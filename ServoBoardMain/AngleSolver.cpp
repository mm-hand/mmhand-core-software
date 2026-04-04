#include "AngleSolver.h"

#include <math.h>
#include <string.h>

#include "pid.h"
#include "TaskSharedData.h"
#include "CalibrationTask.h"

// AngleSolver 模块职责：
// 1) 汇总舵机反馈与编码器反馈，完成关节闭环输入构建；
// 2) 执行目标限幅、双环 PID 解算与两种控制模式下发；
// 3) 处理单关节急停、joint16 双反馈故障与反绕保护状态机。

#ifndef SOLVER_DIAG_LOG_ENABLE
#define SOLVER_DIAG_LOG_ENABLE 0
#endif

// 由 SystemTask 初始化的全局对象与映射表。
extern ServoBusManager servoBus0;
extern ServoBusManager servoBus1;
extern ServoBusManager servoBus2;
extern ServoBusManager servoBus3;
extern AngleSolver angleSolver;
extern JointMapItem jointMap[ENCODER_TOTAL_NUM];
extern MotorMapItem joint16SecondaryMotor;
extern MotorMapItem motorMap[SERVO_TOTAL_NUM];

// 构造函数：清零缓存，等待 init()/setPIDParams() 配置。
AngleSolver::AngleSolver() : _initialized(false)
{
    memset(_zeroOffsets, 0, sizeof(_zeroOffsets));
    memset(_gearRatios, 0, sizeof(_gearRatios));
    memset(_directions, 0, sizeof(_directions));
    memset(_pids, 0, sizeof(_pids));
}

// 初始化解算器基础参数（零偏、传动比、方向）。
void AngleSolver::init(int16_t* zeroOffsets, float* gearRatios, int8_t* directions)
{
    for (int i = 0; i < JOINT_COUNT; i++)
    {
        _zeroOffsets[i] = zeroOffsets[i];
        _gearRatios[i] = gearRatios[i];
        _directions[i] = directions[i];
    }
    _initialized = true;
}

// 设置双环 PID 参数：
// pidParams[0] 为外环（角度环），pidParams[1] 为内环（位置环）。
void AngleSolver::setPIDParams(float pidParams[][PID_PARAMETER_NUM])
{
    for (int i = 0; i < JOINT_COUNT; i++)
    {
        PID_Init(&_pids[i][0], PID_POSITION, pidParams[0]);
        PID_Init(&_pids[i][1], PID_POSITION, pidParams[1]);
    }
}

// 双环解算：
// 1) 外环根据目标角与编码器角度得到位置增量；
// 2) 内环根据目标位置与绝对位置输出舵机目标脉冲。
bool AngleSolver::compute(float* targetDegs, float* magActualDegs,
                          const int32_t* absolutePosition, int16_t* outServoPulses)
{
    for (int i = 0; i < JOINT_COUNT; i++)
    {
        f_PID_Calculate(&_pids[i][0], targetDegs[i], magActualDegs[i]);

        const float loop2Target = _pids[i][0].Output + (float)absolutePosition[i];
        const float loop2Actual = (float)absolutePosition[i];
        f_PID_Calculate(&_pids[i][1], loop2Target, loop2Actual);

        outServoPulses[i] = (int16_t)_pids[i][1].Output + absolutePosition[i];
    }
    return true;
}

// 读取指定关节某一环路的 PID 输出（用于调试上报）。
float AngleSolver::getPidOutput(uint8_t jointIndex, uint8_t loopIndex) const
{
    if (jointIndex >= JOINT_COUNT || loopIndex > 1) {
        return 0.0f;
    }
    return _pids[jointIndex][loopIndex].Output;
}

// 编码器基础常量（14bit，16384counts/圈）。
static const int32_t kEncoderModulo = 16384;
static const int32_t kEncoderHalfTurn = kEncoderModulo / 2;
// 映射时的边界裕量，减少边界跳变导致的误判。
static const int32_t kEncoderMarginCounts = (kEncoderModulo * 10 + 180) / 360;

// 调试包关注的关节索引列表（仅影响调试输出）。
static const uint8_t kDebugJointIndices[] = {4, 5, 6, 7};
static const uint8_t kDebugJointCount =
    (uint8_t)(sizeof(kDebugJointIndices) / sizeof(kDebugJointIndices[0]));

// joint16 双舵机拮抗参数：
// - 副反馈投影到主坐标: S_proj = -S + secondaryOffset
// - 副舵机目标:          T_sub  = -T_main + secondaryOffset + tensionBias
// - 一致性判故障: abs(P - S_proj) > threshold 持续 faultCycles 周期
static const uint8_t kJoint16Index = 16;
static const int32_t kJoint16SecondaryOffset = 0;
static const int32_t kJoint16TensionBias = 0;
static const int32_t kJoint16DiffThresholdCounts = 180;
static const uint8_t kJoint16DiffFaultCycles = 3;

static const uint32_t kCanBusOfflineTimeoutMs = 300;
static const uint16_t kEncoderDisconnectRaw = 0xFFFF;

// 放绳反绕保护阈值（平衡档）。
static const float kReleaseWindowEnterTargetDeltaDeg = -0.3f;
static const float kReleaseWindowExitTargetDeltaDeg = 0.3f;
static const float kReleaseFilterAlpha = 0.25f;
static const float kReleaseActualDeadbandDeg = 0.2f;
static const int32_t kReleaseAccumPrecheckCounts = 256;
static const int32_t kReleaseAccumEarlyFaultCounts = 512;
static const int32_t kReleaseAccumHalfTurnCounts = 2048;
static const int32_t kReleaseAccumFullTurnCounts = 4096;
static const float kReleaseForwardDeltaDeg = 2.0f;
static const float kReleaseMinReturnAccumDeg = 1.5f;
static const uint8_t kReleaseForwardAnomalyCycles = 3;
static const float kReleaseRecoverPullCmdDeg = 2.5f;
static const float kReleaseRecoverTrackErrDeg = 1.5f;
static const uint8_t kReleaseRecoverStableCycles = 8;

typedef struct
{
    uint8_t initialized;
    uint8_t faultActive;
    uint8_t catastrophicFault;
    uint8_t releaseWindowActive;
    int32_t releaseServoAccumCounts;
    float actualReturnAccumDeg;
    float filteredActualDeg;
    uint8_t forwardAnomalyCycles;
    uint8_t recoverStableCycles;
    float prevTargetDeg;
    float prevActualDeg;
    int32_t prevServoPos;
} ReleaseGuardState;

// 协议映射值限幅，避开断连保留值范围。

static int16_t clampMappedCountForProtocol(int32_t value)
{
    if (value > 32766) return 32766;
    if (value < -32768) return -32768;
    return (int16_t)value;
}

static int16_t clampServoPos(int32_t value)
{
    if (value > 30719) return 30719;
    if (value < -30719) return -30719;
    return (int16_t)value;
}

// 按安装方向统一编码器方向，输出范围 [0, kEncoderModulo)。
static int32_t orientEncoderRaw(uint16_t rawValue, int8_t direction)
{
    int32_t oriented = (int32_t)rawValue & (kEncoderModulo - 1);
    if (direction < 0) {
        oriented = (kEncoderModulo - oriented) & (kEncoderModulo - 1);
    }
    return oriented;
}

// 增量折叠到半圈范围，得到最短路径差值。
static int32_t wrapEncoderDelta(int32_t delta)
{
    while (delta > kEncoderHalfTurn) delta -= kEncoderModulo;
    while (delta < -kEncoderHalfTurn) delta += kEncoderModulo;
    return delta;
}

// 优先使用自动标定零偏，失败时回退到手动零偏表。
static int32_t getEncoderOffset(uint8_t jointIndex)
{
    if (jointIndex >= ENCODER_TOTAL_NUM) return 0;
    if (g_jointCalibResult[jointIndex].success) {
        return g_jointCalibResult[jointIndex].offset;
    }
    return g_encoderOffsetManual[jointIndex];
}

// 编码器计数 -> 角度（度）。
static float convertEncoderCountToDeg(int32_t encoderCount)
{
    return (float)encoderCount * 360.0f / (float)kEncoderModulo;
}

// 目标角限幅：约束到 [0, angleScope-bottomReserved-topReserved]（度）。
static float clampJointTargetDegByCalib(float targetDeg, uint8_t jointIndex)
{
    if (!isfinite(targetDeg)) {
        targetDeg = 0.0f;
    }
    if (jointIndex >= ENCODER_TOTAL_NUM) {
        return 0.0f;
    }

    const int32_t maxCount =
        g_jointCalibConfig[jointIndex].angleScope -
        g_jointCalibConfig[jointIndex].bottomReserved -
        g_jointCalibConfig[jointIndex].topReserved;
    const float maxDeg = convertEncoderCountToDeg(maxCount);

    if (!isfinite(maxDeg) || maxDeg <= 0.0f) {
        return 0.0f;
    }
    if (targetDeg < 0.0f) {
        return 0.0f;
    }
    if (targetDeg > maxDeg) {
        return maxDeg;
    }
    return targetDeg;
}

// 单关节急停判定：任一输入状态异常即触发急停。
static bool shouldEmergencyStop(bool canBusOnline,
                                const RemoteSensorData_t& sensorData,
                                const MappedAngleData_t& mappedData,
                                uint8_t jointIndex)
{
    if (!canBusOnline || !sensorData.isValid) {
        return true;
    }
    if (jointIndex >= ENCODER_TOTAL_NUM) {
        return true;
    }
    if (sensorData.errorFlags[jointIndex] != 0) {
        return true;
    }
    if (mappedData.validFlags[jointIndex] == 0) {
        return true;
    }
    return false;
}

// 总线索引 -> 总线管理器实例。
static ServoBusManager* getBusByIndex(uint8_t busIndex)
{
    switch (busIndex)
    {
    case 0: return &servoBus0;
    case 1: return &servoBus1;
    case 2: return &servoBus2;
    case 3: return &servoBus3;
    default: return NULL;
    }
}

// 通过 bus + servoId 找到电机通道索引，找不到返回 -1。
static int findMotorChannel(uint8_t bus, uint8_t id)
{
    for (int i = 0; i < SERVO_TOTAL_NUM; i++)
    {
        if (motorMap[i].busIndex == bus && motorMap[i].servoID == id) {
            return i;
        }
    }
    return -1;
}

// 向某总线读取列表追加目标舵机（自动去重）。
static bool addReadTarget(uint8_t busIds[NUM_BUSES][MAX_SERVOS_PER_BUS],
                          uint8_t busCounts[NUM_BUSES],
                          uint8_t bus,
                          uint8_t id)
{
    if (bus >= NUM_BUSES) {
        return false;
    }
    for (uint8_t i = 0; i < busCounts[bus]; i++)
    {
        if (busIds[bus][i] == id) {
            return true;
        }
    }
    if (busCounts[bus] >= MAX_SERVOS_PER_BUS) {
        return false;
    }
    busIds[bus][busCounts[bus]++] = id;
    return true;
}

static void clearReleaseWindowTracking(ReleaseGuardState* guard)
{
    if (!guard) {
        return;
    }
    guard->releaseWindowActive = 0;
    guard->releaseServoAccumCounts = 0;
    guard->actualReturnAccumDeg = 0.0f;
    guard->forwardAnomalyCycles = 0;
}

static void resetReleaseGuardState(ReleaseGuardState* guard)
{
    if (!guard) {
        return;
    }
    memset(guard, 0, sizeof(*guard));
}

static void resetAllReleaseGuardStates(ReleaseGuardState guards[ENCODER_TOTAL_NUM])
{
    for (uint8_t i = 0; i < ENCODER_TOTAL_NUM; i++)
    {
        resetReleaseGuardState(&guards[i]);
    }
}

static bool updateReleaseGuardState(ReleaseGuardState* guard,
                                    float targetDeg,
                                    float actualDeg,
                                    int32_t servoPos,
                                    bool inputsValid)
{
    if (!guard) {
        return false;
    }

    if (!isfinite(targetDeg)) {
        targetDeg = 0.0f;
    }
    if (!isfinite(actualDeg)) {
        actualDeg = 0.0f;
    }

    if (!guard->initialized)
    {
        guard->initialized = 1;
        guard->filteredActualDeg = actualDeg;
        guard->prevActualDeg = actualDeg;
        guard->prevTargetDeg = targetDeg;
        guard->prevServoPos = servoPos;
        return guard->faultActive != 0;
    }

    const float targetDelta = targetDeg - guard->prevTargetDeg;
    guard->filteredActualDeg =
        kReleaseFilterAlpha * actualDeg + (1.0f - kReleaseFilterAlpha) * guard->filteredActualDeg;
    float actualDelta = guard->filteredActualDeg - guard->prevActualDeg;
    if (fabsf(actualDelta) < kReleaseActualDeadbandDeg) {
        actualDelta = 0.0f;
    }

    int32_t servoDelta = servoPos - guard->prevServoPos;
    if (servoDelta < 0) {
        servoDelta = -servoDelta;
    }

    if (!inputsValid)
    {
        clearReleaseWindowTracking(guard);
        guard->recoverStableCycles = 0;
        guard->initialized = 0;
        guard->prevTargetDeg = targetDeg;
        guard->prevActualDeg = guard->filteredActualDeg;
        guard->prevServoPos = servoPos;
        return guard->faultActive != 0;
    }

    if (targetDelta <= kReleaseWindowEnterTargetDeltaDeg) {
        guard->releaseWindowActive = 1;
    }
    if (targetDelta >= kReleaseWindowExitTargetDeltaDeg) {
        clearReleaseWindowTracking(guard);
    }

    if (guard->releaseWindowActive)
    {
        if (servoDelta > 0) {
            const int32_t remaining = 0x7FFFFFFF - guard->releaseServoAccumCounts;
            guard->releaseServoAccumCounts += (servoDelta > remaining) ? remaining : servoDelta;
        }
        if (actualDelta < 0.0f) {
            guard->actualReturnAccumDeg += -actualDelta;
        }

        if ((guard->releaseServoAccumCounts >= kReleaseAccumPrecheckCounts) &&
            (actualDelta >= kReleaseForwardDeltaDeg))
        {
            if (guard->forwardAnomalyCycles < 255) {
                guard->forwardAnomalyCycles++;
            }
        }
        else
        {
            guard->forwardAnomalyCycles = 0;
        }

        if (guard->releaseServoAccumCounts >= kReleaseAccumFullTurnCounts) {
            guard->faultActive = 1;
            guard->catastrophicFault = 1;
        } else if (guard->releaseServoAccumCounts >= kReleaseAccumHalfTurnCounts) {
            guard->faultActive = 1;
        } else if ((guard->releaseServoAccumCounts >= kReleaseAccumEarlyFaultCounts) &&
                   (guard->actualReturnAccumDeg <= kReleaseMinReturnAccumDeg)) {
            guard->faultActive = 1;
        } else if ((guard->releaseServoAccumCounts >= kReleaseAccumPrecheckCounts) &&
                   (guard->forwardAnomalyCycles >= kReleaseForwardAnomalyCycles)) {
            guard->faultActive = 1;
        }
    }
    else
    {
        guard->releaseServoAccumCounts = 0;
        guard->actualReturnAccumDeg = 0.0f;
        guard->forwardAnomalyCycles = 0;
    }

    if (guard->faultActive && !guard->catastrophicFault)
    {
        bool clearFault = false;
        if (targetDelta >= kReleaseRecoverPullCmdDeg) {
            clearFault = true;
        } else {
            const float trackErr = fabsf(targetDeg - guard->filteredActualDeg);
            if (trackErr <= kReleaseRecoverTrackErrDeg) {
                if (guard->recoverStableCycles < 255) {
                    guard->recoverStableCycles++;
                }
            } else {
                guard->recoverStableCycles = 0;
            }
            if (guard->recoverStableCycles >= kReleaseRecoverStableCycles) {
                clearFault = true;
            }
        }

        if (clearFault)
        {
            guard->faultActive = 0;
            guard->releaseWindowActive = 0;
            guard->releaseServoAccumCounts = 0;
            guard->actualReturnAccumDeg = 0.0f;
            guard->forwardAnomalyCycles = 0;
            guard->recoverStableCycles = 0;
        }
    }

    guard->prevTargetDeg = targetDeg;
    guard->prevActualDeg = guard->filteredActualDeg;
    guard->prevServoPos = servoPos;
    return guard->faultActive != 0;
}

static uint32_t buildReleaseFaultBitmap(const ReleaseGuardState guards[ENCODER_TOTAL_NUM])
{
    uint32_t bitmap = 0;
    for (uint8_t i = 0; i < ENCODER_TOTAL_NUM; i++)
    {
        if (i == kJoint16Index) {
            continue;
        }
        if (guards[i].faultActive) {
            bitmap |= (1UL << i);
        }
    }
    return bitmap;
}

// Solver 任务主循环：
// 1) 读取舵机反馈并上报遥测；
// 2) 构建关节反馈量与传感器映射量；
// 3) 执行限幅与双环解算；
// 4) 根据控制模式与安全状态下发命令。
void taskSolver(void* parameter)
{
    TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;
    if (!sharedData) {
        vTaskDelete(NULL);
        return;
    }

    float localTargets[ENCODER_TOTAL_NUM] = {0.0f};
    float magAngles[ENCODER_TOTAL_NUM] = {0.0f};
    int32_t absolutePosition[ENCODER_TOTAL_NUM] = {0};
    int16_t outPulses[ENCODER_TOTAL_NUM] = {0};

    RemoteSensorData_t sensorData;
    MappedAngleData_t mappedData;
    uint8_t joint16DiffFaultCounter = 0;
    const int joint16PrimaryCh =
        findMotorChannel(jointMap[kJoint16Index].busIndex, jointMap[kJoint16Index].servoID);
    const int joint16SecondaryCh =
        findMotorChannel(joint16SecondaryMotor.busIndex, joint16SecondaryMotor.servoID);
    ReleaseGuardState releaseGuards[ENCODER_TOTAL_NUM];
    resetAllReleaseGuardStates(releaseGuards);
    uint32_t lastReleaseFaultResetToken = sharedData->reverse_release_fault_reset_token;
    sharedData->reverse_release_fault_bitmap = 0;

#if SOLVER_DIAG_LOG_ENABLE
    uint32_t lastDiagLogMs = 0;
#endif

    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t solverPeriodTicks = pdMS_TO_TICKS(10);
    uint8_t readBusPhase = 0;

    while (1)
    {
        const uint32_t releaseFaultResetToken = sharedData->reverse_release_fault_reset_token;
        if (releaseFaultResetToken != lastReleaseFaultResetToken)
        {
            resetAllReleaseGuardStates(releaseGuards);
            sharedData->reverse_release_fault_bitmap = 0;
            lastReleaseFaultResetToken = releaseFaultResetToken;
        }

        bool busWritePending[NUM_BUSES] = {false};
        uint8_t readIds[NUM_BUSES][MAX_SERVOS_PER_BUS] = {0};
        uint8_t readCounts[NUM_BUSES] = {0};
        int16_t jointCmdPos[ENCODER_TOTAL_NUM] = {0};
        uint8_t jointCmdValid[ENCODER_TOTAL_NUM] = {0};

        // 阶段1：收集本周期需要读取的舵机列表。
        for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++)
        {
            addReadTarget(readIds, readCounts, motorMap[ch].busIndex, motorMap[ch].servoID);
        }

        // 阶段2：按总线执行批量同步读取，刷新反馈缓存。
        const uint8_t readStartBus = (readBusPhase == 0) ? 0 : 2;
        for (uint8_t offset = 0; offset < 2; offset++)
        {
            const uint8_t bus = (uint8_t)(readStartBus + offset);
            if (bus >= NUM_BUSES || readCounts[bus] == 0) {
                continue;
            }
            ServoBusManager* pBus = getBusByIndex(bus);
            if (pBus) {
                pBus->syncReadPositions(readIds[bus], readCounts[bus]);
            }
        }
        readBusPhase ^= 1;

        ServoAngleData_t servoData;
        ServoTelemetryData_t telemetryData;
        memset(&servoData, 0, sizeof(servoData));
        memset(&telemetryData, 0, sizeof(telemetryData));
        servoData.timestamp = millis();
        telemetryData.timestamp = servoData.timestamp;

        for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++)
        {
            const uint8_t bus = motorMap[ch].busIndex;
            const uint8_t id = motorMap[ch].servoID;
            ServoBusManager* pBus = getBusByIndex(bus);
            if (pBus && pBus->isOnline(id))
            {
                const int32_t absPos = pBus->getAbsolutePosition(id);
                const ServoFeedback& fb = pBus->getFeedback(id);
                servoData.servoAngles[ch] = absPos;
                servoData.onlineStatus[ch] = 1;
                telemetryData.speed[ch] = fb.speed;
                telemetryData.load[ch] = fb.load;
                telemetryData.voltage[ch] = fb.voltage;
                telemetryData.temperature[ch] = fb.temperature;
                telemetryData.onlineStatus[ch] = 1;
            }
        }

        if (sharedData->servoAngleQueue) {
            xQueueOverwrite(sharedData->servoAngleQueue, &servoData);
        }
        if (sharedData->servoTelemetryQueue) {
            xQueueOverwrite(sharedData->servoTelemetryQueue, &telemetryData);
        }

        // 过载保护逻辑当前关闭，但保留协议状态字段的输出兼容性。
        sharedData->overload_fault_bitmap = 0;

        // 阶段3：从舵机反馈构建“关节侧”绝对位置（闭环输入）。
        memset(absolutePosition, 0, sizeof(absolutePosition));
        for (uint8_t jointIndex = 0; jointIndex < ENCODER_TOTAL_NUM; jointIndex++)
        {
            const uint8_t bus = jointMap[jointIndex].busIndex;
            const uint8_t id = jointMap[jointIndex].servoID;
            const int ch = findMotorChannel(bus, id);
            if (ch >= 0 && servoData.onlineStatus[ch]) {
                absolutePosition[jointIndex] = servoData.servoAngles[ch];
            }
        }

        bool joint16DualFault = false;
        {
            const uint8_t pBus = jointMap[kJoint16Index].busIndex;
            const uint8_t pId = jointMap[kJoint16Index].servoID;
            const uint8_t sBus = joint16SecondaryMotor.busIndex;
            const uint8_t sId = joint16SecondaryMotor.servoID;
            const int pCh = findMotorChannel(pBus, pId);
            const int sCh = findMotorChannel(sBus, sId);
            const bool pOnline = (pCh >= 0) && (servoData.onlineStatus[pCh] != 0);
            const bool sOnline = (sCh >= 0) && (servoData.onlineStatus[sCh] != 0);

            if (pOnline && sOnline)
            {
                const int32_t p = servoData.servoAngles[pCh];
                const int32_t s = servoData.servoAngles[sCh];
                const int32_t sProj = -s + kJoint16SecondaryOffset;
                const int32_t fused = (p + sProj) / 2;
                const int32_t diff = (p >= sProj) ? (p - sProj) : (sProj - p);

                // 双反馈融合后的 joint16 位置用于闭环。
                absolutePosition[kJoint16Index] = fused;

                // 连续超差判故障，抑制单周期抖动误触发。
                if (diff > kJoint16DiffThresholdCounts) {
                    if (joint16DiffFaultCounter < 255) {
                        joint16DiffFaultCounter++;
                    }
                } else {
                    joint16DiffFaultCounter = 0;
                }
                joint16DualFault = (joint16DiffFaultCounter >= kJoint16DiffFaultCycles);
            }
            else
            {
                joint16DiffFaultCounter = 0;
                joint16DualFault = true;
            }
        }
        sharedData->joint16_dual_feedback_fault = joint16DualFault ? 1 : 0;

#if SOLVER_DIAG_LOG_ENABLE
        const uint32_t nowMs = millis();
        if (nowMs - lastDiagLogMs >= 500) {
            lastDiagLogMs = nowMs;
            const int pCh = findMotorChannel(jointMap[kJoint16Index].busIndex, jointMap[kJoint16Index].servoID);
            const int sCh = findMotorChannel(joint16SecondaryMotor.busIndex, joint16SecondaryMotor.servoID);
            const long pAbs = (pCh >= 0) ? (long)servoData.servoAngles[pCh] : 0;
            const long sAbs = (sCh >= 0) ? (long)servoData.servoAngles[sCh] : 0;
            Serial.printf("[Solver] j16 p=%ld s=%ld fault=%d\r\n", pAbs, sAbs, joint16DualFault ? 1 : 0);
        }
#endif

        bool canBusOnline = false;
        if (xQueuePeek(sharedData->canRxQueue, &sensorData, 0) == pdTRUE && sensorData.isValid) {
            const uint32_t nowMs = millis();
            canBusOnline = (nowMs - sensorData.timestamp) <= kCanBusOfflineTimeoutMs;
        }

        memset(&mappedData, 0, sizeof(mappedData));
        mappedData.timestamp = millis();
        mappedData.isValid = canBusOnline;

        if (canBusOnline)
        {
            for (int i = 0; i < ENCODER_TOTAL_NUM; i++)
            {
                if (sensorData.encoderValues[i] == kEncoderDisconnectRaw) {
                    mappedData.validFlags[i] = 0;
                    magAngles[i] = 0.0f;
                    continue;
                }

                const int32_t orientedRaw = orientEncoderRaw(sensorData.encoderValues[i], g_encoderDirection[i]);
                int32_t mappedCount = orientedRaw;
                if (g_jointCalibResult[i].success) {
                    const int32_t offset = getEncoderOffset((uint8_t)i);
                    const int32_t angleScope = g_jointCalibConfig[i].angleScope;
                    const int32_t bottomReserved = g_jointCalibConfig[i].bottomReserved;
                    int32_t delta = orientedRaw - offset + bottomReserved;
                    delta %= kEncoderModulo;
                    if (delta < 0) delta += kEncoderModulo;
                    if (delta > angleScope + kEncoderMarginCounts) {
                        delta -= kEncoderModulo;
                    }
                    mappedCount = delta - bottomReserved;
                }

                mappedData.angleValues[i] = clampMappedCountForProtocol(mappedCount);
                mappedData.validFlags[i] = 1;
                magAngles[i] = convertEncoderCountToDeg(mappedCount);
            }
        }

        if (sharedData->mappedAngleQueue) {
            xQueueOverwrite(sharedData->mappedAngleQueue, &mappedData);
        }

        if (xSemaphoreTake(sharedData->targetAnglesMutex, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            memcpy(localTargets, sharedData->targetAngles, sizeof(localTargets));
            xSemaphoreGive(sharedData->targetAnglesMutex);
        }
        const bool hostControlEnabled = (sharedData->control_enabled != 0);
        const bool controlEnabled = hostControlEnabled;

        // 仅关节控制模式下执行角度限幅（直控模式透传原始目标）。
        if (sharedData->control_mode == CONTROL_MODE_JOINT) {
            for (uint8_t jointIndex = 0; jointIndex < ENCODER_TOTAL_NUM; jointIndex++) {
                localTargets[jointIndex] = clampJointTargetDegByCalib(localTargets[jointIndex], jointIndex);
            }
        }

        // 阶段4：执行双环解算，得到每关节目标脉冲。
        angleSolver.compute(localTargets, magAngles, absolutePosition, outPulses);

        if (sharedData->control_mode == CONTROL_MODE_DIRECT_MOTOR)
        {
            // 直控模式：按 motorTargetRaw 下发；joint16 双反馈故障仅作用于主副舵机对。
            if (controlEnabled) {
                for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++)
                {
                    const uint8_t bus = motorMap[ch].busIndex;
                    const uint8_t id = motorMap[ch].servoID;
                    ServoBusManager* pBus = getBusByIndex(bus);
                    if (!pBus) {
                        continue;
                    }

                    const bool isJoint16Motor =
                        ((int)ch == joint16PrimaryCh) || ((int)ch == joint16SecondaryCh);
                    if (joint16DualFault && isJoint16Motor)
                    {
                        if (pBus->isOnline(id)) {
                            const int16_t holdPos = clampServoPos(servoData.servoAngles[ch]);
                            pBus->setTarget(id, holdPos, 1000, 50);
                            busWritePending[bus] = true;
                        }
                        continue;
                    }

                    const int16_t targetPos = clampServoPos(sharedData->motorTargetRaw[ch]);
                    pBus->setTarget(id, targetPos, 1000, 50);
                    busWritePending[bus] = true;
                }
            }
        }
        else
        {
            // 关节模式：按关节逐一评估急停条件并下发目标。
            for (uint8_t jointIndex = 0; jointIndex < ENCODER_TOTAL_NUM; jointIndex++)
            {
                const uint8_t bus = jointMap[jointIndex].busIndex;
                const uint8_t id = jointMap[jointIndex].servoID;
                ServoBusManager* pBus = getBusByIndex(bus);
                bool releaseFaultActive = false;

                if (jointIndex != kJoint16Index)
                {
                    const int jointMotorCh = findMotorChannel(bus, id);
                    const bool servoOnline = (jointMotorCh >= 0) && (servoData.onlineStatus[jointMotorCh] != 0);
                    const bool mappedValid = canBusOnline && (mappedData.validFlags[jointIndex] != 0);
                    const bool inputsValid = servoOnline && mappedValid;
                    const int32_t servoPos =
                        (jointMotorCh >= 0) ? servoData.servoAngles[jointMotorCh] : absolutePosition[jointIndex];
                    releaseFaultActive = updateReleaseGuardState(&releaseGuards[jointIndex],
                                                                 localTargets[jointIndex],
                                                                 magAngles[jointIndex],
                                                                 servoPos,
                                                                 inputsValid);
                }

                bool emergency =
                    (!controlEnabled) ||
                    shouldEmergencyStop(canBusOnline, sensorData, mappedData, jointIndex);
                if (jointIndex == kJoint16Index) {
                    emergency = emergency || joint16DualFault;
                } else {
                    emergency = emergency || releaseFaultActive;
                }

                if (!pBus) {
                    continue;
                }

                if (emergency)
                {
                    // 急停策略：保持当前舵机位置，避免异常状态下继续运动。
                    if (pBus->isOnline(id)) {
                        const int16_t holdPos = clampServoPos(absolutePosition[jointIndex]);
                        pBus->setTarget(id, holdPos, 1000, 50);
                        busWritePending[bus] = true;
                        jointCmdPos[jointIndex] = holdPos;
                        jointCmdValid[jointIndex] = 1;
                    }

                    if (jointIndex == kJoint16Index) {
                        ServoBusManager* pSecBus = getBusByIndex(joint16SecondaryMotor.busIndex);
                        if (pSecBus && pSecBus->isOnline(joint16SecondaryMotor.servoID)) {
                            const int secCh = findMotorChannel(joint16SecondaryMotor.busIndex, joint16SecondaryMotor.servoID);
                            const int16_t holdSec = (secCh >= 0) ? clampServoPos(servoData.servoAngles[secCh]) : 0;
                            pSecBus->setTarget(joint16SecondaryMotor.servoID, holdSec, 1000, 50);
                            busWritePending[joint16SecondaryMotor.busIndex] = true;
                        }
                    }
                    continue;
                }

                const int16_t targetPos = clampServoPos(outPulses[jointIndex]);
                pBus->setTarget(id, targetPos, 1000, 50);
                busWritePending[bus] = true;
                jointCmdPos[jointIndex] = targetPos;
                jointCmdValid[jointIndex] = 1;

                if (jointIndex == kJoint16Index) {
                    ServoBusManager* pSecBus = getBusByIndex(joint16SecondaryMotor.busIndex);
                    if (pSecBus) {
                        // joint16 拮抗下发：主舵机目标确定后，副舵机按反向映射与偏置补偿计算。
                        const int32_t secTarget = -((int32_t)targetPos) + kJoint16SecondaryOffset + kJoint16TensionBias;
                        pSecBus->setTarget(joint16SecondaryMotor.servoID, clampServoPos(secTarget), 1000, 50);
                        busWritePending[joint16SecondaryMotor.busIndex] = true;
                    }
                }
            }
        }

        sharedData->reverse_release_fault_bitmap = buildReleaseFaultBitmap(releaseGuards);

        if (sharedData->jointDebugQueue)
        {
            // 调试上报：发送指定关节的目标/反馈/PID 输出与命令位置。
            for (uint8_t di = 0; di < kDebugJointCount; di++)
            {
                const uint8_t jointIndex = kDebugJointIndices[di];
                JointDebugData_t debugData;
                memset(&debugData, 0, sizeof(debugData));
                debugData.jointIndex = jointIndex;
                debugData.timestamp = millis();
                debugData.valid = (mappedData.validFlags[jointIndex] != 0) ? 1 : 0;
                debugData.targetDeg = localTargets[jointIndex];
                debugData.magActualDeg = magAngles[jointIndex];
                debugData.loop1Output = angleSolver.getPidOutput(jointIndex, 0);
                debugData.loop2Actual = (float)absolutePosition[jointIndex];
                debugData.loop2Output = angleSolver.getPidOutput(jointIndex, 1);
                debugData.cmdTargetPos = jointCmdPos[jointIndex];
                debugData.cmdValid = jointCmdValid[jointIndex];

                if (xQueueSend(sharedData->jointDebugQueue, &debugData, 0) != pdTRUE) {
                    JointDebugData_t drop;
                    xQueueReceive(sharedData->jointDebugQueue, &drop, 0);
                    xQueueSend(sharedData->jointDebugQueue, &debugData, 0);
                }
            }
        }

        // 阶段5：按总线统一 flush 本周期写缓存，降低串口占用。
        for (uint8_t bus = 0; bus < NUM_BUSES; bus++)
        {
            if (!busWritePending[bus]) {
                continue;
            }
            ServoBusManager* pBus = getBusByIndex(bus);
            if (pBus) {
                pBus->syncWriteAll();
            }
        }

        vTaskDelayUntil(&lastWakeTime, solverPeriodTicks);
    }
}
