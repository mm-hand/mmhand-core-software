#include "ControlTask.h"

#include <math.h>
#include <string.h>

#include "../calibration/CalibrationTask.h"
#include "../hardware/HardwareMap.h"
#include "../system/StateMachineTask.h"
#include "ControlOutputBuilder.h"
#include "ControlSafety.h"
#include "ControlSolver.h"
#include "DualTendonControl.h"

#ifndef SOLVER_DIAG_LOG_ENABLE
#define SOLVER_DIAG_LOG_ENABLE 0
#endif

// Match the legacy RoboticHand protection policy: keep reverse/release guard
// disabled during wiring and manual bring-up unless explicitly enabled.
#ifndef REVERSE_GUARD_ENABLE
#define REVERSE_GUARD_ENABLE 0
#endif

static ControlSolver g_controlSolver;

static const int32_t kEncoderModulo = 16384;
static const int32_t kEncoderMarginCounts = (kEncoderModulo * 10 + 180) / 360;

// 默认上报的关节调试通道。
static const uint8_t kDebugJointIndices[] = {8,9,10,11};
static const uint8_t kDebugJointCount =
    (uint8_t)(sizeof(kDebugJointIndices) / sizeof(kDebugJointIndices[0]));

static const uint32_t kCanBusOfflineTimeoutMs = 300;
static const uint32_t kServoTelemetryTimeoutMs = 500;
static const uint16_t kEncoderDisconnectRaw = 0xFFFF;
static const int16_t kControlOverloadLoadThreshold = 700;
static const int16_t kJointControlOverloadLoadThreshold = 800;
static const uint8_t kControlOverloadLatchCycles = 20;
static const uint8_t kJointControlOverloadLatchCycles = 20;
static const uint16_t kJointControlUnlimitedSpeed = 0;
static const int32_t kDirectRunDoneAbsToleranceCounts = 40;
static const uint8_t kDirectRunDoneStableCycles = 5;
static const float kFeedbackRunDoneToleranceDeg = 1.0f;
static const uint8_t kFeedbackRunDoneStableCycles = 10;
static const float kLocalTestTwoPi = 6.28318530717958647692f;

// 协议字段使用 int16 传输，保留最大正值 32767 作为异常/占位，正常值限制到 32766。

static int16_t clampMappedCountForProtocol(int32_t value)
{
    if (value > 32766) return 32766;
    if (value < -32768) return -32768;
    return (int16_t)value;
}

// 按安装方向统一磁编码器方向，输出范围保持在 [0, kEncoderModulo)。
static int32_t orientEncoderRaw(uint16_t rawValue, int8_t direction)
{
    int32_t oriented = (int32_t)rawValue & (kEncoderModulo - 1);
    if (direction < 0) {
        oriented = (kEncoderModulo - oriented) & (kEncoderModulo - 1);
    }
    return oriented;
}

static int32_t wrapEncoderCount(int32_t value)
{
    value %= kEncoderModulo;
    if (value < 0) value += kEncoderModulo;
    return value;
}

static int32_t forwardEncoderDelta(int32_t from, int32_t to)
{
    return wrapEncoderCount(to - from);
}

// Encoder counts to degrees.
static float convertEncoderCountToDeg(int32_t encoderCount)
{
    return (float)encoderCount * 360.0f / (float)kEncoderModulo;
}

static int32_t degToEncoderCount(float deg)
{
    const float scaled = deg * (float)kEncoderModulo / 360.0f;
    return (int32_t)(scaled + (scaled >= 0.0f ? 0.5f : -0.5f));
}

static float sanitizeLocalTestFreq(float freqHz)
{
    if (!isfinite(freqHz) || freqHz < 0.001f) {
        return 0.001f;
    }
    if (freqHz > 100.0f) {
        return 100.0f;
    }
    return freqHz;
}

static float sanitizeLocalTestAmp(float ampDeg)
{
    if (!isfinite(ampDeg)) {
        return 0.0f;
    }
    if (ampDeg > 180.0f) {
        return 180.0f;
    }
    if (ampDeg < -180.0f) {
        return -180.0f;
    }
    return ampDeg;
}

static float mapCalibratedEncoderToDeg(const JointCalibrationResult& calib, int32_t orientedRaw)
{
    if (calib.scaled && calib.encoderSpan > 0 && calib.totalDeg > 0.0f) {
        int32_t delta = forwardEncoderDelta(calib.startEncoder, orientedRaw);
        if (delta > calib.encoderSpan + kEncoderMarginCounts) {
            delta -= kEncoderModulo;
        }
        return ((float)delta / (float)calib.encoderSpan) * calib.totalDeg + calib.startDeg;
    }

    const int32_t offset = calib.offset;
    int32_t delta = forwardEncoderDelta(offset, orientedRaw);
    int32_t hardMaxFromOffset = forwardEncoderDelta(offset, calib.encoderMax);
    if (hardMaxFromOffset <= 0) {
        hardMaxFromOffset = degToEncoderCount(calib.angleMax);
    }
    if (delta > hardMaxFromOffset + kEncoderMarginCounts) {
        delta -= kEncoderModulo;
    }
    return convertEncoderCountToDeg(delta);
}

static float clampJointTargetDegByCalib(float targetDeg, uint8_t jointIndex)
{
    if (!isfinite(targetDeg)) {
        targetDeg = 0.0f;
    }
    if (jointIndex >= ENCODER_TOTAL_NUM) {
        return 0.0f;
    }

    float maxDeg = 0.0f;
    if (g_jointCalibResult[jointIndex].success) {
        maxDeg = g_jointCalibResult[jointIndex].angleMax;
    } else {
        maxDeg =
            g_jointCalibConfig[jointIndex].angleScope -
            g_jointCalibConfig[jointIndex].bottomReserved -
            g_jointCalibConfig[jointIndex].topReserved;
    }

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

// Find the motorMap channel for a physical bus/id pair.
static int findMotorChannel(uint8_t bus, uint8_t id)
{
    return findMotorChannelByBusId(bus, id);
}

static uint8_t appendJointActuatorHoldTargets(ServoTargetBatch_t* batch,
                                              uint8_t jointIndex,
                                              const ServoAngleData_t& servoData,
                                              uint16_t speed,
                                              uint8_t acc)
{
    if (!batch) {
        return 0;
    }

    JointActuatorRuntimeTarget targets[MAX_JOINT_ACTUATORS];
    uint8_t targetCount = 0;
    const uint8_t actuatorCount = getJointActuatorCount(jointIndex);
    for (uint8_t i = 0; i < actuatorCount && targetCount < MAX_JOINT_ACTUATORS; i++) {
        JointActuatorItem actuator;
        if (!getJointActuator(jointIndex, i, &actuator) ||
            actuator.motorChannel >= SERVO_TOTAL_NUM ||
            servoData.onlineStatus[actuator.motorChannel] == 0) {
            continue;
        }
        const MotorMapItem& motor = motorMap[actuator.motorChannel];
        JointActuatorRuntimeTarget& target = targets[targetCount++];
        target.motorChannel = actuator.motorChannel;
        target.busIndex = motor.busIndex;
        target.servoId = motor.servoID;
        target.sign = actuator.sign;
        target.targetAbs = clampServoPos(servoData.servoAngles[actuator.motorChannel]);
    }

    return appendJointActuatorRuntimeTargets(batch, targets, targetCount, speed, acc);
}

static uint8_t appendDualTendonTargets(ServoTargetBatch_t* batch,
                                       TaskSharedData_t* sharedData,
                                       uint8_t jointIndex,
                                       float qRefDeg,
                                       float qFbDeg,
                                       const ServoAngleData_t& servoData,
                                       uint32_t commandToken,
                                       uint16_t speed,
                                       uint8_t acc)
{
    if (!batch || !sharedData) {
        return 0;
    }

    DualTendonMotorTargets tendonTargets;
    memset(&tendonTargets, 0, sizeof(tendonTargets));
    if (!computeDualTendonMotorTargets(jointIndex,
                                       qRefDeg,
                                       qFbDeg,
                                       &servoData,
                                       commandToken,
                                       &tendonTargets)) {
        return 0;
    }

    JointActuatorRuntimeTarget runtimeTargets[MAX_JOINT_ACTUATORS];
    uint8_t targetCount = 0;
    for (uint8_t i = 0; i < tendonTargets.count && targetCount < MAX_JOINT_ACTUATORS; i++) {
        const uint8_t motorChannel = tendonTargets.motorChannel[i];
        if (motorChannel >= SERVO_TOTAL_NUM || !isMotorChannelValid(motorChannel)) {
            continue;
        }
        const MotorMapItem& motor = motorMap[motorChannel];
        const int16_t clampedTarget = clampServoPosByMotorAbsLimit(sharedData,
                                                                   motorChannel,
                                                                   tendonTargets.targetAbs[i]);
        JointActuatorRuntimeTarget& target = runtimeTargets[targetCount++];
        target.motorChannel = motorChannel;
        target.busIndex = motor.busIndex;
        target.servoId = motor.servoID;
        target.sign = 1;
        target.targetAbs = clampedTarget;
    }

    return appendJointActuatorRuntimeTargets(batch, runtimeTargets, targetCount, speed, acc);
}

static int16_t servoLoadMagnitude(int16_t rawLoad)
{
    int16_t mag = rawLoad;
    if (mag < 0) {
        mag = -mag;
    }
    return (int16_t)(mag & 0x03FF);
}

static uint32_t activeOverloadMotorMask(TaskSharedData_t* sharedData,
                                        const ControlCommandSnapshot_t& commandSnapshot)
{
    if (!sharedData) {
        return 0;
    }
    const uint32_t validMask = validMotorChannelMask();
    if (commandSnapshot.controlMode == CONTROL_MODE_DIRECT_MOTOR) {
        return sharedData->active_motor_mask & validMask;
    }

    uint32_t mask = 0;
    const uint32_t activeJointMask =
        sharedData->active_joint_mask & ((1UL << ENCODER_TOTAL_NUM) - 1UL);
    for (uint8_t jointIndex = 0; jointIndex < ENCODER_TOTAL_NUM; jointIndex++) {
        if ((activeJointMask & (1UL << jointIndex)) == 0) {
            continue;
        }
        const uint8_t actuatorCount = getJointActuatorCount(jointIndex);
        for (uint8_t actuatorIndex = 0; actuatorIndex < actuatorCount; actuatorIndex++) {
            JointActuatorItem actuator;
            if (getJointActuator(jointIndex, actuatorIndex, &actuator) &&
                actuator.motorChannel < SERVO_TOTAL_NUM) {
                mask |= (1UL << actuator.motorChannel);
            }
        }
    }
    return mask & validMask;
}

static bool jointHasOverloadFault(uint8_t jointIndex, uint32_t overloadFaultBitmap)
{
    const uint8_t actuatorCount = getJointActuatorCount(jointIndex);
    for (uint8_t actuatorIndex = 0; actuatorIndex < actuatorCount; actuatorIndex++) {
        JointActuatorItem actuator;
        if (getJointActuator(jointIndex, actuatorIndex, &actuator) &&
            actuator.motorChannel < SERVO_TOTAL_NUM &&
            (overloadFaultBitmap & (1UL << actuator.motorChannel)) != 0) {
            return true;
        }
    }
    return false;
}

static int32_t abs32Local(int32_t value)
{
    return value < 0 ? -value : value;
}

static bool directAbsoluteTargetsReached(TaskSharedData_t* sharedData,
                                         const ServoAngleData_t& servoData,
                                         const ControlCommandSnapshot_t& commandSnapshot)
{
    if (!sharedData ||
        commandSnapshot.controlMode != CONTROL_MODE_DIRECT_MOTOR ||
        commandSnapshot.motorDirectCommandSource != MOTOR_DIRECT_SOURCE_ABSOLUTE) {
        return false;
    }

    const uint32_t requiredMask = sharedData->active_motor_mask & validMotorChannelMask();
    if (requiredMask == 0) {
        return false;
    }

    bool checkedAny = false;
    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        if (((requiredMask >> ch) & 0x1UL) == 0) {
            continue;
        }
        checkedAny = true;
        if (servoData.onlineStatus[ch] == 0) {
            return false;
        }
        const int32_t targetAbs =
            (int32_t)clampServoPosByMotorAbsLimit(sharedData,
                                                  ch,
                                                  commandSnapshot.motorTargetRaw[ch]);
        if (abs32Local(servoData.servoAngles[ch] - targetAbs) > kDirectRunDoneAbsToleranceCounts) {
            return false;
        }
    }
    return checkedAny;
}

static bool feedbackJointTargetsReached(TaskSharedData_t* sharedData,
                                        const MappedAngleData_t& mappedData,
                                        const float* targetDegs,
                                        const float* actualDegs)
{
    if (!sharedData || !targetDegs || !actualDegs || !mappedData.isValid) {
        return false;
    }

    const uint32_t requiredMask = sharedData->active_joint_mask & ((1UL << ENCODER_TOTAL_NUM) - 1UL);
    if (requiredMask == 0) {
        return false;
    }

    bool checkedAny = false;
    for (uint8_t jointIndex = 0; jointIndex < ENCODER_TOTAL_NUM; jointIndex++) {
        if (((requiredMask >> jointIndex) & 0x1UL) == 0) {
            continue;
        }
        checkedAny = true;
        if (mappedData.validFlags[jointIndex] == 0) {
            return false;
        }
        if (fabsf(actualDegs[jointIndex] - targetDegs[jointIndex]) > kFeedbackRunDoneToleranceDeg) {
            return false;
        }
    }

    return checkedAny;
}

static bool readControlCommandSnapshot(TaskSharedData_t* sharedData, ControlCommandSnapshot_t* out)
{
// Read host commands once per control cycle to avoid mixed partial updates.
    if (!sharedData || !out) {
        return false;
    }

    SemaphoreHandle_t lock = sharedData->commandStateMutex ? sharedData->commandStateMutex : sharedData->targetAnglesMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(2)) != pdTRUE) {
        return false;
    }

    memcpy(out->targetAngles, sharedData->targetAngles, sizeof(out->targetAngles));
    memcpy(out->motorTargetRaw, sharedData->motorTargetRaw, sizeof(out->motorTargetRaw));
    memcpy(out->motorSweepTargetRaw, sharedData->motorSweepTargetRaw, sizeof(out->motorSweepTargetRaw));
    out->motorCommandToken = sharedData->motor_command_token;
    out->motorSweepCommandToken = sharedData->motor_sweep_command_token;
    out->jointCommandToken = sharedData->joint_command_token;
    out->motorDirectCommandGeneration = sharedData->motor_direct_command_generation;
    out->motorDirectCommandSource = sharedData->motor_direct_command_source;
    out->controlMode = sharedData->control_mode;
    out->sineTestEnabled = sharedData->sine_test_enabled;
    out->sineTestRequestToken = sharedData->sine_test_request_token;
    out->sineTestStartMs = sharedData->sine_test_start_ms;
    out->sineTestJointMask = sharedData->sine_test_joint_mask;
    out->sineTestHoldMs = sharedData->sine_test_hold_ms;
    memcpy(out->sineTestCenterDeg, sharedData->sine_test_center_deg, sizeof(out->sineTestCenterDeg));
    memcpy(out->sineTestAmpDeg, sharedData->sine_test_amp_deg, sizeof(out->sineTestAmpDeg));
    memcpy(out->sineTestFreqHz, sharedData->sine_test_freq_hz, sizeof(out->sineTestFreqHz));
    memcpy(out->sineTestPhaseRad, sharedData->sine_test_phase_rad, sizeof(out->sineTestPhaseRad));
    memcpy(out->tendonGuardEnabled, (const void*)sharedData->tendon_guard_enabled, sizeof(out->tendonGuardEnabled));
    memcpy(out->tendonGuardSign, (const void*)sharedData->tendon_guard_sign, sizeof(out->tendonGuardSign));
    memcpy(out->tendonGuardX1Abs, (const void*)sharedData->tendon_guard_x1_abs, sizeof(out->tendonGuardX1Abs));

    if (lock) {
        xSemaphoreGive(lock);
    }
    return true;
}

// ============================================================================
// ControlTask - 核心实时控制任务
// ============================================================================
// 运行频率：通常为 100Hz (10ms 周期)
// 职责：
// 1) 收集总线传来的伺服电机绝对位置与磁编码器角度。
// 2) 读取上位机下发的命令快照 (ControlCommandSnapshot_t)。
// 3) 根据不同模式 (CONTROL_MODE_JOINT 闭环 / CONTROL_MODE_DIRECT_MOTOR 直控) 执行分发。
// 4) 执行安全检查（如急停、过载、反向释放保护、工作空间限位）。
// 5) 调用双环 PID 求解器 (ControlSolver) 获得目标脉冲，并组装 Batch 交给底层。
// ============================================================================
void controlTask(void* parameter)
{
    TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;
    if (!sharedData) {
        vTaskDelete(NULL);
        return;
    }

    g_controlSolver.begin();

    float localTargets[ENCODER_TOTAL_NUM] = {0.0f};
    float magAngles[ENCODER_TOTAL_NUM] = {0.0f};
    int32_t absolutePositionRaw[ENCODER_TOTAL_NUM] = {0};
    int32_t absolutePositionTendon[ENCODER_TOTAL_NUM] = {0};
    int32_t outPulsesTendon[ENCODER_TOTAL_NUM] = {0};

    RemoteSensorData_t sensorData;
    MappedAngleData_t mappedData;
    ReleaseGuardState releaseGuards[ENCODER_TOTAL_NUM];
    resetAllReleaseGuardStates(releaseGuards);
    uint32_t lastReleaseFaultResetToken = sharedData->reverse_release_fault_reset_token;
    sharedData->reverse_release_fault_bitmap = 0;
    uint32_t overloadFaultLatched = 0;
    uint8_t overloadHighCount[SERVO_TOTAL_NUM] = {0};
    uint32_t lastOverloadFaultResetToken = sharedData->overload_fault_reset_token;
    sharedData->overload_fault_bitmap = 0;
    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        sharedData->overload_fault_load[ch] = 0;
    }
    ControlCommandSnapshot_t commandSnapshot;
    memset(&commandSnapshot, 0, sizeof(commandSnapshot));
    readControlCommandSnapshot(sharedData, &commandSnapshot);

#if SOLVER_DIAG_LOG_ENABLE
    uint32_t lastDiagLogMs = 0;
#endif

    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t solverPeriodTicks = pdMS_TO_TICKS(10);
    bool prevMotorOnline[SERVO_TOTAL_NUM] = {false};
    bool hotplugHoldMotor[SERVO_TOTAL_NUM] = {false};
    uint32_t lastMotorCommandToken = commandSnapshot.motorCommandToken;
    uint32_t lastMotorSweepCommandToken = commandSnapshot.motorSweepCommandToken;
    uint32_t lastJointCommandToken = commandSnapshot.jointCommandToken;
    uint8_t lastMotorDirectSource = commandSnapshot.motorDirectCommandSource;
    int16_t prevSweepRawTarget[SERVO_TOTAL_NUM] = {0};
    int32_t sweepExpandedTarget[SERVO_TOTAL_NUM] = {0};
    bool sweepAnchorValid[SERVO_TOTAL_NUM] = {false};
    uint8_t directRunDoneStableCount = 0;
    uint8_t feedbackRunDoneStableCount = 0;
    uint32_t lastSineTestRequestToken = commandSnapshot.sineTestRequestToken;
    bool sineTestActive = false;
    uint32_t sineTestStartMs = 0;

    while (1)
    {
        if (!readControlCommandSnapshot(sharedData, &commandSnapshot)) {
            vTaskDelayUntil(&lastWakeTime, solverPeriodTicks);
            continue;
        }

        const uint32_t motorCommandToken = commandSnapshot.motorCommandToken;
        if (motorCommandToken != lastMotorCommandToken)
        {
            memset(hotplugHoldMotor, 0, sizeof(hotplugHoldMotor));
            lastMotorCommandToken = motorCommandToken;
        }

        const uint32_t motorSweepCommandToken = commandSnapshot.motorSweepCommandToken;
        if (motorSweepCommandToken != lastMotorSweepCommandToken)
        {
            memset(hotplugHoldMotor, 0, sizeof(hotplugHoldMotor));
            lastMotorSweepCommandToken = motorSweepCommandToken;
        }

        const uint32_t jointCommandToken = commandSnapshot.jointCommandToken;
        if (jointCommandToken != lastJointCommandToken)
        {
            memset(hotplugHoldMotor, 0, sizeof(hotplugHoldMotor));
            feedbackRunDoneStableCount = 0;
            lastJointCommandToken = jointCommandToken;
        }

        const uint8_t motorDirectSource = commandSnapshot.motorDirectCommandSource;
        if (motorDirectSource != lastMotorDirectSource)
        {
            memset(sweepAnchorValid, 0, sizeof(sweepAnchorValid));
            lastMotorDirectSource = motorDirectSource;
        }

        const uint32_t releaseFaultResetToken = sharedData->reverse_release_fault_reset_token;
        if (releaseFaultResetToken != lastReleaseFaultResetToken)
        {
            resetAllReleaseGuardStates(releaseGuards);
            sharedData->reverse_release_fault_bitmap = 0;
            lastReleaseFaultResetToken = releaseFaultResetToken;
        }

        const uint32_t overloadFaultResetToken = sharedData->overload_fault_reset_token;
        if (overloadFaultResetToken != lastOverloadFaultResetToken)
        {
            overloadFaultLatched = 0;
            memset(overloadHighCount, 0, sizeof(overloadHighCount));
            sharedData->overload_fault_bitmap = 0;
            for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
                sharedData->overload_fault_load[ch] = 0;
            }
            lastOverloadFaultResetToken = overloadFaultResetToken;
        }

        ServoTargetBatch_t targetBatch;
        memset(&targetBatch, 0, sizeof(targetBatch));
        targetBatch.timestamp = millis();
        targetBatch.source = SERVO_TARGET_OWNER_CONTROL;
        int16_t jointCmdPos[ENCODER_TOTAL_NUM] = {0};
        uint8_t jointCmdValid[ENCODER_TOTAL_NUM] = {0};

        ServoAngleData_t servoData;
        memset(&servoData, 0, sizeof(servoData));
        servoData.timestamp = millis();

        if (sharedData->servoFeedbackQueue) {
            xQueuePeek(sharedData->servoFeedbackQueue, &servoData, 0);
        }

        ServoTelemetryData_t telemetryData;
        memset(&telemetryData, 0, sizeof(telemetryData));
        bool telemetryFresh = false;
        if (sharedData->servoTelemetrySnapshotQueue &&
            xQueuePeek(sharedData->servoTelemetrySnapshotQueue, &telemetryData, 0) == pdTRUE) {
            telemetryFresh =
                (uint32_t)(millis() - telemetryData.timestamp) <= kServoTelemetryTimeoutMs;
        }

        for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++)
        {
            const bool onlineNow = (servoData.onlineStatus[ch] != 0);
            if (onlineNow && !prevMotorOnline[ch]) {
                hotplugHoldMotor[ch] = true;
                sweepAnchorValid[ch] = false;
            }
            if (!onlineNow) {
                sweepAnchorValid[ch] = false;
            }
            prevMotorOnline[ch] = onlineNow;
        }

        // 过载保护锁存位由运行期检测维护，并继续写入共享状态供上位机显示。
        sharedData->overload_fault_bitmap |= overloadFaultLatched;
        overloadFaultLatched = sharedData->overload_fault_bitmap;

        // 从主执行舵机反馈构建每个关节的绝对位置，供闭环 PID 与安全判断使用。
        memset(absolutePositionRaw, 0, sizeof(absolutePositionRaw));
        memset(absolutePositionTendon, 0, sizeof(absolutePositionTendon));
        for (uint8_t jointIndex = 0; jointIndex < ENCODER_TOTAL_NUM; jointIndex++)
        {
            const uint8_t bus = jointMap[jointIndex].busIndex;
            const uint8_t id = jointMap[jointIndex].servoID;
            const int ch = findMotorChannel(bus, id);
            if (ch >= 0 && servoData.onlineStatus[ch]) {
                const int32_t rawAbs = servoData.servoAngles[ch];
                absolutePositionRaw[jointIndex] = rawAbs;
                absolutePositionTendon[jointIndex] =
                    motorRawAbsToTendonAbs((uint8_t)ch, rawAbs);
            }
        }

        sharedData->joint16_dual_feedback_fault = 0;

#if SOLVER_DIAG_LOG_ENABLE
        const uint32_t nowMs = millis();
        if (nowMs - lastDiagLogMs >= 500) {
            lastDiagLogMs = nowMs;
            Serial.printf("[Solver] j16 qRef=%.2f qFb=%.2f\r\n",
                          (double)localTargets[kJoint16Index],
                          (double)magAngles[kJoint16Index]);
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
                if (!g_jointCalibResult[i].success) {
                    mappedData.angleValues[i] = 0;
                    mappedData.validFlags[i] = 0;
                    magAngles[i] = 0.0f;
                    continue;
                }

                const JointCalibrationResult& calib = g_jointCalibResult[i];
                const float mappedDeg = mapCalibratedEncoderToDeg(calib, orientedRaw);
                const int32_t mappedCount = degToEncoderCount(mappedDeg);
                mappedData.angleValues[i] = clampMappedCountForProtocol(mappedCount);
                mappedData.validFlags[i] = 1;
                magAngles[i] = mappedDeg;
            }
        }

        if (sharedData->mappedAngleQueue) {
            xQueueOverwrite(sharedData->mappedAngleQueue, &mappedData);
        }

        memcpy(localTargets, commandSnapshot.targetAngles, sizeof(localTargets));
        const bool sineTestEnabled =
            commandSnapshot.sineTestEnabled != 0 &&
            commandSnapshot.controlMode == CONTROL_MODE_JOINT;
        if (sineTestEnabled)
        {
            const uint32_t sineTestToken = commandSnapshot.sineTestRequestToken;
            if (!sineTestActive || sineTestToken != lastSineTestRequestToken)
            {
                sineTestStartMs = commandSnapshot.sineTestStartMs != 0
                                       ? commandSnapshot.sineTestStartMs
                                       : millis();
                sineTestActive = true;
                lastSineTestRequestToken = sineTestToken;
                feedbackRunDoneStableCount = 0;
            }

            const float elapsedSec =
                (float)((uint32_t)(millis() - sineTestStartMs)) * 0.001f;
            const float waveSec =
                elapsedSec > ((float)commandSnapshot.sineTestHoldMs * 0.001f)
                    ? elapsedSec - ((float)commandSnapshot.sineTestHoldMs * 0.001f)
                    : 0.0f;
            const uint32_t activeJointMask = commandSnapshot.sineTestJointMask;
            for (uint8_t jointIndex = 0; jointIndex < ENCODER_TOTAL_NUM; jointIndex++)
            {
                if ((activeJointMask & (1UL << jointIndex)) != 0)
                {
                    const float freqHz = sanitizeLocalTestFreq(commandSnapshot.sineTestFreqHz[jointIndex]);
                    const float ampDeg = sanitizeLocalTestAmp(commandSnapshot.sineTestAmpDeg[jointIndex]);
                    const float phaseRad = commandSnapshot.sineTestPhaseRad[jointIndex];
                    localTargets[jointIndex] =
                        commandSnapshot.sineTestCenterDeg[jointIndex] +
                        ampDeg * sinf(kLocalTestTwoPi * freqHz * waveSec + phaseRad);
                }
            }
        }
        else
        {
            sineTestActive = false;
        }
        const bool hostControlEnabled = (sharedData->control_enabled != 0);
        const uint8_t systemState = sharedData->system_state;
        const uint8_t targetOwner = sharedData->servo_target_owner;
        const bool controlOwnerActive = (targetOwner == SERVO_TARGET_OWNER_CONTROL);
        const bool feedbackRunningState = (systemState == SYSTEM_STATE_RUN_FEEDBACK);
        const bool directRunningState = (systemState == SYSTEM_STATE_RUN_DIRECT);
        const bool feedbackOutputState =
            feedbackRunningState && commandSnapshot.controlMode == CONTROL_MODE_JOINT;
        const bool directOutputState =
            directRunningState && commandSnapshot.controlMode == CONTROL_MODE_DIRECT_MOTOR;
        const bool controlEnabled = hostControlEnabled &&
                                    controlOwnerActive &&
                                    (feedbackOutputState || directOutputState);
        const bool normalOutputAllowed = controlEnabled;

        if (normalOutputAllowed && telemetryFresh)
        {
            const uint32_t overloadMotorMask =
                activeOverloadMotorMask(sharedData, commandSnapshot);
            for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++)
            {
                const uint32_t bit = 1UL << ch;
                if (((overloadMotorMask & bit) == 0) ||
                    telemetryData.onlineStatus[ch] == 0)
                {
                    overloadHighCount[ch] = 0;
                    continue;
                }

                const bool jointControlMode =
                    (commandSnapshot.controlMode == CONTROL_MODE_JOINT);
                const int16_t overloadThreshold = jointControlMode
                                                      ? kJointControlOverloadLoadThreshold
                                                      : kControlOverloadLoadThreshold;
                const uint8_t overloadLatchCycles = jointControlMode
                                                        ? kJointControlOverloadLatchCycles
                                                        : kControlOverloadLatchCycles;
                const int16_t loadMag = servoLoadMagnitude(telemetryData.load[ch]);
                if (loadMag >= overloadThreshold) {
                    if (overloadHighCount[ch] < 255) {
                        overloadHighCount[ch]++;
                    }
                } else {
                    overloadHighCount[ch] = 0;
                }

                if (overloadHighCount[ch] >= overloadLatchCycles &&
                    (overloadFaultLatched & bit) == 0)
                {
                    overloadFaultLatched |= bit;
                    sharedData->overload_fault_load[ch] = telemetryData.load[ch];
                    Serial.printf("[CONTROL] OVERLOAD_HOLD M%02u load=%d threshold=%d cycles=%u mode=%u\r\n",
                                  (unsigned)ch,
                                  loadMag,
                                  overloadThreshold,
                                  (unsigned)overloadLatchCycles,
                                  (unsigned)commandSnapshot.controlMode);
                }
            }
        }
        else
        {
            memset(overloadHighCount, 0, sizeof(overloadHighCount));
        }
        sharedData->overload_fault_bitmap |= overloadFaultLatched;
        overloadFaultLatched = sharedData->overload_fault_bitmap;

        // 关节闭环模式下先按标定结果限制目标角度，避免目标超过当前工作空间。
        if (commandSnapshot.controlMode == CONTROL_MODE_JOINT) {
            for (uint8_t jointIndex = 0; jointIndex < ENCODER_TOTAL_NUM; jointIndex++) {
                localTargets[jointIndex] = clampJointTargetDegByCalib(localTargets[jointIndex], jointIndex);
            }
        }

        // 根据目标角、磁编码器反馈和当前舵机位置计算各关节 PID 输出。
        g_controlSolver.compute(localTargets, magAngles, absolutePositionTendon, outPulsesTendon);

        if (commandSnapshot.controlMode == CONTROL_MODE_DIRECT_MOTOR)
        {
            // 直控模式：motorTargetRaw 按 motor channel 对齐，支持绝对多圈、单圈 raw 和 sweep 输入。
            if (controlEnabled) {
                const uint32_t activeMotorMask =
                    sharedData->active_motor_mask & validMotorChannelMask();
                for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++)
                {
                    if (((activeMotorMask >> ch) & 0x1UL) == 0) {
                        continue;
                    }
                    const uint8_t bus = motorMap[ch].busIndex;
                    const uint8_t id = motorMap[ch].servoID;
                    if (bus >= NUM_BUSES) {
                        continue;
                    }

                    if (!normalOutputAllowed)
                    {
                        if (servoData.onlineStatus[ch] != 0) {
                            const int16_t holdPos = clampServoPos(servoData.servoAngles[ch]);
                            appendServoTarget(&targetBatch, bus, id, holdPos,
                                              SERVO_TARGET_SPEED_DEFAULT, SERVO_TARGET_ACC_DEFAULT);
                        }
                        continue;
                    }

                    int16_t targetPos = clampServoPos(servoData.servoAngles[ch]);
                    if (!hotplugHoldMotor[ch]) {
                        if (motorDirectSource == MOTOR_DIRECT_SOURCE_SWEEP) {
                            const int16_t rawTarget =
                                clampServoSingleTurnRaw(commandSnapshot.motorSweepTargetRaw[ch]);
                            if (!sweepAnchorValid[ch]) {
                                sweepExpandedTarget[ch] =
                                    anchorSingleTurnSweepToCurrent(servoData.servoAngles[ch], rawTarget);
                                prevSweepRawTarget[ch] = rawTarget;
                                sweepAnchorValid[ch] = true;
                            } else if (rawTarget != prevSweepRawTarget[ch]) {
                                const int32_t d = (int32_t)rawTarget - (int32_t)prevSweepRawTarget[ch];
                                sweepExpandedTarget[ch] += wrapServoRawDelta(d);
                                prevSweepRawTarget[ch] = rawTarget;
                            }
                            targetPos = clampServoPosByMotorAbsLimit(sharedData,
                                                                      ch,
                                                                      sweepExpandedTarget[ch]);
                            sweepExpandedTarget[ch] = targetPos;
                        } else if (motorDirectSource == MOTOR_DIRECT_SOURCE_ABSOLUTE) {
                            sweepAnchorValid[ch] = false;
                            targetPos = clampServoPosByMotorAbsLimit(
                                sharedData,
                                ch,
                                (int32_t)commandSnapshot.motorTargetRaw[ch]);
                        } else {
                            sweepAnchorValid[ch] = false;
                            targetPos = clampServoPosByMotorAbsLimit(
                                sharedData,
                                ch,
                                expandSingleTurnTargetNearCurrent(
                                servoData.servoAngles[ch],
                                clampServoSingleTurnRaw(commandSnapshot.motorTargetRaw[ch])
                                ));
                        }
                    } else {
                        sweepAnchorValid[ch] = false;
                    }
                    appendServoTarget(&targetBatch, bus, id, targetPos,
                                      SERVO_TARGET_SPEED_DEFAULT, SERVO_TARGET_ACC_DEFAULT);
                }
            }
        }
        else
        {
            // 负反馈模式：逐关节评估安全条件，PID 输出再映射到一个或两个执行舵机。
            for (uint8_t jointIndex = 0; jointIndex < ENCODER_TOTAL_NUM; jointIndex++)
            {
                const uint8_t bus = jointMap[jointIndex].busIndex;
                const uint8_t id = jointMap[jointIndex].servoID;
                const int jointMotorCh = findMotorChannel(bus, id);
                const bool servoOnline = (jointMotorCh >= 0) && (servoData.onlineStatus[jointMotorCh] != 0);
                const bool mappedValid = canBusOnline && (mappedData.validFlags[jointIndex] != 0);
                const bool guardInputsValid = servoOnline && mappedValid;
                bool releaseFaultActive = false;
                bool holdByTendonGuard = false;
                bool overloadFaultActive = jointHasOverloadFault(jointIndex, overloadFaultLatched);

                if (jointIndex != kJoint16Index)
                {
                    const int32_t servoPos =
                        (jointMotorCh >= 0) ? servoData.servoAngles[jointMotorCh] : absolutePositionRaw[jointIndex];
#if REVERSE_GUARD_ENABLE
                    const bool reverseGuardCalibrationReady = g_jointCalibResult[jointIndex].success;
                    if (controlEnabled && reverseGuardCalibrationReady) {
                        releaseFaultActive = updateReleaseGuardState(&releaseGuards[jointIndex],
                                                                     localTargets[jointIndex],
                                                                     magAngles[jointIndex],
                                                                     servoPos,
                                                                     guardInputsValid);
                        holdByTendonGuard = shouldBlockReleaseByTendonGuard(
                            &commandSnapshot,
                            jointIndex,
                            localTargets[jointIndex],
                            magAngles[jointIndex],
                            absolutePositionRaw[jointIndex],
                            guardInputsValid
                        );
                    }
#else
                    (void)servoPos;
#endif
                }

                bool emergency =
                    (!normalOutputAllowed) ||
                    shouldEmergencyStop(canBusOnline, sensorData, mappedData, jointIndex);
                if (jointIndex == kJoint16Index) {
                    emergency = emergency || overloadFaultActive;
                } else {
                    emergency = emergency || releaseFaultActive || holdByTendonGuard || overloadFaultActive;
                }
                const bool hotplugHoldActive =
                    (jointMotorCh >= 0) &&
                    (jointMotorCh < SERVO_TOTAL_NUM) &&
                    hotplugHoldMotor[jointMotorCh];

                if (bus >= NUM_BUSES) {
                    continue;
                }

                if (!controlEnabled)
                {
                    // 状态机未授权 START/RUN 时，不向舵机追加新的控制目标。
                    continue;
                }

                if (emergency)
                {
                    // 故障或保护触发时保持当前位置，避免异常状态下继续运动。
                    const int16_t holdPos = clampServoPos(absolutePositionRaw[jointIndex]);
                    appendJointActuatorHoldTargets(&targetBatch, jointIndex, servoData,
                                                   SERVO_TARGET_SPEED_DEFAULT, SERVO_TARGET_ACC_DEFAULT);
                    jointCmdPos[jointIndex] = holdPos;
                    jointCmdValid[jointIndex] = servoOnline ? 1 : 0;
                    continue;
                }

                if (hotplugHoldActive)
                {
                    const int16_t holdPos = clampServoPos(absolutePositionRaw[jointIndex]);
                    appendJointActuatorHoldTargets(&targetBatch, jointIndex, servoData,
                                                   SERVO_TARGET_SPEED_DEFAULT, SERVO_TARGET_ACC_DEFAULT);
                    jointCmdPos[jointIndex] = holdPos;
                    jointCmdValid[jointIndex] = servoOnline ? 1 : 0;
                    continue;
                }

                int16_t targetPos = clampServoPos(absolutePositionRaw[jointIndex]);

                // Only active joints follow PID. Inactive joints hold their
                // current absolute position so partial-scope tests do not move
                // unrelated channels.
                const bool jointActive =
                    (sharedData->active_joint_mask & (1UL << jointIndex)) != 0;
                const bool dualTendonJoint = isDualTendonJoint(jointIndex);
                if (jointActive && dualTendonJoint) {
                    const uint8_t appended =
                        appendDualTendonTargets(&targetBatch,
                                                sharedData,
                                                jointIndex,
                                                localTargets[jointIndex],
                                                magAngles[jointIndex],
                                                servoData,
                                                jointCommandToken,
                                                kJointControlUnlimitedSpeed,
                                                SERVO_TARGET_ACC_DEFAULT);
                    if (appended == 0) {
                        appendJointActuatorHoldTargets(&targetBatch, jointIndex, servoData,
                                                       SERVO_TARGET_SPEED_DEFAULT, SERVO_TARGET_ACC_DEFAULT);
                    }
                    jointCmdPos[jointIndex] = targetPos;
                    jointCmdValid[jointIndex] = (appended > 0) ? 1 : 0;
                    continue;
                }

                if (jointActive) {
                    if (jointMotorCh >= 0 && jointMotorCh < SERVO_TOTAL_NUM) {
                        const uint8_t motorCh = (uint8_t)jointMotorCh;
                        const int32_t rawTarget =
                            motorTendonAbsToRawAbs(motorCh, outPulsesTendon[jointIndex]);
                        targetPos = clampServoPosByMotorAbsLimit(sharedData,
                                                                 motorCh,
                                                                 rawTarget);
                    } else {
                        targetPos = clampServoPos(outPulsesTendon[jointIndex]);
                    }
                }

                if (dualTendonJoint) {
                    appendJointActuatorHoldTargets(&targetBatch, jointIndex, servoData,
                                                   SERVO_TARGET_SPEED_DEFAULT, SERVO_TARGET_ACC_DEFAULT);
                } else {
                    appendJointActuatorTargets(&targetBatch, jointIndex, targetPos,
                                               jointActive ? kJointControlUnlimitedSpeed : SERVO_TARGET_SPEED_DEFAULT,
                                               SERVO_TARGET_ACC_DEFAULT);
                }
                jointCmdPos[jointIndex] = targetPos;
                jointCmdValid[jointIndex] = 1;
            }
        }

        sharedData->reverse_release_fault_bitmap =
#if REVERSE_GUARD_ENABLE
            buildReleaseFaultBitmap(releaseGuards);
#else
            0;
#endif

        if (sharedData->jointDebugQueue)
        {
            // 上报少量关节调试数据，便于观察目标、反馈、PID 输出和最终舵机命令。
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
                debugData.loop1Output = g_controlSolver.getPidOutput(jointIndex, 0);
                debugData.loop2Actual = (float)absolutePositionTendon[jointIndex];
                debugData.loop2Output = g_controlSolver.getPidOutput(jointIndex, 1);
                debugData.cmdTargetPos = jointCmdPos[jointIndex];
                debugData.cmdValid = jointCmdValid[jointIndex];

                if (xQueueSend(sharedData->jointDebugQueue, &debugData, 0) != pdTRUE) {
                    JointDebugData_t drop;
                    xQueueReceive(sharedData->jointDebugQueue, &drop, 0);
                    xQueueSend(sharedData->jointDebugQueue, &debugData, 0);
                }
            }
        }

        if (sharedData->servoTargetQueue && targetBatch.count > 0) {
            xQueueOverwrite(sharedData->servoTargetQueue, &targetBatch);
        }

        if (directRunningState &&
            commandSnapshot.controlMode == CONTROL_MODE_DIRECT_MOTOR &&
            commandSnapshot.motorDirectCommandSource == MOTOR_DIRECT_SOURCE_ABSOLUTE) {
            if (directAbsoluteTargetsReached(sharedData, servoData, commandSnapshot)) {
                if (directRunDoneStableCount < 255) {
                    directRunDoneStableCount++;
                }
                if (directRunDoneStableCount >= kDirectRunDoneStableCycles) {
                    postSystemEvent(sharedData, SYSTEM_EVENT_RUN_DIRECT_DONE);
                    directRunDoneStableCount = 0;
                }
            } else {
                directRunDoneStableCount = 0;
            }
        } else {
            directRunDoneStableCount = 0;
        }

        if (feedbackRunningState &&
            commandSnapshot.controlMode == CONTROL_MODE_JOINT &&
            feedbackJointTargetsReached(sharedData, mappedData, localTargets, magAngles)) {
            if (feedbackRunDoneStableCount < 255) {
                feedbackRunDoneStableCount++;
            }
            if (feedbackRunDoneStableCount >= kFeedbackRunDoneStableCycles) {
                postSystemEvent(sharedData, SYSTEM_EVENT_RUN_FEEDBACK_DONE);
                feedbackRunDoneStableCount = 0;
            }
        } else {
            feedbackRunDoneStableCount = 0;
        }

        vTaskDelayUntil(&lastWakeTime, solverPeriodTicks);
    }
}
