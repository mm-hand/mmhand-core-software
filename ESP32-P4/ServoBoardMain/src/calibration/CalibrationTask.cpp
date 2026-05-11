#include "CalibrationTask.h"

#include <string.h>

#include "JointCalibrationProfile.h"

// CalibrationTask 模块职责：
// 1) 管理关节标定参数与标定结果；
// 2) 提供单关节自动标定流程；
// 3) 在当前测试阶段，支持手工标定结果注入，但不作为常驻任务运行。
//
// 数据来源约束：
// - 编码器原始值来自 canRxQueue（RemoteSensorData_t）；
// - 舵机运动与反馈通过 servoCommunicationTask 的队列/快照交互；
// - 标定生命周期与 UI 状态由 StateMachineTask 统一管理。

extern JointMapItem jointMap[ENCODER_TOTAL_NUM];
extern MotorMapItem joint16SecondaryMotor;

// 全局标定配置与结果（按关节索引 0~ENCODER_TOTAL_NUM-1）。
JointCalibrationConfig g_jointCalibConfig[ENCODER_TOTAL_NUM];
JointCalibrationResult g_jointCalibResult[ENCODER_TOTAL_NUM];

// 编码器安装方向：+1 为正向安装，-1 为反向安装。
// 所有角度映射前都需通过该表统一方向，避免同一动作出现正负号不一致。
int8_t g_encoderDirection[ENCODER_TOTAL_NUM] = {
    1, -1, -1, 1,
    1, -1, -1, 1,
    1, -1, -1, 1,
    1, -1, -1, -1,
    1,  1, -1, -1, 1
};

// 手工回退 offset 表：
// 当自动标定结果不可用时，控制任务会读取该表作为零偏。
int32_t g_encoderOffsetManual[ENCODER_TOTAL_NUM] = {0};

static const uint8_t kJoint16Index = 16;
static const int32_t kJoint16SecondaryOffsetDefault = 0;
static const int32_t kJoint16TensionBiasDefault = 0;

// joint16 双舵机标定约束默认配置：
// 主舵机 bus3-id17，副舵机 bus3-id18，默认偏置为0。
JointDualServoConstraint g_joint16DualServoConstraint = {
    1,
    kJoint16Index,
    3,
    17,
    3,
    18,
    kJoint16SecondaryOffsetDefault,
    kJoint16TensionBiasDefault
};

// Manual mechanical limit raw tables. Fill values from the host raw magnetic
// encoder display directly; direction conversion happens in this file.
// -1 means not configured. Raw value 0 is valid.
static const uint8_t kTestJointCount = ENCODER_TOTAL_NUM;
static const int32_t kManualEncoderMinRaw[kTestJointCount] = {
    -1, -1, -1, -1,
    -1, -1, -1, -1,
    -1, -1, -1, -1,
    5766, 6586, 6248, 12130,
    -1, -1, -1, -1, -1
};

static const int32_t kManualEncoderMaxRaw[kTestJointCount] = {
    12037, 12636, 9174, 12822,
    12440, 9738, 3500, 1232,
    3339, 2425, 12643, 6464,
    9061, 3422, 2408, 1457,
    -1, 6542, 6480, 10115, 6535
};

static const int32_t kEncoderModulo = 16384;
static const int32_t kEncoderHalfTurn = kEncoderModulo / 2;

// 按安装方向统一编码器原始值方向，输出范围 [0, modulo)。
static int32_t orientEncoderRaw(uint16_t rawValue, int8_t direction) {
    int32_t oriented = (int32_t)rawValue & (kEncoderModulo - 1);
    if (direction < 0) {
        oriented = (kEncoderModulo - oriented) & (kEncoderModulo - 1);
    }
    return oriented;
}

static bool isManualEncoderRawValid(int32_t rawValue) {
    return rawValue >= 0 && rawValue < kEncoderModulo;
}

static int32_t wrapEncoderCount(int32_t value) {
    value %= kEncoderModulo;
    if (value < 0) value += kEncoderModulo;
    return value;
}

static int32_t forwardEncoderDelta(int32_t from, int32_t to) {
    return wrapEncoderCount(to - from);
}

static float encoderCountToDeg(int32_t count) {
    return (float)count * 360.0f / (float)kEncoderModulo;
}

static int32_t degToEncoderCount(float deg) {
    const float scaled = deg * (float)kEncoderModulo / 360.0f;
    return (int32_t)(scaled + (scaled >= 0.0f ? 0.5f : -0.5f));
}

// 舵机绝对位置限幅，防止下发超出协议/硬件安全范围的目标值。
static int16_t clampToServoPos(int32_t value) {
    if (value > 30719) return 30719;
    if (value < -30719) return -30719;
    return (int16_t)value;
}

// 获取关节双舵机约束：
// - 当前仅 joint16 返回有效约束；
// - 其余关节返回 false，调用方按单舵机流程处理。
bool getJointDualServoConstraint(uint8_t jointIndex, JointDualServoConstraint* out) {
    if (!out) {
        return false;
    }
    if (g_joint16DualServoConstraint.enabled != 0 &&
        g_joint16DualServoConstraint.jointIndex == jointIndex) {
        *out = g_joint16DualServoConstraint;
        return true;
    }
    return false;
}

// 由主舵机目标推导副舵机目标（拮抗关系）：
// secondary = -primary + secondaryOffset + tensionBias。
bool computeSecondaryServoTargetForJoint(uint8_t jointIndex,
                                         int32_t primaryTarget,
                                         int16_t* outSecondaryTarget) {
    if (!outSecondaryTarget) {
        return false;
    }
    JointDualServoConstraint cfg;
    if (!getJointDualServoConstraint(jointIndex, &cfg)) {
        return false;
    }
    // 副舵机目标按拮抗关系计算。
    const int32_t secondaryTarget = -primaryTarget + cfg.secondaryOffset + cfg.tensionBias;
    *outSecondaryTarget = clampToServoPos(secondaryTarget);
    return true;
}

// 计算目标与当前编码器差值，并在整圈边界处做环绕归一化（最短路径）。
static int32_t wrapEncoderDelta(int32_t target, int32_t actual) {
    int32_t delta = target - actual;
    while (delta > kEncoderHalfTurn) delta -= kEncoderModulo;
    while (delta < -kEncoderHalfTurn) delta += kEncoderModulo;
    return delta;
}

// 从 CAN 最新快照读取关节编码器值，并做方向统一。
// 失败条件：队列无数据、数据无效、关节报错、参数越界。
static bool readLatestEncoderRaw(TaskSharedData_t* sharedData, uint8_t jointIndex, int32_t* outEncoder) {
    if (!sharedData || !outEncoder || jointIndex >= ENCODER_TOTAL_NUM) return false;

    RemoteSensorData_t sensorData;
    if (xQueuePeek(sharedData->canRxQueue, &sensorData, 0) != pdTRUE) return false;
    if (!sensorData.isValid) return false;
    if (sensorData.errorFlags[jointIndex]) return false;

    *outEncoder = orientEncoderRaw(sensorData.encoderValues[jointIndex], g_encoderDirection[jointIndex]);
    return true;
}
static int findMotorChannel(uint8_t bus, uint8_t id)
{
    // 将物理 bus/id 转换为 motorMap 通道索引，供反馈数组读取使用。
    for (int i = 0; i < SERVO_TOTAL_NUM; i++) {
        if (motorMap[i].busIndex == bus && motorMap[i].servoID == id) {
            return i;
        }
    }
    return -1;
}

static bool readLatestServoAbs(TaskSharedData_t* sharedData, uint8_t bus, uint8_t id, int32_t* outAbs)
{
    // 标定任务不直接访问舵机总线，只从 servoFeedbackQueue 读取最新快照。
    if (!sharedData || !outAbs) {
        return false;
    }
    const int ch = findMotorChannel(bus, id);
    if (ch < 0) {
        return false;
    }
    ServoAngleData_t servoData;
    if (!sharedData->servoFeedbackQueue || xQueuePeek(sharedData->servoFeedbackQueue, &servoData, 0) != pdTRUE) {
        return false;
    }
    if (servoData.onlineStatus[ch] == 0) {
        return false;
    }
    *outAbs = servoData.servoAngles[ch];
    return true;
}

static bool readLatestServoLoad(TaskSharedData_t* sharedData, uint8_t bus, uint8_t id, int16_t* outLoad)
{
    // 负载来自舵机通信层发布的遥测快照，用于判断是否触碰机械限位。
    if (!sharedData || !outLoad) {
        return false;
    }
    const int ch = findMotorChannel(bus, id);
    if (ch < 0) {
        return false;
    }
    ServoTelemetryData_t telemetry;
    QueueHandle_t queue = sharedData->servoTelemetrySnapshotQueue ? sharedData->servoTelemetrySnapshotQueue : sharedData->servoTelemetryQueue;
    if (!queue || xQueuePeek(queue, &telemetry, 0) != pdTRUE) {
        return false;
    }
    if (telemetry.onlineStatus[ch] == 0) {
        return false;
    }
    *outLoad = telemetry.load[ch];
    return true;
}

static bool appendServoTarget(ServoTargetBatch_t* batch,
                              uint8_t busIndex,
                              uint8_t servoId,
                              int16_t position,
                              uint16_t speed,
                              uint8_t acc)
{
    // CalibrationTask 构造的 batch 仍走 servoTargetQueue，由 ServoCommunicationTask 执行。
    if (!batch || batch->count >= SERVO_TARGET_BATCH_MAX || busIndex >= NUM_BUSES) {
        return false;
    }
    ServoTargetCommand_t& cmd = batch->commands[batch->count++];
    cmd.busIndex = busIndex;
    cmd.servoId = servoId;
    cmd.position = position;
    cmd.speed = speed;
    cmd.acc = acc;
    return true;
}

static bool queueJointTarget(TaskSharedData_t* sharedData,
                             uint8_t jointIndex,
                             int16_t primaryTarget,
                             const JointCalibrationConfig& cfg)
{
    // 校准目标必须带 CALIBRATION source，且只有当前 owner 为 CALIBRATION 才允许入队。
    if (!sharedData || !sharedData->servoTargetQueue || jointIndex >= ENCODER_TOTAL_NUM) {
        return false;
    }

    ServoTargetBatch_t batch;
    memset(&batch, 0, sizeof(batch));
    batch.timestamp = millis();
    batch.source = SERVO_TARGET_OWNER_CALIBRATION;
    appendServoTarget(&batch,
                      jointMap[jointIndex].busIndex,
                      jointMap[jointIndex].servoID,
                      primaryTarget,
                      cfg.tightenSpeed,
                      cfg.tightenAcc);

    JointDualServoConstraint dualCfg;
    int16_t secondaryTarget = 0;
    if (getJointDualServoConstraint(jointIndex, &dualCfg) &&
        computeSecondaryServoTargetForJoint(jointIndex, primaryTarget, &secondaryTarget)) {
        appendServoTarget(&batch,
                          dualCfg.secondaryBusIndex,
                          dualCfg.secondaryServoID,
                          secondaryTarget,
                          cfg.tightenSpeed,
                          cfg.tightenAcc);
    }

    if (batch.count == 0) {
        return false;
    }
    if (sharedData->servo_target_owner != SERVO_TARGET_OWNER_CALIBRATION) {
        return false;
    }
    return xQueueOverwrite(sharedData->servoTargetQueue, &batch) == pdTRUE;
}

// 标定后回退动作：
// 将关节移动到 angleReserved 预留角度，避免长时间停在机械限位附近。
// 成功返回 true；超时或反馈不可用返回 false（不影响标定主结果已写回）。
static bool moveJointToReserved(TaskSharedData_t* sharedData,
                                uint8_t jointIndex,
                                const JointCalibrationConfig& cfg,
                                int32_t offset) {
    if (!sharedData || jointIndex >= ENCODER_TOTAL_NUM) return false;

    const int32_t targetEncoder = offset + degToEncoderCount(cfg.angleReserved);
    const uint8_t bus = jointMap[jointIndex].busIndex;
    const uint8_t servoId = jointMap[jointIndex].servoID;
    const uint32_t beginMs = millis();

    while (millis() - beginMs < 3000) {
        int32_t currentEncoder = 0;
        if (!readLatestEncoderRaw(sharedData, jointIndex, &currentEncoder)) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        int32_t encoderErr = wrapEncoderDelta(targetEncoder, currentEncoder);
        if (abs((int)encoderErr) <= 6) {
            return true;
        }

        int32_t currentServoPos = 0;
        if (!readLatestServoAbs(sharedData, bus, servoId, &currentServoPos)) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        int32_t step = encoderErr / 8;
        if (step == 0) step = (encoderErr > 0) ? 1 : -1;

        int32_t stepLimit = (int32_t)abs((int)cfg.tightenStep) * 3;
        if (stepLimit < 1) stepLimit = 1;
        if (step > stepLimit) step = stepLimit;
        if (step < -stepLimit) step = -stepLimit;

        const int16_t targetPos = clampToServoPos(currentServoPos + step);
        queueJointTarget(sharedData, jointIndex, targetPos, cfg);
        vTaskDelay(pdMS_TO_TICKS(cfg.settleMs));
    }

    return false;
}
// 装载默认标定参数。
void initDefaultCalibrationConfig(JointCalibrationConfig* cfg, uint8_t count) {
    loadJointCalibrationProfile(cfg, count);
}

// 测试模式初始化：
// 1) 装载默认配置；
// 2) 将手工 minraw/maxraw 按方向自动转换成 oriented min/max；
// 3) 用 oriented min/max 推导 angleScope/offset/angleMax；
// 4) 将可用结果同步到手工回退表；
// 5) 保持标定状态为 IDLE（自动标定任务关闭）。
void initManualCalibrationForTest(void) {
    initDefaultCalibrationConfig(g_jointCalibConfig, ENCODER_TOTAL_NUM);
    memset(g_jointCalibResult, 0, sizeof(g_jointCalibResult));
    memset(g_encoderOffsetManual, 0, sizeof(g_encoderOffsetManual));

    for (uint8_t i = 0; i < kTestJointCount; i++) {
        JointCalibrationConfig& cfg = g_jointCalibConfig[i];
        JointCalibrationResult& out = g_jointCalibResult[i];
        const int32_t rawEncoderMin = kManualEncoderMinRaw[i];
        const int32_t rawEncoderMax = kManualEncoderMaxRaw[i];

        // Manual tables hold raw CAN values. Convert both limits into the
        // normalized direction, then derive the mechanical scope.
        const bool rawValid =
            isManualEncoderRawValid(rawEncoderMin) &&
            isManualEncoderRawValid(rawEncoderMax);
        const int32_t encoderMin = rawValid
            ? orientEncoderRaw((uint16_t)rawEncoderMin, g_encoderDirection[i])
            : 0;
        const int32_t encoderMax = rawValid
            ? orientEncoderRaw((uint16_t)rawEncoderMax, g_encoderDirection[i])
            : 0;
        const int32_t derivedAngleScopeCount = rawValid
            ? forwardEncoderDelta(encoderMin, encoderMax)
            : 0;
        const float derivedAngleScopeDeg = encoderCountToDeg(derivedAngleScopeCount);
        const int32_t bottomReservedCount = degToEncoderCount(cfg.bottomReserved);
        const int32_t topReservedCount = degToEncoderCount(cfg.topReserved);
        const float usableAngleMax =
            derivedAngleScopeDeg - cfg.bottomReserved - cfg.topReserved;

        out.success =
            rawValid &&
            derivedAngleScopeCount > 0 &&
            (bottomReservedCount + topReservedCount) < derivedAngleScopeCount &&
            usableAngleMax > 0.0f;
        out.encoderMin = out.success ? encoderMin : 0;
        out.encoderMax = out.success ? encoderMax : 0;
        if (out.success) {
            cfg.angleScope = derivedAngleScopeDeg;
            out.offset = wrapEncoderCount(encoderMin + bottomReservedCount);
            out.angleMax = usableAngleMax;
        } else {
            out.offset = 0;
            out.angleMax = 0.0f;
        }

        // 同步写入回退表，便于调试阶段统一读取路径。
        g_encoderOffsetManual[i] = out.success ? out.offset : 0;
    }

    // 当前测试阶段不运行自动标定任务。
    g_joint16DualServoConstraint.enabled = 1;
    g_joint16DualServoConstraint.jointIndex = kJoint16Index;
    g_joint16DualServoConstraint.primaryBusIndex = jointMap[kJoint16Index].busIndex;
    g_joint16DualServoConstraint.primaryServoID = jointMap[kJoint16Index].servoID;
    g_joint16DualServoConstraint.secondaryBusIndex = joint16SecondaryMotor.busIndex;
    g_joint16DualServoConstraint.secondaryServoID = joint16SecondaryMotor.servoID;
    g_joint16DualServoConstraint.secondaryOffset = kJoint16SecondaryOffsetDefault;
    g_joint16DualServoConstraint.tensionBias = kJoint16TensionBiasDefault;

}

// 单关节自动标定主流程：
// - 逐步“收紧”舵机直到负载跨阈值，判定触及机械限位；
// - 读取该时刻编码器值作为 encoderMax；
// - 根据配置推导 offset/angleMax，并尝试回退到预留角度。
// 返回 false 的典型原因：参数非法、总线不可用、编码器读取失败、超时未触发阈值。
bool runSingleJointCalibration(TaskSharedData_t* sharedData,
                               uint8_t jointIndex,
                               const JointCalibrationConfig& cfg,
                               JointCalibrationResult* out) {
    if (!sharedData || !out || jointIndex >= ENCODER_TOTAL_NUM) return false;

    out->success = false;
    out->encoderMin = 0;
    out->encoderMax = 0;
    out->offset = 0;
    out->angleMax = 0.0f;

    const uint8_t bus = jointMap[jointIndex].busIndex;
    const uint8_t servoId = jointMap[jointIndex].servoID;
    int32_t currentServoPos = 0;
    if (!readLatestServoAbs(sharedData, bus, servoId, &currentServoPos)) {
        return false;
    }
    int16_t commandPos = clampToServoPos(currentServoPos);
    int32_t encoderMin = 0;
    if (!readLatestEncoderRaw(sharedData, jointIndex, &encoderMin)) {
        return false;
    }

    bool seenBelowThreshold = false;
    const uint32_t searchBeginMs = millis();

    while (millis() - searchBeginMs < cfg.maxSearchMs) {
        int32_t tightenStep = (int32_t)cfg.tightenStep;
        if (tightenStep == 0) tightenStep = 1;
        commandPos = clampToServoPos((int32_t)commandPos + tightenStep);
        if (!queueJointTarget(sharedData, jointIndex, commandPos, cfg)) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(cfg.settleMs));

        int16_t load = 0;
        if (!readLatestServoLoad(sharedData, bus, servoId, &load)) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        if (load < cfg.loadThreshold) {
            seenBelowThreshold = true;
        }

        if (seenBelowThreshold && load >= cfg.loadThreshold) {
            int32_t encoderMax = 0;
            if (!readLatestEncoderRaw(sharedData, jointIndex, &encoderMax)) {
                return false;
            }

            const int32_t derivedAngleScopeCount = forwardEncoderDelta(encoderMin, encoderMax);
            const int32_t bottomReservedCount = degToEncoderCount(cfg.bottomReserved);
            const int32_t topReservedCount = degToEncoderCount(cfg.topReserved);
            const float usableAngleMax =
                encoderCountToDeg(derivedAngleScopeCount) - cfg.bottomReserved - cfg.topReserved;
            if (derivedAngleScopeCount <= 0 ||
                (bottomReservedCount + topReservedCount) >= derivedAngleScopeCount ||
                usableAngleMax <= 0.0f) {
                return false;
            }

            out->encoderMin = encoderMin;
            out->encoderMax = encoderMax;
            out->offset = wrapEncoderCount(encoderMin + bottomReservedCount);
            out->angleMax = usableAngleMax;
            out->success = true;

            moveJointToReserved(sharedData, jointIndex, cfg, out->offset);
            return true;
        }
    }

    return false;
}
