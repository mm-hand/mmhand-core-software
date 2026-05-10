#include "CalibrationTask.h"

#include <string.h>

#include "JointCalibrationProfile.h"

// CalibrationTask 模块职责：
// 1) 管理关节标定参数与标定结果；
// 2) 提供单关节自动标定流程；
// 3) 在当前测试阶段，支持手工标定结果注入并停用自动标定任务。
//
// 数据来源约束：
// - 编码器原始值来自 canRxQueue（RemoteSensorData_t）；
// - 舵机运动与反馈通过 servoCommunicationTask 的队列/快照交互；
// - 标定状态通过 g_calibrationUIStatus 对上位机可见。

extern JointMapItem jointMap[ENCODER_TOTAL_NUM];
extern MotorMapItem joint16SecondaryMotor;
extern volatile uint8_t g_calibrationUIStatus;

// 全局标定配置与结果（按关节索引 0~ENCODER_TOTAL_NUM-1）。
JointCalibrationConfig g_jointCalibConfig[ENCODER_TOTAL_NUM];
JointCalibrationResult g_jointCalibResult[ENCODER_TOTAL_NUM];

// 编码器安装方向：+1 为正向安装，-1 为反向安装。
// 所有角度映射前都需通过该表统一方向，避免同一动作出现正负号不一致。
int8_t g_encoderDirection[ENCODER_TOTAL_NUM] = {
    1, -1, -1, 1,
    1, -1, -1, 1,
    1, -1, -1, 1,
    1, -1, 1, 1,
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

// 测试模式手工 encoderMax 表（单位：编码器计数）。
// 约定：值为 0 表示该关节未提供手工标定数据。
static const uint8_t kTestJointCount = ENCODER_TOTAL_NUM;
static const int32_t kManualEncoderMax[kTestJointCount] = {
    12037, 12636, 9174, 12822,
    12440, 9738, 3500, 1232,
    3339, 2425, 12643, 6464,
    8930, 1353, 8687, 12757,
    0, 6542, 6480, 10115, 6535
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
    for (int i = 0; i < SERVO_TOTAL_NUM; i++) {
        if (motorMap[i].busIndex == bus && motorMap[i].servoID == id) {
            return i;
        }
    }
    return -1;
}

static bool readLatestServoAbs(TaskSharedData_t* sharedData, uint8_t bus, uint8_t id, int32_t* outAbs)
{
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

    const int32_t targetEncoder = offset + cfg.angleReserved;
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
// 2) 用手工 encoderMax 推导 offset/angleMax；
// 3) 将可用结果同步到手工回退表；
// 4) 保持标定状态为 IDLE（自动标定任务关闭）。
void initManualCalibrationForTest(void) {
    initDefaultCalibrationConfig(g_jointCalibConfig, ENCODER_TOTAL_NUM);
    memset(g_jointCalibResult, 0, sizeof(g_jointCalibResult));
    memset(g_encoderOffsetManual, 0, sizeof(g_encoderOffsetManual));

    for (uint8_t i = 0; i < kTestJointCount; i++) {
        const JointCalibrationConfig& cfg = g_jointCalibConfig[i];
        JointCalibrationResult& out = g_jointCalibResult[i];

        out.encoderMax = kManualEncoderMax[i];
        out.offset = out.encoderMax - cfg.angleScope + cfg.bottomReserved;
        out.angleMax = cfg.angleScope - cfg.bottomReserved - cfg.topReserved;
        // 约定：encoderMax == 0 代表该关节未提供有效手工标定数据。
        out.success = (out.encoderMax != 0);

        // 同步写入回退表，便于调试阶段统一读取路径。
        g_encoderOffsetManual[i] = out.offset;
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

    g_calibrationUIStatus = CALIB_STATUS_IDLE;
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
    out->encoderMax = 0;
    out->offset = 0;
    out->angleMax = 0;

    const uint8_t bus = jointMap[jointIndex].busIndex;
    const uint8_t servoId = jointMap[jointIndex].servoID;
    int32_t currentServoPos = 0;
    if (!readLatestServoAbs(sharedData, bus, servoId, &currentServoPos)) {
        return false;
    }
    int16_t commandPos = clampToServoPos(currentServoPos);

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

            out->encoderMax = encoderMax;
            out->offset = encoderMax - cfg.angleScope + cfg.bottomReserved;
            out->angleMax = cfg.angleScope - cfg.bottomReserved - cfg.topReserved;
            out->success = true;

            moveJointToReserved(sharedData, jointIndex, cfg, out->offset);
            return true;
        }
    }

    return false;
}
void taskServoCalibration(void* parameter) {
    TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;
    if (!sharedData) {
        vTaskDelete(NULL);
        return;
    }

    g_calibrationUIStatus = CALIB_STATUS_IDLE;
    uint8_t lastState = SYSTEM_STATE_BOOTING;

    for (;;)
    {
        const uint8_t state = sharedData->system_state;
        if (state != lastState) {
            if (state == SYSTEM_STATE_CALIBRATION_IDLE) {
                g_calibrationUIStatus = CALIB_STATUS_IDLE;
            } else if (state == SYSTEM_STATE_CALIBRATION_RUNNING) {
                g_calibrationUIStatus = CALIB_STATUS_RUNNING;
            } else if (state == SYSTEM_STATE_CALIBRATION_SUCCESS) {
                g_calibrationUIStatus = CALIB_STATUS_SUCCESS;
            } else if (state == SYSTEM_STATE_CALIBRATION_FAILED) {
                g_calibrationUIStatus = CALIB_STATUS_FAILED;
            }
            lastState = state;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
