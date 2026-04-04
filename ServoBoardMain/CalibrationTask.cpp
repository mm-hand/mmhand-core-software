#include "CalibrationTask.h"

#include <string.h>

#include "AngleSolver.h"
#include "JointCalibrationProfile.h"
#include "ServoBusManager.h"

// CalibrationTask 模块职责：
// 1) 管理关节标定参数与标定结果；
// 2) 提供单关节自动标定流程；
// 3) 在当前测试阶段，支持手工标定结果注入并停用自动标定任务。
//
// 数据来源约束：
// - 编码器原始值来自 canRxQueue（RemoteSensorData_t）；
// - 舵机运动与反馈来自 ServoBusManager；
// - 标定状态通过 g_calibrationUIStatus 对上位机可见。

extern ServoBusManager servoBus0;
extern ServoBusManager servoBus1;
extern ServoBusManager servoBus2;
extern ServoBusManager servoBus3;
extern JointMapItem jointMap[ENCODER_TOTAL_NUM];
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
    1, -1, -1, 1,
    1,  1, -1, -1, 1
};

// 手工回退 offset 表：
// 当自动标定结果不可用时，AngleSolver 会读取该表作为零偏。
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

// 将 busIndex 映射为总线对象；非法索引返回 NULL。
static ServoBusManager* getBusByIndex(uint8_t busIndex) {
    switch (busIndex) {
        case 0: return &servoBus0;
        case 1: return &servoBus1;
        case 2: return &servoBus2;
        case 3: return &servoBus3;
        default: return NULL;
    }
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

// 标定后回退动作：
// 将关节移动到 angleReserved 预留角度，避免长时间停在机械限位附近。
// 成功返回 true；超时或反馈不可用返回 false（不影响标定主结果已写回）。
static bool moveJointToReserved(TaskSharedData_t* sharedData,
                                ServoBusManager* pBus,
                                uint8_t servoId,
                                uint8_t jointIndex,
                                const JointCalibrationConfig& cfg,
                                int32_t offset) {
    if (!sharedData || !pBus) return false;

    const int32_t targetEncoder = offset + cfg.angleReserved;
    const uint8_t ids[] = {servoId};
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

        if (pBus->syncReadPositions(ids, 1) <= 0) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        const int32_t currentServoPos = pBus->getAbsolutePosition(servoId);
        int32_t step = encoderErr / 8;
        if (step == 0) step = (encoderErr > 0) ? 1 : -1;

        int32_t stepLimit = (int32_t)abs((int)cfg.tightenStep) * 3;
        if (stepLimit < 1) stepLimit = 1;
        if (step > stepLimit) step = stepLimit;
        if (step < -stepLimit) step = -stepLimit;

        const int16_t targetPos = clampToServoPos(currentServoPos + step);
        pBus->setTarget(servoId, targetPos, cfg.tightenSpeed, cfg.tightenAcc);
        JointDualServoConstraint dualCfg;
        if (getJointDualServoConstraint(jointIndex, &dualCfg)) {
            ServoBusManager* pSecBus = getBusByIndex(dualCfg.secondaryBusIndex);
            int16_t secTarget = 0;
            if (pSecBus && computeSecondaryServoTargetForJoint(jointIndex, targetPos, &secTarget)) {
                pSecBus->setTarget(dualCfg.secondaryServoID, secTarget, cfg.tightenSpeed, cfg.tightenAcc);
                pSecBus->syncWriteAll();
            }
        }
        pBus->syncWriteAll();
        vTaskDelay(pdMS_TO_TICKS(cfg.settleMs));
    }

    return false;
}

// 装载默认标定参数（来自 profile 表）。
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
    ServoBusManager* pBus = getBusByIndex(bus);
    if (!pBus) return false;

    const uint8_t ids[] = {servoId};
    pBus->syncReadPositions(ids, 1);
    int16_t commandPos = clampToServoPos(pBus->getAbsolutePosition(servoId));

    bool seenBelowThreshold = false;
    const uint32_t searchBeginMs = millis();

    while (millis() - searchBeginMs < cfg.maxSearchMs) {
        // 每轮按 tightenStep 推进一步，0 步长会被强制为 1，避免死循环。
        int32_t tightenStep = (int32_t)cfg.tightenStep;
        if (tightenStep == 0) tightenStep = 1;
        commandPos = clampToServoPos((int32_t)commandPos + tightenStep);
        pBus->setTarget(servoId, commandPos, cfg.tightenSpeed, cfg.tightenAcc);
        JointDualServoConstraint dualCfg;
        if (getJointDualServoConstraint(jointIndex, &dualCfg)) {
            ServoBusManager* pSecBus = getBusByIndex(dualCfg.secondaryBusIndex);
            int16_t secTarget = 0;
            if (pSecBus && computeSecondaryServoTargetForJoint(jointIndex, commandPos, &secTarget)) {
                pSecBus->setTarget(dualCfg.secondaryServoID, secTarget, cfg.tightenSpeed, cfg.tightenAcc);
                pSecBus->syncWriteAll();
            }
        }
        pBus->syncWriteAll();
        vTaskDelay(pdMS_TO_TICKS(cfg.settleMs));

        const int16_t load = pBus->readLoad(servoId);
        // 采用“先低后高”的边沿触发，降低启动瞬态噪声导致的误判概率。
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

            // 标定成功后尝试回退到预留角；即使回退失败，也保持成功结果。
            moveJointToReserved(sharedData, pBus, servoId, jointIndex, cfg, out->offset);
            return true;
        }
    }

    return false;
}

void taskServoCalibration(void* parameter) {
    (void)parameter;
    // 当前测试阶段：自动标定任务占位关闭，避免后台自主运动影响联调。
    g_calibrationUIStatus = CALIB_STATUS_IDLE;
    vTaskDelete(NULL);
}

