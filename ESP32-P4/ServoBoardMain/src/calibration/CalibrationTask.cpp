#include "CalibrationTask.h"

#include <string.h>

#include "CalibrationMath.h"
#include "../generated/UrdfJointLimits.generated.h"
#include "../hardware/HardwareMap.h"
#include "../shared/SharedSafetyState.h"
#include "../system/FirmwareLog.h"

// CalibrationTask 模块职责：
// 1) 管理关节标定结果和人工磁编参考值；
// 2) 提供标定结果告警、编码器方向和双执行器映射等公共能力；
// 3) 实际机械搜索由 AutoCalibrationTask 的非阻塞状态机执行。
//
// 数据来源约束�?
// - 编码器原始值来�?canRxQueue（RemoteSensorData_t）；
// - 舵机运动与反馈通过 servoCommunicationTask 的队�?快照交互�?
// - 标定生命周期�?UI 状态由 StateMachineTask 统一管理�?

// 全局标定结果（按关节索引 0~ENCODER_TOTAL_NUM-1）。
JointCalibrationResult g_jointCalibResult[ENCODER_TOTAL_NUM];
ManualEncoderReference g_manualEncoderReference[ENCODER_TOTAL_NUM];

int32_t calibrationAutoMinManualOffsetCounts(
    uint8_t jointIndex,
    const JointCalibrationResult& result)
{
    if (jointIndex >= ENCODER_TOTAL_NUM ||
        !result.success ||
        !g_manualEncoderReference[jointIndex].valid) {
        return 0;
    }
    return calibrationSignedDirectedEncoderDelta(
        g_manualEncoderReference[jointIndex].minRaw,
        result.autoMinRaw,
        g_encoderDirection[jointIndex]);
}

bool evaluateCalibrationAssemblyWarning(
    uint8_t jointIndex,
    const JointCalibrationResult& result,
    CalibrationAssemblyWarningInfo* out)
{
    if (!out) {
        return false;
    }
    memset(out, 0, sizeof(*out));
    if (jointIndex >= ENCODER_TOTAL_NUM ||
        !result.success ||
        !g_manualEncoderReference[jointIndex].valid) {
        return false;
    }

    const ManualEncoderReference& manual =
        g_manualEncoderReference[jointIndex];
    const int8_t direction = g_encoderDirection[jointIndex];
    const int32_t minOffset = calibrationSignedDirectedEncoderDelta(
        manual.minRaw, result.autoMinRaw, direction);
    const int32_t maxShortfall = calibrationSignedDirectedEncoderDelta(
        result.autoMaxRaw, manual.maxRaw, direction);
    const int32_t autoSpan = calibrationDirectedEncoderDelta(
        result.autoMinRaw, result.autoMaxRaw, direction);
    const int32_t manualSpan = calibrationDirectedEncoderDelta(
        manual.minRaw, manual.maxRaw, direction);
    const float urdfSpanDeg =
        kUrdfJointLimits[jointIndex].maxDeg -
        kUrdfJointLimits[jointIndex].minDeg;
    const int32_t urdfSpan = calibrationDegToEncoderCount(urdfSpanDeg);
    if (autoSpan < 0 || manualSpan <= 0 || urdfSpan <= 0 ||
        manualSpan >= kCalibrationEncoderModulo ||
        urdfSpan >= kCalibrationEncoderModulo) {
        return false;
    }
    const int32_t spanDifference = manualSpan >= urdfSpan
        ? manualSpan - urdfSpan
        : urdfSpan - manualSpan;

    out->autoMinOffsetCounts = (int16_t)minOffset;
    out->autoMaxShortfallCounts = (int16_t)maxShortfall;
    out->autoSpanCounts = (uint16_t)autoSpan;
    out->manualSpanCounts = (uint16_t)manualSpan;
    out->urdfSpanCounts = (uint16_t)urdfSpan;
    out->spanDifferenceCounts = (uint16_t)spanDifference;
    out->reasonMask = calibrationAssemblyWarningReasonForMetrics(
        minOffset, maxShortfall, spanDifference);
    return true;
}

void clearCalibrationAssemblyWarnings(TaskSharedData_t* sharedData,
                                      uint32_t jointMask)
{
    if (!sharedData) {
        return;
    }
    jointMask &= ((1UL << ENCODER_TOTAL_NUM) - 1UL);
    SharedSafetyGuard guard;
    sharedAtomicFetchAnd(&sharedData->calibration_warning_bitmap, ~jointMask);
    for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; ++joint) {
        if ((jointMask & (1UL << joint)) != 0) {
            sharedData->calibration_auto_min_offset_counts[joint] = 0;
        }
    }
}

bool updateCalibrationAssemblyWarning(
    TaskSharedData_t* sharedData,
    uint8_t jointIndex,
    const JointCalibrationResult& result)
{
    if (!sharedData || jointIndex >= ENCODER_TOTAL_NUM) {
        return false;
    }

    CalibrationAssemblyWarningInfo info;
    const bool valid = evaluateCalibrationAssemblyWarning(
        jointIndex, result, &info);
    const uint32_t bit = 1UL << jointIndex;
    const bool warning = valid && info.reasonMask != CALIB_WARN_NONE;
    {
        SharedSafetyGuard guard;
        sharedData->calibration_auto_min_offset_counts[jointIndex] =
            info.autoMinOffsetCounts;
        if (warning) {
            sharedAtomicFetchOr(&sharedData->calibration_warning_bitmap, bit);
        } else {
            sharedAtomicFetchAnd(&sharedData->calibration_warning_bitmap, ~bit);
        }
    }

    FW_LOG_INFO_PRINTF(
        "[CALIB ASSEMBLY] J%02u manual=%u->%u auto=%u->%u dir=%d "
        "minOffset=%d/%ld maxShortfall=%d/%ld autoSpan=%u "
        "manualSpan=%u urdfSpan=%u spanDiff=%u/%ld "
        "reason=0x%02X warning=%u\r\n",
        (unsigned)jointIndex,
        (unsigned)g_manualEncoderReference[jointIndex].minRaw,
        (unsigned)g_manualEncoderReference[jointIndex].maxRaw,
        (unsigned)result.autoMinRaw,
        (unsigned)result.autoMaxRaw,
        (int)g_encoderDirection[jointIndex],
        (int)info.autoMinOffsetCounts,
        (long)kCalibrationAutoMinAssemblyWarnCounts,
        (int)info.autoMaxShortfallCounts,
        (long)kCalibrationAutoMaxShortfallWarnCounts,
        (unsigned)info.autoSpanCounts,
        (unsigned)info.manualSpanCounts,
        (unsigned)info.urdfSpanCounts,
        (unsigned)info.spanDifferenceCounts,
        (long)kCalibrationUrdfManualSpanWarnCounts,
        (unsigned)info.reasonMask,
        warning ? 1U : 0U);
    return warning;
}
extern const int8_t kDefaultEncoderDirection[ENCODER_TOTAL_NUM] = {
    -1, -1, -1,  1,
    -1, -1, -1,  1,
    -1, -1, -1,  1,
    -1, -1, -1,  1,
    -1,  -1, -1, 1, -1
};

// Runtime encoder direction. It starts from the default installation table, and
// automatic calibration resets the active joint to that fixed table instead of
// learning direction from noisy travel samples. Angle display, manual
// calibration, and PID all read this table.
int8_t g_encoderDirection[ENCODER_TOTAL_NUM] = {
    -1, -1, -1,  1,
    -1, -1, -1,  1,
    -1, -1, -1,  1,
    -1, -1, -1,  1,
    -1, -1, -1,  1, -1
};

void resetEncoderDirectionsToDefault(void)
{
    memcpy(g_encoderDirection, kDefaultEncoderDirection, sizeof(g_encoderDirection));
}

bool setRuntimeEncoderDirection(uint8_t jointIndex, int8_t direction)
{
    if (jointIndex >= ENCODER_TOTAL_NUM || direction == 0) {
        return false;
    }
    g_encoderDirection[jointIndex] = (direction < 0) ? -1 : 1;
    return true;
}

// 手工回退 offset 表：
// 当自动标定结果不可用时，控制任务会读取该表作为零偏�?
int32_t g_encoderOffsetManual[ENCODER_TOTAL_NUM] = {0};

static const uint8_t kJoint16Index = 16;


// joint16 双舵机标定约束默认配置：
// 主舵?bus3-id17，副舵机 bus3-id18，默认偏置为0�?
JointDualServoConstraint g_joint16DualServoConstraint = {
    0,
    kJoint16Index,
    0,
    0,
    0,
    0,
    0,
    0
};

// 获取关节双舵机约束：
// - 当前�?joint16 返回有效约束�?
// - 其余关节返回 false，调用方按单舵机流程处理�?
bool getJointDualServoConstraint(uint8_t jointIndex, JointDualServoConstraint* out) {
    if (!out) {
        return false;
    }
    JointActuatorItem primary;
    JointActuatorItem secondary;
    if (!getJointActuator(jointIndex, 0, &primary) ||
        !getJointActuator(jointIndex, 1, &secondary)) {
        return false;
    }
    const MotorMapItem& p = motorMap[primary.motorChannel];
    const MotorMapItem& s = motorMap[secondary.motorChannel];
    out->enabled = 1;
    out->jointIndex = jointIndex;
    out->primaryBusIndex = p.busIndex;
    out->primaryServoID = p.servoID;
    out->secondaryBusIndex = s.busIndex;
    out->secondaryServoID = s.servoID;
    out->secondaryOffset = secondary.offset;
    out->tensionBias = secondary.tensionBias;
    return true;
}

bool computeSecondaryServoTargetForJoint(uint8_t jointIndex,
                                         int32_t primaryTarget,
                                         int16_t* outSecondaryTarget) {
    (void)jointIndex;
    (void)primaryTarget;
    if (outSecondaryTarget) {
        *outSecondaryTarget = 0;
    }
    return false;
}

// 初始化运行期标定存储。当前非阻塞自动标定状态机直接使用
// AutoCalibrationParams，不再装载旧的逐关节静态 Profile。
void initializeCalibrationRuntimeState(void) {
    memset(g_jointCalibResult, 0, sizeof(g_jointCalibResult));
    memset(g_manualEncoderReference, 0, sizeof(g_manualEncoderReference));
    memset(g_encoderOffsetManual, 0, sizeof(g_encoderOffsetManual));

    FW_LOG_INFO_PRINTLN("[BOOT CALIB] runtime state cleared; auto calibration pending");

    getJointDualServoConstraint(kJoint16Index, &g_joint16DualServoConstraint);
}
