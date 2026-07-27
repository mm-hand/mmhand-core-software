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
// 数据来源约束：
// - 编码器原始值来自 canRxQueue（RemoteSensorData_t）；
// - 舵机运动与反馈通过 servoCommunicationTask 的队列快照交互；
// - 标定生命周期与 UI 状态由 StateMachineTask 统一管理。

// 全局标定结果（按关节索引 0~ENCODER_TOTAL_NUM-1）。
JointCalibrationResult g_jointCalibResult[ENCODER_TOTAL_NUM];
ManualEncoderReference g_manualEncoderReference[ENCODER_TOTAL_NUM];

// 计算 Auto Min 相对 Manual Min 的有符号正向偏移，供装配告警显示。
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

// 汇总单关节的三项装配偏差；这里只计算告警，不改变标定成功状态。
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

// 清除指定关节的装配告警位和对应诊断值。
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

// 将最新装配告警写入共享状态，并输出一条可追溯的诊断日志。
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
// 21 路磁编固定安装方向：+1 表示 raw 增加为关节正向，-1 表示 raw 减少。
extern const int8_t kDefaultEncoderDirection[ENCODER_TOTAL_NUM] = {
    -1, -1, -1,  1,
    -1, -1, -1,  1,
    -1, -1, -1,  1,
    -1, -1, -1,  1,
    -1,  -1, -1, 1, -1
};

// 运行期磁编方向表。自动标定不从噪声行程“学习方向”，而是恢复固定安装表；
// 角度显示、Manual/Auto 跨度计算和 PID 反馈映射都读取这张表。
int8_t g_encoderDirection[ENCODER_TOTAL_NUM] = {
    -1, -1, -1,  1,
    -1, -1, -1,  1,
    -1, -1, -1,  1,
    -1, -1, -1,  1,
    -1, -1, -1,  1, -1
};

// 将全部运行期方向恢复为固定安装方向。
void resetEncoderDirectionsToDefault(void)
{
    memcpy(g_encoderDirection, kDefaultEncoderDirection, sizeof(g_encoderDirection));
}

// 修改单关节运行期方向；输入只接受 ±1。
bool setRuntimeEncoderDirection(uint8_t jointIndex, int8_t direction)
{
    if (jointIndex >= ENCODER_TOTAL_NUM || direction == 0) {
        return false;
    }
    g_encoderDirection[jointIndex] = (direction < 0) ? -1 : 1;
    return true;
}

// 初始化标定 RAM。这里只清空内存；Manual NVS 和 Calib Load 会在启动流程中
// 按各自校验结果恢复数据。
void initializeCalibrationRuntimeState(void) {
    memset(g_jointCalibResult, 0, sizeof(g_jointCalibResult));
    memset(g_manualEncoderReference, 0, sizeof(g_manualEncoderReference));

    FW_LOG_INFO_PRINTLN("[BOOT CALIB] runtime state cleared; auto calibration pending");
}
