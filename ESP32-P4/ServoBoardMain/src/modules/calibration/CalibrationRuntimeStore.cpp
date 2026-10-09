/**
 * @file CalibrationRuntimeStore.cpp
 * @brief 维护 Manual/Auto 标定运行数据及 V4 静态 URDF 角度契约。
 */
#include "CalibrationRuntimeStore.h"

#include <string.h>

#include "CalibrationMath.h"
#include "../../generated/UrdfJointLimits.generated.h"
#include "../../config/topology/HardwareMap.h"
#include "../../platform/rtos/SharedSafetyState.h"
#include "../../safety/SafetyEventReporter.h"
#include "../../safety/SafetyRuleRegistry.h"
#include "../../safety/policies/CalibrationSafetyPolicy.h"
#include "../../platform/log/FirmwareLog.h"

// 模块职责：
// 1) 管理关节标定结果和人工磁编参考值；
// 2) 提供标定结果告警、编码器方向和双执行器映射等公共能力；
// 3) 接收上位机生成的 LUT；P4 不执行机械搜索。
//
// 数据来源约束：
// - 编码器原始值来自 canRxQueue（RemoteSensorData_t）；
// - 标定结果通过已有 Manual/LUT 通信协议写入；
// - 标定生命周期与 UI 状态由上位机管理。

// 全局标定结果（按关节索引 0~ENCODER_TOTAL_NUM-1）。
JointCalibrationResult g_jointCalibResult[ENCODER_TOTAL_NUM];
ManualEncoderReference g_manualEncoderReference[ENCODER_TOTAL_NUM];

bool readJointCalibrationRuntimeSnapshot(
    uint8_t jointIndex,
    JointCalibrationRuntimeSnapshot* out)
{
    if (!out || jointIndex >= ENCODER_TOTAL_NUM) {
        return false;
    }

    // 标定结果包含整张 LUT，普通字段复制与写任务并发在 C++ 中不是合法数据竞争。
    // 统一在短临界区内复制，保证 Result、Manual 和方向属于同一提交代次。
    SharedSafetyGuard guard;
    out->result = g_jointCalibResult[jointIndex];
    out->manual = g_manualEncoderReference[jointIndex];
    out->encoderDirection = g_encoderDirection[jointIndex];
    return true;
}

bool readAllJointCalibrationRuntimeSnapshots(
    JointCalibrationRuntimeSnapshot out[ENCODER_TOTAL_NUM])
{
    if (!out) {
        return false;
    }
    // MotionControl 每周期只加锁一次，避免 21 次重复进入跨核临界区。
    SharedSafetyGuard guard;
    for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; ++joint) {
        out[joint].result = g_jointCalibResult[joint];
        out[joint].manual = g_manualEncoderReference[joint];
        out[joint].encoderDirection = g_encoderDirection[joint];
    }
    return true;
}

bool publishJointCalibrationResult(
    uint8_t jointIndex,
    const JointCalibrationResult& result)
{
    if (jointIndex >= ENCODER_TOTAL_NUM) {
        return false;
    }
    SharedSafetyGuard guard;
    g_jointCalibResult[jointIndex] = result;
    return true;
}



void clearJointCalibrationResult(uint8_t jointIndex)
{
    if (jointIndex >= ENCODER_TOTAL_NUM) {
        return;
    }
    SharedSafetyGuard guard;
    memset(&g_jointCalibResult[jointIndex], 0,
           sizeof(g_jointCalibResult[jointIndex]));
}

void clearAllJointCalibrationResults(void)
{
    SharedSafetyGuard guard;
    for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; ++joint) {
        memset(&g_jointCalibResult[joint], 0,
               sizeof(g_jointCalibResult[joint]));
    }
}

void replaceManualEncoderReferences(
    const ManualEncoderReference references[ENCODER_TOTAL_NUM])
{
    SharedSafetyGuard guard;
    for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; ++joint) {
        if (references) {
            g_manualEncoderReference[joint] = references[joint];
        } else {
            memset(&g_manualEncoderReference[joint], 0,
                   sizeof(g_manualEncoderReference[joint]));
        }
    }
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
    out->reasonMask = p4::safety::calibrationAssemblyWarningReasonForMetrics(
        minOffset, maxShortfall, spanDifference);
    return true;
}

// 清除指定关节的装配告警位和对应诊断值。
void clearCalibrationAssemblyWarnings(SystemRuntimeContext* sharedData,
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
    SystemRuntimeContext* sharedData,
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
        (long)p4::safety::kCalibrationAutoMinAssemblyWarnCounts,
        (int)info.autoMaxShortfallCounts,
        (long)p4::safety::kCalibrationAutoMaxShortfallWarnCounts,
        (unsigned)info.autoSpanCounts,
        (unsigned)info.manualSpanCounts,
        (unsigned)info.urdfSpanCounts,
        (unsigned)info.spanDifferenceCounts,
        (long)p4::safety::kCalibrationUrdfManualSpanWarnCounts,
        (unsigned)info.reasonMask,
        warning ? 1U : 0U);

    // 三类装配告警互相独立，逐位写入统一事件缓存，便于上位机按稳定规则码解释。
    if (warning) {
        const uint8_t warningBits[] = {
            CALIB_WARN_AUTO_MIN_OFFSET,
            CALIB_WARN_AUTO_MAX_SHORTFALL,
            CALIB_WARN_URDF_MANUAL_SPAN_MISMATCH,
        };
        for (uint8_t warningBit : warningBits) {
            if ((info.reasonMask & warningBit) == 0) {
                continue;
            }
            p4::safety::reportSafetyRuleEvent(
                sharedData,
                p4::safety::ruleForCalibrationWarningBit(warningBit),
                info.reasonMask,
                warningBit,
                jointIndex,
                0xFF);
        }
    }
    return warning;
}
// 运行期磁编方向表。默认使用固定安装方向，上位机可通过现有协议同步方向；
// 角度显示、Manual/Auto 跨度计算和 PID 反馈映射都读取这张表。
// +1 表示 raw 增加为关节正向，-1 表示 raw 减少。
int8_t g_encoderDirection[ENCODER_TOTAL_NUM] = {
    -1, -1, -1,  1,
    -1, -1, -1,  1,
    -1, -1, -1,  1,
    -1, -1, -1,  1,
    -1, -1, -1,  1, -1
};

// 修改单关节运行期方向；输入只接受 ±1。
bool setRuntimeEncoderDirection(uint8_t jointIndex, int8_t direction)
{
    if (jointIndex >= ENCODER_TOTAL_NUM || direction == 0) {
        return false;
    }
    SharedSafetyGuard guard;
    g_encoderDirection[jointIndex] = (direction < 0) ? -1 : 1;
    return true;
}

// 初始化标定 RAM。P4 重启后由桌面端 JSON 通过 Manual/Calib Load 协议重新同步数据。
void initializeCalibrationRuntimeState(void) {
    memset(g_jointCalibResult, 0, sizeof(g_jointCalibResult));
    memset(g_manualEncoderReference, 0, sizeof(g_manualEncoderReference));

    FW_LOG_INFO_PRINTLN("[BOOT CALIB] runtime state cleared; waiting for desktop Manual/LUT data");
}
