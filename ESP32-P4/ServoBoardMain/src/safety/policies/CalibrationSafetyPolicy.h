/**
 * @file CalibrationSafetyPolicy.h
 * @brief 集中定义自动标定过程会拒绝或终止机械运动的纯判断规则。
 *
 * Policy 不写标定状态、不发舵机目标、不生成弹窗诊断；调用方根据 decision
 * 设置 CalibrationDiag、保持实际 ABS 并结束对应子状态机。
 */
#ifndef P4_SAFETY_POLICIES_CALIBRATION_SAFETY_POLICY_H
#define P4_SAFETY_POLICIES_CALIBRATION_SAFETY_POLICY_H

#include <stdint.h>

#include "../../data/CalibrationTypes.h"

namespace p4::safety {

/** 装配告警阈值，单位均为磁编码器 counts；告警只提示装配偏差，不否决标定结果。 */
constexpr int32_t kCalibrationAutoMinAssemblyWarnCounts = 350;
constexpr int32_t kCalibrationAutoMaxShortfallWarnCounts = 300;
constexpr int32_t kCalibrationUrdfManualSpanWarnCounts = 360;

/** @brief 根据三项已归一化偏差生成 CALIB_WARN_* 组合位图。 */
constexpr uint8_t calibrationAssemblyWarningReasonForMetrics(
    int32_t autoMinOffsetCounts,
    int32_t autoMaxShortfallCounts,
    int32_t urdfManualSpanDifferenceCounts)
{
    return
        (autoMinOffsetCounts >= kCalibrationAutoMinAssemblyWarnCounts
             ? CALIB_WARN_AUTO_MIN_OFFSET : CALIB_WARN_NONE) |
        (autoMaxShortfallCounts >= kCalibrationAutoMaxShortfallWarnCounts
             ? CALIB_WARN_AUTO_MAX_SHORTFALL : CALIB_WARN_NONE) |
        (urdfManualSpanDifferenceCounts >
                 kCalibrationUrdfManualSpanWarnCounts
             ? CALIB_WARN_URDF_MANUAL_SPAN_MISMATCH : CALIB_WARN_NONE);
}

static_assert(calibrationAssemblyWarningReasonForMetrics(349, 299, 360) ==
                  CALIB_WARN_NONE,
              "Assembly warning lower boundaries changed");
static_assert(calibrationAssemblyWarningReasonForMetrics(350, 299, 360) ==
                  CALIB_WARN_AUTO_MIN_OFFSET,
              "Auto Min warning threshold must be inclusive");
static_assert(calibrationAssemblyWarningReasonForMetrics(349, 300, 360) ==
                  CALIB_WARN_AUTO_MAX_SHORTFALL,
              "Auto Max warning threshold must be inclusive");
static_assert(calibrationAssemblyWarningReasonForMetrics(349, 299, 361) ==
                  CALIB_WARN_URDF_MANUAL_SPAN_MISMATCH,
              "URDF/Manual Span warning must trigger only above 360 counts");

enum class CalibrationSequenceReject : uint8_t {
    None = 0,
    NullContext,
    EmptyOrOversized,
    JointOutOfRange,
    DuplicateJoint,
};

/** @brief 标定关节列表校验结果，并携带首个错误元素和此前已见位图。 */
struct CalibrationSequenceDecision {
    bool allowed;
    CalibrationSequenceReject reason;
    uint8_t itemIndex;
    uint8_t jointIndex;
    uint32_t seenJointMask;
};

CalibrationSequenceDecision validateCalibrationSequence(
    bool contextAvailable,
    const uint8_t* joints,
    uint8_t jointCount,
    uint8_t maximumJointCount);

/** @brief 单腱从紧端释放量达到上限后，禁止继续向松绳方向推进。 */
bool calibrationReleaseExceeded(int32_t releaseCounts,
                                int32_t maximumReleaseCounts);

/** @brief 标定命令必须位于当前 logical 会话闭区间。 */
bool calibrationCommandInsideRange(bool rangeValid,
                                   int32_t commandAbs,
                                   int32_t rangeMinimum,
                                   int32_t rangeMaximum);

/** @brief 负载使用绝对幅值；达到阈值即判为标定硬过载。 */
bool calibrationLoadOverloaded(int16_t loadMagnitude,
                               int16_t overloadThreshold);

} // namespace p4::safety

#endif // P4_SAFETY_POLICIES_CALIBRATION_SAFETY_POLICY_H
