/**
 * @file CalibrationSafetyPolicy.h
 * @brief 定义上位机上传标定结果的装配告警阈值与纯判断规则。
 * @details P4 不运行自动标定运动逻辑；告警只解释已接收的 Manual/LUT 结果。
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

} // namespace p4::safety

#endif // P4_SAFETY_POLICIES_CALIBRATION_SAFETY_POLICY_H
