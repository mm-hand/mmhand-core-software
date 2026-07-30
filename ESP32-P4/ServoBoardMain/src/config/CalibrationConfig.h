/**
 * @file CalibrationConfig.h
 * @brief 集中定义自动标定、双腱工作空间规划和 Rope Init 的固定参数。
 *
 * Manual/Auto/LUT 属于运行数据，不放入本文件。所有 counts、毫秒和角度
 * 常量均在名称中标明单位，避免标定状态机中出现无法追溯的裸数字。
 */
#ifndef MMHAND_CALIBRATION_CONFIG_H
#define MMHAND_CALIBRATION_CONFIG_H

#include <stdint.h>

namespace mmhand::config::calibration {

inline constexpr uint32_t kFeedbackReadyServoTimeoutMs = 500;
inline constexpr uint32_t kEncoderTrustedSampleMaxAgeMs = 300;
inline constexpr uint32_t kInitialFeedbackWaitMs = 1200;
inline constexpr uint32_t kServoRecoveryTimeoutMs = 5000;
inline constexpr uint32_t kServoRecoveryLogIntervalMs = 500;
inline constexpr uint8_t kServoRecoveryStableSamples = 3;
inline constexpr uint32_t kTrustCommitTimeoutMs = 1000;

inline constexpr float kMinTravelDeg = 10.0f;
inline constexpr float kMaxTravelDeg = 359.0f;
inline constexpr int32_t kRejectNearFullWrapSpanCounts = 15360;
inline constexpr float kLutStartDeg = -5.0f;
inline constexpr float kLutZeroDeg = 0.0f;
inline constexpr int32_t kServoZeroPullCounts = 500;

inline constexpr int32_t kDualInitialSlackCounts = 3200;
inline constexpr int32_t kMotorAbsWindowCounts = 4096 * 5;
inline constexpr int32_t kMotorAbsMaxMarginCounts = 500;
inline constexpr int16_t kOverloadLoadThreshold = 800;
inline constexpr int16_t kDualPrimaryTightLoadThreshold = 190;
inline constexpr int16_t kDualSecondaryPreloadLoadThreshold = 190;
inline constexpr int16_t kDualSecondaryFinalLoadThreshold = 250;
inline constexpr int16_t kDualPrimaryFinalLoadThreshold = 120;

inline constexpr int16_t kWorkspaceTightLoadThreshold = 190;
inline constexpr int32_t kWorkspaceReleaseCounts = 3200;
inline constexpr int32_t kWorkspaceTargetToleranceCounts = 60;
inline constexpr float kReverseMinFollowDeg = 12.0f;

inline constexpr float kTendonFitSampleStepDeg = 5.0f;
inline constexpr int32_t kSingleTendonMaxReleaseCounts = 4096 * 2;
inline constexpr uint8_t kAngleStallStableSamples = 2;
inline constexpr int32_t kPreReleaseSyncTargetToleranceCounts = 40;
inline constexpr int32_t kPreReleaseSyncSettleDeltaCounts = 20;
inline constexpr uint8_t kPreReleaseSyncStableSamples = 2;

} // namespace mmhand::config::calibration

#endif // MMHAND_CALIBRATION_CONFIG_H
