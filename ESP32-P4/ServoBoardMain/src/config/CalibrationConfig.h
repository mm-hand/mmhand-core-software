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

inline constexpr uint32_t kFeedbackReadyServoTimeoutMs = 500; ///< 标定允许使用的舵机快照最大年龄。
inline constexpr uint32_t kEncoderTrustedSampleMaxAgeMs = 300; ///< 磁编 raw 样本最大年龄。
inline constexpr uint32_t kInitialFeedbackWaitMs = 1200; ///< 标定启动后等待首帧反馈的时限。
inline constexpr uint32_t kEncoderRecoveryTimeoutMs = 2000; ///< 标定预检等待所选磁编恢复的总时限。
inline constexpr uint32_t kEncoderRecoveryLogIntervalMs = 500; ///< 磁编恢复等待日志的节流周期。
inline constexpr uint8_t kEncoderRecoveryStableSamples = 3; ///< 开始机械运动前要求的连续有效磁编帧数。
inline constexpr uint32_t kServoRecoveryTimeoutMs = 5000; ///< 热插拔多圈恢复总时限。
inline constexpr uint32_t kServoRecoveryLogIntervalMs = 500; ///< 恢复等待日志节流周期。
inline constexpr uint8_t kServoRecoveryStableSamples = 3; ///< 恢复成功前要求的连续稳定帧数。
inline constexpr uint32_t kTrustCommitTimeoutMs = 1000; ///< 多圈可信状态提交时限。

inline constexpr float kMinTravelDeg = 10.0f; ///< 可接受 Auto 磁编行程换算后的最小角度。
inline constexpr float kMaxTravelDeg = 359.0f; ///< 防止环形方向歧义的最大角度。
inline constexpr int32_t kRejectNearFullWrapSpanCounts = 15360; ///< 接近完整 14 bit 一圈时拒绝。
inline constexpr float kLutStartDeg = -5.0f; ///< LUT 在 Auto Max 锚点外保留的起始角。
inline constexpr float kLutZeroDeg = 0.0f; ///< 标定 LUT 的机械紧端零点。
inline constexpr int32_t kServoZeroPullCounts = 500; ///< ABS Limit Max 相对标定紧端的安全余量。

inline constexpr int32_t kDualInitialSlackCounts = 3200; ///< 双腱准备阶段先释放的空程。
inline constexpr int32_t kMotorAbsWindowCounts = 4096 * 5; ///< 自动标定允许的 logical ABS 搜索窗。
inline constexpr int32_t kMotorAbsReleaseMarginCounts = 500; ///< LUT 松端之外的 ABS Limit 余量。
inline constexpr int32_t kMotorAbsTightMarginCounts = 1200; ///< LUT 紧端之外的 ABS Limit 余量。
inline constexpr int16_t kOverloadLoadThreshold = 800; ///< 标定期间的硬过载阈值。
inline constexpr int16_t kDualPrimaryTightLoadThreshold = 190; ///< 主腱首次紧绳阈值。
inline constexpr int16_t kDualSecondaryPreloadLoadThreshold = 190; ///< 副腱预紧阈值。
inline constexpr int16_t kDualSecondaryFinalLoadThreshold = 250; ///< 副腱最终 A 姿态阈值。
inline constexpr int16_t kDualPrimaryFinalLoadThreshold = 120; ///< 普通双腱最终主腱确认阈值。

inline constexpr int16_t kWorkspaceTightLoadThreshold = 190; ///< 辅助手指让位后紧绳阈值。
inline constexpr int32_t kWorkspaceReleaseCounts = 3200; ///< 工作空间姿态切换释放量。
inline constexpr int32_t kWorkspaceTargetToleranceCounts = 60; ///< 姿态缓存复用的 ABS 容差。
inline constexpr float kReverseMinFollowDeg = 12.0f; ///< 松绳阶段确认磁编已跟随的最小角进展。
inline constexpr uint32_t kTargetApplyAckTimeoutMs = 100; ///< 标定批次等待 ServoBus 原子执行确认的时限。

inline constexpr float kTendonFitSampleStepDeg = 5.0f; ///< 标定 LUT 的等角度采样间隔。
inline constexpr int32_t kSingleTendonMaxReleaseCounts = 4096 * 2; ///< 单腱松绳最多两圈保护。
inline constexpr uint8_t kAngleStallStableSamples = 2; ///< 机械端停止判定连续样本数。
inline constexpr int32_t kPreReleaseSyncTargetToleranceCounts = 40; ///< 松绳前 cmd/ABS 误差窗。
inline constexpr int32_t kPreReleaseSyncSettleDeltaCounts = 20; ///< 松绳前相邻 ABS 稳定窗。
inline constexpr uint8_t kPreReleaseSyncStableSamples = 2; ///< 松绳前连续稳定帧数。

} // namespace mmhand::config::calibration

#endif // MMHAND_CALIBRATION_CONFIG_H
