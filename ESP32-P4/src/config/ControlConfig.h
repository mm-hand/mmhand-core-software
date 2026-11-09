/**
 * @file ControlConfig.h
 * @brief 集中定义反馈控制、碰撞检测和控制完成判定使用的固定参数。
 *
 * 本文件只保存不可变配置，不保存 PID 历史状态。修改这些数值会改变机械行为，
 * 必须经过日志对比和实机回归；控制算法不得在实现文件中复制同名“临时阈值”。
 */
#ifndef MMHAND_CONTROL_CONFIG_H
#define MMHAND_CONTROL_CONFIG_H

#include <stdint.h>

// Both collision arming and post-arm detection use this complete window.
#define COLLISION_WINDOW_MS 500

namespace mmhand::config::control {

/** P4 关节反馈控制任务的固定周期，单位秒。 */
inline constexpr float kPeriodSec = 0.010f;

/** 14 bit 磁编码器一圈的原始计数。 */
inline constexpr int32_t kEncoderModulo = 16384;
inline constexpr uint16_t kEncoderDisconnectRaw = 0xFFFF;
/** S3 接线自检分类哨兵（均超出 14 位角度范围）：0x7FFE=开路，0x7FFD=短路。 */
inline constexpr uint16_t kEncoderOpenRaw = 0x7FFE;
inline constexpr uint16_t kEncoderShortRaw = 0x7FFD;

inline constexpr uint32_t kCanBusOfflineTimeoutMs = 1000;
inline constexpr uint32_t kServoTelemetryTimeoutMs = 500;

/** 速度 0 表示使用舵机固件默认速度，适合连续反馈控制。 */
inline constexpr uint16_t kFeedbackServoSpeed = 0;

/** 关节反馈 D 项的测量速度一阶低通时间常数，单位毫秒。 */
inline constexpr float kFeedbackDerivativeFilterMs = 25.0f;
/** 测量速度限幅，单位 deg/s；用于拒绝编码器单帧异常。 */
inline constexpr float kFeedbackDerivativeLimitDegPerS = 400.0f;

inline constexpr int32_t kUpperWorkspaceWarningCounts = 48;
inline constexpr uint8_t kUpperWorkspaceWarningStableFrames = 3;

} // namespace mmhand::config::control

#endif // MMHAND_CONTROL_CONFIG_H
