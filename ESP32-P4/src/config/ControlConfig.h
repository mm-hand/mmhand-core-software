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

inline constexpr uint32_t kCanBusOfflineTimeoutMs = 1000;
inline constexpr uint32_t kServoTelemetryTimeoutMs = 500;

/** 直控与反馈模式的硬过载阈值；碰撞检测不使用该负载值。 */
inline constexpr int16_t kDirectOverloadLoadThreshold = 800;
inline constexpr int16_t kFeedbackOverloadLoadThreshold = 2000;
inline constexpr uint8_t kDirectOverloadLatchCycles = 20;
inline constexpr uint8_t kFeedbackOverloadLatchCycles = 20;

/** 速度 0 表示使用舵机固件默认速度，适合连续反馈控制。 */
inline constexpr uint16_t kFeedbackServoSpeed = 0;

inline constexpr int32_t kUpperWorkspaceWarningCounts = 48;
inline constexpr uint8_t kUpperWorkspaceWarningStableFrames = 3;

} // namespace mmhand::config::control

#endif // MMHAND_CONTROL_CONFIG_H
