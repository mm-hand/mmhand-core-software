/**
 * @file SafetyConfig.h
 * @brief 汇总舵机硬边界、反馈超时和运行安全阈值。
 * @details 安全规则由 SafetyPolicy 执行；本文件只定义不可变配置。
 */
#ifndef MMHAND_SAFETY_CONFIG_H
#define MMHAND_SAFETY_CONFIG_H

#include <stdint.h>

namespace mmhand::config::safety {

/** ST3215 多圈模式单次上电会话允许下发的 hardware ABS 硬边界。 */
inline constexpr int32_t kServoHardwareCommandMin = -30719;
inline constexpr int32_t kServoHardwareCommandMax = 30719;

/**
 * 部分舵机接近端点时会停在约 ±30717，因此 Reset Zero 在硬边界内
 * 提前 4 counts 触发，为检测和复位事务保留可达余量。
 */
inline constexpr int32_t kServoResetZeroTriggerMin = -30715;
inline constexpr int32_t kServoResetZeroTriggerMax = 30715;

/** 总状态机用于判定底层反馈链路离线的时限。 */
inline constexpr uint32_t kStateCanOfflineTimeoutMs = 1000;
inline constexpr uint32_t kStateServoFeedbackTimeoutMs = 500;

/** S3 编码器明确故障持续达到该时间后才置位，过滤瞬时坏帧。 */
inline constexpr uint32_t kEncoderFaultTimeoutMs = 300;

static_assert(kServoResetZeroTriggerMin > kServoHardwareCommandMin &&
                  kServoResetZeroTriggerMax < kServoHardwareCommandMax,
              "Reset Zero trigger must stay inside hardware range");

}

#endif // MMHAND_SAFETY_CONFIG_H
