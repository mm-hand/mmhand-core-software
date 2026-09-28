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

/**
 * 单槽目标邮箱中超过该年龄的批次不得进入驱动。
 * 500 ms 覆盖当前同步读偶发长周期，同时仍能拒绝模式切换前遗留的旧命令。
 */
inline constexpr uint32_t kServoTargetMaxAgeMs = 500;

/** 上位机 1 s 没有任何有效协议帧时，所有舵机运动态的公共租约失效。 */
inline constexpr uint32_t kHostControlLeaseTimeoutMs = 1000;

/** Hold 批次必须在该时间内由 ServoBusTask 确认，否则升级为 E-STOP。 */
inline constexpr uint32_t kHostControlHoldApplyTimeoutMs = 500;

static_assert(kServoResetZeroTriggerMin > kServoHardwareCommandMin &&
                  kServoResetZeroTriggerMax < kServoHardwareCommandMax,
              "Reset Zero trigger must stay inside hardware range");

}

#endif // MMHAND_SAFETY_CONFIG_H
