/**
 * @file ServoConfig.h
 * @brief 定义飞特舵机总线的默认通信与运动参数。
 * @details 这里的值是全局默认值；每条命令仍可携带自己的速度和加速度。
 */
#ifndef MMHAND_SERVO_CONFIG_H
#define MMHAND_SERVO_CONFIG_H

#include <stdint.h>
#include "DimensionsConfig.h"

inline constexpr uint32_t SERVO_PROBE_TIMEOUT_MS = 3; ///< 独立舵机探测的阻塞读取时限。
inline constexpr uint16_t SERVO_TARGET_SPEED_DEFAULT = 400; ///< 标定/直控默认速度。
inline constexpr uint8_t SERVO_TARGET_ACC_DEFAULT = 254; ///< 标定/直控默认加速度。

namespace mmhand::config::servo {

/** ServoBus 服务任务的兜底周期；新目标通知仍可提前唤醒任务。 */
inline constexpr uint32_t kServoBusServicePeriodMs = 10;
/** 四路同步读发出后首次执行非阻塞收集的时间；普通8字节反馈通常已经完整。 */
inline constexpr uint32_t kServoFeedbackCollectDelayMs = 3;
/** 12台舵机、15字节反馈的硬截止时间；超时后解析已收到帧并登记缺失ID。 */
inline constexpr uint32_t kServoFeedbackDeadlineUs = 4000;
/** 100 Hz位置反馈下每10轮读取一次电流，保持电流约10 Hz。 */
inline constexpr uint8_t kServoCurrentReadDivider = 10;

static_assert(kServoCurrentReadDivider > 0,
              "servo current read divider must not be zero");
static_assert(kServoFeedbackCollectDelayMs < kServoBusServicePeriodMs,
              "servo feedback collect point must fit inside one control period");
static_assert(kServoFeedbackDeadlineUs <= kServoBusServicePeriodMs * 1000U,
              "servo feedback deadline must fit inside one control period");
static_assert(MAX_SERVOS_PER_BUS * (15U + 6U) <= 256U,
              "12-servo sync-read response exceeds protocol buffer");

inline constexpr uint32_t kMultiTurnConfigDelayMs = 5; ///< EEPROM 写入和 Reset 后等待时间。
inline constexpr uint8_t kOfflineInvalidateMisses = 3; ///< 连续读失败达到此值才判离线。
/**
 * 即使扫描或恢复流程没有累计同步读失败，反馈超过该时间未刷新也必须判离线。
 * 正常情况下四条 BUS 都在每个10 ms轮次更新；80 ms覆盖八个反馈周期，既允许
 * 短暂调度抖动，又能避免快速掉电重启长期沿用旧 hardware ABS 会话。
 */
inline constexpr uint32_t kFeedbackStaleTimeoutMs = 80;
// Reset Zero 会在 ServoBusTask 内同步执行 RESET、EEPROM 配置和反馈确认。
// 事务结束后预留一个 stale 周期，使四条 BUS 先各完成一次正常同步读，避免
// 把维护操作自身造成的轮询间隔误判为舵机掉电。
inline constexpr uint32_t kFeedbackMaintenanceGraceMs =
    kFeedbackStaleTimeoutMs;
/** 重新发现舵机后必须连续成功三帧，正常约30 ms，才重新发布online。 */
inline constexpr uint8_t kOnlineConfirmReads = 3;
inline constexpr uint16_t kDiscoveryProbeTimeoutMs = 2; ///< 动态 BUS 探测单次最长阻塞时间。
/** 单 BUS 控制容量只有一个事实来源，避免扫描策略与同步收发缓存的上限发生漂移。 */
inline constexpr uint8_t kMaxControllableServosPerBus = MAX_SERVOS_PER_BUS;
/** 离线 Mxx 的定向重扫间隔；包括曾经在线后掉线和首次全量扫描时缺失的舵机。 */
inline constexpr uint32_t kOfflineRouteRescanIntervalMs = 500;
/** 同一条 UART BUS 两次底层重启之间的最短间隔，避免反复抖动影响同 BUS 在线舵机。 */
/** 热插拔定向扫描允许比启动全量扫描多等待少量时间。 */
inline constexpr uint16_t kRecoveryProbeTimeoutMs = 5;
inline constexpr uint8_t kRecoveryMaxAttempts = 3; ///< 单次上线恢复最大尝试次数。
inline constexpr uint32_t kRecoveryRetryIntervalMs = 100; ///< 上线恢复重试间隔。
/** 单台舵机 RESET Zero 完整事务的最大尝试次数；每次都必须收到单播响应。 */
inline constexpr uint8_t kResetZeroTransactionAttempts = 3;
/** RESET 后重新读取反馈的最大次数，用于确认新 turn0 已经真实建立。 */
inline constexpr uint8_t kResetZeroFeedbackAttempts = 3;

static_assert(kResetZeroTransactionAttempts > 0 &&
                  kResetZeroFeedbackAttempts > 0,
              "servo reset-zero attempts must not be zero");

} // namespace mmhand::config::servo

#endif // MMHAND_SERVO_CONFIG_H
