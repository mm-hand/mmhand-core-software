/**
 * @file ServoConfig.h
 * @brief 定义飞特舵机总线的默认通信与运动参数。
 * @details 这里的值是全局默认值；每条命令仍可携带自己的速度和加速度。
 */
#ifndef MMHAND_SERVO_CONFIG_H
#define MMHAND_SERVO_CONFIG_H

#include <stdint.h>
#include "DimensionsConfig.h"

// 该宏参与条件编译，暂时保留宏形式以兼容 Arduino 预处理器。
#define MULTITURN_NVS_PERSISTENCE_ENABLE 1

#ifndef SERVO_CONFIGURE_MULTI_TURN_ON_BOOT
#define SERVO_CONFIGURE_MULTI_TURN_ON_BOOT 1
#endif

inline constexpr uint32_t SERVO_SYNC_READ_TIMEOUT_MS_DEFAULT = 3; ///< 同步读单批等待时限。
inline constexpr uint16_t SERVO_TARGET_SPEED_DEFAULT = 400; ///< 标定/直控默认速度。
inline constexpr uint8_t SERVO_TARGET_ACC_DEFAULT = 200; ///< 标定/直控默认加速度。
/** NVS 快照在单圈零点两侧该距离内时，额外触发一次边界保存。 */
inline constexpr int16_t kMultiTurnEdgeRawMarginCounts = 256;

namespace mmhand::config::servo {

/** 多圈恢复与异步 NVS 协调参数；不改变现有快照布局和键名。 */
inline constexpr uint32_t kNvsRestoreWaitWindowMs = 3000;
inline constexpr uint32_t kNvsSaveMinIntervalMs = 500;
inline constexpr uint32_t kNvsOfflinePeriodicSaveMs = 2000;
inline constexpr uint16_t kNvsRestoreRawToleranceCounts = 256;
inline constexpr int32_t kNvsMoveAbsThreshold = 64;
inline constexpr uint32_t kMultiTurnConfigDelayMs = 5; ///< EEPROM 写入和 Reset 后等待时间。
inline constexpr uint8_t kOfflineInvalidateMisses = 3; ///< 连续读失败达到此值才判离线。
inline constexpr uint8_t kOnlineConfirmReads = 2; ///< 连续成功读取达到此值才判上线。
inline constexpr uint16_t kDiscoveryProbeTimeoutMs = 2; ///< 动态 BUS 探测单次最长阻塞时间。
/** 单 BUS 控制容量只有一个事实来源，避免扫描策略与同步收发缓存的上限发生漂移。 */
inline constexpr uint8_t kMaxControllableServosPerBus = MAX_SERVOS_PER_BUS;
/** 曾经在线后掉线的 Mxx 定向重扫间隔；初次上电缺失的舵机不进入此循环。 */
inline constexpr uint32_t kOfflineRouteRescanIntervalMs = 500;
inline constexpr uint8_t kRecoveryMaxAttempts = 3; ///< 单次上线恢复最大尝试次数。
inline constexpr uint32_t kRecoveryRetryIntervalMs = 100; ///< 上线恢复重试间隔。

} // namespace mmhand::config::servo

#endif // MMHAND_SERVO_CONFIG_H
