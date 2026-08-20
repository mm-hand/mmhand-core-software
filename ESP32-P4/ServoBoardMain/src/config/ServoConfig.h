/**
 * @file ServoConfig.h
 * @brief 定义飞特舵机总线的默认通信与运动参数。
 * @details 这里的值是全局默认值；每条命令仍可携带自己的速度和加速度。
 */
#ifndef MMHAND_SERVO_CONFIG_H
#define MMHAND_SERVO_CONFIG_H

#include <stdint.h>
#include "DimensionsConfig.h"

/**
 * P4 Flash 业务持久化总开关。
 *
 * 当前阶段由上位机 JSON 保存 Manual/Auto/LUT/ABS Limit 与运行多圈锚点，
 * 因此默认关闭 P4 NVS 读写。旧实现继续保留在 ENABLE=1 分支中，实机验证
 * 完成后再统一删除，禁止局部重新打开其中某一种 Flash 数据源。
 */
#define P4_FLASH_PERSISTENCE_ENABLE 0
#define MULTITURN_NVS_PERSISTENCE_ENABLE P4_FLASH_PERSISTENCE_ENABLE

inline constexpr uint32_t SERVO_SYNC_READ_TIMEOUT_MS_DEFAULT = 3; ///< 同步读单批等待时限。
inline constexpr uint16_t SERVO_TARGET_SPEED_DEFAULT = 400; ///< 标定/直控默认速度。
inline constexpr uint8_t SERVO_TARGET_ACC_DEFAULT = 200; ///< 标定/直控默认加速度。
/** NVS 快照在单圈零点两侧该距离内时，额外触发一次边界保存。 */
inline constexpr int16_t kMultiTurnEdgeRawMarginCounts = 256;

namespace mmhand::config::servo {

// JSON 最近圈恢复后的首次明显运动用于核对方向。阈值故意低于
// 正常控制步长，但高于舵机 raw 抖动，避免静止时误判。
inline constexpr int32_t kRestoreDirectionCommandCounts = 64;
inline constexpr int32_t kRestoreDirectionConfirmCounts = 32;
inline constexpr int32_t kRestoreDirectionRejectCounts = 24;
inline constexpr uint32_t kRestoreDirectionObserveMs = 600;

/** 多圈恢复与异步 NVS 协调参数；不改变现有快照布局和键名。 */
inline constexpr uint32_t kNvsRestoreWaitWindowMs = 3000;
inline constexpr uint32_t kNvsSaveMinIntervalMs = 500;
inline constexpr uint32_t kNvsOfflinePeriodicSaveMs = 2000;   //离线期间每隔该时间保存一次快照。
inline constexpr int32_t kNvsMoveAbsThreshold = 64;             ///< 运动期间逻辑 ABS 变化超过该值才触发离线保存。
inline constexpr uint32_t kMultiTurnConfigDelayMs = 5; ///< EEPROM 写入和 Reset 后等待时间。
inline constexpr uint8_t kOfflineInvalidateMisses = 3; ///< 连续读失败达到此值才判离线。
/**
 * 即使扫描或恢复流程没有累计同步读失败，反馈超过该时间未刷新也必须判离线。
 * 正常情况下每条 BUS 约 20 ms 更新一次，200 ms 为十个反馈周期的安全余量。
 */
inline constexpr uint32_t kFeedbackStaleTimeoutMs = 200;
inline constexpr uint8_t kOnlineConfirmReads = 2; ///< 连续成功读取达到此值才判上线。
inline constexpr uint16_t kDiscoveryProbeTimeoutMs = 2; ///< 动态 BUS 探测单次最长阻塞时间。
/** 单 BUS 控制容量只有一个事实来源，避免扫描策略与同步收发缓存的上限发生漂移。 */
inline constexpr uint8_t kMaxControllableServosPerBus = MAX_SERVOS_PER_BUS;
/** 离线 Mxx 的定向重扫间隔；包括曾经在线后掉线和首次全量扫描时缺失的舵机。 */
inline constexpr uint32_t kOfflineRouteRescanIntervalMs = 500;
/** 同一条 UART BUS 两次底层重启之间的最短间隔，避免反复抖动影响同 BUS 在线舵机。 */
inline constexpr uint32_t kTransportRestartMinIntervalMs = 1000;
/** UART 重新 begin 后等待驱动和收发方向稳定，再执行单播探测。 */
inline constexpr uint32_t kTransportRestartSettleMs = 20;
/** 热插拔定向扫描及 BUS 重启确认允许比启动全量扫描多等待少量时间。 */
inline constexpr uint16_t kRecoveryProbeTimeoutMs = 5;
/** 同一 Mxx 连续定向重扫失败达到该次数后，才重启它掉线前所在的 BUS。 */
inline constexpr uint8_t kTransportRestartAfterFailedScans = 2;
inline constexpr uint8_t kRecoveryMaxAttempts = 3; ///< 单次上线恢复最大尝试次数。
inline constexpr uint32_t kRecoveryRetryIntervalMs = 100; ///< 上线恢复重试间隔。

} // namespace mmhand::config::servo

#endif // MMHAND_SERVO_CONFIG_H
