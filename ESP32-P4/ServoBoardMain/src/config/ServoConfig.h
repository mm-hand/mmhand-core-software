/**
 * @file ServoConfig.h
 * @brief 定义飞特舵机总线的默认通信与运动参数。
 * @details 这里的值是全局默认值；每条命令仍可携带自己的速度和加速度。
 */
#ifndef MMHAND_SERVO_CONFIG_H
#define MMHAND_SERVO_CONFIG_H

#include <stdint.h>

// 该宏参与条件编译，暂时保留宏形式以兼容 Arduino 预处理器。
#define MULTITURN_NVS_PERSISTENCE_ENABLE 1

inline constexpr uint32_t SERVO_SYNC_READ_TIMEOUT_MS_DEFAULT = 3;
inline constexpr uint16_t SERVO_TARGET_SPEED_DEFAULT = 400;
inline constexpr uint8_t SERVO_TARGET_ACC_DEFAULT = 200;
/** NVS 快照在单圈零点两侧该距离内时，额外触发一次边界保存。 */
inline constexpr int16_t kMultiTurnEdgeRawMarginCounts = 256;

namespace mmhand::config::servo {

/** 多圈恢复与异步 NVS 协调参数；不改变现有快照布局和键名。 */
inline constexpr uint32_t kNvsRestoreWaitWindowMs = 3000;
inline constexpr uint32_t kHostHeartbeatOfflineMs = 1000;
inline constexpr uint32_t kNvsSaveMinIntervalMs = 500;
inline constexpr uint32_t kNvsOfflinePeriodicSaveMs = 2000;
inline constexpr uint16_t kNvsRestoreRawToleranceCounts = 256;
inline constexpr int32_t kNvsMoveAbsThreshold = 64;

} // namespace mmhand::config::servo

#endif // MMHAND_SERVO_CONFIG_H
