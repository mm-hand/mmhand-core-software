/**
 * @file RecoveryTypes.h
 * @brief 定义舵机掉电恢复与硬件 Reset Zero 事务状态。
 */
#ifndef MMHAND_RECOVERY_TYPES_H
#define MMHAND_RECOVERY_TYPES_H

#include "../config/DimensionsConfig.h"
#include <stdint.h>

inline constexpr uint8_t SERVO_RESET_ZERO_STATUS_IDLE = 0;    ///< 没有 Reset Zero 事务。
inline constexpr uint8_t SERVO_RESET_ZERO_STATUS_RUNNING = 1; ///< 舵机硬件圈数正在归零。
inline constexpr uint8_t SERVO_RESET_ZERO_STATUS_SUCCESS = 2; ///< 最近一次事务成功。
inline constexpr uint8_t SERVO_RESET_ZERO_STATUS_FAILED = 3;  ///< 最近一次事务失败。

#endif // MMHAND_RECOVERY_TYPES_H
