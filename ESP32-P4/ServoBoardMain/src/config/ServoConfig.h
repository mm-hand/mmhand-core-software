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

#endif // MMHAND_SERVO_CONFIG_H
