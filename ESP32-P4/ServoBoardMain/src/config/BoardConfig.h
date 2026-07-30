/**
 * @file BoardConfig.h
 * @brief 定义 ESP32-P4 板级引脚和物理通信接口参数。
 * @details 硬件映射表仍由 HardwareMap 管理，本文件不保存关节到舵机关系。
 */
#ifndef MMHAND_BOARD_CONFIG_H
#define MMHAND_BOARD_CONFIG_H

#include <stdint.h>

inline constexpr int TWAI_TX_PIN = 47;
inline constexpr int TWAI_RX_PIN = 48;

#endif // MMHAND_BOARD_CONFIG_H
