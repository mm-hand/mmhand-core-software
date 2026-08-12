/**
 * @file BoardConfig.h
 * @brief 定义 ESP32-P4 板级引脚和物理通信接口参数。
 * @details 硬件映射表仍由 HardwareMap 管理，本文件不保存关节到舵机关系。
 */
#ifndef MMHAND_BOARD_CONFIG_H
#define MMHAND_BOARD_CONFIG_H

#include "DimensionsConfig.h"
#include <stdint.h>

inline constexpr int TWAI_TX_PIN = 47;
inline constexpr int TWAI_RX_PIN = 48;

/** 四条飞特总线的 UART 引脚；舵机所属 BUS 由启动扫描动态发现。 */
inline constexpr int kServoBusRxPins[NUM_BUSES] = {21, 23, 27, 33};
inline constexpr int kServoBusTxPins[NUM_BUSES] = {20, 22, 26, 32};
/** P4 与上位机之间的运行期 USB-UART 波特率；必须与 desktop 的 BAUDRATE 保持一致。 */
inline constexpr uint32_t kHostUartBaud = 921600;
inline constexpr uint32_t kServoBusBaud = 1000000;
/** 当前 21 关节/27 舵机拓扑版本；写入多圈 NVS 映射签名，数值不得随意修改。 */
inline constexpr uint8_t kHardwareMappingVersion = 3;

#endif // MMHAND_BOARD_CONFIG_H
