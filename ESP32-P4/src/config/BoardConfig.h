/**
 * @file BoardConfig.h
 * @brief 定义 ESP32-P4 板级引脚和物理通信接口参数。
 * @details 硬件映射表仍由 HardwareMap 管理，本文件不保存关节到舵机关系。
 */
#ifndef MMHAND_BOARD_CONFIG_H
#define MMHAND_BOARD_CONFIG_H

#include "DimensionsConfig.h"
#include <stddef.h>
#include <stdint.h>

inline constexpr int TWAI_TX_PIN = 47;
inline constexpr int TWAI_RX_PIN = 48;

/** 四条飞特总线的 UART 引脚；舵机所属 BUS 由启动扫描动态发现。 */
inline constexpr int kServoBusRxPins[NUM_BUSES] = {21, 23, 27, 33};
inline constexpr int kServoBusTxPins[NUM_BUSES] = {20, 22, 26, 32};
/** P4 与上位机之间的运行期 USB-UART 波特率；必须与 desktop 的 BAUDRATE 保持一致。 */
// Runtime host protocol and application logs use the expansion-board USB
// Serial/JTAG connector. Waveshare CH343 Type-C is only a migration/recovery
// upload path. Arduino: USBMode=hwcdc, CDCOnBoot=cdc, JTAGAdapter=builtin.
inline constexpr int kHostUsbDmPin = 24;
inline constexpr int kHostUsbDpPin = 25;
inline constexpr uint32_t kHostUartBaud = 921600; // CDC line coding; not a UART wire rate.
inline constexpr uint32_t kServoBusBaud = 1000000;
/**
 * UART 驱动 RX 环形缓冲区容量。
 *
 * Host 侧需要覆盖最大约 232 byte 的下行帧及任务调度抖动；飞特侧需要覆盖
 * 单 BUS 12 台舵机的最大同步回包（约 252 byte），并为 UART 事件搬运留余量。
 * TX 保持 Arduino 默认的无软件环形缓冲，避免控制命令额外排队。
 */
inline constexpr size_t kHostUartRxBufferBytes = 1024;
inline constexpr size_t kServoBusUartRxBufferBytes = 512;
static_assert(kServoBusUartRxBufferBytes >= MAX_SERVOS_PER_BUS * (15U + 6U) * 2U,
              "servo UART RX ring must hold two maximum sync-read response sets");
#endif // MMHAND_BOARD_CONFIG_H
