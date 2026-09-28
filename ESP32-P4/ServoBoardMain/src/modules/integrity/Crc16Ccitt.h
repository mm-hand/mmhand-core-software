/**
 * @file Crc16Ccitt.h
 * @brief 声明跨通信链路复用的 CRC16-CCITT-FALSE 计算接口。
 * @details 固定参数为 poly=0x1021、init=0xFFFF、refin=false、refout=false、xorout=0。
 */
#ifndef P4_MODULES_INTEGRITY_CRC16_CCITT_H
#define P4_MODULES_INTEGRITY_CRC16_CCITT_H

#include <stddef.h>
#include <stdint.h>

namespace p4::integrity {

/** @brief 逐字节计算 CRC16-CCITT-FALSE；空数据返回初值 0xFFFF。 */
uint16_t crc16CcittFalse(const uint8_t* data, size_t length);

} // namespace p4::integrity

#endif // P4_MODULES_INTEGRITY_CRC16_CCITT_H
