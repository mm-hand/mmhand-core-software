/**
 * @file Crc16Ccitt.cpp
 * @brief 实现 CRC16-CCITT-FALSE 的无查表逐位算法。
 */
#include "Crc16Ccitt.h"

namespace p4::integrity {

uint16_t crc16CcittFalse(const uint8_t* data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    if (!data && length != 0U) {
        return crc;
    }
    for (size_t index = 0; index < length; ++index) {
        crc ^= static_cast<uint16_t>(data[index]) << 8;
        for (uint8_t bit = 0; bit < 8U; ++bit) {
            crc = (crc & 0x8000U) != 0U
                ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
                : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

} // namespace p4::integrity
