/**
 * @file Crc32Ieee.cpp
 * @brief 实现 CRC-32/ISO-HDLC 的无查表逐位增量算法。
 */
#include "Crc32Ieee.h"

namespace p4::integrity {

uint32_t crc32IeeeUpdate(uint32_t previousCrc,
                        const uint8_t* data,
                        size_t length)
{
    uint32_t crc = ~previousCrc;
    if (!data && length != 0U) {
        return previousCrc;
    }
    for (size_t index = 0; index < length; ++index) {
        crc ^= data[index];
        for (uint8_t bit = 0; bit < 8U; ++bit) {
            const uint32_t mask =
                static_cast<uint32_t>(-static_cast<int32_t>(crc & 1U));
            crc = (crc >> 1) ^ (0xEDB88320UL & mask);
        }
    }
    return ~crc;
}

} // namespace p4::integrity
