/**
 * @file Crc32Ieee.h
 * @brief 声明 NVS 快照和配置签名复用的 CRC-32/ISO-HDLC 增量接口。
 * @details 固定反射多项式 0xEDB88320，支持把上一次返回值继续传入分段计算。
 */
#ifndef P4_MODULES_INTEGRITY_CRC32_IEEE_H
#define P4_MODULES_INTEGRITY_CRC32_IEEE_H

#include <stddef.h>
#include <stdint.h>

namespace p4::integrity {

/** @brief 从 previousCrc 继续计算 CRC32；首次调用传 0。 */
uint32_t crc32IeeeUpdate(uint32_t previousCrc,
                        const uint8_t* data,
                        size_t length);

} // namespace p4::integrity

#endif // P4_MODULES_INTEGRITY_CRC32_IEEE_H
