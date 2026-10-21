/** @file RampParameters.cpp
 * @brief 验证并提交渐进快速换向参数快照。
 */
#include "RampParameters.h"
#include "../system/rtos/SharedSafetyState.h"
#include <math.h>
#include <string.h>

namespace mmhand::control {
namespace {
RampParameters current;
uint32_t u32(const uint8_t* p) {
    return uint32_t(p[0]) << 24 | uint32_t(p[1]) << 16 | uint32_t(p[2]) << 8 | p[3];
}
uint32_t crc32(const uint8_t* data, size_t length) {
    uint32_t crc = 0xFFFFFFFFUL;
    while (length--) {
        crc ^= *data++;
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320UL : 0);
    }
    return crc ^ 0xFFFFFFFFUL;
}
}
RampParameters readRampParameters() {
    SharedSafetyGuard guard;
    return current;
}
bool rampParametersConfigured() {
    SharedSafetyGuard guard;
    return current.configured;
}
bool applyRampParameterPayload(const uint8_t* payload, size_t length) {
    constexpr size_t bytes = RampParameterCount * 4;
    if (!payload || length != 9 + bytes || payload[0] != 1) return false;
    RampParameters next;
    next.revision = u32(payload + 1);
    next.crc = u32(payload + 5 + bytes);
    if (!next.revision || crc32(payload + 5, bytes) != next.crc) return false;
    for (size_t i = 0; i < RampParameterCount; ++i) {
        const uint32_t bits = u32(payload + 5 + 4 * i);
        memcpy(&next.values[i], &bits, 4);
        if (!isfinite(next.values[i]) || next.values[i] < kRampParameterMinimum[i] ||
            next.values[i] > kRampParameterMaximum[i]) return false;
    }
    // The Host hard watchdog is 1000 counts; never accept a freeze threshold above it.
    for (uint8_t type = 0; type < 5; ++type) {
        if (next.values[ReversalRampLoadLimitAa + type] > 1000.0f) return false;
    }
    next.configured = true;
    SharedSafetyGuard guard;
    current = next;
    return true;
}
} // namespace mmhand::control
