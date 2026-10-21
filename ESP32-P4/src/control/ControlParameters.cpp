/** @file ControlParameters.cpp
 * @brief 验证并提交上位机下发的控制参数快照。
 */
#include "ControlParameters.h"
#include "../system/rtos/SharedSafetyState.h"
#include <math.h>
#include <string.h>

namespace mmhand::control {
namespace {
Parameters current;
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
Parameters readParameters() {
    SharedSafetyGuard guard;
    return current;
}
bool parametersConfigured() {
    SharedSafetyGuard guard;
    return current.configured;
}
bool applyParameterPayload(const uint8_t* payload, size_t length) {
    constexpr size_t bytes = ParameterCount * 4;
    if (!payload || length != 9 + bytes || payload[0] != 3) return false;
    Parameters next;
    next.revision = u32(payload + 1);
    next.crc = u32(payload + 5 + bytes);
    if (!next.revision || crc32(payload + 5, bytes) != next.crc) return false;
    for (size_t i = 0; i < ParameterCount; ++i) {
        const uint32_t bits = u32(payload + 5 + 4 * i);
        memcpy(&next.values[i], &bits, 4);
        if (!isfinite(next.values[i]) || next.values[i] < kParameterMinimum[i] ||
            next.values[i] > kParameterMaximum[i]) return false;
    }
    if (next[FeedbackFastWindowMs] >= next[FeedbackSlowWindowMs] ||
        next[Ros2InputEnterDeg] >= next[Ros2InputExitDeg]) return false;
    next.configured = true;
    SharedSafetyGuard guard;
    current = next;
    return true;
}
} // namespace mmhand::control
