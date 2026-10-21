/** @file RampParameters.h
 * @brief 渐进快速换向参数（B0/vB/负载阈值/超时）的校验与快照。
 */
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "../generated/RampParameters.generated.h"

namespace mmhand::control {
struct RampParameters {
    uint32_t revision = 0;
    uint32_t crc = 0;
    bool configured = false;
    float values[RampParameterCount] = {};
    float operator[](RampParameter key) const { return values[key]; }
};
RampParameters readRampParameters();
bool applyRampParameterPayload(const uint8_t* payload, size_t length);
bool rampParametersConfigured();
} // namespace mmhand::control
