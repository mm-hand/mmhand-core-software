/** @file ControlParameters.h
 * @brief 声明控制参数快照及其校验、发布接口。
 */
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "../generated/ControlParameters.generated.h"

namespace mmhand::control {
struct Parameters {
    uint32_t revision = 0;
    uint32_t crc = 0;
    bool configured = false;
    float values[ParameterCount] = {};
    float operator[](Parameter key) const { return values[key]; }
};
Parameters readParameters();
bool applyParameterPayload(const uint8_t* payload, size_t length);
bool parametersConfigured();
} // namespace mmhand::control
