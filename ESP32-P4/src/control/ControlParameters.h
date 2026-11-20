/** @file ControlParameters.h
 * @brief 声明控制参数快照及其校验、发布接口。
 */
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "../config/topology/HardwareMap.h"
#include "../generated/ControlParameters.generated.h"

namespace mmhand::control {
struct Parameters {
    uint32_t revision = 0;
    uint32_t crc = 0;
    bool configured = false;
    float values[ParameterCount] = {};
    float operator[](Parameter key) const { return values[key]; }
};
// 双腱关节（AA/CMC）两侧同档；单腱沿用 release/tight 两档。
inline void lutMarginsForJoint(uint8_t joint, const Parameters& p, int32_t* release, int32_t* tight) {
    const bool dual = getJointActuatorCount(joint) > 1;
    *tight = dual ? kLutMarginCountsDual : (int32_t)p[OutputLutTightMarginCounts];
    *release = dual ? *tight : (int32_t)p[OutputLutReleaseMarginCounts];
}
Parameters readParameters();
bool applyParameterPayload(const uint8_t* payload, size_t length);
bool parametersConfigured();
} // namespace mmhand::control
