/**
 * @file JointAngleMapper.cpp
 * @brief 实现 14 bit 环形磁编码器与标定后关节角之间的连续映射。
 */
#include "JointAngleMapper.h"

#include "../../system/calibration/CalibrationRuntimeStore.h"

#include <math.h>

#include "../../config/ControlConfig.h"
#include "../../generated/UrdfJointLimits.generated.h"
#include "../../safety/policies/ControlBoundarySafetyPolicy.h"

namespace p4::control {

using namespace mmhand::config::control;

int32_t degToEncoderCount(float deg)
{
    const float scaled =
        deg * static_cast<float>(kEncoderModulo) / 360.0f;
    return static_cast<int32_t>(
        scaled + (scaled >= 0.0f ? 0.5f : -0.5f));
}

float clampJointTargetDeg(float targetDeg, uint8_t jointIndex)
{
    if (!isfinite(targetDeg) || jointIndex >= ENCODER_TOTAL_NUM) {
        return 0.0f;
    }

    // V4 的控制目标始终受静态 URDF Min/Max 约束。Auto 标定只负责
    // 证明该关节可控并生成 Tendon LUT，不能再缩短或平移目标角区间。
    const float minDeg = kUrdfJointLimits[jointIndex].minDeg;
    const float maxDeg = kUrdfJointLimits[jointIndex].maxDeg;

    const p4::safety::BoundaryClampDecision decision =
        p4::safety::clampFloatToBoundary(targetDeg, minDeg, maxDeg);
    return decision.valid ? decision.applied : 0.0f;
}

} // namespace p4::control
