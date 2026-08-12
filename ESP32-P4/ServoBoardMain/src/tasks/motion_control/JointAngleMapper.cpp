/**
 * @file JointAngleMapper.cpp
 * @brief 实现 14 bit 环形磁编码器与标定后关节角之间的连续映射。
 */
#include "JointAngleMapper.h"

#include "../../modules/calibration/CalibrationRuntimeStore.h"

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

    float minDeg = 0.0f;
    float maxDeg = 0.0f;
    // 有效 Auto 使用动态 Current Min/URDF Max；未标定仅给 UI/诊断提供静态 URDF 范围。
    const JointCalibrationResult& calibration =
        g_jointCalibResult[jointIndex];
    if (calibration.success &&
        calibration.mappingVersion == kManualScaledUrdfFeedbackMappingVersion) {
        minDeg = calibration.angleMin;
        maxDeg = calibration.angleMax;
    } else {
        minDeg = kUrdfJointLimits[jointIndex].minDeg;
        maxDeg = kUrdfJointLimits[jointIndex].maxDeg;
    }

    const p4::safety::BoundaryClampDecision decision =
        p4::safety::clampFloatToBoundary(targetDeg, minDeg, maxDeg);
    return decision.valid ? decision.applied : 0.0f;
}

} // namespace p4::control
