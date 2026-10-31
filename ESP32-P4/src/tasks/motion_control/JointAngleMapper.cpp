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

float clampJointTargetDeg(float targetDeg, uint8_t jointIndex,
                          const JointCalibrationResult& calibration)
{
    if (!isfinite(targetDeg) || jointIndex >= ENCODER_TOTAL_NUM) {
        return 0.0f;
    }

    // V5: 目标角钳到当前标定端点，等价于钳 Manual counts 窗口。
    // 标定结果无效时退回标称 URDF 端点，控制入口仍会拒绝无效关节。
    float minDeg = kUrdfJointLimits[jointIndex].minDeg;
    float maxDeg = kUrdfJointLimits[jointIndex].maxDeg;
    if (calibration.mappingVersion == kManualScaledUrdfFeedbackMappingVersion &&
        isfinite(calibration.angleMin) && isfinite(calibration.angleMax) &&
        calibration.angleMax > calibration.angleMin) {
        minDeg = calibration.angleMin;
        maxDeg = calibration.angleMax;
    }

    const p4::safety::BoundaryClampDecision decision =
        p4::safety::clampFloatToBoundary(targetDeg, minDeg, maxDeg);
    return decision.valid ? decision.applied : 0.0f;
}

} // namespace p4::control
