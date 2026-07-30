/**
 * @file JointAngleMapper.cpp
 * @brief 实现 14 bit 环形磁编码器与标定后关节角之间的连续映射。
 */
#include "JointAngleMapper.h"

#include <math.h>

#include "../config/ControlConfig.h"
#include "../generated/UrdfJointLimits.generated.h"

namespace p4::control {

using namespace mmhand::config::control;

int32_t orientEncoderRaw(uint16_t rawValue, int8_t direction)
{
    int32_t oriented = static_cast<int32_t>(rawValue) & (kEncoderModulo - 1);
    if (direction < 0) {
        oriented = (kEncoderModulo - oriented) & (kEncoderModulo - 1);
    }
    return oriented;
}

int32_t wrapEncoderCount(int32_t value)
{
    value %= kEncoderModulo;
    if (value < 0) {
        value += kEncoderModulo;
    }
    return value;
}

int32_t forwardEncoderDelta(int32_t from, int32_t to)
{
    return wrapEncoderCount(to - from);
}

float encoderCountToDeg(int32_t encoderCount)
{
    return static_cast<float>(encoderCount) * 360.0f /
           static_cast<float>(kEncoderModulo);
}

int32_t degToEncoderCount(float deg)
{
    const float scaled =
        deg * static_cast<float>(kEncoderModulo) / 360.0f;
    return static_cast<int32_t>(
        scaled + (scaled >= 0.0f ? 0.5f : -0.5f));
}

float mapCalibratedEncoderToDeg(const JointCalibrationResult& calibration,
                                int32_t orientedRaw)
{
    if (calibration.scaled &&
        calibration.encoderSpan > 0 &&
        calibration.totalDeg > 0.0f) {
        int32_t delta =
            forwardEncoderDelta(calibration.startEncoder, orientedRaw);
        if (delta > calibration.encoderSpan + kEncoderMarginCounts) {
            delta -= kEncoderModulo;
        }
        return (static_cast<float>(delta) /
                static_cast<float>(calibration.encoderSpan)) *
                   calibration.totalDeg +
               calibration.startDeg;
    }

    const int32_t offset = calibration.offset;
    int32_t delta = forwardEncoderDelta(offset, orientedRaw);
    int32_t hardMaxFromOffset =
        forwardEncoderDelta(offset, calibration.encoderMax);
    if (hardMaxFromOffset <= 0) {
        hardMaxFromOffset = degToEncoderCount(calibration.angleMax);
    }
    if (delta > hardMaxFromOffset + kEncoderMarginCounts) {
        delta -= kEncoderModulo;
    }
    return encoderCountToDeg(delta);
}

float clampJointTargetDeg(float targetDeg, uint8_t jointIndex)
{
    if (!isfinite(targetDeg) || jointIndex >= ENCODER_TOTAL_NUM) {
        return 0.0f;
    }

    float minDeg = 0.0f;
    float maxDeg = 0.0f;
    const JointCalibrationResult& calibration =
        g_jointCalibResult[jointIndex];
    if (calibration.success) {
        minDeg = calibration.mappingVersion >= 2
                     ? calibration.angleMin
                     : 0.0f;
        maxDeg = calibration.angleMax;
    } else {
        minDeg = kUrdfJointLimits[jointIndex].minDeg;
        maxDeg = kUrdfJointLimits[jointIndex].maxDeg;
    }

    if (!isfinite(minDeg) || !isfinite(maxDeg) || minDeg >= maxDeg) {
        return 0.0f;
    }
    if (targetDeg < minDeg) {
        return minDeg;
    }
    if (targetDeg > maxDeg) {
        return maxDeg;
    }
    return targetDeg;
}

} // namespace p4::control
