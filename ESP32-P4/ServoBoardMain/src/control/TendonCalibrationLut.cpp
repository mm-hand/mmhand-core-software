/**
 * @file TendonCalibrationLut.cpp
 * @brief 实现单腱和双腱标定 LUT 的校验、查表与插值。
 */
#include "TendonCalibrationLut.h"

#include <math.h>

namespace {

// LUT 的采样横坐标沿用自动标定内部行程角，而控制输入使用 URDF 角度。
// 映射在这里集中完成，避免单电机与双电机各自重复换算造成双重缩放。
bool mapUrdfToLutDeg(const JointCalibrationResult& calibration,
                     float urdfDeg,
                     float* outLutDeg)
{
    if (!outLutDeg || !isfinite(urdfDeg)) {
        return false;
    }

    float lutDeg = urdfDeg;
    if (calibration.mappingVersion >= 2) {
        const float urdfSpan = calibration.angleMax - calibration.angleMin;
        if (!isfinite(urdfSpan) || urdfSpan <= 0.0f ||
            !isfinite(calibration.totalDeg) || calibration.totalDeg <= 0.0f) {
            return false;
        }
        lutDeg = calibration.startDeg +
            ((urdfDeg - calibration.angleMin) / urdfSpan) * calibration.totalDeg;
    }
    *outLutDeg = lutDeg;
    return isfinite(lutDeg);
}

bool calibrationHasUsableTendonLut(const JointCalibrationResult& calibration,
                                   uint8_t actuatorIndex)
{
    return actuatorIndex < TENDON_LUT_ACTUATOR_COUNT &&
        calibration.success && calibration.scaled &&
        calibration.tendonLutValid != 0 &&
        calibration.tendonLutCount >= 3 &&
        calibration.tendonLutCount <= TENDON_LUT_MAX_SAMPLES &&
        isfinite(calibration.tendonLutAngleStartDeg) &&
        isfinite(calibration.tendonLutAngleStepDeg) &&
        calibration.tendonLutAngleStepDeg > 0.0f;
}

} // namespace

bool tendonCalibrationMotorAbsRange(const JointCalibrationResult& calibration,
                                    uint8_t actuatorIndex,
                                    int32_t* outMinAbs,
                                    int32_t* outMaxAbs)
{
    if (!outMinAbs || !outMaxAbs ||
        !calibrationHasUsableTendonLut(calibration, actuatorIndex)) {
        return false;
    }

    int32_t minAbs = calibration.tendonLutMotorAbs[actuatorIndex][0];
    int32_t maxAbs = minAbs;
    for (uint8_t i = 1; i < calibration.tendonLutCount; i++) {
        const int32_t value = calibration.tendonLutMotorAbs[actuatorIndex][i];
        if (value < minAbs) minAbs = value;
        if (value > maxAbs) maxAbs = value;
    }
    *outMinAbs = minAbs;
    *outMaxAbs = maxAbs;
    return true;
}

bool interpolateTendonCalibrationMotorAbs(const JointCalibrationResult& calibration,
                                          uint8_t actuatorIndex,
                                          float urdfDeg,
                                          float* outMotorAbs)
{
    if (!outMotorAbs ||
        !calibrationHasUsableTendonLut(calibration, actuatorIndex)) {
        return false;
    }

    float lutDeg = 0.0f;
    if (!mapUrdfToLutDeg(calibration, urdfDeg, &lutDeg)) {
        return false;
    }

    const float firstDeg = calibration.tendonLutAngleStartDeg;
    const float stepDeg = calibration.tendonLutAngleStepDeg;
    if (lutDeg <= firstDeg) {
        *outMotorAbs = (float)calibration.tendonLutMotorAbs[actuatorIndex][0];
        return true;
    }

    const uint8_t lastIndex = calibration.tendonLutCount - 1;
    const float lastDeg = firstDeg + stepDeg * (float)lastIndex;
    if (lutDeg >= lastDeg) {
        *outMotorAbs =
            (float)calibration.tendonLutMotorAbs[actuatorIndex][lastIndex];
        return true;
    }

    const float position = (lutDeg - firstDeg) / stepDeg;
    uint8_t lowerIndex = (uint8_t)floorf(position);
    if (lowerIndex >= lastIndex) {
        lowerIndex = lastIndex - 1;
    }
    const float t = position - (float)lowerIndex;
    const float lower =
        (float)calibration.tendonLutMotorAbs[actuatorIndex][lowerIndex];
    const float upper =
        (float)calibration.tendonLutMotorAbs[actuatorIndex][lowerIndex + 1];
    *outMotorAbs = lower + (upper - lower) * t;
    return true;
}
