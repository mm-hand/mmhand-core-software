/**
 * @file TendonCalibrationLut.cpp
 * @brief 实现单腱和双腱标定 LUT 的校验、查表与插值。
 */
#include "TendonCalibrationLut.h"

#include "../../generated/UrdfJointLimits.generated.h"
#include "../../config/topology/HardwareMap.h"

#include <math.h>

namespace {

bool calibrationHasUsableTendonLut(const JointCalibrationResult& calibration,
                                   uint8_t actuatorIndex)
{
    return actuatorIndex < TENDON_LUT_ACTUATOR_COUNT &&
        calibration.success && calibration.scaled &&
        calibration.tendonLutValid != 0 &&
        // 2 点即主机拟合直线的首末端点；P4 直接做线性插值，不再展开 24 点。
        calibration.tendonLutCount >= 2 &&
        calibration.tendonLutCount <= TENDON_LUT_MAX_SAMPLES &&
        isfinite(calibration.tendonLutAngleStartDeg) &&
        isfinite(calibration.tendonLutAngleStepDeg) &&
        calibration.tendonLutAngleStepDeg > 0.0f;
}

// 新 LUT 使用固定 5° 标称步长，最后一个样本可能是不足 5° 的 URDF Max。
// V4 中 angleMax 恒等于生成的 URDF Max，因此末段按同一 URDF 上限解释。
float tendonLutSampleAngleDeg(const JointCalibrationResult& calibration,
                              uint8_t sampleIndex)
{
    const float nominalDeg = calibration.tendonLutAngleStartDeg +
        calibration.tendonLutAngleStepDeg * (float)sampleIndex;
    const uint8_t lastIndex = calibration.tendonLutCount - 1U;
    if (sampleIndex == lastIndex &&
        isfinite(calibration.angleMax) &&
        calibration.angleMax > calibration.tendonLutAngleStartDeg &&
        calibration.angleMax < nominalDeg) {
        return calibration.angleMax;
    }
    return nominalDeg;
}

} // namespace

bool tendonCalibrationContainsAngle(const JointCalibrationResult& calibration,
                                    float urdfDeg)
{
    if (!isfinite(urdfDeg) || !calibrationHasUsableTendonLut(calibration, 0)) {
        return false;
    }
    const float firstDeg = tendonLutSampleAngleDeg(calibration, 0);
    const float lastDeg = tendonLutSampleAngleDeg(
        calibration, calibration.tendonLutCount - 1U);
    return urdfDeg >= firstDeg && urdfDeg <= lastDeg;
}

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

bool tendonCalibrationMotorAbsRangeWithMargins(
    const JointCalibrationResult& calibration,
    uint8_t actuatorIndex,
    uint8_t motorChannel,
    int32_t releaseMarginCounts,
    int32_t tightMarginCounts,
    int32_t* outMinAbs,
    int32_t* outMaxAbs)
{
    int32_t lutMinAbs = 0;
    int32_t lutMaxAbs = 0;
    if (!outMinAbs || !outMaxAbs || releaseMarginCounts < 0 ||
        tightMarginCounts < 0 ||
        !tendonCalibrationMotorAbsRange(calibration,
                                        actuatorIndex,
                                        &lutMinAbs,
                                        &lutMaxAbs)) {
        return false;
    }

    // shortenSign 决定数值较大的一端是否为紧端，不能默认 ABS 增大必然收绳。
    const int8_t shortenSign = getMotorTendonShortenAbsSign(motorChannel);
    if (shortenSign == 0) {
        return false;
    }
    const int32_t releaseAbs = shortenSign > 0 ? lutMinAbs : lutMaxAbs;
    const int32_t tightAbs = shortenSign > 0 ? lutMaxAbs : lutMinAbs;
    const int32_t releaseWithMargin =
        motorApplyTendonDelta(motorChannel, releaseAbs, -releaseMarginCounts);
    const int32_t tightWithMargin =
        motorApplyTendonDelta(motorChannel, tightAbs, tightMarginCounts);

    *outMinAbs = releaseWithMargin < tightWithMargin
        ? releaseWithMargin : tightWithMargin;
    *outMaxAbs = releaseWithMargin > tightWithMargin
        ? releaseWithMargin : tightWithMargin;
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

    if (!isfinite(urdfDeg) ||
        calibration.mappingVersion != kManualScaledUrdfFeedbackMappingVersion) {
        return false;
    }

    const float firstDeg = calibration.tendonLutAngleStartDeg;
    const uint8_t lastIndex = calibration.tendonLutCount - 1;
    const float lastDeg = tendonLutSampleAngleDeg(calibration, lastIndex);
    if (!isfinite(lastDeg) || lastDeg <= firstDeg) {
        return false;
    }
    uint8_t upperIndex = 1U;
    if (urdfDeg >= lastDeg) {
        upperIndex = lastIndex;
    } else if (urdfDeg > firstDeg) {
        while (upperIndex < lastIndex &&
               urdfDeg > tendonLutSampleAngleDeg(calibration, upperIndex)) {
            ++upperIndex;
        }
    }
    const uint8_t lowerIndex = upperIndex - 1U;
    const float lowerDeg = tendonLutSampleAngleDeg(calibration, lowerIndex);
    const float upperDeg = tendonLutSampleAngleDeg(calibration, upperIndex);
    if (!isfinite(lowerDeg) || !isfinite(upperDeg) || upperDeg <= lowerDeg) {
        return false;
    }
    // t 允许小于 0 或大于 1，使反馈角越过端点后仍沿端部斜率连续变化，
    // 不会在紧绳端或松绳端被 LUT 固定为常数。
    const float t = (urdfDeg - lowerDeg) / (upperDeg - lowerDeg);
    const float lower =
        (float)calibration.tendonLutMotorAbs[actuatorIndex][lowerIndex];
    const float upper =
        (float)calibration.tendonLutMotorAbs[actuatorIndex][upperIndex];
    *outMotorAbs = lower + (upper - lower) * t;
    return true;
}
