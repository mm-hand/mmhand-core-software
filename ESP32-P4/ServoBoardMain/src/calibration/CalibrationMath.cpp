#include "CalibrationMath.h"

#include <math.h>
#include <string.h>

#include "../generated/UrdfJointLimits.generated.h"
#include "../system/FirmwareLog.h"

static_assert(kManualScaledUrdfFeedbackMappingVersion == 3,
              "Manual-scaled calibration contract must remain v3");

int32_t calibrationOrientEncoderRaw(uint16_t rawValue, int8_t direction)
{
    int32_t oriented = (int32_t)rawValue & (kCalibrationEncoderModulo - 1);
    if (direction < 0) {
        oriented = (kCalibrationEncoderModulo - oriented) & (kCalibrationEncoderModulo - 1);
    }
    return oriented;
}

int32_t calibrationWrapEncoderCount(int32_t value)
{
    value %= kCalibrationEncoderModulo;
    if (value < 0) {
        value += kCalibrationEncoderModulo;
    }
    return value;
}

int32_t calibrationForwardEncoderDelta(int32_t from, int32_t to)
{
    return calibrationWrapEncoderCount(to - from);
}

int32_t calibrationDirectedEncoderDelta(uint16_t fromRaw,
                                        uint16_t toRaw,
                                        int8_t direction)
{
    if (direction != 1 && direction != -1) {
        return -1;
    }
    const int32_t from = (int32_t)fromRaw & (kCalibrationEncoderModulo - 1);
    const int32_t to = (int32_t)toRaw & (kCalibrationEncoderModulo - 1);
    return direction > 0
        ? calibrationWrapEncoderCount(to - from)
        : calibrationWrapEncoderCount(from - to);
}

int32_t calibrationSignedDirectedEncoderDelta(uint16_t fromRaw,
                                              uint16_t toRaw,
                                              int8_t direction)
{
    const int32_t forward = calibrationDirectedEncoderDelta(
        fromRaw, toRaw, direction);
    if (forward < 0) {
        return 0;
    }
    return forward > kCalibrationEncoderHalfTurn
        ? forward - kCalibrationEncoderModulo
        : forward;
}

int32_t calibrationWrapEncoderDelta(int32_t target, int32_t actual)
{
    int32_t delta = target - actual;
    while (delta > kCalibrationEncoderHalfTurn) {
        delta -= kCalibrationEncoderModulo;
    }
    while (delta < -kCalibrationEncoderHalfTurn) {
        delta += kCalibrationEncoderModulo;
    }
    return delta;
}

float calibrationEncoderCountToDeg(int32_t count)
{
    return (float)count * 360.0f / (float)kCalibrationEncoderModulo;
}

int32_t calibrationDegToEncoderCount(float deg)
{
    const float scaled = deg * (float)kCalibrationEncoderModulo / 360.0f;
    return (int32_t)(scaled + (scaled >= 0.0f ? 0.5f : -0.5f));
}

bool mapEncoderWindowToUrdf(int32_t orientedRaw,
                            int32_t encoderAtUrdfMin,
                            int32_t encoderAtUrdfMax,
                            float urdfMinDeg,
                            float urdfMaxDeg,
                            EncoderUrdfMapResult* out)
{
    if (!out) {
        return false;
    }

    memset(out, 0, sizeof(*out));
    if (!isfinite(urdfMinDeg) || !isfinite(urdfMaxDeg) ||
        urdfMaxDeg <= urdfMinDeg) {
        return false;
    }

    const int32_t minEncoder = calibrationWrapEncoderCount(encoderAtUrdfMin);
    const int32_t maxEncoder = calibrationWrapEncoderCount(encoderAtUrdfMax);
    const int32_t span = calibrationForwardEncoderDelta(minEncoder, maxEncoder);

    // The signed-distance representation is unambiguous only below half a
    // magnetic encoder revolution. Auto calibration will enforce the same
    // limit before it writes a v2 result.
    if (span <= 0 || span >= kCalibrationEncoderHalfTurn) {
        return false;
    }

    const int32_t currentEncoder = calibrationWrapEncoderCount(orientedRaw);
    const int32_t signedTravel =
        calibrationWrapEncoderDelta(currentEncoder, minEncoder);
    const bool inside = signedTravel >= 0 && signedTravel <= span;

    int32_t outsideCounts = 0;
    float mappedDeg = 0.0f;
    if (inside) {
        const float progress = (float)signedTravel / (float)span;
        mappedDeg = urdfMinDeg + progress * (urdfMaxDeg - urdfMinDeg);
    } else if (signedTravel < 0) {
        outsideCounts = signedTravel;
        mappedDeg = urdfMinDeg + calibrationEncoderCountToDeg(outsideCounts);
    } else {
        outsideCounts = signedTravel - span;
        mappedDeg = urdfMaxDeg + calibrationEncoderCountToDeg(outsideCounts);
    }

    out->valid = true;
    out->insideCalibratedWindow = inside;
    out->feedbackReady = inside;
    out->signedTravelCounts = signedTravel;
    out->outsideCounts = outsideCounts;
    out->urdfDeg = mappedDeg;
    return true;
}

bool mapJointCalibrationEncoderToUrdf(uint8_t jointIndex,
                                      const JointCalibrationResult& calibration,
                                      int32_t orientedRaw,
                                      EncoderUrdfMapResult* out)
{
    if (jointIndex >= ENCODER_TOTAL_NUM ||
        !calibration.success ||
        calibration.mappingVersion != kUrdfFeedbackMappingVersion ||
        calibration.urdfConfigCrc != kUrdfJointLimitConfigCrc ||
        fabsf(calibration.angleMin - kUrdfJointLimits[jointIndex].minDeg) > 0.0001f ||
        fabsf(calibration.angleMax - kUrdfJointLimits[jointIndex].maxDeg) > 0.0001f) {
        if (out) {
            memset(out, 0, sizeof(*out));
        }
        return false;
    }

    return mapEncoderWindowToUrdf(orientedRaw,
                                  calibration.encoderAtUrdfMin,
                                  calibration.encoderAtUrdfMax,
                                  calibration.angleMin,
                                  calibration.angleMax,
                                  out);
}

bool applyGeneratedUrdfMappingToCalibration(uint8_t jointIndex,
                                            int32_t encoderAtUrdfMin,
                                            int32_t encoderAtUrdfMax,
                                            JointCalibrationResult* result)
{
    if (!result || jointIndex >= ENCODER_TOTAL_NUM) {
        return false;
    }

    const int32_t minEncoder = calibrationWrapEncoderCount(encoderAtUrdfMin);
    const int32_t maxEncoder = calibrationWrapEncoderCount(encoderAtUrdfMax);
    const int32_t span = calibrationForwardEncoderDelta(minEncoder, maxEncoder);
    const UrdfJointLimitDeg& limit = kUrdfJointLimits[jointIndex];
    const float urdfSpanDeg = limit.maxDeg - limit.minDeg;
    if (span <= 0 || span >= kCalibrationEncoderHalfTurn ||
        !isfinite(limit.minDeg) || !isfinite(limit.maxDeg) ||
        !isfinite(urdfSpanDeg) || urdfSpanDeg <= 0.0f) {
        return false;
    }

    result->mappingVersion = kUrdfFeedbackMappingVersion;
    result->encoderAtUrdfMin = minEncoder;
    result->encoderAtUrdfMax = maxEncoder;
    result->angleMin = limit.minDeg;
    result->angleMax = limit.maxDeg;
    result->urdfConfigCrc = kUrdfJointLimitConfigCrc;

    // Keep legacy start/tight/zero/total fields unchanged. The current
    // calibration return path and dual-tendon LUT still use that internal
    // coordinate system; runtime v2 mapping uses only the additive fields.
    result->encoderMin = minEncoder;
    result->encoderMax = maxEncoder;
    return true;
}

bool applyManualScaledUrdfMappingToCalibration(
    uint8_t jointIndex,
    const ManualEncoderReference& manual,
    uint16_t autoMinRaw,
    uint16_t autoMaxRaw,
    int8_t direction,
    uint32_t manualEntryCrc,
    JointCalibrationResult* result)
{
    if (!result || jointIndex >= ENCODER_TOTAL_NUM || !manual.valid ||
        manual.minRaw >= kCalibrationEncoderModulo ||
        manual.maxRaw >= kCalibrationEncoderModulo ||
        autoMinRaw >= kCalibrationEncoderModulo ||
        autoMaxRaw >= kCalibrationEncoderModulo) {
        return false;
    }

    const int32_t manualSpan = calibrationDirectedEncoderDelta(
        manual.minRaw, manual.maxRaw, direction);
    const int32_t autoSpan = calibrationDirectedEncoderDelta(
        autoMinRaw, autoMaxRaw, direction);
    if (manualSpan <= 0 || manualSpan >= kCalibrationEncoderModulo ||
        autoSpan <= 0 || autoSpan >= kCalibrationEncoderModulo) {
        return false;
    }

    const UrdfJointLimitDeg& limit = kUrdfJointLimits[jointIndex];
    const float urdfSpanDeg = limit.maxDeg - limit.minDeg;
    const float degPerCount = urdfSpanDeg / (float)manualSpan;
    const float currentMaxDeg = limit.maxDeg;
    const float currentMinDeg = currentMaxDeg - (float)autoSpan * degPerCount;
    if (!isfinite(urdfSpanDeg) || urdfSpanDeg <= 0.0f ||
        !isfinite(degPerCount) || degPerCount <= 0.0f ||
        !isfinite(currentMinDeg) || !isfinite(currentMaxDeg)) {
        return false;
    }

    result->mappingVersion = kManualScaledUrdfFeedbackMappingVersion;
    result->autoMinRaw = autoMinRaw;
    result->autoMaxRaw = autoMaxRaw;
    result->manualSpanCounts = manualSpan;
    result->autoSpanCounts = autoSpan;
    result->manualReferenceCrc = manualEntryCrc;
    result->currentMinDeg = currentMinDeg;
    result->currentMaxDeg = currentMaxDeg;
    result->angleMin = currentMinDeg;
    result->angleMax = currentMaxDeg;
    result->urdfConfigCrc = kManualScaledUrdfConfigCrc;
    const float autoManualRatio = (float)autoSpan / (float)manualSpan;
    FW_LOG_INFO_PRINTF(
        "[CALIB MAP V3] J%02u dir=%d manual=%ld(%u->%u) "
        "auto=%ld(%u->%u) ratio=%.6f "
        "urdf=[%.3f,%.3f] current=[%.3f,%.3f] crc=0x%08lX\r\n",
        (unsigned)jointIndex,
        (int)direction,
        (long)manualSpan,
        (unsigned)manual.minRaw,
        (unsigned)manual.maxRaw,
        (long)autoSpan,
        (unsigned)autoMinRaw,
        (unsigned)autoMaxRaw,
        autoManualRatio,
        limit.minDeg,
        limit.maxDeg,
        currentMinDeg,
        currentMaxDeg,
        (unsigned long)manualEntryCrc);
    if (autoManualRatio > 1.0f) {
        FW_LOG_WARN_PRINTF(
            "[CALIB MAP V3] WARN J%02u auto/manual ratio %.6f > 1; "
            "result kept for diagnostics and control\r\n",
            (unsigned)jointIndex,
            autoManualRatio);
    }
    return true;
}

static int32_t signedDirectedCoordinateFromWindowStart(
    uint16_t windowStartRaw,
    uint16_t currentRaw,
    int8_t direction,
    int32_t windowSpan)
{
    const int32_t forward = calibrationDirectedEncoderDelta(
        windowStartRaw, currentRaw, direction);
    if (forward <= windowSpan) {
        return forward;
    }

    const int32_t aboveWindow = forward - windowSpan;
    const int32_t belowWindow = kCalibrationEncoderModulo - forward;
    return belowWindow < aboveWindow ? -belowWindow : forward;
}

bool mapManualScaledEncoderToUrdf(uint8_t jointIndex,
                                  const JointCalibrationResult& calibration,
                                  const ManualEncoderReference& manual,
                                  uint16_t currentRaw,
                                  int8_t direction,
                                  EncoderUrdfMapResult* out)
{
    if (!out) {
        return false;
    }
    memset(out, 0, sizeof(*out));
    if (jointIndex >= ENCODER_TOTAL_NUM ||
        calibration.mappingVersion != kManualScaledUrdfFeedbackMappingVersion ||
        calibration.urdfConfigCrc != kManualScaledUrdfConfigCrc ||
        !manual.valid ||
        manual.minRaw >= kCalibrationEncoderModulo ||
        manual.maxRaw >= kCalibrationEncoderModulo ||
        currentRaw >= kCalibrationEncoderModulo ||
        calibration.autoMinRaw >= kCalibrationEncoderModulo ||
        calibration.autoMaxRaw >= kCalibrationEncoderModulo ||
        calibration.manualSpanCounts <= 0 ||
        calibration.manualSpanCounts >= kCalibrationEncoderModulo ||
        calibration.autoSpanCounts <= 0 ||
        calibration.autoSpanCounts >= kCalibrationEncoderModulo ||
        !isfinite(calibration.currentMinDeg) ||
        !isfinite(calibration.currentMaxDeg) ||
        calibration.currentMaxDeg <= calibration.currentMinDeg) {
        return false;
    }

    const int32_t directedTravel = calibrationDirectedEncoderDelta(
        calibration.autoMinRaw, currentRaw, direction);
    if (directedTravel < 0) {
        return false;
    }

    int32_t signedTravel = directedTravel;
    int32_t belowMinCounts = 0;
    int32_t aboveMaxCounts = 0;
    if (directedTravel > calibration.autoSpanCounts) {
        const int32_t distanceAboveMax =
            directedTravel - calibration.autoSpanCounts;
        const int32_t distanceBelowMin =
            kCalibrationEncoderModulo - directedTravel;
        if (distanceBelowMin < distanceAboveMax) {
            belowMinCounts = distanceBelowMin;
            signedTravel = -distanceBelowMin;
        } else {
            aboveMaxCounts = distanceAboveMax;
        }
    }

    const float urdfSpanDeg =
        kUrdfJointLimits[jointIndex].maxDeg -
        kUrdfJointLimits[jointIndex].minDeg;
    const float manualDegPerCount =
        urdfSpanDeg / (float)calibration.manualSpanCounts;
    // Express both samples in one signed coordinate system. A raw sample just
    // below Manual Min must be -N, not 16384-N. The old unsigned-forward
    // representation made J13's -100 count Auto Min look like +16284 counts
    // and shifted the live Current angle by almost one full revolution.
    const int32_t manualCoordinate =
        signedDirectedCoordinateFromWindowStart(
            manual.minRaw, currentRaw, direction,
            calibration.manualSpanCounts);
    const int32_t autoMinCoordinate =
        signedDirectedCoordinateFromWindowStart(
            manual.minRaw, calibration.autoMinRaw, direction,
            calibration.manualSpanCounts);

    float mappedDeg = 0.0f;
    if (belowMinCounts == 0 && aboveMaxCounts == 0) {
        // The calibrated Auto window is authoritative: its endpoints must map
        // exactly to currentMin/currentMax even when Auto extends slightly
        // outside the manually measured window.
        mappedDeg = calibration.currentMinDeg +
            (float)signedTravel * manualDegPerCount;
    } else if (manualCoordinate >= 0 &&
               manualCoordinate <= calibration.manualSpanCounts) {
        mappedDeg = calibration.currentMinDeg +
            (float)(manualCoordinate - autoMinCoordinate) *
                manualDegPerCount;
    } else if (manualCoordinate < 0) {
        const float manualMinMapped = calibration.currentMinDeg -
            (float)autoMinCoordinate * manualDegPerCount;
        mappedDeg = manualMinMapped +
            calibrationEncoderCountToDeg(manualCoordinate);
    } else {
        const float manualMaxMapped = calibration.currentMinDeg +
            (float)(calibration.manualSpanCounts - autoMinCoordinate) *
                manualDegPerCount;
        mappedDeg = manualMaxMapped + calibrationEncoderCountToDeg(
            manualCoordinate - calibration.manualSpanCounts);
    }
    if (!isfinite(mappedDeg)) {
        return false;
    }

    out->valid = true;
    out->insideCalibratedWindow =
        belowMinCounts == 0 && aboveMaxCounts == 0;
    out->feedbackReady = belowMinCounts == 0;
    out->signedTravelCounts = signedTravel;
    out->outsideCounts = belowMinCounts > 0
        ? -belowMinCounts : aboveMaxCounts;
    out->belowMinCounts = belowMinCounts;
    out->aboveMaxCounts = aboveMaxCounts;
    out->urdfDeg = mappedDeg;
    return true;
}

void applyRange100CalibrationConfig(JointCalibrationConfig* cfg)
{
    if (!cfg) {
        return;
    }

    cfg->angleScope = kRange100TotalDeg;
    cfg->bottomReserved = -kRange100StartDeg;
    cfg->topReserved = kRange100TopGuardDeg;
    cfg->angleReserved = -kRange100StartDeg;
}

static int32_t roundToInt32(float value)
{
    return (int32_t)(value + (value >= 0.0f ? 0.5f : -0.5f));
}

bool buildRange100CalibrationResult(uint8_t jointIndex,
                                    int32_t startEncoder,
                                    int32_t tightEncoder,
                                    int32_t startServoAbs,
                                    int32_t tightServoAbs,
                                    JointCalibrationResult* out)
{
    (void)jointIndex;
    if (!out) {
        return false;
    }

    memset(out, 0, sizeof(*out));

    const int32_t span = calibrationForwardEncoderDelta(startEncoder, tightEncoder);
    if (span <= 0 || kRange100TotalDeg <= 0.0f ||
        kRange100ZeroFraction < 0.0f || kRange100ZeroFraction > 1.0f) {
        return false;
    }

    const int32_t zeroOffsetCount = roundToInt32((float)span * kRange100ZeroFraction);
    const int32_t zeroEncoder = calibrationWrapEncoderCount(startEncoder + zeroOffsetCount);
    const int32_t servoTravel = tightServoAbs - startServoAbs;
    const int32_t zeroServoAbs =
        startServoAbs + roundToInt32((float)servoTravel * kRange100ZeroFraction);

    out->success = true;
    out->encoderMin = startEncoder;
    out->encoderMax = tightEncoder;
    out->offset = zeroEncoder;
    out->angleMax = kRange100ControlMaxDeg;
    out->mappingVersion = 1;
    out->startEncoder = startEncoder;
    out->tightEncoder = tightEncoder;
    out->zeroEncoder = zeroEncoder;
    out->encoderSpan = span;
    out->startServoAbs = startServoAbs;
    out->tightServoAbs = tightServoAbs;
    out->zeroServoAbs = zeroServoAbs;
    out->startDeg = kRange100StartDeg;
    out->tightDeg = kRange100TightDeg;
    out->totalDeg = kRange100TotalDeg;
    out->scaled = true;
    return true;
}
