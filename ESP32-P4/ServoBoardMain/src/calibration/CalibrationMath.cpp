#include "CalibrationMath.h"

#include <math.h>
#include <string.h>

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
