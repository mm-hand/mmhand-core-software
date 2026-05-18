#ifndef CALIBRATION_MATH_H
#define CALIBRATION_MATH_H

#include "CalibrationTask.h"

static const int32_t kCalibrationEncoderModulo = 16384;
static const int32_t kCalibrationEncoderHalfTurn = kCalibrationEncoderModulo / 2;

static const float kRange100StartDeg = -5.0f;
static const float kRange100TightDeg = 90.0f;
static const float kRange100TotalDeg = kRange100TightDeg - kRange100StartDeg;
static const float kRange100ZeroFraction = (0.0f - kRange100StartDeg) / kRange100TotalDeg;
static const float kRange100ControlMaxDeg = 83.98f;
static const float kRange100TopGuardDeg = kRange100TightDeg - kRange100ControlMaxDeg;

int32_t calibrationOrientEncoderRaw(uint16_t rawValue, int8_t direction);
int32_t calibrationWrapEncoderCount(int32_t value);
int32_t calibrationForwardEncoderDelta(int32_t from, int32_t to);
int32_t calibrationWrapEncoderDelta(int32_t target, int32_t actual);
float calibrationEncoderCountToDeg(int32_t count);
int32_t calibrationDegToEncoderCount(float deg);

void applyRange100CalibrationConfig(JointCalibrationConfig* cfg);

bool buildRange100CalibrationResult(uint8_t jointIndex,
                                    int32_t startEncoder,
                                    int32_t tightEncoder,
                                    int32_t startServoAbs,
                                    int32_t tightServoAbs,
                                    JointCalibrationResult* out);

#endif // CALIBRATION_MATH_H
