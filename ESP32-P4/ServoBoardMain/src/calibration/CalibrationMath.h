#ifndef CALIBRATION_MATH_H
#define CALIBRATION_MATH_H

#include "CalibrationTask.h"

static const int32_t kCalibrationEncoderModulo = 16384;
static const int32_t kCalibrationEncoderHalfTurn = kCalibrationEncoderModulo / 2;
static const int32_t kFeedbackReadyEnterOutsideToleranceCounts = 24;



int32_t calibrationOrientEncoderRaw(uint16_t rawValue, int8_t direction);
int32_t calibrationWrapEncoderCount(int32_t value);
int32_t calibrationForwardEncoderDelta(int32_t from, int32_t to);
int32_t calibrationWrapEncoderDelta(int32_t target, int32_t actual);
float calibrationEncoderCountToDeg(int32_t count);
int32_t calibrationDegToEncoderCount(float deg);

// Result of mapping one direction-oriented magnetic encoder sample through a
// v2 calibration window. outsideCounts is signed: negative means the sample
// is below the URDF-min endpoint, positive means it is above the URDF-max
// endpoint. It must never be converted to an absolute value.
struct EncoderUrdfMapResult {
    bool valid;
    bool insideCalibratedWindow;
    bool feedbackReady;
    int32_t signedTravelCounts;
    int32_t outsideCounts;
    int32_t belowMinCounts;
    int32_t aboveMaxCounts;
    float urdfDeg;
};

// Positive travel following the configured encoder direction. Endpoints are
// raw 14-bit values and may cross zero.
int32_t calibrationDirectedEncoderDelta(uint16_t fromRaw,
                                        uint16_t toRaw,
                                        int8_t direction);

// Shortest signed displacement following the configured positive encoder
// direction. Raw endpoints may cross 0/16383.
int32_t calibrationSignedDirectedEncoderDelta(uint16_t fromRaw,
                                              uint16_t toRaw,
                                              int8_t direction);

bool applyManualScaledUrdfMappingToCalibration(
    uint8_t jointIndex,
    const ManualEncoderReference& manual,
    uint16_t autoMinRaw,
    uint16_t autoMaxRaw,
    int8_t direction,
    uint32_t manualEntryCrc,
    JointCalibrationResult* result);

// Above-max feedback remains valid but is reported separately. The runtime
// controller will later clamp commands and emit a warning without leaving
// READY solely because the upper boundary was exceeded.
bool mapManualScaledEncoderToUrdf(uint8_t jointIndex,
                                  const JointCalibrationResult& calibration,
                                  const ManualEncoderReference& manual,
                                  uint16_t currentRaw,
                                  int8_t direction,
                                  EncoderUrdfMapResult* out);

// Map a direction-oriented encoder raw value into a complete URDF degree
// interval. Inside the measured endpoint window this is a direct linear fit.
// Outside the window urdfDeg is only a signed 16384-count diagnostic
// extrapolation from the nearest endpoint; callers must not use it for PID.
bool mapEncoderWindowToUrdf(int32_t orientedRaw,
                            int32_t encoderAtUrdfMin,
                            int32_t encoderAtUrdfMax,
                            float urdfMinDeg,
                            float urdfMaxDeg,
                            EncoderUrdfMapResult* out);

// Convenience wrapper for the staged v2 JointCalibrationResult fields.
bool mapJointCalibrationEncoderToUrdf(uint8_t jointIndex,
                                      const JointCalibrationResult& calibration,
                                      int32_t orientedRaw,
                                      EncoderUrdfMapResult* out);

// Bind a measured, direction-oriented endpoint pair to the generated URDF
// interval for one joint. This converts the result to mapping v2 and stamps
// the generated config CRC. Spans at or above half a revolution are rejected
// until continuous unwrapped encoder tracking is available.
bool applyGeneratedUrdfMappingToCalibration(uint8_t jointIndex,
                                            int32_t encoderAtUrdfMin,
                                            int32_t encoderAtUrdfMax,
                                            JointCalibrationResult* result);



#endif // CALIBRATION_MATH_H
