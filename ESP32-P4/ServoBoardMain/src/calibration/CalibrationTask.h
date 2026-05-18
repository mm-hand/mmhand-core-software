#ifndef CALIBRATION_TASK_H
#define CALIBRATION_TASK_H

#include <Arduino.h>
#include "../shared/TaskSharedData.h"

// Calibration service module. The calibration lifecycle is owned by the state
// machine; this file only exposes calibration data and helper algorithms.
#define CALIB_STATUS_IDLE      0
#define CALIB_STATUS_RUNNING   1
#define CALIB_STATUS_SUCCESS   2
#define CALIB_STATUS_FAILED    3

// Per-joint calibration config:
// - angleScope/bottomReserved/topReserved/angleReserved are all degrees.
// - Manual RANGE100 start/tight points overwrite the usable control range.
// - Load and motion parameters are used by the optional single-joint search.
struct JointCalibrationConfig {
    float angleScope;
    float bottomReserved;
    float topReserved;
    float angleReserved;

    int16_t loadThreshold;
    int16_t tightenStep;
    uint16_t tightenSpeed;
    uint8_t tightenAcc;

    uint32_t settleMs;
    uint32_t maxSearchMs;
};

// Per-joint calibration result:
// - encoderMin/encoderMax/offset are encoder counts.
// - angleMax is the usable control range in degrees.
struct JointCalibrationResult {
    bool success;
    int32_t encoderMin;
    int32_t encoderMax;
    int32_t offset;
    float angleMax;

    // RANGE100 manual/boot calibration fields:
    // startEncoder maps to startDeg, tightEncoder maps to tightDeg, and
    // zeroEncoder is the 0 deg point between them.
    int32_t startEncoder;
    int32_t tightEncoder;
    int32_t zeroEncoder;
    int32_t encoderSpan;
    int32_t startServoAbs;
    int32_t tightServoAbs;
    int32_t zeroServoAbs;
    float startDeg;
    float tightDeg;
    float totalDeg;
    bool scaled;
};

extern JointCalibrationConfig g_jointCalibConfig[ENCODER_TOTAL_NUM];
extern JointCalibrationResult g_jointCalibResult[ENCODER_TOTAL_NUM];

// Default encoder direction table, runtime encoder direction table, and
// fallback offset table. Automatic calibration may update the runtime table.
extern const int8_t kDefaultEncoderDirection[ENCODER_TOTAL_NUM];
extern int8_t g_encoderDirection[ENCODER_TOTAL_NUM];
extern int32_t g_encoderOffsetManual[ENCODER_TOTAL_NUM];
void resetEncoderDirectionsToDefault(void);
bool setRuntimeEncoderDirection(uint8_t jointIndex, int8_t direction);

// Dual-servo constraint used by calibration helpers.
struct JointDualServoConstraint {
    uint8_t enabled;
    uint8_t jointIndex;
    uint8_t primaryBusIndex;
    uint8_t primaryServoID;
    uint8_t secondaryBusIndex;
    uint8_t secondaryServoID;
    int32_t secondaryOffset;
    int32_t tensionBias;
};

extern JointDualServoConstraint g_joint16DualServoConstraint;

bool getJointDualServoConstraint(uint8_t jointIndex, JointDualServoConstraint* out);
bool computeSecondaryServoTargetForJoint(uint8_t jointIndex, int32_t primaryTarget, int16_t* outSecondaryTarget);

void initDefaultCalibrationConfig(JointCalibrationConfig* cfg, uint8_t count);
void initManualCalibrationForTest(void);

// Run one-joint calibration search. Not called automatically by the state machine.
bool runSingleJointCalibration(TaskSharedData_t* sharedData,
                               uint8_t jointIndex,
                               const JointCalibrationConfig& cfg,
                               JointCalibrationResult* out);

bool queueJointCalibrationTarget(TaskSharedData_t* sharedData,
                                 uint8_t jointIndex,
                                 int16_t primaryTarget,
                                 const JointCalibrationConfig& cfg);

#endif // CALIBRATION_TASK_H
