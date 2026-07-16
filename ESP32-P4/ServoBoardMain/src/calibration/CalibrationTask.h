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
#define CALIB_STATUS_WARN      4

static inline bool calibrationStatusHasUsableResult(uint8_t status)
{
    return status == CALIB_STATUS_SUCCESS || status == CALIB_STATUS_WARN;
}

// Auto Min may legitimately differ slightly from the operator-entered
// Manual Min. A larger positive offset indicates an assembly warning, but the
// calibration remains usable and must not enter the fault path.
static const int32_t kCalibrationAutoMinAssemblyWarnCounts = 350;
static const int32_t kCalibrationAutoMaxShortfallWarnCounts = 300;
static const int32_t kCalibrationAutoSpanWarnMinCounts = 2300;
static const int32_t kCalibrationAutoSpanWarnMaxCounts = 6300;

static const uint8_t CALIB_WARN_NONE = 0x00;
static const uint8_t CALIB_WARN_AUTO_MIN_OFFSET = 0x01;
static const uint8_t CALIB_WARN_AUTO_MAX_SHORTFALL = 0x02;
static const uint8_t CALIB_WARN_AUTO_SPAN_RANGE = 0x04;

static constexpr uint8_t calibrationAssemblyWarningReasonForMetrics(
    int32_t autoMinOffsetCounts,
    int32_t autoMaxShortfallCounts,
    int32_t autoSpanCounts)
{
    return
        (autoMinOffsetCounts >= kCalibrationAutoMinAssemblyWarnCounts
             ? CALIB_WARN_AUTO_MIN_OFFSET : CALIB_WARN_NONE) |
        (autoMaxShortfallCounts >= kCalibrationAutoMaxShortfallWarnCounts
             ? CALIB_WARN_AUTO_MAX_SHORTFALL : CALIB_WARN_NONE) |
        (autoSpanCounts < kCalibrationAutoSpanWarnMinCounts ||
         autoSpanCounts > kCalibrationAutoSpanWarnMaxCounts
             ? CALIB_WARN_AUTO_SPAN_RANGE : CALIB_WARN_NONE);
}

static_assert(calibrationAssemblyWarningReasonForMetrics(349, 299, 2300) ==
                  CALIB_WARN_NONE,
              "Assembly warning lower boundaries changed");
static_assert(calibrationAssemblyWarningReasonForMetrics(350, 299, 2300) ==
                  CALIB_WARN_AUTO_MIN_OFFSET,
              "Auto Min warning threshold must be inclusive");
static_assert(calibrationAssemblyWarningReasonForMetrics(349, 300, 6300) ==
                  CALIB_WARN_AUTO_MAX_SHORTFALL,
              "Auto Max warning threshold must be inclusive");
static_assert(calibrationAssemblyWarningReasonForMetrics(349, 299, 2299) ==
                  CALIB_WARN_AUTO_SPAN_RANGE &&
              calibrationAssemblyWarningReasonForMetrics(349, 299, 6301) ==
                  CALIB_WARN_AUTO_SPAN_RANGE,
              "Auto Span warning range must remain 2300..6300 inclusive");

#define DUAL_TENDON_LUT_ACTUATOR_COUNT 2
#define DUAL_TENDON_LUT_MAX_SAMPLES 24

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

// Device-specific magnetic encoder reference entered by the operator.
// Min means the fully released tendon endpoint and max means the fully
// tightened endpoint; the raw numeric values may cross 0/16383.
// Direction comes from g_encoderDirection and is not stored twice.
struct ManualEncoderReference {
    uint8_t valid;
    uint16_t minRaw;
    uint16_t maxRaw;
};

struct CalibrationAssemblyWarningInfo {
    uint8_t reasonMask;
    int16_t autoMinOffsetCounts;
    int16_t autoMaxShortfallCounts;
    uint16_t autoSpanCounts;
};

// Per-joint calibration result:
// - encoderMin/encoderMax/offset and the legacy RANGE100 fields are retained
//   so old Calib Load snapshots can still be parsed.
// - Feedback mapping v2 uses encoderAtUrdfMin/encoderAtUrdfMax as the two
//   measured, direction-oriented magnetic encoder endpoints. The interval
//   maps linearly to [angleMin, angleMax] in URDF degrees without assuming
//   that the calibrated encoder span represents any fixed mechanical angle.
struct JointCalibrationResult {
    bool success;
    int32_t encoderMin;
    int32_t encoderMax;
    int32_t offset;
    float angleMax;

    // Feedback mapping v2. Retained while the current controller and Calib
    // Load protocol still consume full-URDF endpoint mapping.
    uint8_t mappingVersion;
    int32_t encoderAtUrdfMin;
    int32_t encoderAtUrdfMax;
    float angleMin;
    uint32_t urdfConfigCrc;

    // Manual-scaled feedback mapping v3. Manual endpoints live in
    // g_manualEncoderReference. manualReferenceCrc is the CRC of this joint's
    // endpoint pair, so editing one joint invalidates only that joint's Auto
    // result. The full-table CRC remains reserved for NVS/readback integrity.
    uint16_t autoMinRaw;
    uint16_t autoMaxRaw;
    int32_t manualSpanCounts;
    int32_t autoSpanCounts;
    uint32_t manualReferenceCrc;
    float currentMinDeg;
    float currentMaxDeg;

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

    uint8_t dualTendonLutValid;
    uint8_t dualTendonLutCount;
    uint8_t dualTendonLutMotor[DUAL_TENDON_LUT_ACTUATOR_COUNT];
    float dualTendonLutAngleStartDeg;
    float dualTendonLutAngleStepDeg;
    int32_t dualTendonLutMotorAbs[DUAL_TENDON_LUT_ACTUATOR_COUNT][DUAL_TENDON_LUT_MAX_SAMPLES];
};

extern JointCalibrationConfig g_jointCalibConfig[ENCODER_TOTAL_NUM];
extern JointCalibrationResult g_jointCalibResult[ENCODER_TOTAL_NUM];
extern ManualEncoderReference g_manualEncoderReference[ENCODER_TOTAL_NUM];

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

int32_t calibrationAutoMinManualOffsetCounts(
    uint8_t jointIndex,
    const JointCalibrationResult& result);
bool evaluateCalibrationAssemblyWarning(
    uint8_t jointIndex,
    const JointCalibrationResult& result,
    CalibrationAssemblyWarningInfo* out);
bool updateCalibrationAssemblyWarning(
    TaskSharedData_t* sharedData,
    uint8_t jointIndex,
    const JointCalibrationResult& result);
void clearCalibrationAssemblyWarnings(TaskSharedData_t* sharedData,
                                      uint32_t jointMask);

#endif // CALIBRATION_TASK_H
