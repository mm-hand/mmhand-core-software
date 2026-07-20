#include "AutoCalibrationTask.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "CalibrationMath.h"
#include "CalibrationTask.h"
#include "../control/ControlTask.h"
#include "../hardware/HardwareMap.h"
#include "../persistence/ManualCalibrationNvsStore.h"
#include "../shared/SharedSafetyState.h"
#include "../system/FirmwareLog.h"
#include "../system/StateMachineTask.h"
#include "../system/TaskRuntimeMonitor.h"

static TaskHandle_t g_autoCalibrationTaskHandle = NULL;

const uint8_t kDefaultAutoCalibrationJoints[] = {
    0, 1, 2, 3,
    4, 5, 6, 7,
    8, 9, 10, 11,
    12, 13, 14, 15,
    20, 16, 17, 18, 19
};
const uint8_t kDefaultAutoCalibrationJointsCount = sizeof(kDefaultAutoCalibrationJoints) / sizeof(kDefaultAutoCalibrationJoints[0]);
// The thumb is mechanically independent of the four side-sway dual joints.
// Only these joints participate in automatic calibration workspace planning.
static const uint8_t kWorkspaceDualJoints[] = {0, 4, 8, 12};
static const uint8_t kWorkspaceDualJointsCount =
    sizeof(kWorkspaceDualJoints) / sizeof(kWorkspaceDualJoints[0]);
static const uint8_t kWorkspaceDualHelperCount = kWorkspaceDualJointsCount - 1;

// Keep the four fingers in joint order, then calibrate the thumb as:
// J20 -> J16 -> J17 -> J18 -> J19.
static uint8_t autoCalibrationSequenceRank(uint8_t jointIndex)
{
    if (jointIndex < 16) {
        return jointIndex;
    }
    switch (jointIndex) {
        case 20:
            return 16;
        case 16:
            return 17;
        case 17:
            return 18;
        case 18:
            return 19;
        case 19:
            return 20;
        default:
            return jointIndex;
    }
}

static const uint32_t kAutoFeedbackReadyCanTimeoutMs = 500;
static const uint32_t kAutoFeedbackReadyServoTimeoutMs = 500;
static const uint32_t kAutoFeedbackReadyTelemetryTimeoutMs = 800;
static const uint32_t kAutoEncoderTrustedSampleMaxAgeMs = 300;
static const uint32_t kAutoInitialFeedbackWaitMs = 1200;
static const float kAutoCalibrationMinTravelDeg = 10.0f;
static const float kAutoCalibrationMaxTravelDeg = 359.0f;
static const int32_t kAutoCalibrationRejectNearFullWrapSpan = kCalibrationEncoderModulo - 1024;
static const float kAutoCalibrationStartDeg = -5.0f;
static const float kAutoCalibrationZeroDeg = 0.0f;
static const int32_t kAutoServoZeroPullCounts = 500;
static const int32_t kDualAutoSlackCounts = 3200;
static const int32_t kCalibratedMotorAbsWindowCounts = 4096 * 5;
static const int32_t kCalibratedMotorAbsMaxMarginCounts = 500;
static const int16_t kAutoCalibrationOverloadLoadThreshold = 800;
static const int16_t kDualPrimaryTightLoadThreshold = 190;
static const int16_t kDualSecondaryPreloadLoadThreshold = 190;
static const int16_t kDualSecondaryFinalLoadThreshold = 250;
static const int16_t kDualPrimaryFinalLoadThreshold = 120;
static const int16_t kWorkspaceTightLoadThreshold = 190;
static const int32_t kWorkspaceReleaseCounts = 3200;
static const int32_t kWorkspaceTargetToleranceCounts = 60;
static const float kReverseMinFollowDeg = 12.0f;
static const float kDualTendonFitSampleStepDeg = 5.0f;
static const uint8_t kDualTendonRawSampleCapacity =
    DUAL_TENDON_LUT_MAX_SAMPLES * 3;
static const uint8_t kAngleStallStableSamples = 2;
static const int32_t kPreReleaseSyncTargetToleranceCounts = 40;
static const int32_t kPreReleaseSyncSettleDeltaCounts = 20;
static const uint8_t kPreReleaseSyncStableSamples = 2;

struct DualTendonRawFitSample {
    int32_t encoder;
    int32_t primaryAbs;
    int32_t secondaryAbs;
};

struct DualTendonRawFitBuffer {
    uint8_t count;
    float nextTravelDeg;
    DualTendonRawFitSample samples[kDualTendonRawSampleCapacity];
};

enum DualWorkspacePose : uint8_t {
    DUAL_WORKSPACE_POSE_NONE = 0,
    // Primary released by 3200 counts; secondary tightened to load 190.
    DUAL_WORKSPACE_POSE_A = 1,
    // Secondary released by 3200 counts; primary tightened to load 190.
    DUAL_WORKSPACE_POSE_B = 2,
};

struct DualWorkspaceCacheEntry {
    uint8_t valid;
    uint8_t currentPose;
    uint8_t primaryMotor;
    uint8_t secondaryMotor;
    int32_t primaryTightAbs;
    int32_t primaryReleaseAbs;
    int32_t secondaryTightAbs;
    int32_t secondaryReleaseAbs;
};

struct DualWorkspacePlanStep {
    uint8_t jointIndex;
    DualWorkspacePose pose;
};

enum DualWorkspacePlanStage : int32_t {
    DUAL_WORKSPACE_PLAN_STAGE_PREFLIGHT = 101,
    DUAL_WORKSPACE_PLAN_STAGE_EXECUTE = 102,
};

// Deliberately RAM-only. A position established in one calibration sequence
// must never be trusted after a reset, power cycle, or manual motion.
static DualWorkspaceCacheEntry g_dualWorkspaceCache[kWorkspaceDualJointsCount];

static int32_t abs32(int32_t value)
{
    return value < 0 ? -value : value;
}


static int16_t clampToServoPosLocal(int32_t value)
{
    if (value > 30719) {
        return 30719;
    }
    if (value < -30719) {
        return -30719;
    }
    return (int16_t)value;
}

static int32_t clampToServoAbsRangeLocal(int32_t value)
{
    return (int32_t)clampToServoPosLocal(value);
}

static void setCalibratedMotorAbsLimit(TaskSharedData_t* sharedData,
                                       uint8_t motorChannel,
                                       int32_t tightAbs)
{
    if (!sharedData ||
        motorChannel >= SERVO_TOTAL_NUM ||
        !isMotorChannelValid(motorChannel)) {
        return;
    }

    const int32_t tightClamped = clampToServoAbsRangeLocal(tightAbs);
    const int32_t slackClamped = clampToServoAbsRangeLocal(
        motorApplyTendonDelta(motorChannel, tightClamped, -kCalibratedMotorAbsWindowCounts));
    const int32_t minAbs = (slackClamped < tightClamped) ? slackClamped : tightClamped;
    const int32_t baseMaxAbs = (slackClamped > tightClamped) ? slackClamped : tightClamped;
    const int32_t maxAbs = clampToServoAbsRangeLocal(
        baseMaxAbs + kCalibratedMotorAbsMaxMarginCounts);
    {
        SharedSafetyGuard guard;
        sharedData->motor_abs_limit_min[motorChannel] = minAbs;
        sharedData->motor_abs_limit_max[motorChannel] = maxAbs;
        sharedAtomicFetchOr(&sharedData->motor_abs_limit_valid_bitmap,
                            (uint32_t)(1UL << motorChannel));
    }
    FW_LOG_INFO_PRINTF("[AUTO CALIB] ABS_LIMIT M%02u min=%ld max=%ld baseMax=%ld tight=%ld slack=%ld shortenSign=%d window=%ld maxMargin=%ld\r\n",
                   (unsigned)motorChannel,
                   (long)minAbs,
                   (long)maxAbs,
                   (long)baseMaxAbs,
                   (long)tightClamped,
                   (long)slackClamped,
                   (int)getMotorTendonShortenAbsSign(motorChannel),
                   (long)kCalibratedMotorAbsWindowCounts,
                   (long)kCalibratedMotorAbsMaxMarginCounts);
}

static void clearCalibratedMotorAbsLimitsForJoint(TaskSharedData_t* sharedData,
                                                  uint8_t jointIndex)
{
    if (!sharedData || jointIndex >= ENCODER_TOTAL_NUM) {
        return;
    }

    uint32_t clearedMask = 0;
    JointActuatorGroup group;
    if (getJointActuatorGroup(jointIndex, &group)) {
        for (uint8_t i = 0; i < group.count; i++) {
            const uint8_t motorChannel = group.actuators[i].motorChannel;
            if (motorChannel < SERVO_TOTAL_NUM && isMotorChannelValid(motorChannel)) {
                clearedMask |= (1UL << motorChannel);
            }
        }
    } else {
        const int primaryCh = findMotorChannelByBusId(jointMap[jointIndex].busIndex,
                                                      jointMap[jointIndex].servoID);
        if (primaryCh >= 0 && primaryCh < SERVO_TOTAL_NUM) {
            clearedMask |= (1UL << (uint8_t)primaryCh);
        }
    }

    if (clearedMask == 0) {
        return;
    }

    {
        SharedSafetyGuard guard;
        sharedAtomicFetchAnd(&sharedData->motor_abs_limit_valid_bitmap, ~clearedMask);
        sharedAtomicFetchAnd(&sharedData->motor_abs_limit_locked_bitmap, ~clearedMask);
        for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
            if ((clearedMask & (1UL << ch)) != 0) {
                sharedData->motor_abs_limit_min[ch] = 0;
                sharedData->motor_abs_limit_max[ch] = 0;
            }
        }
    }
    FW_LOG_INFO_PRINTF("[AUTO CALIB] ABS_LIMIT_CLEAR J%02u mask=0x%08lX\r\n",
                  (unsigned)jointIndex,
                  (unsigned long)clearedMask);
}

static int16_t normalizeServoRawSingleTurn(int32_t value)
{
    int32_t raw = value % 4096;
    if (raw < 0) {
        raw += 4096;
    }
    return (int16_t)raw;
}

static int32_t servoTurnFromAbs(int32_t abs)
{
    const int32_t raw = normalizeServoRawSingleTurn(abs);
    return (abs - raw) / 4096;
}

static int16_t servoLoadMagnitude(int16_t rawLoad)
{
    int16_t mag = rawLoad;
    if (mag < 0) {
        mag = -mag;
    }
    return (int16_t)(mag & 0x03FF);
}

static void setCalibrationDiag(TaskSharedData_t* sharedData,
                               uint8_t reason,
                               uint8_t phase,
                               uint8_t joint,
                               uint8_t motor,
                               int32_t valueA,
                               int32_t valueB,
                               int32_t valueC,
                               uint32_t bitmap)
{
    if (!sharedData || reason == CALIB_DIAG_REASON_NONE) {
        return;
    }
    CalibrationDiagData_t diag;
    memset(&diag, 0, sizeof(diag));
    diag.valid = 1;
    diag.reason = reason;
    diag.phase = phase;
    diag.joint = joint;
    diag.motor = motor;
    diag.valueA = valueA;
    diag.valueB = valueB;
    diag.valueC = valueC;
    diag.bitmap = bitmap;
    diag.timestamp = millis();
    {
        SharedSafetyGuard guard;
        diag.sequence = sharedData->calibration_diag.sequence + 1;
        sharedData->calibration_diag = diag;
    }
}

static void clearCalibrationDiag(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return;
    }
    SharedSafetyGuard guard;
    const uint32_t nextSequence = sharedData->calibration_diag.sequence + 1;
    memset(&sharedData->calibration_diag, 0, sizeof(sharedData->calibration_diag));
    sharedData->calibration_diag.sequence = nextSequence;
}

static bool hasCalibrationDiag(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return false;
    }
    SharedSafetyGuard guard;
    return sharedData->calibration_diag.valid != 0;
}

static uint8_t calibrationDiagReason(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return CALIB_DIAG_REASON_NONE;
    }
    SharedSafetyGuard guard;
    return sharedData->calibration_diag.valid
        ? sharedData->calibration_diag.reason
        : CALIB_DIAG_REASON_NONE;
}

static bool encoderOnlineAndHealthy(TaskSharedData_t* sharedData, uint32_t bit)
{
    if (!sharedData) {
        return false;
    }
    SharedSafetyGuard guard;
    const uint32_t onlineMask = sharedAtomicLoad(&sharedData->encoder_online_bitmap);
    const uint32_t faultMask = sharedAtomicLoad(&sharedData->encoder_fault_bitmap);
    return (onlineMask & bit) != 0 && (faultMask & bit) == 0;
}

static bool autoCalibrationStillOwnsControl(TaskSharedData_t* sharedData)
{
    return sharedData &&
           sharedData->system_state == SYSTEM_STATE_CALIBRATION_RUNNING &&
           sharedData->servo_target_owner == SERVO_TARGET_OWNER_CALIBRATION &&
           sharedData->control_enabled == 0;
}

static bool isServoCommandInsideLimit(int32_t commandAbs)
{
    return commandAbs >= -30719 && commandAbs <= 30719;
}

static int findMotorChannel(uint8_t bus, uint8_t id)
{
    return findMotorChannelByBusId(bus, id);
}

static bool isJointPrimaryServoMultiTurnKnown(TaskSharedData_t* sharedData, uint8_t jointIndex)
{
    if (!sharedData || jointIndex >= ENCODER_TOTAL_NUM) {
        return false;
    }
    const int ch = findMotorChannel(jointMap[jointIndex].busIndex,
                                    jointMap[jointIndex].servoID);
    if (ch < 0 || ch >= SERVO_TOTAL_NUM) {
        return false;
    }
    return (sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap) &
            (1UL << ch)) == 0;
}

static void clearJointServoMultiTurnUnknown(TaskSharedData_t* sharedData, uint8_t jointIndex)
{
    if (!sharedData || jointIndex >= ENCODER_TOTAL_NUM) {
        return;
    }
    uint32_t clearMask = 0;
    JointActuatorGroup group;
    if (getJointActuatorGroup(jointIndex, &group)) {
        for (uint8_t i = 0; i < group.count; i++) {
            const uint8_t motorChannel = group.actuators[i].motorChannel;
            if (motorChannel < SERVO_TOTAL_NUM && isMotorChannelValid(motorChannel)) {
                clearMask |= (1UL << motorChannel);
            }
        }
    } else {
        const int primaryCh = findMotorChannel(jointMap[jointIndex].busIndex,
                                               jointMap[jointIndex].servoID);
        if (primaryCh >= 0 && primaryCh < SERVO_TOTAL_NUM) {
            clearMask |= (1UL << primaryCh);
        }
    }

    if (clearMask != 0) {
        sharedAtomicFetchAnd(&sharedData->multi_turn_unknown_bitmap, ~clearMask);
        FW_LOG_INFO_PRINTF("[AUTO CALIB] MULTITURN_KNOWN J%02u clearMask=0x%08lX unknown=0x%08lX\r\n",
                      jointIndex,
                      (unsigned long)clearMask,
                      (unsigned long)sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap));
    }
}

static bool readLatestEncoderSample(TaskSharedData_t* sharedData,
                                    uint8_t jointIndex,
                                    uint16_t* outRaw,
                                    int32_t* outOriented)
{
    if (!sharedData || !outRaw || !outOriented || jointIndex >= ENCODER_TOTAL_NUM) {
        return false;
    }

    RemoteSensorData_t sensorData;
    const uint32_t nowMs = millis();
    const uint32_t bit = (1UL << jointIndex);
    if (!sharedData->canRxQueue ||
        xQueuePeek(sharedData->canRxQueue, &sensorData, 0) != pdTRUE ||
        !sensorData.isValid ||
        (uint32_t)(nowMs - sensorData.timestamp) > kAutoEncoderTrustedSampleMaxAgeMs ||
        !encoderOnlineAndHealthy(sharedData, bit)) {
        return false;
    }

    *outRaw = sensorData.encoderValues[jointIndex];
    *outOriented = calibrationOrientEncoderRaw(sensorData.encoderValues[jointIndex],
                                               g_encoderDirection[jointIndex]);
    return true;
}

static bool readLatestEncoderOriented(TaskSharedData_t* sharedData,
                                      uint8_t jointIndex,
                                      int32_t* outEncoder)
{
    uint16_t raw = 0;
    return readLatestEncoderSample(sharedData, jointIndex, &raw, outEncoder);
}

static bool readLatestServoAbs(TaskSharedData_t* sharedData,
                               uint8_t bus,
                               uint8_t id,
                               int32_t* outAbs)
{
    if (!sharedData || !outAbs) {
        return false;
    }
    const int ch = findMotorChannel(bus, id);
    if (ch < 0) {
        return false;
    }

    ServoAngleData_t servoData;
    if (!sharedData->servoFeedbackQueue ||
        xQueuePeek(sharedData->servoFeedbackQueue, &servoData, 0) != pdTRUE ||
        servoData.onlineStatus[ch] == 0) {
        return false;
    }

    *outAbs = servoData.servoAngles[ch];
    return true;
}

static bool readLatestServoRaw(TaskSharedData_t* sharedData,
                               uint8_t bus,
                               uint8_t id,
                               int16_t* outRaw)
{
    if (!sharedData || !outRaw) {
        return false;
    }
    const int ch = findMotorChannel(bus, id);
    if (ch < 0) {
        return false;
    }

    ServoAngleData_t servoData;
    if (!sharedData->servoFeedbackQueue ||
        xQueuePeek(sharedData->servoFeedbackQueue, &servoData, 0) != pdTRUE ||
        servoData.onlineStatus[ch] == 0) {
        return false;
    }

    *outRaw = normalizeServoRawSingleTurn(servoData.servoRawPositions[ch]);
    return true;
}

static bool readLatestServoLoad(TaskSharedData_t* sharedData,
                                uint8_t bus,
                                uint8_t id,
                                int16_t* outLoad)
{
    if (!sharedData || !outLoad) {
        return false;
    }
    const int ch = findMotorChannel(bus, id);
    if (ch < 0) {
        return false;
    }

    ServoTelemetryData_t telemetry;
    QueueHandle_t queue = sharedData->servoTelemetrySnapshotQueue
        ? sharedData->servoTelemetrySnapshotQueue
        : sharedData->servoTelemetryQueue;
    if (!queue ||
        xQueuePeek(queue, &telemetry, 0) != pdTRUE ||
        telemetry.onlineStatus[ch] == 0) {
        return false;
    }

    *outLoad = telemetry.load[ch];
    return true;
}

static bool readLatestMotorServoAbs(TaskSharedData_t* sharedData,
                                    uint8_t motorChannel,
                                    int32_t* outAbs)
{
    if (!sharedData || !outAbs || !isMotorChannelValid(motorChannel)) {
        return false;
    }

    ServoAngleData_t servoData;
    if (!sharedData->servoFeedbackQueue ||
        xQueuePeek(sharedData->servoFeedbackQueue, &servoData, 0) != pdTRUE ||
        servoData.onlineStatus[motorChannel] == 0) {
        return false;
    }

    *outAbs = servoData.servoAngles[motorChannel];
    return true;
}

static bool readLatestMotorServoRaw(TaskSharedData_t* sharedData,
                                    uint8_t motorChannel,
                                    int16_t* outRaw)
{
    if (!sharedData || !outRaw || !isMotorChannelValid(motorChannel)) {
        return false;
    }

    ServoAngleData_t servoData;
    if (!sharedData->servoFeedbackQueue ||
        xQueuePeek(sharedData->servoFeedbackQueue, &servoData, 0) != pdTRUE ||
        servoData.onlineStatus[motorChannel] == 0) {
        return false;
    }

    *outRaw = normalizeServoRawSingleTurn(servoData.servoRawPositions[motorChannel]);
    return true;
}

static bool readLatestMotorServoLoad(TaskSharedData_t* sharedData,
                                     uint8_t motorChannel,
                                     int16_t* outLoad)
{
    if (!sharedData || !outLoad || !isMotorChannelValid(motorChannel)) {
        return false;
    }

    ServoTelemetryData_t telemetry;
    QueueHandle_t queue = sharedData->servoTelemetrySnapshotQueue
        ? sharedData->servoTelemetrySnapshotQueue
        : sharedData->servoTelemetryQueue;
    if (!queue ||
        xQueuePeek(queue, &telemetry, 0) != pdTRUE ||
        telemetry.onlineStatus[motorChannel] == 0) {
        return false;
    }

    *outLoad = telemetry.load[motorChannel];
    return true;
}

static bool calibrationBatchOverloadSafe(TaskSharedData_t* sharedData,
                                         const ServoTargetBatch_t* batch)
{
    if (!sharedData || !batch) {
        return false;
    }
    for (uint8_t i = 0; i < batch->count; i++) {
        const ServoTargetCommand_t& cmd = batch->commands[i];
        const int ch = findMotorChannel(cmd.busIndex, cmd.servoId);
        if (ch < 0 || ch >= SERVO_TOTAL_NUM || !isMotorChannelValid((uint8_t)ch)) {
            continue;
        }

        int16_t loadRaw = 0;
        if (!readLatestMotorServoLoad(sharedData, (uint8_t)ch, &loadRaw)) {
            continue;
        }
        const int16_t loadMag = servoLoadMagnitude(loadRaw);
        if (loadMag >= kAutoCalibrationOverloadLoadThreshold) {
            const uint32_t bit = 1UL << (uint8_t)ch;
            {
                SharedSafetyGuard guard;
                sharedAtomicFetchOr(&sharedData->overload_fault_bitmap, bit);
                sharedData->overload_fault_load[ch] = loadRaw;
            }
            FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL OVERLOAD M%02d load=%d threshold=%d\r\n",
                          ch,
                          loadMag,
                          kAutoCalibrationOverloadLoadThreshold);
            setCalibrationDiag(sharedData,
                               CALIB_DIAG_REASON_OVERLOAD,
                               CALIB_DIAG_PHASE_SEQUENCE,
                               0xFF,
                               (uint8_t)ch,
                               loadMag,
                               kAutoCalibrationOverloadLoadThreshold,
                               loadRaw,
                               bit);
            return false;
        }
    }
    return true;
}

static void logAutoInitialFeedbackDiag(TaskSharedData_t* sharedData,
                                       uint8_t jointIndex,
                                       uint8_t bus,
                                       uint8_t id,
                                       bool encoderOk,
                                       bool absOk,
                                       bool rawOk)
{
    const int ch = findMotorChannel(bus, id);
    const uint32_t nowMs = millis();
    bool sensorQueueOk = false;
    bool sensorValid = false;
    uint8_t sensorErr = 0xFF;
    uint32_t sensorAge = 0xFFFFFFFFUL;
    RemoteSensorData_t sensorData;
    if (sharedData &&
        sharedData->canRxQueue &&
        xQueuePeek(sharedData->canRxQueue, &sensorData, 0) == pdTRUE) {
        sensorQueueOk = true;
        sensorValid = sensorData.isValid;
        if (jointIndex < ENCODER_TOTAL_NUM) {
            sensorErr = sensorData.errorFlags[jointIndex];
        }
        sensorAge = nowMs - sensorData.timestamp;
    }

    bool servoQueueOk = false;
    uint8_t servoOnline = 0;
    uint32_t servoAge = 0xFFFFFFFFUL;
    int32_t servoAbs = 0;
    int16_t servoRaw = 0;
    ServoAngleData_t servoData;
    if (sharedData &&
        sharedData->servoFeedbackQueue &&
        xQueuePeek(sharedData->servoFeedbackQueue, &servoData, 0) == pdTRUE) {
        servoQueueOk = true;
        servoAge = nowMs - servoData.timestamp;
        if (ch >= 0 && ch < SERVO_TOTAL_NUM) {
            servoOnline = servoData.onlineStatus[ch];
            servoAbs = servoData.servoAngles[ch];
            servoRaw = servoData.servoRawPositions[ch];
        }
    }

    FW_LOG_INFO_PRINTF("[AUTO CALIB] INIT_FEEDBACK J%02u bus%u id%u ch=%d encoderOk=%u absOk=%u rawOk=%u sensorQ=%u valid=%u err=0x%02X age=%lu servoQ=%u online=%u age=%lu abs=%ld raw=%d\r\n",
                  jointIndex,
                  bus,
                  id,
                  ch,
                  encoderOk ? 1 : 0,
                  absOk ? 1 : 0,
                  rawOk ? 1 : 0,
                  sensorQueueOk ? 1 : 0,
                  sensorValid ? 1 : 0,
                  sensorErr,
                  (unsigned long)sensorAge,
                  servoQueueOk ? 1 : 0,
                  servoOnline,
                  (unsigned long)servoAge,
                  (long)servoAbs,
                  (int)servoRaw);
}

static int32_t roundToInt32Local(float value)
{
    return (int32_t)(value + (value >= 0.0f ? 0.5f : -0.5f));
}

static int32_t signedEncoderDeltaLocal(int32_t fromEncoder, int32_t toEncoder)
{
    return calibrationWrapEncoderDelta(toEncoder, fromEncoder);
}

static int32_t encoderTravelCountLocal(int32_t fromEncoder, int32_t toEncoder, int8_t travelSign)
{
    const int32_t signedDelta = signedEncoderDeltaLocal(fromEncoder, toEncoder);
    return (travelSign < 0) ? -signedDelta : signedDelta;
}

static int32_t encoderForPositiveTravelResult(int32_t encoder, int8_t travelSign)
{
    if (travelSign >= 0) {
        return calibrationWrapEncoderCount(encoder);
    }
    return calibrationWrapEncoderCount(kCalibrationEncoderModulo - encoder);
}

static void resetDualTendonRawFitBuffer(DualTendonRawFitBuffer* buffer)
{
    if (!buffer) {
        return;
    }
    memset(buffer, 0, sizeof(*buffer));
}

static bool appendDualTendonRawFitSample(DualTendonRawFitBuffer* buffer,
                                         int32_t encoder,
                                         int32_t primaryAbs,
                                         int32_t secondaryAbs)
{
    if (!buffer || buffer->count >= kDualTendonRawSampleCapacity) {
        return false;
    }
    if (buffer->count > 0) {
        DualTendonRawFitSample& prev = buffer->samples[buffer->count - 1];
        if (prev.encoder == encoder &&
            prev.primaryAbs == primaryAbs &&
            prev.secondaryAbs == secondaryAbs) {
            return true;
        }
    }
    DualTendonRawFitSample& sample = buffer->samples[buffer->count++];
    sample.encoder = calibrationWrapEncoderCount(encoder);
    sample.primaryAbs = primaryAbs;
    sample.secondaryAbs = secondaryAbs;
    return true;
}

static void maybeAppendDualTendonTravelFitSample(DualTendonRawFitBuffer* buffer,
                                                 float travelDeg,
                                                 int32_t encoder,
                                                 int32_t primaryAbs,
                                                 int32_t secondaryAbs)
{
    if (!buffer || !isfinite(travelDeg)) {
        return;
    }
    if (travelDeg + 0.1f < buffer->nextTravelDeg) {
        return;
    }
    while (travelDeg + 0.1f >= buffer->nextTravelDeg) {
        buffer->nextTravelDeg += kDualTendonFitSampleStepDeg;
    }
    appendDualTendonRawFitSample(buffer, encoder, primaryAbs, secondaryAbs);
}

static float dualFitSampleAngleDeg(const JointCalibrationResult& result,
                                   int32_t encoder)
{
    if (result.encoderSpan <= 0 || result.totalDeg <= 0.0f) {
        return NAN;
    }
    const int32_t delta =
        calibrationForwardEncoderDelta(result.startEncoder,
                                       calibrationWrapEncoderCount(encoder));
    return result.startDeg +
        ((float)delta / (float)result.encoderSpan) * result.totalDeg;
}

static bool interpolateDualFitAbs(float targetDeg,
                                  const float* sampleDeg,
                                  const int32_t* sampleAbs,
                                  uint8_t sampleCount,
                                  int32_t* outAbs)
{
    if (!sampleDeg || !sampleAbs || !outAbs || sampleCount == 0) {
        return false;
    }
    if (targetDeg <= sampleDeg[0]) {
        *outAbs = sampleAbs[0];
        return true;
    }
    for (uint8_t i = 1; i < sampleCount; i++) {
        if (targetDeg <= sampleDeg[i]) {
            const float span = sampleDeg[i] - sampleDeg[i - 1];
            if (span <= 0.0f) {
                return false;
            }
            const float t = (targetDeg - sampleDeg[i - 1]) / span;
            const float value =
                (float)sampleAbs[i - 1] +
                ((float)sampleAbs[i] - (float)sampleAbs[i - 1]) * t;
            *outAbs = roundToInt32Local(value);
            return true;
        }
    }
    *outAbs = sampleAbs[sampleCount - 1];
    return true;
}

static bool buildDualTendonLutFromRawSamples(JointCalibrationResult* result,
                                             uint8_t primaryMotor,
                                             uint8_t secondaryMotor,
                                             const DualTendonRawFitBuffer& raw)
{
    if (!result) {
        return false;
    }
    result->dualTendonLutValid = 0;
    result->dualTendonLutCount = 0;
    memset(result->dualTendonLutMotor, 0xFF, sizeof(result->dualTendonLutMotor));
    memset(result->dualTendonLutMotorAbs, 0, sizeof(result->dualTendonLutMotorAbs));
    result->dualTendonLutAngleStartDeg = result->startDeg;
    result->dualTendonLutAngleStepDeg = kDualTendonFitSampleStepDeg;

    if (!result->success || !result->scaled ||
        result->encoderSpan <= 0 ||
        result->totalDeg <= 0.0f ||
        raw.count < 3) {
        return false;
    }

    float sampleDeg[kDualTendonRawSampleCapacity];
    int32_t primaryAbs[kDualTendonRawSampleCapacity];
    int32_t secondaryAbs[kDualTendonRawSampleCapacity];
    uint8_t sampleCount = 0;

    for (uint8_t i = 0; i < raw.count; i++) {
        const float angleDeg =
            dualFitSampleAngleDeg(*result, raw.samples[i].encoder);
        if (!isfinite(angleDeg) ||
            angleDeg < result->startDeg - kDualTendonFitSampleStepDeg ||
            angleDeg > result->tightDeg + kDualTendonFitSampleStepDeg) {
            continue;
        }
        if (sampleCount >= kDualTendonRawSampleCapacity) {
            continue;
        }

        uint8_t insertAt = sampleCount;
        while (insertAt > 0 && sampleDeg[insertAt - 1] > angleDeg) {
            sampleDeg[insertAt] = sampleDeg[insertAt - 1];
            primaryAbs[insertAt] = primaryAbs[insertAt - 1];
            secondaryAbs[insertAt] = secondaryAbs[insertAt - 1];
            insertAt--;
        }
        if (insertAt > 0 && fabsf(sampleDeg[insertAt - 1] - angleDeg) < 0.2f) {
            sampleDeg[insertAt - 1] = angleDeg;
            primaryAbs[insertAt - 1] = raw.samples[i].primaryAbs;
            secondaryAbs[insertAt - 1] = raw.samples[i].secondaryAbs;
            continue;
        }
        sampleDeg[insertAt] = angleDeg;
        primaryAbs[insertAt] = raw.samples[i].primaryAbs;
        secondaryAbs[insertAt] = raw.samples[i].secondaryAbs;
        sampleCount++;
    }

    if (sampleCount < 3 ||
        sampleDeg[sampleCount - 1] - sampleDeg[0] < kDualTendonFitSampleStepDeg * 2.0f ||
        sampleDeg[0] > result->startDeg + kDualTendonFitSampleStepDeg ||
        sampleDeg[sampleCount - 1] < result->tightDeg - kDualTendonFitSampleStepDeg) {
        return false;
    }

    uint8_t lutCount =
        (uint8_t)ceilf(result->totalDeg / kDualTendonFitSampleStepDeg) + 1;
    if (lutCount > DUAL_TENDON_LUT_MAX_SAMPLES) {
        lutCount = DUAL_TENDON_LUT_MAX_SAMPLES;
    }
    if (lutCount < 3) {
        return false;
    }

    result->dualTendonLutMotor[0] = primaryMotor;
    result->dualTendonLutMotor[1] = secondaryMotor;
    result->dualTendonLutAngleStartDeg = result->startDeg;
    result->dualTendonLutAngleStepDeg = kDualTendonFitSampleStepDeg;
    for (uint8_t i = 0; i < lutCount; i++) {
        const float targetDeg =
            result->dualTendonLutAngleStartDeg +
            result->dualTendonLutAngleStepDeg * (float)i;
        if (!interpolateDualFitAbs(targetDeg,
                                   sampleDeg,
                                   primaryAbs,
                                   sampleCount,
                                   &result->dualTendonLutMotorAbs[0][i]) ||
            !interpolateDualFitAbs(targetDeg,
                                   sampleDeg,
                                   secondaryAbs,
                                   sampleCount,
                                   &result->dualTendonLutMotorAbs[1][i])) {
            result->dualTendonLutValid = 0;
            result->dualTendonLutCount = 0;
            return false;
        }
    }
    result->dualTendonLutCount = lutCount;
    result->dualTendonLutValid = 1;
    return true;
}

static bool buildAutoZeroTightCalibrationResultWithRange(uint8_t jointIndex,
                                                         int32_t tautEncoder,
                                                         int32_t zeroEncoder,
                                                         int32_t tightEncoder,
                                                         int32_t tautServoAbs,
                                                         int32_t zeroServoAbs,
                                                         int32_t tightServoAbs,
                                                         float startDeg,
                                                         JointCalibrationResult* out)
{
    (void)zeroEncoder;
    if (!out) {
        return false;
    }

    memset(out, 0, sizeof(*out));

    const int32_t startToTightSpan =
        calibrationForwardEncoderDelta(tautEncoder, tightEncoder);
    const float totalDeg = calibrationEncoderCountToDeg(startToTightSpan);
    if (startToTightSpan <= 0 ||
        !isfinite(totalDeg) ||
        totalDeg < kAutoCalibrationMinTravelDeg ||
        totalDeg >= kAutoCalibrationMaxTravelDeg) {
        return false;
    }
    const float tightDeg = startDeg + totalDeg;

    const float zeroFraction =
        (kAutoCalibrationZeroDeg - startDeg) / totalDeg;
    if (zeroFraction <= 0.0f || zeroFraction >= 1.0f) {
        return false;
    }

    const int32_t zeroOffsetCount =
        roundToInt32Local((float)startToTightSpan * zeroFraction);
    const int32_t computedZeroEncoder =
        calibrationWrapEncoderCount(tautEncoder + zeroOffsetCount);

    out->success = true;
    out->encoderMin = tautEncoder;
    out->encoderMax = tightEncoder;
    out->offset = computedZeroEncoder;
    out->angleMax = tightDeg;
    out->startEncoder = tautEncoder;
    out->tightEncoder = tightEncoder;
    out->zeroEncoder = computedZeroEncoder;
    out->encoderSpan = startToTightSpan;
    out->startServoAbs = tautServoAbs;
    out->tightServoAbs = tightServoAbs;
    out->zeroServoAbs = zeroServoAbs;
    out->startDeg = startDeg;
    out->tightDeg = tightDeg;
    out->totalDeg = totalDeg;
    out->scaled = true;

    // The legacy calibration fields above use direction-oriented encoder
    // coordinates. V3 autoMinRaw/autoMaxRaw are a wire/NVS contract and must
    // contain the physical 14-bit raw values. calibrationOrientEncoderRaw()
    // is self-inverse for direction=-1, so applying it here converts the
    // oriented endpoints back to raw without changing direction=+1 joints.
    const int8_t encoderDirection = g_encoderDirection[jointIndex];
    const uint16_t autoMinRaw = (uint16_t)calibrationOrientEncoderRaw(
        (uint16_t)calibrationWrapEncoderCount(tautEncoder), encoderDirection);
    const uint16_t autoMaxRaw = (uint16_t)calibrationOrientEncoderRaw(
        (uint16_t)calibrationWrapEncoderCount(tightEncoder), encoderDirection);
    if (!applyManualScaledUrdfMappingToCalibration(
            jointIndex,
            g_manualEncoderReference[jointIndex],
            autoMinRaw,
            autoMaxRaw,
            encoderDirection,
            manualEncoderReferenceEntryCrc(
                jointIndex, g_manualEncoderReference[jointIndex]),
            out)) {
        memset(out, 0, sizeof(*out));
        return false;
    }
    return true;
}

static bool buildAutoZeroTightCalibrationResult(uint8_t jointIndex,
                                                int32_t tautEncoder,
                                                int32_t zeroEncoder,
                                                int32_t tightEncoder,
                                                int32_t tautServoAbs,
                                                int32_t zeroServoAbs,
                                                int32_t tightServoAbs,
                                                JointCalibrationResult* out)
{
    return buildAutoZeroTightCalibrationResultWithRange(jointIndex,
                                                        tautEncoder,
                                                        zeroEncoder,
                                                        tightEncoder,
                                                        tautServoAbs,
                                                        zeroServoAbs,
                                                        tightServoAbs,
                                                        kAutoCalibrationStartDeg,
                                                        out);
}

static bool buildDualZeroTightCalibrationResult(uint8_t jointIndex,
                                                int32_t startEncoder,
                                                int32_t zeroEncoder,
                                                int32_t tightEncoder,
                                                int32_t startServoAbs,
                                                int32_t zeroServoAbs,
                                                int32_t tightServoAbs,
                                                JointCalibrationResult* out)
{
    return buildAutoZeroTightCalibrationResultWithRange(jointIndex,
                                                        startEncoder,
                                                        zeroEncoder,
                                                        tightEncoder,
                                                        startServoAbs,
                                                        zeroServoAbs,
                                                        tightServoAbs,
                                                        kAutoCalibrationStartDeg,
                                                        out);
}

static void applyDynamicAutoCalibrationConfig(JointCalibrationConfig* cfg,
                                              const JointCalibrationResult& result)
{
    if (!cfg || !result.success || result.totalDeg <= 0.0f) {
        return;
    }

    cfg->angleScope = result.totalDeg;
    cfg->bottomReserved = -result.startDeg;
    cfg->topReserved = 0.0f;
    cfg->angleReserved = -result.startDeg;
}

static uint8_t buildAutoCalibrationJointListFromActiveScope(TaskSharedData_t* sharedData,
                                                            uint8_t* outJoints,
                                                            uint8_t maxCount)
{
    if (!sharedData || !outJoints || maxCount == 0) {
        return 0;
    }

    uint32_t activeMask = 0;
    uint32_t selectedActiveMotorMask = 0;
    {
        SharedSafetyGuard guard;
        activeMask = sharedAtomicLoad(&sharedData->active_joint_mask) &
                     ((1UL << ENCODER_TOTAL_NUM) - 1UL);
        selectedActiveMotorMask = sharedAtomicLoad(&sharedData->active_motor_mask);
    }
    if (activeMask == 0 && selectedActiveMotorMask != 0) {
        const uint32_t activeMotorMask =
            selectedActiveMotorMask & validMotorChannelMask();
        for (uint8_t jointIndex = 0; jointIndex < ENCODER_TOTAL_NUM; jointIndex++) {
            JointActuatorGroup group;
            if (!getJointActuatorGroup(jointIndex, &group)) {
                continue;
            }
            for (uint8_t i = 0; i < group.count; i++) {
                const uint8_t motorChannel = group.actuators[i].motorChannel;
                if (motorChannel < SERVO_TOTAL_NUM &&
                    (activeMotorMask & (1UL << motorChannel)) != 0) {
                    activeMask |= (1UL << jointIndex);
                    break;
                }
            }
        }

        if (activeMask != 0) {
            FW_LOG_INFO_PRINTF("[AUTO CALIB] ACTIVE_SCOPE inferred joints=0x%08lX from motor=0x%08lX\r\n",
                          (unsigned long)activeMask,
                          (unsigned long)activeMotorMask);
        }
    }

    uint8_t count = 0;
    for (uint8_t jointIndex = 0;
         jointIndex < ENCODER_TOTAL_NUM && count < maxCount;
         jointIndex++) {
        if ((activeMask & (1UL << jointIndex)) == 0) {
            continue;
        }
        outJoints[count++] = jointIndex;
    }

    return count;
}

static void syncJointTargetToDeg(TaskSharedData_t* sharedData,
                                 uint8_t jointIndex,
                                 float targetDeg)
{
    if (!sharedData || jointIndex >= ENCODER_TOTAL_NUM) {
        return;
    }

    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }

    sharedData->targetAngles[jointIndex] = targetDeg;
    sharedData->joint_command_token++;

    if (lock) {
        xSemaphoreGive(lock);
    }
}

static void syncJointTargetToZero(TaskSharedData_t* sharedData, uint8_t jointIndex)
{
    syncJointTargetToDeg(sharedData, jointIndex, 0.0f);
}

AutoCalibrationParams defaultAutoCalibrationParams(void)
{
    AutoCalibrationParams params;
    params.tightenSpeedCountsPerSec = 400;
    params.servoSpeed = 400;
    params.servoAcc = 20;
    params.loopMs = 20;
    params.windowMs = 200;
    params.traceMs = 200;
    params.windowServoDeltaCounts = 50;
    params.zeroReachAbsTolerance = 200;
    params.stallActualDeg = 0.75f;
    params.startLoadDelta = 25;
    params.tightLoadHigh = 250;
    params.startLoadStableSamples = 3;
    return params;
}

enum AutoCalibState {
    AUTO_CALIB_STATE_IDLE = 0,
    AUTO_CALIB_STATE_INIT,
    AUTO_CALIB_STATE_TIGHTEN,
    AUTO_CALIB_STATE_PRE_RELEASE_SYNC,
    AUTO_CALIB_STATE_DONE,
    AUTO_CALIB_STATE_FAILED
};

struct JointCalibContext {
    AutoCalibState state;
    uint8_t jointIndex;
    uint8_t bus;
    uint8_t servoId;
    uint8_t motorChannel;
    int32_t directionSign;
    int8_t encoderTravelSign;
    JointCalibrationConfig motionCfg;
    uint32_t stateBeginMs;
    uint32_t lastCommandMs;
    uint32_t lastTraceMs;
    int32_t commandAbs;
    int32_t commandRemainder;
    int32_t targetAbs;
    int16_t baselineLoad;
    bool startFound;
    bool zeroFound;
    uint8_t startLoadStable;
    uint16_t startSearchRaw;
    int32_t startSearchEncoder;
    int32_t startSearchAbs;
    int32_t startEncoder;
    int32_t startServoAbs;
    int32_t zeroEncoder;
    int32_t zeroServoAbs;
    int32_t windowStartEncoder;
    int32_t windowStartAbs;
    uint32_t windowStartMs;
    JointCalibrationResult result;
    bool resultValid;
    bool initStarted;
    bool releaseCommandStarted;
    int32_t preReleaseSyncTargetAbs;
    int32_t preReleaseSyncWindowStartAbs;
    uint32_t preReleaseSyncWindowStartMs;
    uint8_t preReleaseSyncStableSamples;
    uint8_t releaseAngleStallStable;
};

static bool appendServoTargetLocal(ServoTargetBatch_t* batch,
                                   uint8_t busIndex,
                                   uint8_t servoId,
                                   int16_t position,
                                   uint16_t speed,
                                   uint8_t acc)
{
    if (!batch) {
        return false;
    }
    for (uint8_t i = 0; i < batch->count; i++) {
        ServoTargetCommand_t& existing = batch->commands[i];
        if (existing.busIndex == busIndex && existing.servoId == servoId) {
            existing.position = position;
            existing.speed = speed;
            existing.acc = acc;
            return true;
        }
    }
    if (batch->count >= SERVO_TARGET_BATCH_MAX) {
        return false;
    }
    ServoTargetCommand_t& cmd = batch->commands[batch->count++];
    cmd.busIndex = busIndex;
    cmd.servoId = servoId;
    cmd.position = position;
    cmd.speed = speed;
    cmd.acc = acc;
    return true;
}

static bool appendJointCalibrationTargetToBatch(ServoTargetBatch_t* batch,
                                                uint8_t jointIndex,
                                                int16_t primaryTarget,
                                                const JointCalibrationConfig& cfg)
{
    if (!batch || jointIndex >= ENCODER_TOTAL_NUM) {
        return false;
    }
    const uint8_t originalCount = batch->count;
    if (!appendServoTargetLocal(batch,
                                jointMap[jointIndex].busIndex,
                                jointMap[jointIndex].servoID,
                                primaryTarget,
                                cfg.tightenSpeed,
                                cfg.tightenAcc)) {
        return false;
    }

    JointDualServoConstraint dualCfg;
    int16_t secondaryTarget = 0;
    if (getJointDualServoConstraint(jointIndex, &dualCfg) &&
        computeSecondaryServoTargetForJoint(jointIndex, primaryTarget, &secondaryTarget)) {
        if (!appendServoTargetLocal(batch,
                                    dualCfg.secondaryBusIndex,
                                    dualCfg.secondaryServoID,
                                    secondaryTarget,
                                    cfg.tightenSpeed,
                                    cfg.tightenAcc)) {
            batch->count = originalCount;
            return false;
        }
    }
    return true;
}

static bool publishCalibrationBatch(TaskSharedData_t* sharedData, ServoTargetBatch_t* batch)
{
    if (!sharedData || !batch || batch->count == 0) {
        return true;
    }
    if (!sharedData->servoTargetQueue ||
        sharedData->servo_target_owner != SERVO_TARGET_OWNER_CALIBRATION) {
        return false;
    }
    if (!calibrationBatchOverloadSafe(sharedData, batch)) {
        return false;
    }
    batch->source = SERVO_TARGET_OWNER_CALIBRATION;
    batch->timestamp = millis();
    return xQueueOverwrite(sharedData->servoTargetQueue, batch) == pdTRUE;
}

static bool appendCalibrationHoldTargetsExcept(TaskSharedData_t* sharedData,
                                               ServoTargetBatch_t* batch,
                                               const uint8_t* jointList,
                                               uint8_t jointCount,
                                               uint8_t activeJoint,
                                               uint32_t skipJointMask,
                                               const AutoCalibrationParams& params)
{
    if (!sharedData || !batch || !jointList) {
        return false;
    }

    for (uint8_t i = 0; i < jointCount; i++) {
        const uint8_t jointIndex = jointList[i];
        if (jointIndex == activeJoint || jointIndex >= ENCODER_TOTAL_NUM ||
            ((skipJointMask & (1UL << jointIndex)) != 0)) {
            continue;
        }

        JointActuatorGroup group;
        if (getJointActuatorGroup(jointIndex, &group)) {
            for (uint8_t actuatorIndex = 0; actuatorIndex < group.count; actuatorIndex++) {
                const uint8_t motorChannel = group.actuators[actuatorIndex].motorChannel;
                int32_t holdAbs = 0;
                if (!readLatestMotorServoAbs(sharedData, motorChannel, &holdAbs)) {
                    FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL hold feedback missing J%02u M%02u\r\n",
                                  jointIndex,
                                  motorChannel);
                    return false;
                }
                const MotorMapItem& map = motorMap[motorChannel];
                if (!appendServoTargetLocal(batch,
                                            map.busIndex,
                                            map.servoID,
                                            clampToServoPosLocal(holdAbs),
                                            params.servoSpeed,
                                            params.servoAcc)) {
                    FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL append hold target J%02u M%02u abs=%ld\r\n",
                                  jointIndex,
                                  motorChannel,
                                  (long)holdAbs);
                    return false;
                }
            }
        } else {
            const uint8_t bus = jointMap[jointIndex].busIndex;
            const uint8_t id = jointMap[jointIndex].servoID;
            int32_t holdAbs = 0;
            if (!readLatestServoAbs(sharedData, bus, id, &holdAbs)) {
                FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL hold feedback missing J%02u bus%u id%u\r\n",
                              jointIndex,
                              bus,
                              id);
                return false;
            }
            if (!appendServoTargetLocal(batch,
                                        bus,
                                        id,
                                        clampToServoPosLocal(holdAbs),
                                        params.servoSpeed,
                                        params.servoAcc)) {
                FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL append hold target J%02u abs=%ld\r\n",
                              jointIndex,
                              (long)holdAbs);
                return false;
            }
        }
    }

    return true;
}

static bool getDualAntagonistAutoCalibrationMotors(uint8_t jointIndex,
                                                   uint8_t* primaryMotor,
                                                   uint8_t* secondaryMotor)
{
    // Apply the antagonist calibration flow to every dual-actuator joint
    // described by jointActuatorGroups, including thumb J16/J20.
    if (!primaryMotor || !secondaryMotor || jointIndex >= ENCODER_TOTAL_NUM) {
        return false;
    }

    JointActuatorGroup group;
    if (!getJointActuatorGroup(jointIndex, &group) || group.count < 2) {
        return false;
    }
    if (group.actuators[0].sign <= 0 || group.actuators[1].sign >= 0) {
        return false;
    }
    if (!isMotorChannelValid(group.actuators[0].motorChannel) ||
        !isMotorChannelValid(group.actuators[1].motorChannel)) {
        return false;
    }

    *primaryMotor = group.actuators[0].motorChannel;
    *secondaryMotor = group.actuators[1].motorChannel;
    return true;
}

static bool appendMotorCalibrationTargetToBatch(ServoTargetBatch_t* batch,
                                                uint8_t motorChannel,
                                                int32_t targetAbs,
                                                const AutoCalibrationParams& params)
{
    if (!batch || !isMotorChannelValid(motorChannel)) {
        return false;
    }
    const MotorMapItem& map = motorMap[motorChannel];
    return appendServoTargetLocal(batch,
                                  map.busIndex,
                                  map.servoID,
                                  clampToServoPosLocal(targetAbs),
                                  params.servoSpeed,
                                  params.servoAcc);
}

static bool publishMotorCalibrationTarget(TaskSharedData_t* sharedData,
                                          uint8_t motorChannel,
                                          int32_t targetAbs,
                                          const AutoCalibrationParams& params)
{
    if (!isServoCommandInsideLimit(targetAbs)) {
        FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL motor target outside limit M%02u target=%ld\r\n",
                      motorChannel,
                      (long)targetAbs);
        return false;
    }

    ServoTargetBatch_t batch;
    memset(&batch, 0, sizeof(batch));
    batch.source = SERVO_TARGET_OWNER_CALIBRATION;
    batch.timestamp = millis();
    if (!appendMotorCalibrationTargetToBatch(&batch, motorChannel, targetAbs, params)) {
        return false;
    }
    return publishCalibrationBatch(sharedData, &batch);
}

// Explicit calibration target publishing: do not rewrite the target from
// feedback here; trajectory generation owns the command.
static bool publishMotorCalibrationTargetDirect(TaskSharedData_t* sharedData,
                                                uint8_t motorChannel,
                                                int32_t targetAbs,
                                                const AutoCalibrationParams& params)
{
    if (!isServoCommandInsideLimit(targetAbs)) {
        FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL motor target outside limit M%02u target=%ld\r\n",
                      motorChannel,
                      (long)targetAbs);
        return false;
    }

    ServoTargetBatch_t batch;
    memset(&batch, 0, sizeof(batch));
    batch.source = SERVO_TARGET_OWNER_CALIBRATION;
    batch.timestamp = millis();
    if (!appendMotorCalibrationTargetToBatch(&batch, motorChannel, targetAbs, params)) {
        return false;
    }
    return publishCalibrationBatch(sharedData, &batch);
}

static bool publishDualMotorCalibrationTargets(TaskSharedData_t* sharedData,
                                               uint8_t movingMotor,
                                               int32_t movingTargetAbs,
                                               uint8_t holdMotor,
                                               int32_t holdTargetAbs,
                                               const AutoCalibrationParams& params);

static bool publishDualMotorCalibrationTargets(TaskSharedData_t* sharedData,
                                               uint8_t movingMotor,
                                               int32_t movingTargetAbs,
                                               uint8_t holdMotor,
                                               int32_t holdTargetAbs,
                                               const AutoCalibrationParams& params)
{
    if (!isServoCommandInsideLimit(movingTargetAbs) ||
        !isServoCommandInsideLimit(holdTargetAbs)) {
        FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL dual target outside limit moving=M%02u target=%ld hold=M%02u target=%ld\r\n",
                      movingMotor,
                      (long)movingTargetAbs,
                      holdMotor,
                      (long)holdTargetAbs);
        return false;
    }

    ServoTargetBatch_t batch;
    memset(&batch, 0, sizeof(batch));
    batch.source = SERVO_TARGET_OWNER_CALIBRATION;
    batch.timestamp = millis();
    if (!appendMotorCalibrationTargetToBatch(&batch, movingMotor, movingTargetAbs, params)) {
        return false;
    }
    if (holdMotor != movingMotor &&
        !appendMotorCalibrationTargetToBatch(&batch, holdMotor, holdTargetAbs, params)) {
        return false;
    }
    return publishCalibrationBatch(sharedData, &batch);
}

static bool waitMotorTargetReached(TaskSharedData_t* sharedData,
                                   uint8_t jointIndex,
                                   const char* phase,
                                   uint8_t motorChannel,
                                   int32_t targetAbs,
                                   const AutoCalibrationParams& params)
{
    uint32_t lastTraceMs = 0;
    while (autoCalibrationStillOwnsControl(sharedData)) {
        // This helper is only used by DUAL_SLACK_SECONDARY. The 3200-count
        // release is an explicitly bounded trajectory generated by the
        // calibration state machine.
        if (!publishMotorCalibrationTargetDirect(sharedData,
                                                 motorChannel,
                                                 targetAbs,
                                                 params)) {
            return false;
        }

        int32_t currentAbs = 0;
        if (readLatestMotorServoAbs(sharedData, motorChannel, &currentAbs)) {
            const uint32_t nowMs = millis();
            const int32_t absErr = abs32(currentAbs - targetAbs);
            if (absErr <= params.zeroReachAbsTolerance) {
                FW_LOG_INFO_PRINTF("[AUTO CALIB] %s_REACHED J%02u M%02u targetAbs=%ld currentAbs=%ld absErr=%ld\r\n",
                              phase,
                              jointIndex,
                              motorChannel,
                              (long)targetAbs,
                              (long)currentAbs,
                              (long)absErr);
                return true;
            }

            if ((uint32_t)(nowMs - lastTraceMs) >= params.traceMs) {
                lastTraceMs = nowMs;
                FW_LOG_INFO_PRINTF("[AUTO CALIB] %s_WAIT J%02u M%02u targetAbs=%ld currentAbs=%ld absErr=%ld/%ld\r\n",
                              phase,
                              jointIndex,
                              motorChannel,
                              (long)targetAbs,
                              (long)currentAbs,
                              (long)absErr,
                              (long)params.zeroReachAbsTolerance);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(params.loopMs));
    }
    return false;
}

static bool waitDualInitialFeedback(TaskSharedData_t* sharedData,
                                    uint8_t jointIndex,
                                    uint8_t primaryMotor,
                                    uint8_t secondaryMotor,
                                    int32_t* primaryAbs,
                                    int32_t* secondaryAbs,
                                    uint16_t* encoderRaw,
                                    int32_t* encoderOriented,
                                    const AutoCalibrationParams& params)
{
    (void)params;
    uint32_t lastTraceMs = 0;
    while (autoCalibrationStillOwnsControl(sharedData)) {
        const bool primaryOk =
            readLatestMotorServoAbs(sharedData, primaryMotor, primaryAbs);
        const bool secondaryOk =
            readLatestMotorServoAbs(sharedData, secondaryMotor, secondaryAbs);
        const bool encoderOk =
            readLatestEncoderSample(sharedData, jointIndex, encoderRaw, encoderOriented);
        if (primaryOk && secondaryOk && encoderOk) {
            return true;
        }

        const uint32_t nowMs = millis();
        if ((uint32_t)(nowMs - lastTraceMs) >= 200) {
            lastTraceMs = nowMs;
            FW_LOG_INFO_PRINTF("[AUTO CALIB] DUAL_INIT_WAIT J%02u M%02u=%u M%02u=%u encoder=%u\r\n",
                          jointIndex,
                          primaryMotor,
                          primaryOk ? 1 : 0,
                          secondaryMotor,
                          secondaryOk ? 1 : 0,
                          encoderOk ? 1 : 0);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    return false;
}

static bool findDualTightPoint(TaskSharedData_t* sharedData,
                               uint8_t jointIndex,
                               const char* phase,
                               uint8_t motorChannel,
                               int8_t motorShortenSign,
                               int32_t startAbs,
                               int32_t startEncoder,
                               int16_t loadThreshold,
                               bool holdOtherMotor,
                               uint8_t holdMotorChannel,
                               int32_t holdMotorAbs,
                               const AutoCalibrationParams& params,
                               int32_t* outTightAbs,
                               uint16_t* outTightRaw,
                               int32_t* outTightEncoder,
                               int16_t* outTightLoad)
{
    int32_t commandAbs = startAbs;
    int32_t commandRemainder = 0;
    uint32_t lastCommandMs = millis();
    uint32_t lastTraceMs = 0;
    uint32_t windowStartMs = millis();
    int32_t windowStartAbs = startAbs;
    int32_t windowStartEncoder = startEncoder;
    uint8_t angleStallStable = 0;

    while (autoCalibrationStillOwnsControl(sharedData)) {
        const uint32_t nowMs = millis();
        uint16_t currentRaw = 0;
        int32_t currentEncoder = 0;
        int32_t currentAbs = 0;
        if (!readLatestEncoderSample(sharedData, jointIndex, &currentRaw, &currentEncoder) ||
            !readLatestMotorServoAbs(sharedData, motorChannel, &currentAbs)) {
            FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL %s lost feedback J%02u M%02u\r\n",
                          phase,
                          jointIndex,
                          motorChannel);
            return false;
        }

        int16_t loadRaw = 0;
        const bool hasLoad = readLatestMotorServoLoad(sharedData, motorChannel, &loadRaw);
        const int16_t loadMag = hasLoad ? servoLoadMagnitude(loadRaw) : 0;
        const int32_t totalAbsDelta = abs32(currentAbs - startAbs);
        const int32_t totalEncoderDelta =
            calibrationWrapEncoderDelta(currentEncoder, startEncoder);
        const float totalAngleDeltaDeg =
            fabsf(calibrationEncoderCountToDeg(totalEncoderDelta));
        const bool loadHigh = hasLoad && loadMag >= loadThreshold;
        bool tightFoundByStall = false;

        if ((uint32_t)(nowMs - windowStartMs) >= params.windowMs) {
            const int32_t windowAbsDelta = abs32(currentAbs - windowStartAbs);
            const int32_t windowEncoderDelta =
                calibrationWrapEncoderDelta(currentEncoder, windowStartEncoder);
            const float windowAngleDeltaDeg =
                fabsf(calibrationEncoderCountToDeg(windowEncoderDelta));
            const bool servoStillTightening =
                windowAbsDelta >= params.windowServoDeltaCounts;
            const bool angleStalled =
                windowAngleDeltaDeg <= params.stallActualDeg;
            if (angleStalled) {
                if (angleStallStable < 255) {
                    angleStallStable++;
                }
            } else {
                angleStallStable = 0;
            }

            FW_LOG_INFO_PRINTF("[AUTO CALIB] CHECK_%s J%02u M%02u abs=%ld raw=%u oriented=%ld dCount=%ld winCount=%ld winAbs=%ld/%ld winDeg=%.3f/%.3f totalDeg=%.3f load=%d/%d cond servo=%u angle=%u angleStable=%u/%u load=%u hold=%u\r\n",
                          phase,
                          jointIndex,
                          motorChannel,
                          (long)currentAbs,
                          (unsigned)currentRaw,
                          (long)currentEncoder,
                          (long)totalEncoderDelta,
                          (long)windowEncoderDelta,
                          (long)windowAbsDelta,
                          (long)params.windowServoDeltaCounts,
                          windowAngleDeltaDeg,
                          params.stallActualDeg,
                          totalAngleDeltaDeg,
                          loadMag,
                          loadThreshold,
                          servoStillTightening ? 1 : 0,
                          angleStalled ? 1 : 0,
                          (unsigned)angleStallStable,
                          (unsigned)kAngleStallStableSamples,
                          loadHigh ? 1 : 0,
                          holdOtherMotor ? 1 : 0);

            tightFoundByStall =
                loadHigh &&
                servoStillTightening &&
                angleStallStable >= kAngleStallStableSamples;
            windowStartAbs = currentAbs;
            windowStartEncoder = currentEncoder;
            windowStartMs = nowMs;
        }

        if (tightFoundByStall) {
            if (outTightAbs) {
                *outTightAbs = currentAbs;
            }
            if (outTightRaw) {
                *outTightRaw = currentRaw;
            }
            if (outTightEncoder) {
                *outTightEncoder = currentEncoder;
            }
            if (outTightLoad) {
                *outTightLoad = loadMag;
            }
            FW_LOG_INFO_PRINTF("[AUTO CALIB] %s_FOUND J%02u M%02u raw=%u oriented=%ld abs=%ld shortenSign=%d load=%d byStall=%u totalDeg=%.3f loadThreshold=%d hold=%u\r\n",
                          phase,
                          jointIndex,
                          motorChannel,
                          (unsigned)currentRaw,
                          (long)currentEncoder,
                          (long)currentAbs,
                          (int)motorShortenSign,
                          loadMag,
                          tightFoundByStall ? 1 : 0,
                          totalAngleDeltaDeg,
                          loadThreshold,
                          holdOtherMotor ? 1 : 0);
            if (holdOtherMotor) {
                if (!publishDualMotorCalibrationTargets(sharedData,
                                                        motorChannel,
                                                        currentAbs,
                                                        holdMotorChannel,
                                                        holdMotorAbs,
                                                        params)) {
                    return false;
                }
            } else if (!publishMotorCalibrationTarget(sharedData, motorChannel, currentAbs, params)) {
                return false;
            }
            return true;
        }

        const uint32_t dtMs = nowMs - lastCommandMs;
        if (dtMs >= params.loopMs) {
            const int32_t numerator =
                (int32_t)dtMs * params.tightenSpeedCountsPerSec + commandRemainder;
            const int32_t stepCounts = numerator / 1000;
            commandRemainder = numerator % 1000;
            if (stepCounts > 0) {
                commandAbs = motorApplyTendonDelta(motorChannel, commandAbs, stepCounts);
                if (holdOtherMotor) {
                    if (!publishDualMotorCalibrationTargets(sharedData,
                                                            motorChannel,
                                                            commandAbs,
                                                            holdMotorChannel,
                                                            holdMotorAbs,
                                                            params)) {
                        return false;
                    }
                } else {
                    if (!publishMotorCalibrationTarget(sharedData, motorChannel, commandAbs, params)) {
                        return false;
                    }
                }
            }
            lastCommandMs = nowMs;
        }

        if ((uint32_t)(nowMs - lastTraceMs) >= params.traceMs) {
            lastTraceMs = nowMs;
            FW_LOG_INFO_PRINTF("[AUTO CALIB] TRACE_%s J%02u M%02u cmd=%ld abs=%ld shortenSign=%d raw=%u oriented=%ld dCount=%ld totalAbs=%ld totalDeg=%.3f load=%d\r\n",
                          phase,
                          jointIndex,
                          motorChannel,
                          (long)commandAbs,
                          (long)currentAbs,
                          (int)motorShortenSign,
                          (unsigned)currentRaw,
                          (long)currentEncoder,
                          (long)totalEncoderDelta,
                          (long)totalAbsDelta,
                          totalAngleDeltaDeg,
                          loadMag);
        }

        vTaskDelay(pdMS_TO_TICKS(params.loopMs));
    }
    FW_LOG_INFO_PRINTF("[AUTO CALIB] CANCEL %s J%02u M%02u state=%u owner=%u enabled=%u\r\n",
                  phase,
                  jointIndex,
                  motorChannel,
                  sharedData ? (unsigned)sharedData->system_state : 255,
                  sharedData ? (unsigned)sharedData->servo_target_owner : 255,
                  sharedData ? (unsigned)sharedData->control_enabled : 255);
    return false;
}

static int8_t workspaceCacheIndex(uint8_t jointIndex)
{
    for (uint8_t i = 0; i < kWorkspaceDualJointsCount; ++i) {
        if (kWorkspaceDualJoints[i] == jointIndex) {
            return (int8_t)i;
        }
    }
    return -1;
}

static bool isWorkspaceDualJoint(uint8_t jointIndex)
{
    return workspaceCacheIndex(jointIndex) >= 0;
}

static void clearDualWorkspaceCache(void)
{
    memset(g_dualWorkspaceCache, 0, sizeof(g_dualWorkspaceCache));
}

enum DualWorkspacePairAvailability : uint8_t {
    DUAL_WORKSPACE_PAIR_READY = 0,
    DUAL_WORKSPACE_PAIR_ALL_OFFLINE,
    DUAL_WORKSPACE_PAIR_PARTIAL_ONLINE,
    DUAL_WORKSPACE_PAIR_TURN_UNKNOWN,
    DUAL_WORKSPACE_PAIR_FEEDBACK_STALE,
    DUAL_WORKSPACE_PAIR_MAP_INVALID,
};

static DualWorkspacePairAvailability classifyDualWorkspacePair(
    TaskSharedData_t* sharedData,
    uint8_t jointIndex,
    uint8_t* outPrimaryMotor,
    uint8_t* outSecondaryMotor,
    uint32_t* outMotorMask)
{
    uint8_t primaryMotor = 0;
    uint8_t secondaryMotor = 0;
    if (!sharedData ||
        !getDualAntagonistAutoCalibrationMotors(jointIndex,
                                                &primaryMotor,
                                                &secondaryMotor)) {
        return DUAL_WORKSPACE_PAIR_MAP_INVALID;
    }
    const uint32_t motorMask =
        (1UL << primaryMotor) | (1UL << secondaryMotor);
    if (outPrimaryMotor) *outPrimaryMotor = primaryMotor;
    if (outSecondaryMotor) *outSecondaryMotor = secondaryMotor;
    if (outMotorMask) *outMotorMask = motorMask;

    ServoAngleData_t servoData;
    if (!sharedData->servoFeedbackQueue ||
        xQueuePeek(sharedData->servoFeedbackQueue, &servoData, 0) != pdTRUE ||
        (uint32_t)(millis() - servoData.timestamp) >
            kAutoFeedbackReadyServoTimeoutMs) {
        return DUAL_WORKSPACE_PAIR_FEEDBACK_STALE;
    }

    const bool primaryOnline = servoData.onlineStatus[primaryMotor] != 0;
    const bool secondaryOnline = servoData.onlineStatus[secondaryMotor] != 0;
    if (!primaryOnline && !secondaryOnline) {
        return DUAL_WORKSPACE_PAIR_ALL_OFFLINE;
    }
    if (primaryOnline != secondaryOnline) {
        return DUAL_WORKSPACE_PAIR_PARTIAL_ONLINE;
    }
    if ((sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap) & motorMask) != 0) {
        return DUAL_WORKSPACE_PAIR_TURN_UNKNOWN;
    }
    return DUAL_WORKSPACE_PAIR_READY;
}

static const char* workspacePairAvailabilityText(
    DualWorkspacePairAvailability availability)
{
    switch (availability) {
        case DUAL_WORKSPACE_PAIR_READY: return "ready";
        case DUAL_WORKSPACE_PAIR_ALL_OFFLINE: return "all-offline";
        case DUAL_WORKSPACE_PAIR_PARTIAL_ONLINE: return "partial-online";
        case DUAL_WORKSPACE_PAIR_TURN_UNKNOWN: return "turn-unknown";
        case DUAL_WORKSPACE_PAIR_FEEDBACK_STALE: return "feedback-stale";
        default: return "map-invalid";
    }
}

static bool buildDynamicWorkspaceMotorMask(
    TaskSharedData_t* sharedData,
    const uint8_t* activeJoints,
    uint8_t activeJointCount,
    uint32_t* outMotorMask)
{
    if (!sharedData || !activeJoints || !outMotorMask) return false;
    uint32_t motorMask = 0;
    for (uint8_t active = 0; active < activeJointCount; ++active) {
        const uint8_t targetJoint = activeJoints[active];
        if (!isWorkspaceDualJoint(targetJoint)) continue;
        for (uint8_t i = 0; i < kWorkspaceDualJointsCount; ++i) {
            const uint8_t candidateJoint = kWorkspaceDualJoints[i];
            uint8_t primaryMotor = 0;
            uint8_t secondaryMotor = 0;
            uint32_t pairMask = 0;
            const DualWorkspacePairAvailability availability =
                classifyDualWorkspacePair(sharedData,
                                          candidateJoint,
                                          &primaryMotor,
                                          &secondaryMotor,
                                          &pairMask);
            if (availability == DUAL_WORKSPACE_PAIR_READY) {
                motorMask |= pairMask;
                continue;
            }
            if (candidateJoint != targetJoint &&
                availability == DUAL_WORKSPACE_PAIR_ALL_OFFLINE) {
                FW_LOG_INFO_PRINTF(
                    "[AUTO CALIB] WORKSPACE_MASK_SKIP target=J%02u helper=J%02u "
                    "M%02u/M%02u reason=all-offline\r\n",
                    (unsigned)targetJoint,
                    (unsigned)candidateJoint,
                    (unsigned)primaryMotor,
                    (unsigned)secondaryMotor);
                continue;
            }
            FW_LOG_WARN_PRINTF(
                "[AUTO CALIB] WORKSPACE_MASK_REJECT target=J%02u joint=J%02u "
                "M%02u/M%02u reason=%s unknown=0x%08lX\r\n",
                (unsigned)targetJoint,
                (unsigned)candidateJoint,
                (unsigned)primaryMotor,
                (unsigned)secondaryMotor,
                workspacePairAvailabilityText(availability),
                (unsigned long)(sharedAtomicLoad(
                    &sharedData->multi_turn_unknown_bitmap) & pairMask));
            setCalibrationDiag(sharedData,
                               CALIB_DIAG_REASON_WORKSPACE_PREP_FAILED,
                               CALIB_DIAG_PHASE_DUAL_PRELOAD,
                               targetJoint,
                               primaryMotor,
                               candidateJoint,
                               (int32_t)availability,
                               secondaryMotor,
                               pairMask);
            return false;
        }
    }
    *outMotorMask = motorMask & validMotorChannelMask();
    return true;
}

static void setCalibrationWorkspaceReadMask(TaskSharedData_t* sharedData,
                                            uint32_t motorMask)
{
    if (!sharedData) {
        return;
    }
    SharedSafetyGuard guard;
    sharedAtomicStore(&sharedData->calibration_aux_motor_mask,
                      motorMask & validMotorChannelMask());
}

static uint8_t buildDualWorkspacePlan(TaskSharedData_t* sharedData,
                                      uint8_t targetJoint,
                                      DualWorkspacePlanStep* steps,
                                      uint8_t capacity)
{
    if (!steps || capacity < kWorkspaceDualHelperCount ||
        !isWorkspaceDualJoint(targetJoint)) {
        return 0;
    }

    uint8_t count = 0;
    // Establish all non-target fingers first. Lower-index fingers move from
    // low to high into A; higher-index fingers move from high to low into B.
    // The target is deliberately excluded: its calibration routine performs
    // the required slack/primary-tight sequence itself, or reuses an existing
    // B cache established earlier as a helper.
    for (uint8_t i = 0; i < kWorkspaceDualJointsCount; ++i) {
        const uint8_t jointIndex = kWorkspaceDualJoints[i];
        if (jointIndex >= targetJoint) {
            continue;
        }
        const DualWorkspacePairAvailability availability =
            classifyDualWorkspacePair(sharedData, jointIndex, NULL, NULL, NULL);
        if (availability == DUAL_WORKSPACE_PAIR_ALL_OFFLINE) {
            FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_PLAN_SKIP target=J%02u helper=J%02u reason=all-offline\r\n",
                               (unsigned)targetJoint,
                               (unsigned)jointIndex);
            continue;
        }
        if (availability != DUAL_WORKSPACE_PAIR_READY || count >= capacity) {
            FW_LOG_WARN_PRINTF("[AUTO CALIB] WORKSPACE_PLAN_REJECT target=J%02u helper=J%02u reason=%s\r\n",
                               (unsigned)targetJoint,
                               (unsigned)jointIndex,
                               workspacePairAvailabilityText(availability));
            return 0xFF;
        }
        steps[count++] = {jointIndex, DUAL_WORKSPACE_POSE_A};
    }

    for (int8_t i = (int8_t)kWorkspaceDualJointsCount - 1; i >= 0; --i) {
        const uint8_t jointIndex = kWorkspaceDualJoints[(uint8_t)i];
        if (jointIndex <= targetJoint) {
            continue;
        }
        const DualWorkspacePairAvailability availability =
            classifyDualWorkspacePair(sharedData, jointIndex, NULL, NULL, NULL);
        if (availability == DUAL_WORKSPACE_PAIR_ALL_OFFLINE) {
            FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_PLAN_SKIP target=J%02u helper=J%02u reason=all-offline\r\n",
                               (unsigned)targetJoint,
                               (unsigned)jointIndex);
            continue;
        }
        if (availability != DUAL_WORKSPACE_PAIR_READY || count >= capacity) {
            FW_LOG_WARN_PRINTF("[AUTO CALIB] WORKSPACE_PLAN_REJECT target=J%02u helper=J%02u reason=%s\r\n",
                               (unsigned)targetJoint,
                               (unsigned)jointIndex,
                               workspacePairAvailabilityText(availability));
            return 0xFF;
        }
        steps[count++] = {jointIndex, DUAL_WORKSPACE_POSE_B};
    }

    return count;
}

static void setWorkspacePlanFailureDiag(TaskSharedData_t* sharedData,
                                        uint8_t targetJoint,
                                        uint8_t helperJoint,
                                        DualWorkspacePose pose,
                                        DualWorkspacePlanStage stage)
{
    const uint8_t existingReason = calibrationDiagReason(sharedData);
    if (existingReason != CALIB_DIAG_REASON_NONE &&
        existingReason != CALIB_DIAG_REASON_WORKSPACE_PREP_FAILED) {
        return;
    }

    uint8_t primaryMotor = 0xFF;
    uint8_t secondaryMotor = 0xFF;
    uint32_t motorMask = 0;
    if (getDualAntagonistAutoCalibrationMotors(helperJoint,
                                               &primaryMotor,
                                               &secondaryMotor)) {
        motorMask = (1UL << primaryMotor) | (1UL << secondaryMotor);
    }
    setCalibrationDiag(sharedData,
                       CALIB_DIAG_REASON_WORKSPACE_PREP_FAILED,
                       CALIB_DIAG_PHASE_DUAL_PRELOAD,
                       targetJoint,
                       primaryMotor,
                       helperJoint,
                       (int32_t)pose,
                       (int32_t)stage,
                       motorMask);
}

static void logDualWorkspacePlan(uint8_t targetJoint,
                                 const DualWorkspacePlanStep* steps,
                                 uint8_t count)
{
    FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_PLAN target=J%02u helpers=%u\r\n",
                       (unsigned)targetJoint,
                       (unsigned)count);
    for (uint8_t i = 0; i < count; ++i) {
        FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_PLAN_HELPER target=J%02u joint=J%02u pose=%c\r\n",
                           (unsigned)targetJoint,
                           (unsigned)steps[i].jointIndex,
                           steps[i].pose == DUAL_WORKSPACE_POSE_A ? 'A' : 'B');
    }
}

static bool waitDualWorkspaceTargets(TaskSharedData_t* sharedData,
                                     uint8_t jointIndex,
                                     const char* phase,
                                     uint8_t primaryMotor,
                                     int32_t primaryTargetAbs,
                                     uint8_t secondaryMotor,
                                     int32_t secondaryTargetAbs,
                                     const AutoCalibrationParams& params)
{
    uint32_t lastTraceMs = 0;
    while (autoCalibrationStillOwnsControl(sharedData)) {
        int32_t primaryAbs = 0;
        int32_t secondaryAbs = 0;
        if (!readLatestMotorServoAbs(sharedData, primaryMotor, &primaryAbs) ||
            !readLatestMotorServoAbs(sharedData, secondaryMotor, &secondaryAbs)) {
            setCalibrationDiag(sharedData,
                               CALIB_DIAG_REASON_WORKSPACE_PREP_FAILED,
                               CALIB_DIAG_PHASE_DUAL_PRELOAD,
                               jointIndex,
                               primaryMotor,
                               primaryMotor,
                               secondaryMotor,
                               0,
                               (1UL << primaryMotor) | (1UL << secondaryMotor));
            return false;
        }
        if (!publishDualMotorCalibrationTargets(sharedData,
                                                primaryMotor,
                                                primaryTargetAbs,
                                                secondaryMotor,
                                                secondaryTargetAbs,
                                                params)) {
            setCalibrationDiag(sharedData,
                               CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED,
                               CALIB_DIAG_PHASE_DUAL_PRELOAD,
                               jointIndex,
                               primaryMotor,
                               primaryTargetAbs,
                               secondaryTargetAbs,
                               0,
                               (1UL << primaryMotor) | (1UL << secondaryMotor));
            FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_%s publish failed J%02u M%02u/M%02u\r\n",
                               phase,
                               (unsigned)jointIndex,
                               (unsigned)primaryMotor,
                               (unsigned)secondaryMotor);
            return false;
        }

        const int32_t primaryErr = abs32(primaryAbs - primaryTargetAbs);
        const int32_t secondaryErr = abs32(secondaryAbs - secondaryTargetAbs);
        if (primaryErr <= kWorkspaceTargetToleranceCounts &&
            secondaryErr <= kWorkspaceTargetToleranceCounts) {
            FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_%s_REACHED J%02u M%02u=%ld/%ld M%02u=%ld/%ld\r\n",
                               phase,
                               (unsigned)jointIndex,
                               (unsigned)primaryMotor,
                               (long)primaryAbs,
                               (long)primaryTargetAbs,
                               (unsigned)secondaryMotor,
                               (long)secondaryAbs,
                               (long)secondaryTargetAbs);
            return true;
        }

        const uint32_t nowMs = millis();
        if ((uint32_t)(nowMs - lastTraceMs) >= params.traceMs) {
            lastTraceMs = nowMs;
            FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_%s_WAIT J%02u M%02u err=%ld M%02u err=%ld\r\n",
                               phase,
                               (unsigned)jointIndex,
                               (unsigned)primaryMotor,
                               (long)primaryErr,
                               (unsigned)secondaryMotor,
                               (long)secondaryErr);
        }
        vTaskDelay(pdMS_TO_TICKS(params.loopMs));
    }
    setCalibrationDiag(sharedData,
                       CALIB_DIAG_REASON_CONTROL_LOST,
                       CALIB_DIAG_PHASE_DUAL_PRELOAD,
                       jointIndex,
                       primaryMotor,
                       sharedData ? sharedData->system_state : -1,
                       sharedData ? sharedData->servo_target_owner : -1,
                       sharedData ? sharedData->control_enabled : -1,
                       sharedData ? sharedAtomicLoad(&sharedData->active_joint_mask) : 0);
    FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_%s cancel J%02u state=%u owner=%u enabled=%u\r\n",
                       phase,
                       (unsigned)jointIndex,
                       sharedData ? (unsigned)sharedData->system_state : 255U,
                       sharedData ? (unsigned)sharedData->servo_target_owner : 255U,
                       sharedData ? (unsigned)sharedData->control_enabled : 255U);
    return false;
}

static bool waitWorkspaceFeedbackKnown(TaskSharedData_t* sharedData,
                                       uint8_t jointIndex,
                                       uint8_t primaryMotor,
                                       uint8_t secondaryMotor,
                                       int32_t* primaryAbs,
                                       int32_t* secondaryAbs)
{
    const uint32_t motorMask = (1UL << primaryMotor) | (1UL << secondaryMotor);
    const uint32_t startMs = millis();
    while (autoCalibrationStillOwnsControl(sharedData)) {
        const uint32_t unknownMask =
            sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap) & motorMask;
        if (unknownMask != 0) {
            const uint8_t firstUnknown = (unknownMask & (1UL << primaryMotor))
                ? primaryMotor : secondaryMotor;
            setCalibrationDiag(sharedData,
                               CALIB_DIAG_REASON_WORKSPACE_PREP_FAILED,
                               CALIB_DIAG_PHASE_DUAL_PRELOAD,
                               jointIndex,
                               firstUnknown,
                               (int32_t)unknownMask,
                               0,
                               0,
                               unknownMask);
            FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE fail Turn UNKNOWN J%02u mask=0x%08lX; New Device Reset required\r\n",
                               (unsigned)jointIndex,
                               (unsigned long)unknownMask);
            return false;
        }

        const bool primaryOk = readLatestMotorServoAbs(sharedData, primaryMotor, primaryAbs);
        const bool secondaryOk = readLatestMotorServoAbs(sharedData, secondaryMotor, secondaryAbs);
        int16_t primaryLoad = 0;
        int16_t secondaryLoad = 0;
        const bool primaryLoadOk = readLatestMotorServoLoad(sharedData, primaryMotor, &primaryLoad);
        const bool secondaryLoadOk = readLatestMotorServoLoad(sharedData, secondaryMotor, &secondaryLoad);
        if (primaryOk && secondaryOk && primaryLoadOk && secondaryLoadOk) {
            return true;
        }

        if ((uint32_t)(millis() - startMs) >= kAutoInitialFeedbackWaitMs) {
            const uint32_t missingMask =
                (primaryOk && primaryLoadOk ? 0UL : (1UL << primaryMotor)) |
                (secondaryOk && secondaryLoadOk ? 0UL : (1UL << secondaryMotor));
            setCalibrationDiag(sharedData,
                               CALIB_DIAG_REASON_WORKSPACE_PREP_FAILED,
                               CALIB_DIAG_PHASE_DUAL_PRELOAD,
                               jointIndex,
                               primaryOk && primaryLoadOk ? secondaryMotor : primaryMotor,
                               primaryOk ? 1 : 0,
                               secondaryOk ? 1 : 0,
                               0,
                               missingMask);
            FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE fail feedback J%02u primary=%u secondary=%u load=%u/%u\r\n",
                               (unsigned)jointIndex,
                               primaryOk ? 1U : 0U,
                               secondaryOk ? 1U : 0U,
                               primaryLoadOk ? 1U : 0U,
                               secondaryLoadOk ? 1U : 0U);
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    setCalibrationDiag(sharedData,
                       CALIB_DIAG_REASON_CONTROL_LOST,
                       CALIB_DIAG_PHASE_DUAL_PRELOAD,
                       jointIndex,
                       primaryMotor,
                       sharedData ? sharedData->system_state : -1,
                       sharedData ? sharedData->servo_target_owner : -1,
                       sharedData ? sharedData->control_enabled : -1,
                       sharedData ? sharedAtomicLoad(&sharedData->active_joint_mask) : 0);
    FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE feedback cancel J%02u M%02u/M%02u state=%u owner=%u enabled=%u\r\n",
                       (unsigned)jointIndex,
                       (unsigned)primaryMotor,
                       (unsigned)secondaryMotor,
                       sharedData ? (unsigned)sharedData->system_state : 255U,
                       sharedData ? (unsigned)sharedData->servo_target_owner : 255U,
                       sharedData ? (unsigned)sharedData->control_enabled : 255U);
    return false;
}

static bool preflightDualWorkspacePlan(TaskSharedData_t* sharedData,
                                       uint8_t targetJoint,
                                       const DualWorkspacePlanStep* steps,
                                       uint8_t count)
{
    for (uint8_t i = 0; i < count; ++i) {
        const DualWorkspacePlanStep& step = steps[i];
        uint8_t primaryMotor = 0;
        uint8_t secondaryMotor = 0;
        if (!getDualAntagonistAutoCalibrationMotors(step.jointIndex,
                                                    &primaryMotor,
                                                    &secondaryMotor)) {
            setWorkspacePlanFailureDiag(sharedData,
                                        targetJoint,
                                        step.jointIndex,
                                        step.pose,
                                        DUAL_WORKSPACE_PLAN_STAGE_PREFLIGHT);
            FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_PREFLIGHT fail target=J%02u helper=J%02u no dual map\r\n",
                               (unsigned)targetJoint,
                               (unsigned)step.jointIndex);
            return false;
        }

        int32_t primaryAbs = 0;
        int32_t secondaryAbs = 0;
        FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_PREFLIGHT target=J%02u role=HELPER joint=J%02u M%02u/M%02u\r\n",
                           (unsigned)targetJoint,
                           (unsigned)step.jointIndex,
                           (unsigned)primaryMotor,
                           (unsigned)secondaryMotor);
        if (!waitWorkspaceFeedbackKnown(sharedData,
                                        step.jointIndex,
                                        primaryMotor,
                                        secondaryMotor,
                                        &primaryAbs,
                                        &secondaryAbs)) {
            setWorkspacePlanFailureDiag(sharedData,
                                        targetJoint,
                                        step.jointIndex,
                                        step.pose,
                                        DUAL_WORKSPACE_PLAN_STAGE_PREFLIGHT);
            FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_PREFLIGHT fail target=J%02u helper=J%02u pose=%c\r\n",
                               (unsigned)targetJoint,
                               (unsigned)step.jointIndex,
                               step.pose == DUAL_WORKSPACE_POSE_A ? 'A' : 'B');
            return false;
        }
    }
    return true;
}

static bool tightenWorkspaceMotor(TaskSharedData_t* sharedData,
                                  uint8_t jointIndex,
                                  const char* phase,
                                  uint8_t motor,
                                  uint8_t holdMotor,
                                  int32_t holdAbs,
                                  const AutoCalibrationParams& params,
                                  int32_t* outTightAbs)
{
    int32_t startAbs = 0;
    if (!readLatestMotorServoAbs(sharedData, motor, &startAbs)) {
        return false;
    }
    int32_t commandAbs = startAbs;
    int32_t commandRemainder = 0;
    uint32_t lastCommandMs = millis();
    uint32_t lastTraceMs = 0;
    while (autoCalibrationStillOwnsControl(sharedData)) {
        int32_t currentAbs = 0;
        int16_t loadRaw = 0;
        if (!readLatestMotorServoAbs(sharedData, motor, &currentAbs) ||
            !readLatestMotorServoLoad(sharedData, motor, &loadRaw)) {
            return false;
        }
        const int16_t loadMag = servoLoadMagnitude(loadRaw);
        if (loadMag >= kWorkspaceTightLoadThreshold) {
            if (!publishDualMotorCalibrationTargets(sharedData,
                                                    motor,
                                                    currentAbs,
                                                    holdMotor,
                                                    holdAbs,
                                                    params)) {
                setCalibrationDiag(sharedData,
                                   CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED,
                                   CALIB_DIAG_PHASE_DUAL_PRELOAD,
                                   jointIndex,
                                   motor,
                                   currentAbs,
                                   holdAbs,
                                   0,
                                   (1UL << motor) | (1UL << holdMotor));
                return false;
            }
            if (outTightAbs) {
                *outTightAbs = currentAbs;
            }
            FW_LOG_INFO_PRINTF("[AUTO CALIB] %s_FOUND J%02u M%02u abs=%ld load=%d/%d hold=M%02u abs=%ld\r\n",
                               phase,
                               (unsigned)jointIndex,
                               (unsigned)motor,
                               (long)currentAbs,
                               (int)loadMag,
                               (int)kWorkspaceTightLoadThreshold,
                               (unsigned)holdMotor,
                               (long)holdAbs);
            return true;
        }
        const uint32_t nowMs = millis();
        const uint32_t dtMs = nowMs - lastCommandMs;
        if (dtMs >= params.loopMs) {
            const int32_t numerator =
                (int32_t)dtMs * params.tightenSpeedCountsPerSec + commandRemainder;
            const int32_t stepCounts = numerator / 1000;
            commandRemainder = numerator % 1000;
            if (stepCounts > 0) {
                commandAbs = motorApplyTendonDelta(motor, commandAbs, stepCounts);
                if (!publishDualMotorCalibrationTargets(sharedData,
                                                        motor,
                                                        commandAbs,
                                                        holdMotor,
                                                        holdAbs,
                                                        params)) {
                    setCalibrationDiag(sharedData,
                                       CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED,
                                       CALIB_DIAG_PHASE_DUAL_PRELOAD,
                                       jointIndex,
                                       motor,
                                       commandAbs,
                                       holdAbs,
                                       0,
                                       (1UL << motor) | (1UL << holdMotor));
                    return false;
                }
            }
            lastCommandMs = nowMs;
        }
        if ((uint32_t)(nowMs - lastTraceMs) >= params.traceMs) {
            lastTraceMs = nowMs;
            FW_LOG_INFO_PRINTF("[AUTO CALIB] TRACE_%s J%02u M%02u cmd=%ld abs=%ld load=%d/%d\r\n",
                               phase,
                               (unsigned)jointIndex,
                               (unsigned)motor,
                               (long)commandAbs,
                               (long)currentAbs,
                               (int)loadMag,
                               (int)kWorkspaceTightLoadThreshold);
        }
        vTaskDelay(pdMS_TO_TICKS(params.loopMs));
    }
    setCalibrationDiag(sharedData,
                       CALIB_DIAG_REASON_CONTROL_LOST,
                       CALIB_DIAG_PHASE_DUAL_PRELOAD,
                       jointIndex,
                       motor,
                       sharedData ? sharedData->system_state : -1,
                       sharedData ? sharedData->servo_target_owner : -1,
                       sharedData ? sharedData->control_enabled : -1,
                       sharedData ? sharedAtomicLoad(&sharedData->active_joint_mask) : 0);
    FW_LOG_INFO_PRINTF("[AUTO CALIB] %s cancel J%02u M%02u state=%u owner=%u enabled=%u\r\n",
                       phase,
                       (unsigned)jointIndex,
                       (unsigned)motor,
                       sharedData ? (unsigned)sharedData->system_state : 255U,
                       sharedData ? (unsigned)sharedData->servo_target_owner : 255U,
                       sharedData ? (unsigned)sharedData->control_enabled : 255U);
    return false;
}

static bool ensureWorkspaceDualPose(TaskSharedData_t* sharedData,
                                    uint8_t jointIndex,
                                    DualWorkspacePose desiredPose,
                                    const AutoCalibrationParams& params)
{
    const int8_t cacheIndex = workspaceCacheIndex(jointIndex);
    if (cacheIndex < 0 || desiredPose == DUAL_WORKSPACE_POSE_NONE) {
        return false;
    }

    uint8_t primaryMotor = 0;
    uint8_t secondaryMotor = 0;
    if (!getDualAntagonistAutoCalibrationMotors(jointIndex, &primaryMotor, &secondaryMotor)) {
        return false;
    }
    int32_t primaryAbs = 0;
    int32_t secondaryAbs = 0;
    if (!waitWorkspaceFeedbackKnown(sharedData,
                                    jointIndex,
                                    primaryMotor,
                                    secondaryMotor,
                                    &primaryAbs,
                                    &secondaryAbs)) {
        return false;
    }

    DualWorkspaceCacheEntry* entry = &g_dualWorkspaceCache[(uint8_t)cacheIndex];
    if (entry->valid &&
        (entry->primaryMotor != primaryMotor || entry->secondaryMotor != secondaryMotor)) {
        memset(entry, 0, sizeof(*entry));
    }
    entry->primaryMotor = primaryMotor;
    entry->secondaryMotor = secondaryMotor;

    if (entry->valid && entry->currentPose == desiredPose) {
        const int32_t primaryTarget = desiredPose == DUAL_WORKSPACE_POSE_A
            ? entry->primaryReleaseAbs : entry->primaryTightAbs;
        const int32_t secondaryTarget = desiredPose == DUAL_WORKSPACE_POSE_A
            ? entry->secondaryTightAbs : entry->secondaryReleaseAbs;
        const int32_t primaryError = abs32(primaryAbs - primaryTarget);
        const int32_t secondaryError = abs32(secondaryAbs - secondaryTarget);
        if (primaryError <= kWorkspaceTargetToleranceCounts &&
            secondaryError <= kWorkspaceTargetToleranceCounts) {
            // The pose was already reached by an earlier workspace step or a
            // completed calibration. Reuse it without issuing servo targets.
            FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_REUSE_NO_COMMAND J%02u pose=%c M%02u=%ld/%ld err=%ld M%02u=%ld/%ld err=%ld\r\n",
                               (unsigned)jointIndex,
                               desiredPose == DUAL_WORKSPACE_POSE_A ? 'A' : 'B',
                               (unsigned)primaryMotor,
                               (long)primaryAbs,
                               (long)primaryTarget,
                               (long)primaryError,
                               (unsigned)secondaryMotor,
                               (long)secondaryAbs,
                               (long)secondaryTarget,
                               (long)secondaryError);
            return true;
        }

        // The RAM cache cannot describe the current mechanical pose anymore.
        // Rebuild the requested pose from measured feedback instead of issuing
        // targets from stale cached ABS values.
        FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_REUSE_INVALIDATE J%02u pose=%c M%02u err=%ld M%02u err=%ld tolerance=%ld\r\n",
                           (unsigned)jointIndex,
                           desiredPose == DUAL_WORKSPACE_POSE_A ? 'A' : 'B',
                           (unsigned)primaryMotor,
                           (long)primaryError,
                           (unsigned)secondaryMotor,
                           (long)secondaryError,
                           (long)kWorkspaceTargetToleranceCounts);
        memset(entry, 0, sizeof(*entry));
        entry->primaryMotor = primaryMotor;
        entry->secondaryMotor = secondaryMotor;
    }

    if (entry->valid && entry->currentPose == DUAL_WORKSPACE_POSE_A &&
        desiredPose == DUAL_WORKSPACE_POSE_B) {
        entry->secondaryReleaseAbs = motorApplyTendonDelta(secondaryMotor,
                                                            entry->secondaryTightAbs,
                                                            -kWorkspaceReleaseCounts);
        if (!waitDualWorkspaceTargets(sharedData, jointIndex, "A_TO_B_RELEASE",
                                      primaryMotor, entry->primaryReleaseAbs,
                                      secondaryMotor, entry->secondaryReleaseAbs, params) ||
            !tightenWorkspaceMotor(sharedData, jointIndex, "WORKSPACE_B_TIGHT",
                                   primaryMotor, secondaryMotor, entry->secondaryReleaseAbs,
                                   params, &entry->primaryTightAbs)) {
            return false;
        }
        entry->currentPose = DUAL_WORKSPACE_POSE_B;
        entry->valid = 1;
        return true;
    }

    if (entry->valid && entry->currentPose == DUAL_WORKSPACE_POSE_B &&
        desiredPose == DUAL_WORKSPACE_POSE_A) {
        entry->primaryReleaseAbs = motorApplyTendonDelta(primaryMotor,
                                                          entry->primaryTightAbs,
                                                          -kWorkspaceReleaseCounts);
        if (!waitDualWorkspaceTargets(sharedData, jointIndex, "B_TO_A_RELEASE",
                                      primaryMotor, entry->primaryReleaseAbs,
                                      secondaryMotor, entry->secondaryReleaseAbs, params) ||
            !tightenWorkspaceMotor(sharedData, jointIndex, "WORKSPACE_A_TIGHT",
                                   secondaryMotor, primaryMotor, entry->primaryReleaseAbs,
                                   params, &entry->secondaryTightAbs)) {
            return false;
        }
        entry->currentPose = DUAL_WORKSPACE_POSE_A;
        entry->valid = 1;
        return true;
    }

    // No trusted in-sequence cache exists. Establish the requested workspace
    // directly from the measured pose: release one tendon by 3200 counts
    // first, keep its antagonist fixed, then tighten that antagonist to the
    // workspace load threshold. This keeps the physical sequence consistent
    // with the requested A/B pose and avoids an unintended pre-tighten of the
    // tendon that is supposed to be released.
    const uint8_t releaseMotor = desiredPose == DUAL_WORKSPACE_POSE_A
        ? primaryMotor : secondaryMotor;
    const uint8_t tightenMotor = desiredPose == DUAL_WORKSPACE_POSE_A
        ? secondaryMotor : primaryMotor;
    const int32_t releaseStartAbs = desiredPose == DUAL_WORKSPACE_POSE_A
        ? primaryAbs : secondaryAbs;
    const int32_t holdAbs = desiredPose == DUAL_WORKSPACE_POSE_A
        ? secondaryAbs : primaryAbs;
    const int32_t releaseTargetAbs = motorApplyTendonDelta(releaseMotor,
                                                            releaseStartAbs,
                                                            -kWorkspaceReleaseCounts);
    if (!isServoCommandInsideLimit(releaseTargetAbs)) {
        setCalibrationDiag(sharedData,
                           CALIB_DIAG_REASON_ABS_LIMIT_INVALID,
                           CALIB_DIAG_PHASE_DUAL_PRELOAD,
                           jointIndex,
                           releaseMotor,
                           releaseTargetAbs,
                           -30719,
                           30719,
                           1UL << releaseMotor);
        return false;
    }

    const int32_t primaryReleaseTarget = desiredPose == DUAL_WORKSPACE_POSE_A
        ? releaseTargetAbs : holdAbs;
    const int32_t secondaryReleaseTarget = desiredPose == DUAL_WORKSPACE_POSE_A
        ? holdAbs : releaseTargetAbs;
    if (!waitDualWorkspaceTargets(sharedData,
                                  jointIndex,
                                  "DIRECT_RELEASE",
                                  primaryMotor,
                                  primaryReleaseTarget,
                                  secondaryMotor,
                                  secondaryReleaseTarget,
                                  params)) {
        return false;
    }

    int32_t tightenAbs = 0;
    if (!tightenWorkspaceMotor(sharedData,
                               jointIndex,
                               desiredPose == DUAL_WORKSPACE_POSE_A
                                   ? "WORKSPACE_A_TIGHT" : "WORKSPACE_B_TIGHT",
                               tightenMotor,
                               releaseMotor,
                               releaseTargetAbs,
                               params,
                               &tightenAbs)) {
        return false;
    }

    if (desiredPose == DUAL_WORKSPACE_POSE_A) {
        entry->primaryReleaseAbs = releaseTargetAbs;
        entry->secondaryTightAbs = tightenAbs;
    } else {
        entry->primaryTightAbs = tightenAbs;
        entry->secondaryReleaseAbs = releaseTargetAbs;
    }
    entry->currentPose = desiredPose;
    entry->valid = 1;
    FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE established J%02u pose=%c release=M%02u start=%ld target=%ld tight=M%02u abs=%ld load=%d\r\n",
                       (unsigned)jointIndex,
                       desiredPose == DUAL_WORKSPACE_POSE_A ? 'A' : 'B',
                       (unsigned)releaseMotor,
                       (long)releaseStartAbs,
                       (long)releaseTargetAbs,
                       (unsigned)tightenMotor,
                       (long)tightenAbs,
                       (int)kWorkspaceTightLoadThreshold);
    return true;
}

static bool prepareWorkspaceForDualCalibration(TaskSharedData_t* sharedData,
                                               uint8_t targetJoint,
                                               const AutoCalibrationParams& params)
{
    if (!isWorkspaceDualJoint(targetJoint)) {
        return true;
    }

    uint8_t targetPrimaryMotor = 0;
    uint8_t targetSecondaryMotor = 0;
    int32_t targetPrimaryAbs = 0;
    int32_t targetSecondaryAbs = 0;
    if (!getDualAntagonistAutoCalibrationMotors(targetJoint,
                                                &targetPrimaryMotor,
                                                &targetSecondaryMotor) ||
        !waitWorkspaceFeedbackKnown(sharedData,
                                    targetJoint,
                                    targetPrimaryMotor,
                                    targetSecondaryMotor,
                                    &targetPrimaryAbs,
                                    &targetSecondaryAbs)) {
        setWorkspacePlanFailureDiag(sharedData,
                                    targetJoint,
                                    targetJoint,
                                    DUAL_WORKSPACE_POSE_NONE,
                                    DUAL_WORKSPACE_PLAN_STAGE_PREFLIGHT);
        FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_TARGET_PREFLIGHT fail target=J%02u\r\n",
                           (unsigned)targetJoint);
        return false;
    }
    FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_TARGET_PREFLIGHT ok target=J%02u M%02u=%ld M%02u=%ld; no target pose command\r\n",
                       (unsigned)targetJoint,
                       (unsigned)targetPrimaryMotor,
                       (long)targetPrimaryAbs,
                       (unsigned)targetSecondaryMotor,
                       (long)targetSecondaryAbs);

    DualWorkspacePlanStep steps[kWorkspaceDualHelperCount] = {};
    const uint8_t stepCount = buildDualWorkspacePlan(sharedData,
                                                      targetJoint,
                                                      steps,
                                                      kWorkspaceDualHelperCount);
    if (stepCount == 0xFF) {
        setWorkspacePlanFailureDiag(sharedData,
                                    targetJoint,
                                    targetJoint,
                                    DUAL_WORKSPACE_POSE_B,
                                    DUAL_WORKSPACE_PLAN_STAGE_PREFLIGHT);
        FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_PLAN invalid target=J%02u count=%u\r\n",
                           (unsigned)targetJoint,
                           (unsigned)stepCount);
        return false;
    }

    logDualWorkspacePlan(targetJoint, steps, stepCount);
    // Do not move a helper until target and all helper pairs have known,
    // readable ABS/load feedback. A missing channel must not leave a partial
    // workspace arrangement behind.
    if (!preflightDualWorkspacePlan(sharedData, targetJoint, steps, stepCount)) {
        return false;
    }

    for (uint8_t i = 0; i < stepCount; ++i) {
        const DualWorkspacePlanStep& step = steps[i];
        FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_HELPER_BEGIN target=J%02u joint=J%02u pose=%c\r\n",
                           (unsigned)targetJoint,
                           (unsigned)step.jointIndex,
                           step.pose == DUAL_WORKSPACE_POSE_A ? 'A' : 'B');
        if (!ensureWorkspaceDualPose(sharedData,
                                     step.jointIndex,
                                     step.pose,
                                     params)) {
            setWorkspacePlanFailureDiag(sharedData,
                                        targetJoint,
                                        step.jointIndex,
                                        step.pose,
                                        DUAL_WORKSPACE_PLAN_STAGE_EXECUTE);
            FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_HELPER_FAIL target=J%02u joint=J%02u pose=%c\r\n",
                               (unsigned)targetJoint,
                               (unsigned)step.jointIndex,
                               step.pose == DUAL_WORKSPACE_POSE_A ? 'A' : 'B');
            return false;
        }
        FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_HELPER_DONE target=J%02u joint=J%02u pose=%c\r\n",
                           (unsigned)targetJoint,
                           (unsigned)step.jointIndex,
                           step.pose == DUAL_WORKSPACE_POSE_A ? 'A' : 'B');
    }
    return true;
}

static bool findDualSecondaryPreloadPoint(TaskSharedData_t* sharedData,
                                          uint8_t jointIndex,
                                          uint8_t secondaryMotor,
                                          int8_t secondaryShortenSign,
                                          int32_t secondaryStartAbs,
                                          uint8_t primaryMotor,
                                          int32_t primaryHoldAbs,
                                          int16_t loadThreshold,
                                          const AutoCalibrationParams& params,
                                          int32_t* outPreloadAbs,
                                          uint16_t* outPreloadRaw,
                                          int32_t* outPreloadEncoder,
                                          int16_t* outPreloadLoad)
{
    uint32_t lastTraceMs = 0;
    int32_t commandAbs = secondaryStartAbs;
    int32_t commandRemainder = 0;
    uint32_t lastCommandMs = millis();

    // Match the CUI trajectory: start from measured ABS and progressively
    // advance by the configured speed step, without a remote preload target.
    FW_LOG_INFO_PRINTF("[AUTO CALIB] DUAL_PRELOAD_SECONDARY_BEGIN J%02u secondary=M%02u startAbs=%ld shortenSign=%d primaryHold=M%02u holdAbs=%ld loadThreshold=%d\r\n",
                  jointIndex,
                  secondaryMotor,
                  (long)secondaryStartAbs,
                  (int)secondaryShortenSign,
                  primaryMotor,
                  (long)primaryHoldAbs,
                  loadThreshold);

    while (autoCalibrationStillOwnsControl(sharedData)) {
        const uint32_t nowMs = millis();
        uint16_t currentRaw = 0;
        int32_t currentEncoder = 0;
        int32_t currentAbs = 0;
        if (!readLatestEncoderSample(sharedData, jointIndex, &currentRaw, &currentEncoder) ||
            !readLatestMotorServoAbs(sharedData, secondaryMotor, &currentAbs)) {
            FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL DUAL_PRELOAD_SECONDARY lost feedback J%02u M%02u\r\n",
                          jointIndex,
                          secondaryMotor);
            return false;
        }

        int16_t loadRaw = 0;
        const bool hasLoad =
            readLatestMotorServoLoad(sharedData, secondaryMotor, &loadRaw);
        const int16_t loadMag = hasLoad ? servoLoadMagnitude(loadRaw) : 0;
        if (hasLoad && loadMag >= loadThreshold) {
            if (outPreloadAbs) {
                *outPreloadAbs = currentAbs;
            }
            if (outPreloadRaw) {
                *outPreloadRaw = currentRaw;
            }
            if (outPreloadEncoder) {
                *outPreloadEncoder = currentEncoder;
            }
            if (outPreloadLoad) {
                *outPreloadLoad = loadMag;
            }
            FW_LOG_INFO_PRINTF("[AUTO CALIB] DUAL_PRELOAD_SECONDARY_FOUND J%02u M%02u raw=%u oriented=%ld abs=%ld load=%d/%d primaryHoldAbs=%ld\r\n",
                          jointIndex,
                          secondaryMotor,
                          (unsigned)currentRaw,
                          (long)currentEncoder,
                          (long)currentAbs,
                          loadMag,
                          loadThreshold,
                          (long)primaryHoldAbs);
            return publishDualMotorCalibrationTargets(sharedData,
                                                      secondaryMotor,
                                                      currentAbs,
                                                      primaryMotor,
                                                      primaryHoldAbs,
                                                      params);
        }

        const uint32_t dtMs = nowMs - lastCommandMs;
        if (dtMs >= params.loopMs) {
            const int32_t numerator =
                (int32_t)dtMs * params.tightenSpeedCountsPerSec + commandRemainder;
            const int32_t stepCounts = numerator / 1000;
            commandRemainder = numerator % 1000;
            if (stepCounts > 0) {
                commandAbs = motorApplyTendonDelta(secondaryMotor,
                                                    commandAbs,
                                                    stepCounts);
                if (!publishDualMotorCalibrationTargets(
                        sharedData,
                        secondaryMotor,
                        commandAbs,
                        primaryMotor,
                        primaryHoldAbs,
                        params)) {
                    return false;
                }
            }
            lastCommandMs = nowMs;
        }

        if ((uint32_t)(nowMs - lastTraceMs) >= params.traceMs) {
            lastTraceMs = nowMs;
            FW_LOG_INFO_PRINTF("[AUTO CALIB] TRACE_DUAL_PRELOAD_SECONDARY J%02u M%02u cmd=%ld abs=%ld raw=%u oriented=%ld load=%d/%d primaryHoldAbs=%ld\r\n",
                          jointIndex,
                          secondaryMotor,
                          (long)commandAbs,
                          (long)currentAbs,
                          (unsigned)currentRaw,
                          (long)currentEncoder,
                          loadMag,
                          loadThreshold,
                          (long)primaryHoldAbs);
        }

        vTaskDelay(pdMS_TO_TICKS(params.loopMs));
    }
    FW_LOG_INFO_PRINTF("[AUTO CALIB] CANCEL DUAL_PRELOAD_SECONDARY J%02u secondary=M%02u primary=M%02u cmdAbs=%ld holdAbs=%ld state=%u owner=%u enabled=%u\r\n",
                  jointIndex,
                  secondaryMotor,
                  primaryMotor,
                  (long)commandAbs,
                  (long)primaryHoldAbs,
                  sharedData ? (unsigned)sharedData->system_state : 255,
                  sharedData ? (unsigned)sharedData->servo_target_owner : 255,
                  sharedData ? (unsigned)sharedData->control_enabled : 255);
    return false;
}

static bool finishDualTendonReadyPreload(TaskSharedData_t* sharedData,
                                         uint8_t jointIndex,
                                         uint8_t primaryMotor,
                                         int32_t primaryStartAbs,
                                         int16_t primaryLoadThreshold,
                                         uint8_t secondaryMotor,
                                         int32_t secondaryStartAbs,
                                         const AutoCalibrationParams& params,
                                         int32_t* outPrimaryAbs,
                                         int32_t* outSecondaryAbs,
                                         uint16_t* outEncoderRaw,
                                         int32_t* outEncoder,
                                         int16_t* outPrimaryLoad,
                                         int16_t* outSecondaryLoad)
{
    int32_t primaryCommandAbs = primaryStartAbs;
    int32_t secondaryCommandAbs = secondaryStartAbs;
    int32_t commandRemainder = 0;
    uint32_t lastCommandMs = millis();
    uint32_t lastTraceMs = 0;

    FW_LOG_INFO_PRINTF("[AUTO CALIB] DUAL_FINAL_READY_PRELOAD_BEGIN J%02u primary=M%02u startAbs=%ld loadThreshold=%d secondary=M%02u holdAbs=%ld; secondary already reached final threshold\r\n",
                       jointIndex,
                       primaryMotor,
                       (long)primaryStartAbs,
                       primaryLoadThreshold,
                       secondaryMotor,
                       (long)secondaryStartAbs);

    // Synchronize both commands to the measured exchange endpoint before the
    // primary motor starts tightening. This prevents a stale exchange target
    // from surviving into the final preload phase.
    if (!publishDualMotorCalibrationTargets(sharedData,
                                            primaryMotor,
                                            primaryCommandAbs,
                                            secondaryMotor,
                                            secondaryCommandAbs,
                                            params)) {
        return false;
    }

    while (autoCalibrationStillOwnsControl(sharedData)) {
        const uint32_t nowMs = millis();
        uint16_t currentRaw = 0;
        int32_t currentEncoder = 0;
        int32_t currentPrimaryAbs = 0;
        int32_t currentSecondaryAbs = 0;
        int16_t primaryLoadRaw = 0;
        int16_t secondaryLoadRaw = 0;
        if (!readLatestEncoderSample(sharedData,
                                     jointIndex,
                                     &currentRaw,
                                     &currentEncoder) ||
            !readLatestMotorServoAbs(sharedData,
                                     primaryMotor,
                                     &currentPrimaryAbs) ||
            !readLatestMotorServoAbs(sharedData,
                                     secondaryMotor,
                                     &currentSecondaryAbs) ||
            !readLatestMotorServoLoad(sharedData,
                                      primaryMotor,
                                      &primaryLoadRaw) ||
            !readLatestMotorServoLoad(sharedData,
                                      secondaryMotor,
                                      &secondaryLoadRaw)) {
            FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL DUAL_FINAL_READY_PRELOAD lost feedback J%02u primary=M%02u secondary=M%02u\r\n",
                               jointIndex,
                               primaryMotor,
                               secondaryMotor);
            setCalibrationDiag(sharedData,
                               CALIB_DIAG_REASON_WORKSPACE_PREP_FAILED,
                               CALIB_DIAG_PHASE_DUAL_PRELOAD,
                               jointIndex,
                               primaryMotor,
                               primaryMotor,
                               secondaryMotor,
                               0,
                               (1UL << primaryMotor) | (1UL << secondaryMotor));
            return false;
        }

        const int16_t primaryLoad = servoLoadMagnitude(primaryLoadRaw);
        const int16_t secondaryLoad = servoLoadMagnitude(secondaryLoadRaw);
        const bool primaryReady = primaryLoad > primaryLoadThreshold;
        if (primaryReady) {
            if (outPrimaryAbs) {
                *outPrimaryAbs = currentPrimaryAbs;
            }
            if (outSecondaryAbs) {
                *outSecondaryAbs = currentSecondaryAbs;
            }
            if (outEncoderRaw) {
                *outEncoderRaw = currentRaw;
            }
            if (outEncoder) {
                *outEncoder = currentEncoder;
            }
            if (outPrimaryLoad) {
                *outPrimaryLoad = primaryLoad;
            }
            if (outSecondaryLoad) {
                *outSecondaryLoad = secondaryLoad;
            }
            FW_LOG_INFO_PRINTF("[AUTO CALIB] DUAL_FINAL_READY_PRELOAD_FOUND J%02u primary=M%02u abs=%ld load=%d/%d secondary=M%02u abs=%ld load=%d diagnostic raw=%u oriented=%ld\r\n",
                               jointIndex,
                               primaryMotor,
                               (long)currentPrimaryAbs,
                               primaryLoad,
                               primaryLoadThreshold,
                               secondaryMotor,
                               (long)currentSecondaryAbs,
                               secondaryLoad,
                               (unsigned)currentRaw,
                               (long)currentEncoder);
            return publishDualMotorCalibrationTargets(sharedData,
                                                      primaryMotor,
                                                      currentPrimaryAbs,
                                                      secondaryMotor,
                                                      currentSecondaryAbs,
                                                      params);
        }

        const uint32_t dtMs = nowMs - lastCommandMs;
        if (dtMs >= params.loopMs) {
            const int32_t numerator =
                (int32_t)dtMs * params.tightenSpeedCountsPerSec +
                commandRemainder;
            const int32_t stepCounts = numerator / 1000;
            commandRemainder = numerator % 1000;
            if (stepCounts > 0) {
                primaryCommandAbs = motorApplyTendonDelta(primaryMotor,
                                                           primaryCommandAbs,
                                                           stepCounts);
                if (!publishDualMotorCalibrationTargets(sharedData,
                                                        primaryMotor,
                                                        primaryCommandAbs,
                                                        secondaryMotor,
                                                        secondaryCommandAbs,
                                                        params)) {
                    return false;
                }
            }
            lastCommandMs = nowMs;
        }

        if ((uint32_t)(nowMs - lastTraceMs) >= params.traceMs) {
            lastTraceMs = nowMs;
            FW_LOG_INFO_PRINTF("[AUTO CALIB] TRACE_DUAL_FINAL_READY_PRELOAD J%02u primary=M%02u cmd=%ld abs=%ld load=%d/%d secondary=M%02u holdCmd=%ld abs=%ld load=%d diagnostic raw=%u oriented=%ld\r\n",
                               jointIndex,
                               primaryMotor,
                               (long)primaryCommandAbs,
                               (long)currentPrimaryAbs,
                               primaryLoad,
                               primaryLoadThreshold,
                               secondaryMotor,
                               (long)secondaryCommandAbs,
                               (long)currentSecondaryAbs,
                               secondaryLoad,
                               (unsigned)currentRaw,
                               (long)currentEncoder);
        }

        vTaskDelay(pdMS_TO_TICKS(params.loopMs));
    }

    FW_LOG_INFO_PRINTF("[AUTO CALIB] CANCEL DUAL_FINAL_READY_PRELOAD J%02u primary=M%02u cmd=%ld secondary=M%02u cmd=%ld state=%u owner=%u enabled=%u\r\n",
                       jointIndex,
                       primaryMotor,
                       (long)primaryCommandAbs,
                       secondaryMotor,
                       (long)secondaryCommandAbs,
                       sharedData ? (unsigned)sharedData->system_state : 255,
                       sharedData ? (unsigned)sharedData->servo_target_owner : 255,
                       sharedData ? (unsigned)sharedData->control_enabled : 255);
    return false;
}

static bool findDualExchangeLimitPoint(TaskSharedData_t* sharedData,
                                       uint8_t jointIndex,
                                       uint8_t secondaryMotor,
                                       int8_t secondaryShortenSign,
                                       int32_t secondaryStartAbs,
                                       uint8_t primaryMotor,
                                       int8_t primaryShortenSign,
                                       int32_t primaryStartAbs,
                                       int32_t startEncoder,
                                       int16_t finalLoadThreshold,
                                       const AutoCalibrationParams& params,
                                       int32_t* outSecondaryLimitAbs,
                                       int32_t* outPrimaryFinalAbs,
                                       uint16_t* outLimitRaw,
                                       int32_t* outLimitEncoder,
                                       int16_t* outSecondaryLoad,
                                       DualTendonRawFitBuffer* fitBuffer)
{
    int32_t secondaryCommandAbs = secondaryStartAbs;
    int32_t primaryCommandAbs = primaryStartAbs;
    int32_t commandRemainder = 0;
    const uint32_t startMs = millis();
    uint32_t lastCommandMs = startMs;
    uint32_t lastTraceMs = 0;
    uint32_t windowStartMs = startMs;
    int32_t windowStartEncoder = startEncoder;
    int32_t windowStartSecondaryAbs = secondaryStartAbs;
    int32_t windowStartPrimaryAbs = primaryStartAbs;
    uint8_t angleStallStable = 0;

    FW_LOG_INFO_PRINTF("[AUTO CALIB] DUAL_EXCHANGE_NEG3_BEGIN J%02u secondary=M%02u secondaryAbs=%ld secondaryShortenSign=%d primary=M%02u primaryAbs=%ld primaryShortenSign=%d requiredTravelDeg=%.3f finalLoadThreshold=%d\r\n",
                  jointIndex,
                  secondaryMotor,
                  (long)secondaryStartAbs,
                  (int)secondaryShortenSign,
                  primaryMotor,
                  (long)primaryStartAbs,
                  (int)primaryShortenSign,
                  kReverseMinFollowDeg,
                  finalLoadThreshold);

    while (autoCalibrationStillOwnsControl(sharedData)) {
        const uint32_t nowMs = millis();
        uint16_t currentRaw = 0;
        int32_t currentEncoder = 0;
        int32_t currentSecondaryAbs = 0;
        int32_t currentPrimaryAbs = 0;
        if (!readLatestEncoderSample(sharedData, jointIndex, &currentRaw, &currentEncoder) ||
            !readLatestMotorServoAbs(sharedData, secondaryMotor, &currentSecondaryAbs) ||
            !readLatestMotorServoAbs(sharedData, primaryMotor, &currentPrimaryAbs)) {
            FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL DUAL_EXCHANGE_NEG3 lost feedback J%02u M%02u/M%02u\r\n",
                          jointIndex,
                          secondaryMotor,
                          primaryMotor);
            return false;
        }

        int16_t loadRaw = 0;
        const bool hasLoad =
            readLatestMotorServoLoad(sharedData, secondaryMotor, &loadRaw);
        const int16_t loadMag = hasLoad ? servoLoadMagnitude(loadRaw) : 0;
        const bool secondaryLoadHigh = hasLoad && loadMag >= finalLoadThreshold;
        const int32_t totalEncoderDelta =
            calibrationWrapEncoderDelta(currentEncoder, startEncoder);
        const float totalAngleDeltaDeg =
            fabsf(calibrationEncoderCountToDeg(totalEncoderDelta));
        maybeAppendDualTendonTravelFitSample(fitBuffer,
                                             totalAngleDeltaDeg,
                                             currentEncoder,
                                             currentPrimaryAbs,
                                             currentSecondaryAbs);
        bool limitFound = false;

        if ((uint32_t)(nowMs - windowStartMs) >= params.windowMs) {
            const int32_t windowEncoderDelta =
                calibrationWrapEncoderDelta(currentEncoder, windowStartEncoder);
            const float windowAngleDeltaDeg =
                fabsf(calibrationEncoderCountToDeg(windowEncoderDelta));
            const int32_t windowSecondaryAbsDelta =
                abs32(currentSecondaryAbs - windowStartSecondaryAbs);
            const int32_t windowPrimaryAbsDelta =
                abs32(currentPrimaryAbs - windowStartPrimaryAbs);
            const bool enoughEncoderTravel =
                totalAngleDeltaDeg >= kReverseMinFollowDeg;
            const bool angleStalled =
                windowAngleDeltaDeg <= params.stallActualDeg;
            const bool secondaryStillReeling =
                windowSecondaryAbsDelta >= params.windowServoDeltaCounts;
            const bool primaryStillPaying =
                windowPrimaryAbsDelta >= params.windowServoDeltaCounts;
            if (angleStalled) {
                if (angleStallStable < 255) {
                    angleStallStable++;
                }
            } else {
                angleStallStable = 0;
            }
            FW_LOG_INFO_PRINTF("[AUTO CALIB] CHECK_DUAL_EXCHANGE_NEG3 J%02u secondary=M%02u abs=%ld primary=M%02u abs=%ld raw=%u oriented=%ld dCount=%ld winCount=%ld winSecAbs=%ld/%ld winPriAbs=%ld/%ld winDeg=%.3f/%.3f totalDeg=%.3f/%.3f load=%d/%d cond load=%u sec=%u pri=%u angle=%u angleStable=%u/%u travel=%u\r\n",
                          jointIndex,
                          secondaryMotor,
                          (long)currentSecondaryAbs,
                          primaryMotor,
                          (long)currentPrimaryAbs,
                          (unsigned)currentRaw,
                          (long)currentEncoder,
                          (long)totalEncoderDelta,
                          (long)windowEncoderDelta,
                          (long)windowSecondaryAbsDelta,
                          (long)params.windowServoDeltaCounts,
                          (long)windowPrimaryAbsDelta,
                          (long)params.windowServoDeltaCounts,
                          windowAngleDeltaDeg,
                          params.stallActualDeg,
                          totalAngleDeltaDeg,
                          kReverseMinFollowDeg,
                          loadMag,
                          finalLoadThreshold,
                          secondaryLoadHigh ? 1 : 0,
                          secondaryStillReeling ? 1 : 0,
                          primaryStillPaying ? 1 : 0,
                          angleStalled ? 1 : 0,
                          (unsigned)angleStallStable,
                          (unsigned)kAngleStallStableSamples,
                          enoughEncoderTravel ? 1 : 0);

            limitFound =
                secondaryLoadHigh &&
                enoughEncoderTravel &&
                secondaryStillReeling &&
                primaryStillPaying;
            windowStartEncoder = currentEncoder;
            windowStartSecondaryAbs = currentSecondaryAbs;
            windowStartPrimaryAbs = currentPrimaryAbs;
            windowStartMs = nowMs;
        }

        if (limitFound) {
            if (outSecondaryLimitAbs) {
                *outSecondaryLimitAbs = currentSecondaryAbs;
            }
            if (outPrimaryFinalAbs) {
                *outPrimaryFinalAbs = currentPrimaryAbs;
            }
            if (outLimitRaw) {
                *outLimitRaw = currentRaw;
            }
            if (outLimitEncoder) {
                *outLimitEncoder = currentEncoder;
            }
            if (outSecondaryLoad) {
                *outSecondaryLoad = loadMag;
            }
            FW_LOG_INFO_PRINTF("[AUTO CALIB] DUAL_EXCHANGE_NEG3_FOUND J%02u secondary=M%02u raw=%u oriented=%ld secondaryAbs=%ld primaryAbs=%ld totalDeg=%.3f load=%d/%d\r\n",
                          jointIndex,
                          secondaryMotor,
                          (unsigned)currentRaw,
                          (long)currentEncoder,
                          (long)currentSecondaryAbs,
                          (long)currentPrimaryAbs,
                          totalAngleDeltaDeg,
                          loadMag,
                          finalLoadThreshold);
            return publishDualMotorCalibrationTargets(sharedData,
                                                      secondaryMotor,
                                                      currentSecondaryAbs,
                                                      primaryMotor,
                                                      currentPrimaryAbs,
                                                      params);
        }

        const uint32_t dtMs = nowMs - lastCommandMs;
        if (dtMs >= params.loopMs) {
            const int32_t numerator =
                (int32_t)dtMs * params.tightenSpeedCountsPerSec + commandRemainder;
            const int32_t stepCounts = numerator / 1000;
            commandRemainder = numerator % 1000;
            if (stepCounts > 0) {
                secondaryCommandAbs = motorApplyTendonDelta(secondaryMotor,
                                                            secondaryCommandAbs,
                                                            stepCounts);
                primaryCommandAbs = motorApplyTendonDelta(primaryMotor,
                                                          primaryCommandAbs,
                                                          -stepCounts);
                if (!publishDualMotorCalibrationTargets(sharedData,
                                                        secondaryMotor,
                                                        secondaryCommandAbs,
                                                        primaryMotor,
                                                        primaryCommandAbs,
                                                        params)) {
                    return false;
                }
            }
            lastCommandMs = nowMs;
        }

        if ((uint32_t)(nowMs - lastTraceMs) >= params.traceMs) {
            lastTraceMs = nowMs;
            FW_LOG_INFO_PRINTF("[AUTO CALIB] TRACE_DUAL_EXCHANGE_NEG3 J%02u secondary=M%02u cmd=%ld abs=%ld primary=M%02u cmd=%ld abs=%ld raw=%u oriented=%ld totalDeg=%.3f load=%d/%d\r\n",
                          jointIndex,
                          secondaryMotor,
                          (long)secondaryCommandAbs,
                          (long)currentSecondaryAbs,
                          primaryMotor,
                          (long)primaryCommandAbs,
                          (long)currentPrimaryAbs,
                          (unsigned)currentRaw,
                          (long)currentEncoder,
                          totalAngleDeltaDeg,
                          loadMag,
                          finalLoadThreshold);
        }

        vTaskDelay(pdMS_TO_TICKS(params.loopMs));
    }
    FW_LOG_INFO_PRINTF("[AUTO CALIB] CANCEL DUAL_EXCHANGE_NEG3 J%02u secondary=M%02u cmd=%ld primary=M%02u cmd=%ld state=%u owner=%u enabled=%u\r\n",
                  jointIndex,
                  secondaryMotor,
                  (long)secondaryCommandAbs,
                  primaryMotor,
                  (long)primaryCommandAbs,
                  sharedData ? (unsigned)sharedData->system_state : 255,
                  sharedData ? (unsigned)sharedData->servo_target_owner : 255,
                  sharedData ? (unsigned)sharedData->control_enabled : 255);
    return false;
}

static bool runDualAntagonistAutoCalibrationJoint(TaskSharedData_t* sharedData,
                                                  uint8_t jointIndex,
                                                  const AutoCalibrationParams& params,
                                                  DualWorkspaceCacheEntry* workspaceState)
{
    uint8_t primaryMotor = 0;
    uint8_t secondaryMotor = 0;
    if (!getDualAntagonistAutoCalibrationMotors(jointIndex, &primaryMotor, &secondaryMotor)) {
        return false;
    }
    setRuntimeEncoderDirection(jointIndex, kDefaultEncoderDirection[jointIndex]);
    clearCalibratedMotorAbsLimitsForJoint(sharedData, jointIndex);
    const int8_t primaryShortenSign = getMotorTendonShortenAbsSign(primaryMotor);
    const int8_t secondaryShortenSign = getMotorTendonShortenAbsSign(secondaryMotor);
    const int16_t primaryTightLoadThreshold = kDualPrimaryTightLoadThreshold;
    const int16_t secondaryPreloadLoadThreshold = kDualSecondaryPreloadLoadThreshold;
    const int16_t secondaryFinalLoadThreshold = kDualSecondaryFinalLoadThreshold;
    const int16_t primaryFinalLoadThreshold = kDualPrimaryFinalLoadThreshold;

    uint16_t initialRaw = 0;
    int32_t initialEncoder = 0;
    int32_t primaryAbs = 0;
    int32_t secondaryAbs = 0;
    if (!waitDualInitialFeedback(sharedData,
                                 jointIndex,
                                 primaryMotor,
                                 secondaryMotor,
                                 &primaryAbs,
                                 &secondaryAbs,
                                 &initialRaw,
                                 &initialEncoder,
                                 params)) {
        FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL DUAL_INIT J%02u M%02u/M%02u missing feedback\r\n",
                      jointIndex,
                      primaryMotor,
                      secondaryMotor);
        return false;
    }

    FW_LOG_INFO_PRINTF("[AUTO CALIB] DUAL_BEGIN J%02u primary=M%02u primaryShortenSign=%d secondary=M%02u secondaryShortenSign=%d slack=%ld primaryLoad=%d secondaryPreloadLoad=%d secondaryFinalLoad=%d primaryFinalLoad=%d startDeg=%.3f tightDeg=dynamic initialEncoder=%ld primaryAbs=%ld secondaryAbs=%ld\r\n",
                  jointIndex,
                  primaryMotor,
                  (int)primaryShortenSign,
                  secondaryMotor,
                  (int)secondaryShortenSign,
                  (long)kDualAutoSlackCounts,
                   primaryTightLoadThreshold,
                   secondaryPreloadLoadThreshold,
                   secondaryFinalLoadThreshold,
                   primaryFinalLoadThreshold,
                   kAutoCalibrationStartDeg,
                  (long)initialEncoder,
                  (long)primaryAbs,
                  (long)secondaryAbs);

    int32_t primary80Encoder = 0;
    int32_t primary80Abs = 0;
    uint16_t primary80Raw = 0;
    int16_t primary80Load = 0;
    const bool workspaceBMatchesMotors = workspaceState && workspaceState->valid &&
        workspaceState->currentPose == DUAL_WORKSPACE_POSE_B &&
        workspaceState->primaryMotor == primaryMotor &&
        workspaceState->secondaryMotor == secondaryMotor;
    bool reuseWorkspaceStart = false;
    if (workspaceBMatchesMotors) {
        const int32_t primaryWorkspaceError =
            abs32(primaryAbs - workspaceState->primaryTightAbs);
        const int32_t secondaryWorkspaceError =
            abs32(secondaryAbs - workspaceState->secondaryReleaseAbs);
        if (primaryWorkspaceError <= kWorkspaceTargetToleranceCounts &&
            secondaryWorkspaceError <= kWorkspaceTargetToleranceCounts) {
            reuseWorkspaceStart = true;
        } else {
            FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_TARGET_B_INVALIDATE J%02u M%02u err=%ld M%02u err=%ld tolerance=%ld\r\n",
                               (unsigned)jointIndex,
                               (unsigned)primaryMotor,
                               (long)primaryWorkspaceError,
                               (unsigned)secondaryMotor,
                               (long)secondaryWorkspaceError,
                               (long)kWorkspaceTargetToleranceCounts);
            workspaceState->valid = 0;
        }
    }
    if (reuseWorkspaceStart) {
        int16_t primaryLoadRaw = 0;
        const bool primaryLoadAvailable =
            readLatestMotorServoLoad(sharedData, primaryMotor, &primaryLoadRaw);
        primary80Abs = primaryAbs;
        primary80Raw = initialRaw;
        primary80Encoder = initialEncoder;
        // B is a position cache. The servo's instantaneous load commonly
        // returns to zero after the prior tightening command has settled, so
        // only the verified ABS pose decides whether it can be reused.
        primary80Load = primaryLoadAvailable ? servoLoadMagnitude(primaryLoadRaw) : 0;
        FW_LOG_INFO_PRINTF("[AUTO CALIB] DUAL_WORKSPACE_B_REUSE J%02u primary=M%02u abs=%ld load=%d available=%u secondary=M%02u abs=%ld; ABS matched, skip duplicate slack/POS80\r\n",
                           (unsigned)jointIndex,
                           (unsigned)primaryMotor,
                           (long)primary80Abs,
                           (int)primary80Load,
                           primaryLoadAvailable ? 1U : 0U,
                           (unsigned)secondaryMotor,
                           (long)secondaryAbs);
    } else {
        const int32_t secondarySlackTarget =
            motorApplyTendonDelta(secondaryMotor, secondaryAbs, -kDualAutoSlackCounts);
        FW_LOG_INFO_PRINTF("[AUTO CALIB] DUAL_SLACK_SECONDARY_BEGIN J%02u M%02u targetAbs=%ld from=%ld shortenSign=%d\r\n",
                      jointIndex,
                      secondaryMotor,
                      (long)secondarySlackTarget,
                      (long)secondaryAbs,
                      (int)secondaryShortenSign);
        if (!waitMotorTargetReached(sharedData,
                                    jointIndex,
                                    "DUAL_SLACK_SECONDARY",
                                    secondaryMotor,
                                    secondarySlackTarget,
                                    params)) {
            return false;
        }

        if (!readLatestEncoderOriented(sharedData, jointIndex, &initialEncoder) ||
            !readLatestMotorServoAbs(sharedData, primaryMotor, &primaryAbs)) {
            FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL DUAL_PRIMARY_START J%02u missing feedback\r\n",
                          jointIndex);
            return false;
        }
        if (!findDualTightPoint(sharedData,
                                jointIndex,
                                "POS80",
                                primaryMotor,
                                primaryShortenSign,
                                primaryAbs,
                                initialEncoder,
                                primaryTightLoadThreshold,
                                false,
                                0,
                                0,
                                params,
                                &primary80Abs,
                                &primary80Raw,
                                &primary80Encoder,
                                &primary80Load)) {
            return false;
        }
    }

    int32_t secondaryStartAbs = 0;
    int32_t secondaryStartEncoder = 0;
    if (!readLatestMotorServoAbs(sharedData, secondaryMotor, &secondaryStartAbs) ||
        !readLatestEncoderOriented(sharedData, jointIndex, &secondaryStartEncoder)) {
        FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL DUAL_SECONDARY_START J%02u missing feedback\r\n",
                      jointIndex);
        return false;
    }

    int32_t neg3Encoder = 0;
    int32_t secondaryNeg3Abs = 0;
    int32_t primaryNeg3Abs = primary80Abs;
    uint16_t neg3Raw = 0;
    int16_t neg3Load = 0;
    int32_t secondaryPreloadAbs = secondaryStartAbs;
    uint16_t secondaryPreloadRaw = 0;
    int32_t secondaryPreloadEncoder = secondaryStartEncoder;
    int16_t secondaryPreloadLoad = 0;
    DualTendonRawFitBuffer fitSamples;
    resetDualTendonRawFitBuffer(&fitSamples);
    if (!findDualSecondaryPreloadPoint(sharedData,
                                       jointIndex,
                                       secondaryMotor,
                                       secondaryShortenSign,
                                       secondaryStartAbs,
                                       primaryMotor,
                                       primary80Abs,
                                       secondaryPreloadLoadThreshold,
                                       params,
                                       &secondaryPreloadAbs,
                                       &secondaryPreloadRaw,
                                       &secondaryPreloadEncoder,
                                       &secondaryPreloadLoad)) {
        return false;
    }

    int32_t exchangeStartEncoder = 0;
    if (!readLatestEncoderOriented(sharedData, jointIndex, &exchangeStartEncoder)) {
        FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL DUAL_EXCHANGE_START J%02u missing encoder\r\n",
                      jointIndex);
        return false;
    }
    appendDualTendonRawFitSample(&fitSamples,
                                 exchangeStartEncoder,
                                 primary80Abs,
                                 secondaryPreloadAbs);
    fitSamples.nextTravelDeg = kDualTendonFitSampleStepDeg;

    if (!findDualExchangeLimitPoint(sharedData,
                                    jointIndex,
                                    secondaryMotor,
                                    secondaryShortenSign,
                                    secondaryPreloadAbs,
                                    primaryMotor,
                                    primaryShortenSign,
                                    primary80Abs,
                                    exchangeStartEncoder,
                                    secondaryFinalLoadThreshold,
                                    params,
                                    &secondaryNeg3Abs,
                                    &primaryNeg3Abs,
                                    &neg3Raw,
                                    &neg3Encoder,
                                    &neg3Load,
                                    &fitSamples)) {
        return false;
    }
    appendDualTendonRawFitSample(&fitSamples,
                                 neg3Encoder,
                                 primaryNeg3Abs,
                                 secondaryNeg3Abs);

    // Preserve the mechanical Auto Min/LUT endpoint above. READY must use a
    // different final pose in which both tendons are loaded: secondary has
    // already reached 250, then primary is restored to 240 while secondary
    // remains at the exchange endpoint command.
    int32_t primaryReadyAbs = primaryNeg3Abs;
    int32_t secondaryReadyAbs = secondaryNeg3Abs;
    uint16_t readyRaw = neg3Raw;
    int32_t readyEncoder = neg3Encoder;
    int16_t primaryReadyLoad = 0;
    int16_t secondaryReadyLoad = neg3Load;
    if (!finishDualTendonReadyPreload(sharedData,
                                     jointIndex,
                                     primaryMotor,
                                     primaryNeg3Abs,
                                     primaryFinalLoadThreshold,
                                     secondaryMotor,
                                     secondaryNeg3Abs,
                                     params,
                                     &primaryReadyAbs,
                                     &secondaryReadyAbs,
                                     &readyRaw,
                                     &readyEncoder,
                                     &primaryReadyLoad,
                                     &secondaryReadyLoad)) {
        return false;
    }

    const int32_t signedSpan = signedEncoderDeltaLocal(neg3Encoder, primary80Encoder);
    const int8_t encoderTravelSign = 1;
    const int32_t resultStartEncoder =
        encoderForPositiveTravelResult(neg3Encoder, encoderTravelSign);
    const int32_t resultTightEncoder =
        encoderForPositiveTravelResult(primary80Encoder, encoderTravelSign);
    const int32_t resultSpan =
        calibrationForwardEncoderDelta(resultStartEncoder, resultTightEncoder);
    if (resultSpan <= 0 || resultSpan >= kAutoCalibrationRejectNearFullWrapSpan) {
        FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL DUAL_RESULT J%02u invalid span=%ld neg3=%ld pos80=%ld signed=%ld\r\n",
                      jointIndex,
                      (long)resultSpan,
                      (long)neg3Encoder,
                      (long)primary80Encoder,
                      (long)signedSpan);
        return false;
    }

    const float dynamicDualTotalDeg = calibrationEncoderCountToDeg(resultSpan);
    const float zeroFraction =
        (dynamicDualTotalDeg > 0.0f)
            ? ((kAutoCalibrationZeroDeg - kAutoCalibrationStartDeg) /
               dynamicDualTotalDeg)
            : 0.0f;
    const int32_t resultZeroEncoder =
        calibrationWrapEncoderCount(
            resultStartEncoder + roundToInt32Local((float)resultSpan * zeroFraction));
    JointCalibrationResult result;
    if (!buildDualZeroTightCalibrationResult(jointIndex,
                                             resultStartEncoder,
                                             resultZeroEncoder,
                                             resultTightEncoder,
                                             secondaryNeg3Abs,
                                             primary80Abs,
                                             primary80Abs,
                                             &result)) {
        FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL DUAL_RESULT_BUILD J%02u\r\n", jointIndex);
        return false;
    }
    const bool lutBuilt =
        buildDualTendonLutFromRawSamples(&result,
                                         primaryMotor,
                                         secondaryMotor,
                                         fitSamples);
    EncoderUrdfMapResult readyMap = {};
    if (!mapManualScaledEncoderToUrdf(
            jointIndex,
            result,
            g_manualEncoderReference[jointIndex],
            readyRaw,
            g_encoderDirection[jointIndex],
            &readyMap) ||
        !readyMap.valid ||
        readyMap.outsideCounts < -kFeedbackReadyEnterOutsideToleranceCounts) {
        FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL DUAL_FINAL_READY_MAP J%02u raw=%u oriented=%ld valid=%u ready=%u outside=%ld enterTolerance=%ld\r\n",
                           jointIndex,
                           (unsigned)readyRaw,
                           (long)readyEncoder,
                           readyMap.valid ? 1U : 0U,
                           readyMap.feedbackReady ? 1U : 0U,
                           (long)readyMap.outsideCounts,
                           (long)kFeedbackReadyEnterOutsideToleranceCounts);
        setCalibrationDiag(sharedData,
                           CALIB_DIAG_REASON_WORKSPACE_PREP_FAILED,
                           CALIB_DIAG_PHASE_DUAL_PRELOAD,
                           jointIndex,
                           primaryMotor,
                           readyEncoder,
                           readyMap.outsideCounts,
                           secondaryMotor,
                           (1UL << primaryMotor) | (1UL << secondaryMotor));
        return false;
    }

    FW_LOG_INFO_PRINTF("[AUTO CALIB] DUAL_ENCODER_RESULT J%02u defaultDir=%d runtimeDir=%d signed=%ld resultStart=%ld resultZero=%ld resultTight=%ld\r\n",
                  jointIndex,
                  (int)kDefaultEncoderDirection[jointIndex],
                  (int)g_encoderDirection[jointIndex],
                  (long)signedSpan,
                  (long)result.startEncoder,
                  (long)result.zeroEncoder,
                  (long)result.tightEncoder);
    FW_LOG_INFO_PRINTF("[AUTO CALIB] DUAL_LUT J%02u built=%u rawSamples=%u lutSamples=%u startDeg=%.3f stepDeg=%.3f primary=M%02u secondary=M%02u\r\n",
                  jointIndex,
                  lutBuilt ? 1U : 0U,
                  (unsigned)fitSamples.count,
                  (unsigned)result.dualTendonLutCount,
                  result.dualTendonLutAngleStartDeg,
                  result.dualTendonLutAngleStepDeg,
                  primaryMotor,
                  secondaryMotor);

    applyDynamicAutoCalibrationConfig(&g_jointCalibConfig[jointIndex], result);
    g_jointCalibResult[jointIndex] = result;
    updateCalibrationAssemblyWarning(sharedData, jointIndex, result);
    g_encoderOffsetManual[jointIndex] = result.offset;
    syncJointTargetToDeg(sharedData, jointIndex, readyMap.urdfDeg);
    clearJointServoMultiTurnUnknown(sharedData, jointIndex);
    const int32_t primaryLimitTightAbs =
        motorRawAbsToTendonAbs(primaryMotor, primaryReadyAbs) >
                motorRawAbsToTendonAbs(primaryMotor, primary80Abs)
            ? primaryReadyAbs
            : primary80Abs;
    const int32_t secondaryLimitTightAbs =
        motorRawAbsToTendonAbs(secondaryMotor, secondaryReadyAbs) >
                motorRawAbsToTendonAbs(secondaryMotor, secondaryNeg3Abs)
            ? secondaryReadyAbs
            : secondaryNeg3Abs;
    setCalibratedMotorAbsLimit(sharedData, primaryMotor, primaryLimitTightAbs);
    setCalibratedMotorAbsLimit(sharedData, secondaryMotor, secondaryLimitTightAbs);
    invalidateDualTendonReleaseAnchors((1UL << primaryMotor) |
                                      (1UL << secondaryMotor));

    if (workspaceState) {
        // Cache the final two-tendon preload pose, not the earlier mechanical
        // endpoint where primary was released. Later joints can reuse this A
        // pose without introducing slack again.
        workspaceState->primaryMotor = primaryMotor;
        workspaceState->secondaryMotor = secondaryMotor;
        workspaceState->primaryReleaseAbs = primaryReadyAbs;
        workspaceState->secondaryTightAbs = secondaryReadyAbs;
        workspaceState->currentPose = DUAL_WORKSPACE_POSE_A;
        workspaceState->valid = 1;
        FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_POST_CALIB_CACHE J%02u pose=A primaryReady=M%02u:%ld secondaryReady=M%02u:%ld load=%d/%d\r\n",
                           (unsigned)jointIndex,
                           (unsigned)primaryMotor,
                           (long)primaryReadyAbs,
                           (unsigned)secondaryMotor,
                           (long)secondaryReadyAbs,
                           primaryReadyLoad,
                           secondaryReadyLoad);
    }

    FW_LOG_INFO_PRINTF("[AUTO CALIB] DUAL_RESULT J%02u neg3Raw=%u neg3=%ld pos80Raw=%u pos80=%ld span=%ld zero=%ld primary80Abs=%ld primaryNeg3Abs=%ld secondaryPreloadRaw=%u secondaryPreloadEncoder=%ld secondaryPreloadAbs=%ld secondaryNeg3Abs=%ld loadPrimary=%d loadSecondaryPreload=%d loadSecondaryNeg3=%d readyRaw=%u readyEncoder=%ld readyDeg=%.3f primaryReadyAbs=%ld primaryReadyLoad=%d secondaryReadyAbs=%ld secondaryReadyLoad=%d\r\n",
                  jointIndex,
                  (unsigned)neg3Raw,
                  (long)result.startEncoder,
                  (unsigned)primary80Raw,
                  (long)result.tightEncoder,
                  (long)result.encoderSpan,
                  (long)result.zeroEncoder,
                  (long)primary80Abs,
                  (long)primaryNeg3Abs,
                  (unsigned)secondaryPreloadRaw,
                  (long)secondaryPreloadEncoder,
                  (long)secondaryPreloadAbs,
                  (long)secondaryNeg3Abs,
                   primary80Load,
                   secondaryPreloadLoad,
                   neg3Load,
                   (unsigned)readyRaw,
                   (long)readyEncoder,
                   readyMap.urdfDeg,
                   (long)primaryReadyAbs,
                   primaryReadyLoad,
                   (long)secondaryReadyAbs,
                   secondaryReadyLoad);
    FW_LOG_INFO_PRINTF("[AUTO CALIB] AUTO_RANGE_DYNAMIC J%02u startDeg=%.3f zeroDeg=0.000 tightDeg=%.3f totalDeg=%.3f angleMax=%.3f span=%ld\r\n",
                  jointIndex,
                  result.startDeg,
                  result.tightDeg,
                  result.totalDeg,
                  result.angleMax,
                  (long)result.encoderSpan);

    return true;
}

static void failAutoCalibContext(JointCalibContext* ctx, const char* reason)
{
    if (!ctx || ctx->state == AUTO_CALIB_STATE_FAILED) {
        return;
    }
    FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL J%02u %s\r\n",
                  ctx->jointIndex,
                  reason ? reason : "unknown");
    ctx->state = AUTO_CALIB_STATE_FAILED;
}

static void failAutoCalibContextWithDiag(JointCalibContext* ctx,
                                         TaskSharedData_t* sharedData,
                                         const char* reasonText,
                                         uint8_t reason,
                                         uint8_t phase,
                                         uint8_t motor,
                                         int32_t valueA,
                                         int32_t valueB,
                                         int32_t valueC,
                                         uint32_t bitmap)
{
    if (!ctx || ctx->state == AUTO_CALIB_STATE_FAILED) {
        return;
    }
    setCalibrationDiag(sharedData,
                       reason,
                       phase,
                       ctx->jointIndex,
                       motor,
                       valueA,
                       valueB,
                       valueC,
                       bitmap);
    failAutoCalibContext(ctx, reasonText);
}

static bool initTightenStateFromFeedback(JointCalibContext* ctx,
                                         TaskSharedData_t* sharedData,
                                         const AutoCalibrationParams& params)
{
    if (!ctx || !sharedData) {
        return false;
    }

    uint16_t raw = 0;
    int32_t encoder = 0;
    int32_t absNow = 0;
    if (!readLatestEncoderSample(sharedData, ctx->jointIndex, &raw, &encoder) ||
        !readLatestServoAbs(sharedData, ctx->bus, ctx->servoId, &absNow)) {
        return false;
    }

    int16_t baselineLoadRaw = 0;
    const bool hasBaselineLoad =
        readLatestServoLoad(sharedData, ctx->bus, ctx->servoId, &baselineLoadRaw);
    ctx->baselineLoad = hasBaselineLoad ? servoLoadMagnitude(baselineLoadRaw) : 0;
    ctx->startSearchRaw = raw;
    ctx->startSearchEncoder = encoder;
    ctx->startSearchAbs = absNow;
    ctx->startFound = false;
    ctx->zeroFound = false;
    ctx->startLoadStable = 0;
    ctx->startEncoder = 0;
    ctx->startServoAbs = 0;
    ctx->zeroEncoder = 0;
    ctx->zeroServoAbs = 0;
    ctx->encoderTravelSign = 1;
    ctx->windowStartEncoder = encoder;
    ctx->windowStartAbs = absNow;
    ctx->windowStartMs = millis();
    ctx->commandAbs = absNow;
    ctx->commandRemainder = 0;
    ctx->lastCommandMs = millis();
    ctx->lastTraceMs = 0;
    ctx->stateBeginMs = millis();
    ctx->state = AUTO_CALIB_STATE_TIGHTEN;

    FW_LOG_INFO_PRINTF("[AUTO CALIB] AUTO_RANGE_DYNAMIC_BEGIN J%02u startDeg=%.3f tightDeg=dynamic totalDeg=dynamic angleMax=dynamic\r\n",
                  ctx->jointIndex,
                  kAutoCalibrationStartDeg);
    FW_LOG_INFO_PRINTF("[AUTO CALIB] START J%02u bus%u id%u abs=%ld raw=%u oriented=%ld dir=%d baselineLoad=%d speedCps=%ld\r\n",
                  ctx->jointIndex,
                  ctx->bus,
                  ctx->servoId,
                  (long)ctx->startSearchAbs,
                  (unsigned)ctx->startSearchRaw,
                  (long)ctx->startSearchEncoder,
                  (int)g_encoderDirection[ctx->jointIndex],
                  ctx->baselineLoad,
                  (long)params.tightenSpeedCountsPerSec);
    return true;
}

static void initAutoCalibContext(JointCalibContext* ctx, uint8_t jointIndex)
{
    if (!ctx) {
        return;
    }
    memset(ctx, 0, sizeof(*ctx));
    ctx->state = AUTO_CALIB_STATE_INIT;
    ctx->jointIndex = jointIndex;
    ctx->motorChannel = 0xFF;
}

// ============================================================================
// stepAutoCalibrationJoint - 单关节自动标定的并行状态机步进函数
// ============================================================================
// 核心逻辑（非阻塞）：
// 在每一帧定时调度中被调用。它将原先阻塞式的“死循环拉绳”重构成了状态机步进�?// 主要卡点判定条件 (AUTO_CALIB_STATE_TIGHTEN)�?// 1) 舵机在指定的时间窗口内，绝对位置必须移动了一定的距离，否则认为拉紧停滞�?// 2) 电流反馈 (load) 必须达到并稳定超过设定的阈�?(tightLoadHigh)�?// 3) 磁编码器的角度必须在连续多次采样中变化小于容�?(stallActualDeg)，证明机械已经完全抵住死区�?// 只有当以上条件同时满足时，才判定为“成功寻找到物理零位”，进而校准磁编与舵机�?// ============================================================================
static void stepAutoCalibrationJoint(JointCalibContext* ctx,
                                     TaskSharedData_t* sharedData,
                                     const AutoCalibrationParams& params,
                                     ServoTargetBatch_t* batch)
{
    if (!ctx ||
        ctx->state == AUTO_CALIB_STATE_DONE ||
        ctx->state == AUTO_CALIB_STATE_FAILED) {
        return;
    }

    const uint32_t nowMs = millis();
    if (!autoCalibrationStillOwnsControl(sharedData)) {
        FW_LOG_INFO_PRINTF("[AUTO CALIB] CANCEL J%02u state=%u owner=%u enabled=%u\r\n",
                      ctx->jointIndex,
                      sharedData ? (unsigned)sharedData->system_state : 255,
                      sharedData ? (unsigned)sharedData->servo_target_owner : 255,
                      sharedData ? (unsigned)sharedData->control_enabled : 255);
        ctx->state = AUTO_CALIB_STATE_FAILED;
        return;
    }

    switch (ctx->state) {
        case AUTO_CALIB_STATE_INIT:
        {
            if (!sharedData ||
                ctx->jointIndex >= ENCODER_TOTAL_NUM ||
                sharedData->servo_target_owner != SERVO_TARGET_OWNER_CALIBRATION) {
                failAutoCalibContext(ctx, "invalid init or owner");
                return;
            }

            JointCalibrationConfig& cfg = g_jointCalibConfig[ctx->jointIndex];
            ctx->motionCfg = cfg;
            ctx->motionCfg.tightenSpeed = params.servoSpeed;
            ctx->motionCfg.tightenAcc = params.servoAcc;
            ctx->motionCfg.settleMs = params.loopMs;
            ctx->bus = jointMap[ctx->jointIndex].busIndex;
            ctx->servoId = jointMap[ctx->jointIndex].servoID;
            const int motorChannel = findMotorChannel(ctx->bus, ctx->servoId);
            if (motorChannel < 0 || motorChannel >= SERVO_TOTAL_NUM) {
                failAutoCalibContext(ctx, "primary motor channel not mapped");
                return;
            }
            ctx->motorChannel = (uint8_t)motorChannel;
            ctx->directionSign = getMotorTendonShortenAbsSign(ctx->motorChannel);
            if (!ctx->initStarted) {
                ctx->initStarted = true;
                ctx->stateBeginMs = nowMs;
                ctx->lastCommandMs = 0;
                ctx->lastTraceMs = 0;
                setRuntimeEncoderDirection(ctx->jointIndex,
                                           kDefaultEncoderDirection[ctx->jointIndex]);
                clearCalibratedMotorAbsLimitsForJoint(sharedData, ctx->jointIndex);
            }

            if (!isJointPrimaryServoMultiTurnKnown(sharedData, ctx->jointIndex)) {
                FW_LOG_INFO_PRINTF("[AUTO CALIB] WARN J%02u multi-turn unknown; auto calibration will rebuild zero from current relative abs unknown=0x%08lX\r\n",
                              ctx->jointIndex,
                              (unsigned long)sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap));
            }

            uint16_t raw = 0;
            int32_t encoder = 0;
            int32_t absNow = 0;
            int16_t rawServo = 0;
            const bool encoderOk =
                readLatestEncoderSample(sharedData, ctx->jointIndex, &raw, &encoder);
            const bool absOk =
                readLatestServoAbs(sharedData, ctx->bus, ctx->servoId, &absNow);
            const bool rawOk =
                readLatestServoRaw(sharedData, ctx->bus, ctx->servoId, &rawServo);
            if (!encoderOk || !absOk || !rawOk) {
                if ((nowMs - ctx->lastTraceMs) >= params.traceMs) {
                    ctx->lastTraceMs = nowMs;
                    logAutoInitialFeedbackDiag(sharedData,
                                               ctx->jointIndex,
                                               ctx->bus,
                                               ctx->servoId,
                                               encoderOk,
                                               absOk,
                                               rawOk);
                }
                if ((uint32_t)(nowMs - ctx->stateBeginMs) < kAutoInitialFeedbackWaitMs) {
                    return;
                }
                logAutoInitialFeedbackDiag(sharedData,
                                           ctx->jointIndex,
                                           ctx->bus,
                                           ctx->servoId,
                                           encoderOk,
                                           absOk,
                                           rawOk);
                failAutoCalibContextWithDiag(ctx,
                                             sharedData,
                                             "missing initial encoder/servo feedback after wait",
                                             !encoderOk ? CALIB_DIAG_REASON_ENCODER_ERROR : CALIB_DIAG_REASON_SERVO_OFFLINE,
                                             CALIB_DIAG_PHASE_INIT,
                                             ctx->motorChannel,
                                             encoderOk ? 1 : 0,
                                             absOk ? 1 : 0,
                                             rawOk ? 1 : 0,
                                             sharedData
                                                 ? sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap)
                                                 : 0);
                return;
            }

            const int32_t turn = servoTurnFromAbs(absNow);
            if (turn != 0) {
                FW_LOG_INFO_PRINTF("[AUTO CALIB] SKIP_NORMALIZE_TURN J%02u bus%u id%u abs=%ld raw=%d turn=%ld; calibrate from current abs\r\n",
                              ctx->jointIndex,
                              ctx->bus,
                              ctx->servoId,
                              (long)absNow,
                              (int)rawServo,
                              (long)turn);
            }

            if (!initTightenStateFromFeedback(ctx, sharedData, params)) {
                failAutoCalibContextWithDiag(ctx,
                                             sharedData,
                                             "missing feedback after init",
                                             CALIB_DIAG_REASON_FEEDBACK_LOST,
                                             CALIB_DIAG_PHASE_INIT,
                                             ctx->motorChannel,
                                             0,
                                             0,
                                             0,
                                             0);
            }
            return;
        }

        case AUTO_CALIB_STATE_TIGHTEN:
        {
            uint16_t currentRaw = 0;
            int32_t currentEncoder = 0;
            int32_t currentAbs = 0;
            if (!readLatestEncoderSample(sharedData, ctx->jointIndex, &currentRaw, &currentEncoder) ||
                !readLatestServoAbs(sharedData, ctx->bus, ctx->servoId, &currentAbs)) {
                failAutoCalibContextWithDiag(ctx,
                                             sharedData,
                                             "lost encoder/servo feedback",
                                             CALIB_DIAG_REASON_FEEDBACK_LOST,
                                             ctx->startFound ? CALIB_DIAG_PHASE_RELEASE : CALIB_DIAG_PHASE_FIND_MAX,
                                             ctx->motorChannel,
                                             0,
                                             0,
                                             0,
                                             0);
                return;
            }

            int16_t loadRaw = 0;
            const bool hasLoad = readLatestServoLoad(sharedData, ctx->bus, ctx->servoId, &loadRaw);
            const int16_t loadMag = hasLoad ? servoLoadMagnitude(loadRaw) : 0;
            const int32_t totalAbsDelta = abs32(currentAbs - ctx->startSearchAbs);
            const int32_t totalEncoderDelta =
                calibrationWrapEncoderDelta(currentEncoder, ctx->startSearchEncoder);
            const float totalAngleDeltaDeg =
                fabsf(calibrationEncoderCountToDeg(totalEncoderDelta));
            const bool loadHigh = hasLoad && loadMag >= params.tightLoadHigh;

            if ((nowMs - ctx->lastTraceMs) >= params.traceMs) {
                ctx->lastTraceMs = nowMs;
                FW_LOG_INFO_PRINTF("[AUTO CALIB] TRACE J%02u phase=%s cmd=%ld abs=%ld raw=%u oriented=%ld dCount=%ld totalAbs=%ld totalDeg=%.3f load=%d startFound=%u elapsed=%lu\r\n",
                              ctx->jointIndex,
                              ctx->startFound ? "RELEASE_TO_TIGHT" : "FIND_MAX",
                              (long)ctx->commandAbs,
                              (long)currentAbs,
                              (unsigned)currentRaw,
                              (long)currentEncoder,
                              (long)totalEncoderDelta,
                              (long)totalAbsDelta,
                              totalAngleDeltaDeg,
                              loadMag,
                              ctx->startFound ? 1 : 0,
                              (unsigned long)(nowMs - ctx->stateBeginMs));
            }

            if (!ctx->startFound) {
                bool maxFoundByStall = false;
                if ((nowMs - ctx->windowStartMs) >= params.windowMs) {
                    const int32_t windowAbsDelta = abs32(currentAbs - ctx->windowStartAbs);
                    const int32_t windowEncoderDelta =
                        calibrationWrapEncoderDelta(currentEncoder, ctx->windowStartEncoder);
                    const float windowAngleDeltaDeg =
                        fabsf(calibrationEncoderCountToDeg(windowEncoderDelta));
                    const bool servoStillTightening =
                        windowAbsDelta >= params.windowServoDeltaCounts;
                    const bool angleStalled =
                        windowAngleDeltaDeg <= params.stallActualDeg;
                    FW_LOG_INFO_PRINTF("[AUTO CALIB] CHECK_MAX J%02u abs=%ld raw=%u oriented=%ld dCount=%ld winCount=%ld winAbs=%ld/%ld winDeg=%.3f/%.3f totalDeg=%.3f load=%d/%d cond servo=%u angle=%u requireTravel=0 load=%u\r\n",
                                  ctx->jointIndex,
                                  (long)currentAbs,
                                  (unsigned)currentRaw,
                                  (long)currentEncoder,
                                  (long)totalEncoderDelta,
                                  (long)windowEncoderDelta,
                                  (long)windowAbsDelta,
                                  (long)params.windowServoDeltaCounts,
                                  windowAngleDeltaDeg,
                                  params.stallActualDeg,
                                  totalAngleDeltaDeg,
                                  loadMag,
                                  params.tightLoadHigh,
                                  servoStillTightening ? 1 : 0,
                                  angleStalled ? 1 : 0,
                                  loadHigh ? 1 : 0);

                    maxFoundByStall =
                        loadHigh && servoStillTightening && angleStalled;
                    ctx->windowStartAbs = currentAbs;
                    ctx->windowStartEncoder = currentEncoder;
                    ctx->windowStartMs = nowMs;
                }

                if (maxFoundByStall) {
                    ctx->encoderTravelSign = 1;
                    ctx->startFound = true;
                    ctx->startEncoder = currentEncoder;
                    ctx->startServoAbs = currentAbs;
                    ctx->windowStartEncoder = currentEncoder;
                    ctx->windowStartAbs = currentAbs;
                    ctx->windowStartMs = nowMs;
                    ctx->lastCommandMs = nowMs;
                    ctx->commandAbs = currentAbs;
                    ctx->commandRemainder = 0;
                    ctx->releaseCommandStarted = false;
                    ctx->preReleaseSyncTargetAbs = currentAbs;
                    ctx->preReleaseSyncWindowStartAbs = currentAbs;
                    ctx->preReleaseSyncWindowStartMs = nowMs;
                    ctx->preReleaseSyncStableSamples = 0;
                    ctx->releaseAngleStallStable = 0;
                    ctx->state = AUTO_CALIB_STATE_PRE_RELEASE_SYNC;
                    if (!appendJointCalibrationTargetToBatch(batch,
                                                             ctx->jointIndex,
                                                             clampToServoPosLocal(ctx->preReleaseSyncTargetAbs),
                                                             ctx->motionCfg)) {
                        failAutoCalibContextWithDiag(ctx,
                                                     sharedData,
                                                     "pre-release sync target append failed",
                                                     CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED,
                                                     CALIB_DIAG_PHASE_RELEASE,
                                                     ctx->motorChannel,
                                                     ctx->preReleaseSyncTargetAbs,
                                                     0,
                                                     0,
                                                     0);
                        return;
                    }
                    FW_LOG_INFO_PRINTF("[AUTO CALIB] START_FOUND J%02u maxDeg=dynamic raw=%u oriented=%ld abs=%ld load=%d byLoad=%u byStall=%u totalDeg=%.3f loadThreshold=%d encTravelSign=%d\r\n",
                                  ctx->jointIndex,
                                  (unsigned)currentRaw,
                                  (long)ctx->startEncoder,
                                  (long)ctx->startServoAbs,
                                  loadMag,
                                  loadHigh ? 1 : 0,
                                  maxFoundByStall ? 1 : 0,
                                  totalAngleDeltaDeg,
                                  params.tightLoadHigh,
                                  (int)ctx->encoderTravelSign);
                    FW_LOG_INFO_PRINTF("[AUTO CALIB] PRE_RELEASE_SYNC_BEGIN J%02u targetAbs=%ld tolerance=%ld settleDelta=%ld stable=%u\r\n",
                                  ctx->jointIndex,
                                  (long)ctx->preReleaseSyncTargetAbs,
                                  (long)kPreReleaseSyncTargetToleranceCounts,
                                  (long)kPreReleaseSyncSettleDeltaCounts,
                                  (unsigned)kPreReleaseSyncStableSamples);
                    return;
                }

                const uint32_t dtMs = nowMs - ctx->lastCommandMs;
                if (dtMs >= params.loopMs) {
                    const int32_t numerator =
                        (int32_t)dtMs * params.tightenSpeedCountsPerSec + ctx->commandRemainder;
                    const int32_t stepCounts = numerator / 1000;
                    ctx->commandRemainder = numerator % 1000;
                    if (stepCounts > 0) {
                        ctx->commandAbs = motorApplyTendonDelta(ctx->motorChannel,
                                                                ctx->commandAbs,
                                                                stepCounts);
                        if (!isServoCommandInsideLimit(ctx->commandAbs)) {
                            appendJointCalibrationTargetToBatch(batch,
                                                                ctx->jointIndex,
                                                                clampToServoPosLocal(ctx->commandAbs),
                                                                ctx->motionCfg);
                            failAutoCalibContextWithDiag(ctx,
                                                         sharedData,
                                                         "find max command outside +/-30719",
                                                         CALIB_DIAG_REASON_ABS_LIMIT_INVALID,
                                                         CALIB_DIAG_PHASE_FIND_MAX,
                                                         ctx->motorChannel,
                                                         ctx->commandAbs,
                                                         -30719,
                                                         30719,
                                                         0);
                            return;
                        }
                        if (!appendJointCalibrationTargetToBatch(batch,
                                                                 ctx->jointIndex,
                                                                 clampToServoPosLocal(ctx->commandAbs),
                                                                 ctx->motionCfg)) {
                            failAutoCalibContextWithDiag(ctx,
                                                         sharedData,
                                                         "find max target append failed",
                                                         CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED,
                                                         CALIB_DIAG_PHASE_FIND_MAX,
                                                         ctx->motorChannel,
                                                         ctx->commandAbs,
                                                         0,
                                                         0,
                                                         0);
                            return;
                        }
                    }
                    ctx->lastCommandMs = nowMs;
                }
                return;
            }
            const uint32_t dtMs = nowMs - ctx->lastCommandMs;
            if (dtMs >= params.loopMs) {
                const int32_t numerator =
                    (int32_t)dtMs * params.tightenSpeedCountsPerSec + ctx->commandRemainder;
                const int32_t stepCounts = numerator / 1000;
                    ctx->commandRemainder = numerator % 1000;
                    if (stepCounts > 0) {
                        ctx->commandAbs = motorApplyTendonDelta(ctx->motorChannel,
                                                            ctx->commandAbs,
                                                            -stepCounts);
                    if (!isServoCommandInsideLimit(ctx->commandAbs)) {
                        appendJointCalibrationTargetToBatch(batch,
                                                            ctx->jointIndex,
                                                            clampToServoPosLocal(ctx->commandAbs),
                                                            ctx->motionCfg);
                        failAutoCalibContextWithDiag(ctx,
                                                     sharedData,
                                                     "release command outside +/-30719",
                                                     CALIB_DIAG_REASON_ABS_LIMIT_INVALID,
                                                     CALIB_DIAG_PHASE_RELEASE,
                                                     ctx->motorChannel,
                                                     ctx->commandAbs,
                                                     -30719,
                                                     30719,
                                                     0);
                        return;
                    }
                    if (!appendJointCalibrationTargetToBatch(batch,
                                                             ctx->jointIndex,
                                                             clampToServoPosLocal(ctx->commandAbs),
                                                             ctx->motionCfg)) {
                        failAutoCalibContextWithDiag(ctx,
                                                     sharedData,
                                                     "release target append failed",
                                                     CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED,
                                                     CALIB_DIAG_PHASE_RELEASE,
                                                     ctx->motorChannel,
                                                     ctx->commandAbs,
                                                     0,
                                                     0,
                                                     0);
                        return;
                    }
                    if (!ctx->releaseCommandStarted) {
                        ctx->releaseCommandStarted = true;
                        FW_LOG_INFO_PRINTF("[AUTO CALIB] BEGIN_RELEASE J%02u fromMaxAbs=%ld firstTargetAbs=%ld step=%ld directionSign=%ld targetDeg=%.3f\r\n",
                                      ctx->jointIndex,
                                      (long)ctx->startServoAbs,
                                      (long)ctx->commandAbs,
                                      (long)stepCounts,
                                      (long)ctx->directionSign,
                                      kAutoCalibrationStartDeg);
                    }
                }
                ctx->lastCommandMs = nowMs;
            }

            if ((nowMs - ctx->windowStartMs) >= params.windowMs) {
                const int32_t windowAbsDelta = abs32(currentAbs - ctx->windowStartAbs);
                const int32_t windowEncoderDelta =
                    calibrationWrapEncoderDelta(currentEncoder, ctx->windowStartEncoder);
                const float windowAngleDeltaDeg =
                    fabsf(calibrationEncoderCountToDeg(windowEncoderDelta));
                const bool servoStillReleasing =
                    windowAbsDelta >= params.windowServoDeltaCounts;
                const bool angleStalled =
                    windowAngleDeltaDeg <= params.stallActualDeg;
                if (angleStalled) {
                    if (ctx->releaseAngleStallStable < 255) {
                        ctx->releaseAngleStallStable++;
                    }
                } else {
                    ctx->releaseAngleStallStable = 0;
                }
                const int32_t releaseFromMaxCount =
                    calibrationWrapEncoderDelta(currentEncoder, ctx->startEncoder);
                const float releaseFromMaxDeg =
                    fabsf(calibrationEncoderCountToDeg(releaseFromMaxCount));
                const bool enoughReleaseTravel =
                    releaseFromMaxDeg >= kReverseMinFollowDeg;

                FW_LOG_INFO_PRINTF("[AUTO CALIB] CHECK_RELEASE J%02u abs=%ld raw=%u oriented=%ld releaseCount=%ld releaseDeg=%.3f/%.3f winCount=%ld winAbs=%ld/%ld winDeg=%.3f/%.3f load=%d cond servo=%u angle=%u angleStable=%u/%u travel=%u\r\n",
                              ctx->jointIndex,
                              (long)currentAbs,
                              (unsigned)currentRaw,
                              (long)currentEncoder,
                              (long)releaseFromMaxCount,
                              releaseFromMaxDeg,
                              kReverseMinFollowDeg,
                              (long)windowEncoderDelta,
                              (long)windowAbsDelta,
                              (long)params.windowServoDeltaCounts,
                              windowAngleDeltaDeg,
                              params.stallActualDeg,
                              loadMag,
                              servoStillReleasing ? 1 : 0,
                              angleStalled ? 1 : 0,
                              (unsigned)ctx->releaseAngleStallStable,
                              (unsigned)kAngleStallStableSamples,
                              enoughReleaseTravel ? 1 : 0);

                if (servoStillReleasing &&
                    ctx->releaseAngleStallStable >= kAngleStallStableSamples &&
                    enoughReleaseTravel) {
                    ctx->zeroFound = true;
                    ctx->zeroEncoder = currentEncoder;
                    ctx->zeroServoAbs = currentAbs;
                    const int32_t signedTightToMax =
                        signedEncoderDeltaLocal(ctx->zeroEncoder, ctx->startEncoder);
                    ctx->encoderTravelSign = 1;

                    const int32_t maxEncoder = ctx->startEncoder;
                    const int32_t maxServoAbs = ctx->startServoAbs;
                    const int32_t tightEncoder = ctx->zeroEncoder;
                    const int32_t tightServoAbs = ctx->zeroServoAbs;
                    if (maxServoAbs == tightServoAbs) {
                        failAutoCalibContext(ctx, "zero find has no servo travel");
                        return;
                    }
                    const int32_t resultTautEncoder =
                        encoderForPositiveTravelResult(ctx->zeroEncoder, ctx->encoderTravelSign);
                    const int32_t resultTightEncoder =
                        encoderForPositiveTravelResult(ctx->startEncoder, ctx->encoderTravelSign);
                    const int32_t resultSpan =
                        calibrationForwardEncoderDelta(resultTautEncoder, resultTightEncoder);
                    const float dynamicTotalDeg = calibrationEncoderCountToDeg(resultSpan);
                    const float zeroFraction =
                        (dynamicTotalDeg > 0.0f)
                            ? ((kAutoCalibrationZeroDeg - kAutoCalibrationStartDeg) /
                               dynamicTotalDeg)
                            : 0.0f;
                    const int32_t resultZeroEncoder =
                        calibrationWrapEncoderCount(
                            resultTautEncoder +
                            roundToInt32Local((float)resultSpan * zeroFraction));
                    const int32_t servoZeroAbs =
                        motorApplyTendonDelta(ctx->motorChannel,
                                              tightServoAbs,
                                              kAutoServoZeroPullCounts);

                    JointCalibrationResult result;
                    if (!buildAutoZeroTightCalibrationResult(ctx->jointIndex,
                                                             resultTautEncoder,
                                                             resultZeroEncoder,
                                                             resultTightEncoder,
                                                             tightServoAbs,
                                                             servoZeroAbs,
                                                             maxServoAbs,
                                                             &result)) {
                        failAutoCalibContextWithDiag(ctx,
                                                     sharedData,
                                                     "invalid max/release result",
                                                     CALIB_DIAG_REASON_ZERO_CROSS_INVALID,
                                                     CALIB_DIAG_PHASE_RELEASE,
                                                     ctx->motorChannel,
                                                     resultTautEncoder,
                                                     resultZeroEncoder,
                                                     resultTightEncoder,
                                                     0);
                        return;
                    }
                    if (result.encoderSpan >= kAutoCalibrationRejectNearFullWrapSpan) {
                        FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL J%02u rejected wrap-like span=%ld start=%ld tight=%ld; endpoint span is invalid\r\n",
                                      ctx->jointIndex,
                                      (long)result.encoderSpan,
                                      (long)tightEncoder,
                                      (long)maxEncoder);
                        failAutoCalibContextWithDiag(ctx,
                                                     sharedData,
                                                     "rejected wrap-like span",
                                                     CALIB_DIAG_REASON_WRAP_TOO_LARGE,
                                                     CALIB_DIAG_PHASE_RELEASE,
                                                     ctx->motorChannel,
                                                     result.encoderSpan,
                                                     tightEncoder,
                                                     maxEncoder,
                                                     0);
                        return;
                    }

                    FW_LOG_INFO_PRINTF("[AUTO CALIB] ENCODER_RESULT J%02u defaultDir=%d runtimeDir=%d tightToMax=%ld resultTaut=%ld resultZero=%ld resultTight=%ld\r\n",
                                  ctx->jointIndex,
                                  (int)kDefaultEncoderDirection[ctx->jointIndex],
                                  (int)g_encoderDirection[ctx->jointIndex],
                                  (long)signedTightToMax,
                                  (long)resultTautEncoder,
                                  (long)resultZeroEncoder,
                                  (long)resultTightEncoder);

                    applyDynamicAutoCalibrationConfig(&g_jointCalibConfig[ctx->jointIndex], result);
                    g_jointCalibResult[ctx->jointIndex] = result;
                    updateCalibrationAssemblyWarning(sharedData,
                                                     ctx->jointIndex,
                                                     result);
                    g_encoderOffsetManual[ctx->jointIndex] = result.offset;
                    ctx->result = result;
                    ctx->resultValid = true;
                    ctx->targetAbs = currentAbs;
                    syncJointTargetToDeg(sharedData,
                                         ctx->jointIndex,
                                         result.currentMinDeg);
                    clearJointServoMultiTurnUnknown(sharedData, ctx->jointIndex);
                    setCalibratedMotorAbsLimit(sharedData,
                                               ctx->motorChannel,
                                               result.tightServoAbs);
                    ctx->state = AUTO_CALIB_STATE_DONE;
                    ctx->stateBeginMs = nowMs;
                    ctx->lastTraceMs = nowMs;

                    FW_LOG_INFO_PRINTF("[AUTO CALIB] TIGHT_FOUND J%02u startDeg=%.3f raw=%u encoder=%ld abs=%ld load=%d releaseDeg=%.3f winDeg=%.3f\r\n",
                                  ctx->jointIndex,
                                  result.startDeg,
                                  (unsigned)currentRaw,
                                  (long)tightEncoder,
                                  (long)tightServoAbs,
                                  loadMag,
                                  releaseFromMaxDeg,
                                  windowAngleDeltaDeg);
                    FW_LOG_INFO_PRINTF("[AUTO CALIB] AUTO_RANGE_DYNAMIC J%02u startDeg=%.3f zeroDeg=0.000 tightDeg=%.3f totalDeg=%.3f angleMax=%.3f span=%ld\r\n",
                                  ctx->jointIndex,
                                  result.startDeg,
                                  result.tightDeg,
                                  result.totalDeg,
                                  result.angleMax,
                                  (long)result.encoderSpan);
                    FW_LOG_INFO_PRINTF("[AUTO CALIB] ZERO_FOUND J%02u zeroDeg=%.3f encoder=%ld servoZero=%ld formula=tendon(tightAbs)+pullCounts(%ld) shortenSign=%ld\r\n",
                                  ctx->jointIndex,
                                  kAutoCalibrationZeroDeg,
                                  (long)result.zeroEncoder,
                                  (long)result.zeroServoAbs,
                                  (long)kAutoServoZeroPullCounts,
                                  (long)ctx->directionSign);
                    FW_LOG_INFO_PRINTF("[AUTO CALIB] RESULT J%02u start=%ld zero=%ld tight=%ld span=%ld servoStart=%ld servoZero=%ld servoTight=%ld\r\n",
                                  ctx->jointIndex,
                                  (long)result.startEncoder,
                                  (long)result.zeroEncoder,
                                  (long)result.tightEncoder,
                                  (long)result.encoderSpan,
                                  (long)result.startServoAbs,
                                  (long)result.zeroServoAbs,
                                  (long)result.tightServoAbs);
                    FW_LOG_INFO_PRINTF("[AUTO CALIB] SINGLE_DONE_AT_AUTO_MIN J%02u holdAbs=%ld currentMinDeg=%.3f; no return motion\r\n",
                                  ctx->jointIndex,
                                  (long)ctx->targetAbs,
                                  result.currentMinDeg);

                    if (!appendJointCalibrationTargetToBatch(
                            batch,
                            ctx->jointIndex,
                            clampToServoPosLocal(ctx->targetAbs),
                            ctx->motionCfg)) {
                        failAutoCalibContextWithDiag(
                            ctx,
                            sharedData,
                            "auto-min hold target append failed",
                            CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED,
                            CALIB_DIAG_PHASE_RELEASE,
                            ctx->motorChannel,
                            ctx->targetAbs,
                            0,
                            0,
                            0);
                    }
                    return;
                }

                ctx->windowStartAbs = currentAbs;
                ctx->windowStartEncoder = currentEncoder;
                ctx->windowStartMs = nowMs;
            }
            return;
        }

        case AUTO_CALIB_STATE_PRE_RELEASE_SYNC:
        {
            int32_t currentAbs = 0;
            if (!readLatestMotorServoAbs(sharedData, ctx->motorChannel, &currentAbs)) {
                failAutoCalibContextWithDiag(ctx,
                                             sharedData,
                                             "pre-release sync lost servo feedback",
                                             CALIB_DIAG_REASON_FEEDBACK_LOST,
                                             CALIB_DIAG_PHASE_RELEASE,
                                             ctx->motorChannel,
                                             0,
                                             0,
                                             0,
                                             0);
                return;
            }

            const int32_t targetError =
                abs32(currentAbs - ctx->preReleaseSyncTargetAbs);
            if (targetError > kPreReleaseSyncTargetToleranceCounts) {
                // A previous tighten trajectory is still being executed.  Replace
                // the hold target with the measured ABS and restart settling.
                ctx->preReleaseSyncTargetAbs = currentAbs;
                ctx->preReleaseSyncWindowStartAbs = currentAbs;
                ctx->preReleaseSyncWindowStartMs = nowMs;
                ctx->preReleaseSyncStableSamples = 0;
            }

            if (!appendJointCalibrationTargetToBatch(batch,
                                                     ctx->jointIndex,
                                                     clampToServoPosLocal(ctx->preReleaseSyncTargetAbs),
                                                     ctx->motionCfg)) {
                failAutoCalibContextWithDiag(ctx,
                                             sharedData,
                                             "pre-release sync target append failed",
                                             CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED,
                                             CALIB_DIAG_PHASE_RELEASE,
                                             ctx->motorChannel,
                                             ctx->preReleaseSyncTargetAbs,
                                             0,
                                             0,
                                             0);
                return;
            }

            if ((uint32_t)(nowMs - ctx->preReleaseSyncWindowStartMs) < params.windowMs) {
                return;
            }

            const int32_t windowAbsDelta =
                abs32(currentAbs - ctx->preReleaseSyncWindowStartAbs);
            const bool targetReached =
                abs32(currentAbs - ctx->preReleaseSyncTargetAbs) <=
                kPreReleaseSyncTargetToleranceCounts;
            const bool absSettled =
                windowAbsDelta <= kPreReleaseSyncSettleDeltaCounts;
            if (targetReached && absSettled) {
                if (ctx->preReleaseSyncStableSamples < 255) {
                    ctx->preReleaseSyncStableSamples++;
                }
            } else {
                ctx->preReleaseSyncStableSamples = 0;
            }

            FW_LOG_INFO_PRINTF("[AUTO CALIB] PRE_RELEASE_SYNC J%02u targetAbs=%ld currentAbs=%ld err=%ld/%ld windowAbs=%ld/%ld stable=%u/%u\r\n",
                          ctx->jointIndex,
                          (long)ctx->preReleaseSyncTargetAbs,
                          (long)currentAbs,
                          (long)abs32(currentAbs - ctx->preReleaseSyncTargetAbs),
                          (long)kPreReleaseSyncTargetToleranceCounts,
                          (long)windowAbsDelta,
                          (long)kPreReleaseSyncSettleDeltaCounts,
                          (unsigned)ctx->preReleaseSyncStableSamples,
                          (unsigned)kPreReleaseSyncStableSamples);

            if (ctx->preReleaseSyncStableSamples < kPreReleaseSyncStableSamples) {
                ctx->preReleaseSyncWindowStartAbs = currentAbs;
                ctx->preReleaseSyncWindowStartMs = nowMs;
                return;
            }

            ctx->commandAbs = currentAbs;
            ctx->commandRemainder = 0;
            ctx->windowStartAbs = currentAbs;
            ctx->windowStartMs = nowMs;
            ctx->lastCommandMs = nowMs;
            ctx->releaseAngleStallStable = 0;
            ctx->state = AUTO_CALIB_STATE_TIGHTEN;
            FW_LOG_INFO_PRINTF("[AUTO CALIB] PRE_RELEASE_SYNC_READY J%02u releaseStartAbs=%ld\r\n",
                          ctx->jointIndex,
                          (long)currentAbs);
            return;
        }

        default:
            return;
    }
}

bool runAutoCalibrationJoint(TaskSharedData_t* sharedData,
                             uint8_t jointIndex,
                             const AutoCalibrationParams& params)
{
    return runAutoCalibrationSequence(sharedData, &jointIndex, 1, params);
}

bool runAutoCalibrationSequence(TaskSharedData_t* sharedData,
                                const uint8_t* jointList,
                                uint8_t jointCount,
                                const AutoCalibrationParams& params)
{
    if (!sharedData || !jointList || jointCount == 0 || jointCount > ENCODER_TOTAL_NUM) {
        setCalibrationDiag(sharedData,
                           CALIB_DIAG_REASON_JOINT_LIST_INVALID,
                           CALIB_DIAG_PHASE_SEQUENCE,
                           0xFF,
                           0xFF,
                           jointCount,
                           ENCODER_TOTAL_NUM,
                           jointList ? 1 : 0,
                           0);
        return false;
    }

    clearDualWorkspaceCache();

    uint32_t seenJoints = 0;
    uint8_t orderedJoints[ENCODER_TOTAL_NUM] = {0};
    for (uint8_t i = 0; i < jointCount; i++) {
        if (jointList[i] >= ENCODER_TOTAL_NUM) {
            setCalibrationDiag(sharedData,
                               CALIB_DIAG_REASON_JOINT_LIST_INVALID,
                               CALIB_DIAG_PHASE_SEQUENCE,
                               jointList[i],
                               0xFF,
                               jointList[i],
                               ENCODER_TOTAL_NUM,
                               i,
                               seenJoints);
            return false;
        }
        const uint32_t bit = (1UL << jointList[i]);
        if ((seenJoints & bit) != 0) {
            FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL duplicate joint J%02u in sequence\r\n",
                          jointList[i]);
            setCalibrationDiag(sharedData,
                               CALIB_DIAG_REASON_DUPLICATE_JOINT,
                               CALIB_DIAG_PHASE_SEQUENCE,
                               jointList[i],
                               0xFF,
                               i,
                               jointCount,
                               0,
                               seenJoints);
            return false;
        }
        seenJoints |= bit;
        orderedJoints[i] = jointList[i];
    }

    for (uint8_t i = 0; i < jointCount; i++) {
        for (uint8_t j = (uint8_t)(i + 1); j < jointCount; j++) {
            if (autoCalibrationSequenceRank(orderedJoints[j]) <
                autoCalibrationSequenceRank(orderedJoints[i])) {
                const uint8_t tmp = orderedJoints[i];
                orderedJoints[i] = orderedJoints[j];
                orderedJoints[j] = tmp;
            }
        }
    }

    // Serial calibration is intentional: tightening one tendon changes the
    // mechanical load and encoder angle of neighboring joints, so parallel
    // calibration can bias the detected zero/tight points. Keep the per-joint
    // non-blocking state machine, but run one context to completion before the
    // next joint starts.
    uint32_t completedJointMask = 0;
    uint32_t failedJointMask = 0;
    uint8_t firstFailedJoint = 0xFF;

    for (uint8_t i = 0; i < jointCount; i++) {
        const uint32_t currentJointBit = (1UL << orderedJoints[i]);
        uint8_t dualPrimaryMotor = 0;
        uint8_t dualSecondaryMotor = 0;
        if (getDualAntagonistAutoCalibrationMotors(orderedJoints[i],
                                                   &dualPrimaryMotor,
                                                   &dualSecondaryMotor)) {
            const bool isWorkspaceTarget = isWorkspaceDualJoint(orderedJoints[i]);
            DualWorkspaceCacheEntry* workspaceState = NULL;
            if (isWorkspaceTarget) {
                if (!prepareWorkspaceForDualCalibration(sharedData,
                                                        orderedJoints[i],
                                                        params)) {
                    FW_LOG_INFO_PRINTF("[AUTO CALIB] SEQUENCE_SERIAL workspace failed J%02u; stop remaining sequence\r\n",
                                       orderedJoints[i]);
                    failedJointMask |= currentJointBit;
                    if (firstFailedJoint == 0xFF) {
                        firstFailedJoint = orderedJoints[i];
                    }
                    break;
                }
                const int8_t cacheIndex = workspaceCacheIndex(orderedJoints[i]);
                if (cacheIndex < 0) {
                    setWorkspacePlanFailureDiag(sharedData,
                                                orderedJoints[i],
                                                orderedJoints[i],
                                                DUAL_WORKSPACE_POSE_NONE,
                                                DUAL_WORKSPACE_PLAN_STAGE_EXECUTE);
                    failedJointMask |= currentJointBit;
                    if (firstFailedJoint == 0xFF) {
                        firstFailedJoint = orderedJoints[i];
                    }
                    break;
                }
                workspaceState = &g_dualWorkspaceCache[(uint8_t)cacheIndex];
                if (workspaceState->valid &&
                    workspaceState->currentPose == DUAL_WORKSPACE_POSE_B) {
                    FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_TARGET_B_CANDIDATE J%02u M%02u/M%02u; verify ABS before reuse\r\n",
                                       (unsigned)orderedJoints[i],
                                       (unsigned)dualPrimaryMotor,
                                       (unsigned)dualSecondaryMotor);
                } else {
                    FW_LOG_INFO_PRINTF("[AUTO CALIB] WORKSPACE_TARGET_DIRECT J%02u M%02u/M%02u; no target B preset\r\n",
                                       (unsigned)orderedJoints[i],
                                       (unsigned)dualPrimaryMotor,
                                       (unsigned)dualSecondaryMotor);
                }
            }
            FW_LOG_INFO_PRINTF("[AUTO CALIB] SEQUENCE_SERIAL begin J%02u (%u/%u) dual M%02u/M%02u\r\n",
                          orderedJoints[i],
                          (unsigned)(i + 1),
                          (unsigned)jointCount,
                          dualPrimaryMotor,
                          dualSecondaryMotor);
            if (!runDualAntagonistAutoCalibrationJoint(sharedData,
                                                       orderedJoints[i],
                                                       params,
                                                       workspaceState)) {
                FW_LOG_INFO_PRINTF("[AUTO CALIB] SEQUENCE_SERIAL failed J%02u dual\r\n",
                              orderedJoints[i]);
                if (!hasCalibrationDiag(sharedData)) {
                    setCalibrationDiag(sharedData,
                                       CALIB_DIAG_REASON_JOINT_STEP_FAILED,
                                       CALIB_DIAG_PHASE_DUAL_EXCHANGE,
                                       orderedJoints[i],
                                       dualPrimaryMotor,
                                       i,
                                       jointCount,
                                       dualSecondaryMotor,
                                       seenJoints);
                }
                failedJointMask |= currentJointBit;
                if (firstFailedJoint == 0xFF) {
                    firstFailedJoint = orderedJoints[i];
                }
                continue;
            }
            FW_LOG_INFO_PRINTF("[AUTO CALIB] SEQUENCE_SERIAL done J%02u dual\r\n",
                          orderedJoints[i]);
            completedJointMask |= currentJointBit;
            continue;
        }

        JointCalibContext ctx;
        initAutoCalibContext(&ctx, orderedJoints[i]);
        FW_LOG_INFO_PRINTF("[AUTO CALIB] SEQUENCE_SERIAL begin J%02u (%u/%u)\r\n",
                      orderedJoints[i],
                      (unsigned)(i + 1),
                      (unsigned)jointCount);

        while (ctx.state != AUTO_CALIB_STATE_DONE &&
               ctx.state != AUTO_CALIB_STATE_FAILED) {
            if (!autoCalibrationStillOwnsControl(sharedData)) {
                FW_LOG_INFO_PRINTF("[AUTO CALIB] CANCEL sequence state=%u owner=%u enabled=%u\r\n",
                              sharedData ? (unsigned)sharedData->system_state : 255,
                              sharedData ? (unsigned)sharedData->servo_target_owner : 255,
                              sharedData ? (unsigned)sharedData->control_enabled : 255);
                setCalibrationDiag(sharedData,
                                   CALIB_DIAG_REASON_CONTROL_LOST,
                                   CALIB_DIAG_PHASE_SEQUENCE,
                                   orderedJoints[i],
                                   0xFF,
                                   sharedData ? sharedData->system_state : -1,
                                   sharedData ? sharedData->servo_target_owner : -1,
                                   sharedData ? sharedData->control_enabled : -1,
                                   sharedData
                                       ? sharedAtomicLoad(&sharedData->active_joint_mask)
                                       : 0);
                return false;
            }

            ServoTargetBatch_t batch;
            memset(&batch, 0, sizeof(batch));
            batch.source = SERVO_TARGET_OWNER_CALIBRATION;
            batch.timestamp = millis();

            if (!appendCalibrationHoldTargetsExcept(sharedData,
                                                    &batch,
                                                    orderedJoints,
                                                    jointCount,
                                                    ctx.jointIndex,
                                                    failedJointMask,
                                                    params)) {
                FW_LOG_INFO_PRINTLN("[AUTO CALIB] FAIL sequence append non-active holds");
                setCalibrationDiag(sharedData,
                                   CALIB_DIAG_REASON_HOLD_TARGET_FAILED,
                                   CALIB_DIAG_PHASE_SEQUENCE,
                                   ctx.jointIndex,
                                   0xFF,
                                   i,
                                   jointCount,
                                   batch.count,
                                   seenJoints);
                return false;
            }

            stepAutoCalibrationJoint(&ctx, sharedData, params, &batch);

            if (!publishCalibrationBatch(sharedData, &batch)) {
                FW_LOG_INFO_PRINTLN("[AUTO CALIB] FAIL sequence publish target batch");
                setCalibrationDiag(sharedData,
                                   CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED,
                                   CALIB_DIAG_PHASE_SEQUENCE,
                                   ctx.jointIndex,
                                   0xFF,
                                   batch.count,
                                   sharedData ? sharedData->servo_target_owner : -1,
                                   sharedData ? sharedData->system_state : -1,
                                   seenJoints);
                return false;
            }

            vTaskDelay(pdMS_TO_TICKS(params.loopMs));
        }

        if (ctx.state == AUTO_CALIB_STATE_FAILED) {
            FW_LOG_INFO_PRINTF("[AUTO CALIB] SEQUENCE_SERIAL failed J%02u\r\n", orderedJoints[i]);
            if (!hasCalibrationDiag(sharedData)) {
                setCalibrationDiag(sharedData,
                                   CALIB_DIAG_REASON_JOINT_STEP_FAILED,
                                   CALIB_DIAG_PHASE_SEQUENCE,
                                   orderedJoints[i],
                                   ctx.motorChannel,
                                   ctx.state,
                                   ctx.releaseAngleStallStable,
                                   i,
                                   seenJoints);
            }
            failedJointMask |= currentJointBit;
            if (firstFailedJoint == 0xFF) {
                firstFailedJoint = orderedJoints[i];
            }
            continue;
        }

        FW_LOG_INFO_PRINTF("[AUTO CALIB] SEQUENCE_SERIAL done J%02u\r\n", orderedJoints[i]);
        completedJointMask |= currentJointBit;
    }

    if (failedJointMask != 0) {
        FW_LOG_INFO_PRINTF("[AUTO CALIB] SEQUENCE_SERIAL partial failed=0x%08lX completed=0x%08lX selected=0x%08lX\r\n",
                      (unsigned long)failedJointMask,
                      (unsigned long)completedJointMask,
                      (unsigned long)seenJoints);
        if (!hasCalibrationDiag(sharedData)) {
            setCalibrationDiag(sharedData,
                               CALIB_DIAG_REASON_JOINT_STEP_FAILED,
                               CALIB_DIAG_PHASE_SEQUENCE,
                               firstFailedJoint,
                               0xFF,
                               completedJointMask,
                               failedJointMask,
                               jointCount,
                               failedJointMask);
        }
        return false;
    }

    return true;
}

static bool startAutoCalibrationTaskInternal(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return false;
    }
    if (g_autoCalibrationTaskHandle != NULL) {
        FW_LOG_INFO_PRINTLN("[AUTO CALIB] task already running");
        return true;
    }
    clearCalibrationDiag(sharedData);

    const BaseType_t ok = xTaskCreate(autoCalibrationTask,
                                      "autoCalibrationTask",
                                      AUTO_CALIB_TASK_STACK_SIZE,
                                      sharedData,
                                      TASK_AUTO_CALIB_PRIORITY,
                                      &g_autoCalibrationTaskHandle);
    if (ok == pdPASS) {
        taskRuntimeRegister(
            TASK_RUNTIME_AUTO_CALIB,
            g_autoCalibrationTaskHandle,
            AUTO_CALIB_TASK_STACK_SIZE
        );
    }
    return ok == pdPASS;
}

bool startAutoCalibrationTask(TaskSharedData_t* sharedData)
{
    return startAutoCalibrationTaskInternal(sharedData);
}

void autoCalibrationTask(void* parameter)
{
    TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;
    const AutoCalibrationParams params = defaultAutoCalibrationParams();

    uint8_t activeJoints[ENCODER_TOTAL_NUM] = {0};
    const uint8_t activeJointCount =
        buildAutoCalibrationJointListFromActiveScope(sharedData,
                                                     activeJoints,
                                                     ENCODER_TOTAL_NUM);
    if (activeJointCount == 0) {
        FW_LOG_INFO_PRINTLN("[AUTO CALIB] FAIL active scope has no joints");
        setCalibrationDiag(sharedData,
                           CALIB_DIAG_REASON_ACTIVE_SCOPE_EMPTY,
                           CALIB_DIAG_PHASE_SEQUENCE,
                           0xFF,
                           0xFF,
                           sharedData
                               ? (int32_t)sharedAtomicLoad(&sharedData->active_joint_mask)
                               : 0,
                           sharedData
                               ? (int32_t)sharedAtomicLoad(&sharedData->active_motor_mask)
                               : 0,
                           0,
                           0);
    } else {
        char activeScopeLog[128] = {0};
        size_t used = 0;
        int written = snprintf(activeScopeLog,
                               sizeof(activeScopeLog),
                               "[AUTO CALIB] ACTIVE_SCOPE joints=");
        if (written > 0) {
            used = (size_t)written;
            if (used >= sizeof(activeScopeLog)) {
                used = sizeof(activeScopeLog) - 1;
            }
        }
        for (uint8_t i = 0; i < activeJointCount; i++) {
            if (used >= sizeof(activeScopeLog) - 1) {
                break;
            }
            written = snprintf(activeScopeLog + used,
                               sizeof(activeScopeLog) - used,
                               "%sJ%02u",
                               (i == 0) ? "" : ",",
                               activeJoints[i]);
            if (written <= 0) {
                break;
            }
            const size_t appended = (size_t)written;
            used += appended;
            if (used >= sizeof(activeScopeLog)) {
                used = sizeof(activeScopeLog) - 1;
                break;
            }
        }
        activeScopeLog[used] = '\0';
        FW_LOG_INFO_PRINTLN(activeScopeLog);
    }

    bool needsWorkspaceReadMask = false;
    for (uint8_t i = 0; i < activeJointCount; ++i) {
        if (isWorkspaceDualJoint(activeJoints[i])) {
            needsWorkspaceReadMask = true;
            break;
        }
    }
    uint32_t workspaceReadMask = 0;
    const bool workspaceReady =
        !needsWorkspaceReadMask ||
        buildDynamicWorkspaceMotorMask(sharedData,
                                       activeJoints,
                                       activeJointCount,
                                       &workspaceReadMask);
    setCalibrationWorkspaceReadMask(sharedData,
                                    workspaceReady ? workspaceReadMask : 0);
    const bool ok = (activeJointCount > 0) &&
                    workspaceReady &&
                    runAutoCalibrationSequence(sharedData,
                                               activeJoints,
                                               activeJointCount,
                                               params);
    setCalibrationWorkspaceReadMask(sharedData, 0);

    if (!ok && sharedData && !hasCalibrationDiag(sharedData)) {
        setCalibrationDiag(sharedData,
                           CALIB_DIAG_REASON_JOINT_STEP_FAILED,
                           CALIB_DIAG_PHASE_SEQUENCE,
                           0xFF,
                           0xFF,
                           0,
                           0,
                           0,
                           0);
    }

    postSystemEvent(sharedData, ok ? SYSTEM_EVENT_CALIBRATION_DONE : SYSTEM_EVENT_CALIBRATION_FAILED);
    taskRuntimeUnregister(TASK_RUNTIME_AUTO_CALIB);
    g_autoCalibrationTaskHandle = NULL;
    vTaskDelete(NULL);
}
