#include "AutoCalibrationTask.h"

#include <math.h>
#include <string.h>

#include "CalibrationMath.h"
#include "CalibrationTask.h"
#include "../hardware/HardwareMap.h"
#include "../shared/SharedSafetyState.h"
#include "../system/FirmwareLog.h"
#include "../system/StateMachineTask.h"
#include "../system/TaskRuntimeMonitor.h"

static TaskHandle_t g_autoCalibrationTaskHandle = NULL;
static volatile uint8_t g_autoCalibrationTaskMode = 0;

enum AutoCalibrationTaskMode {
    AUTO_CAL_TASK_MODE_SEQUENCE = 0,
    AUTO_CAL_TASK_MODE_SIDE_SWAY_ADJUST = 1
};

const uint8_t kDefaultAutoCalibrationJoints[] = {
    0, 1, 2, 3,
    4, 5, 6, 7,
    8, 9, 10, 11,
    12, 13, 14, 15,
    20, 16, 17, 18, 19
};
const uint8_t kDefaultAutoCalibrationJointsCount = sizeof(kDefaultAutoCalibrationJoints) / sizeof(kDefaultAutoCalibrationJoints[0]);
static const uint8_t kSideSwayAdjustJoints[] = {12, 8, 4, 0};
static const uint8_t kSideSwayAdjustJointsCount = sizeof(kSideSwayAdjustJoints) / sizeof(kSideSwayAdjustJoints[0]);

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
static const int32_t kAutoServoZeroPullToleranceCounts = 40;
static const uint32_t kAutoTightClampHoldMs = 120;
static const int32_t kDualAutoSlackCounts = 3200;
static const int32_t kDualSecondaryPreloadTargetLeadCounts = 4096 * 5;
static const int32_t kCalibratedMotorAbsWindowCounts = 4096 * 5;
static const int32_t kAutoCalibrationMaxReverseReleaseCounts = 4096 * 3;
static const int16_t kAutoCalibrationOverloadLoadThreshold = 800;
static const int16_t kDualPrimaryTightLoadThreshold = 190;
static const int16_t kDualSecondaryPreloadLoadThreshold = 190;
static const int16_t kSideSwayAdjustLoadThreshold = 190;
static const int32_t kSideSwaySecondaryPreReleaseCounts = 3200;
static const float kReverseMinFollowDeg = 12.0f;
static const float kDualTendonFitSampleStepDeg = 5.0f;
static const uint8_t kDualTendonRawSampleCapacity =
    DUAL_TENDON_LUT_MAX_SAMPLES * 3;
static const uint8_t kAngleStallStableSamples = 2;
static const float kReleaseMinDirectionDeg = 3.0f;
static const float kReleaseWrongWindowDeg = 1.0f;
static const uint32_t kReleaseWrongIgnoreMs = 600;
static const uint32_t kReleasePreHoldMs = 500;
static const uint8_t kReleaseDirectionStableSamples = 2;
static const uint8_t kReleaseWrongDirectionStableSamples = 3;
static const int32_t kReleaseServoWrongDirectionCounts = 120;
static const int32_t kSideSwayMaxCommandLeadCounts = 400;
static const float kAutoZeroMappedToleranceDeg = 6.0f;
static const float kAutoZeroCrossMaxAbsDeg = 160.0f;

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
    const int32_t maxAbs = (slackClamped > tightClamped) ? slackClamped : tightClamped;
    {
        SharedSafetyGuard guard;
        sharedData->motor_abs_limit_min[motorChannel] = minAbs;
        sharedData->motor_abs_limit_max[motorChannel] = maxAbs;
        sharedAtomicFetchOr(&sharedData->motor_abs_limit_valid_bitmap,
                            (uint32_t)(1UL << motorChannel));
        sharedAtomicFetchOr(&sharedData->motor_abs_limit_locked_bitmap,
                            (uint32_t)(1UL << motorChannel));
    }
    FW_LOG_INFO_PRINTF("[AUTO CALIB] ABS_LIMIT M%02u min=%ld max=%ld tight=%ld slack=%ld shortenSign=%d window=%ld\r\n",
                  (unsigned)motorChannel,
                  (long)minAbs,
                  (long)maxAbs,
                  (long)tightClamped,
                  (long)slackClamped,
                  (int)getMotorTendonShortenAbsSign(motorChannel),
                  (long)kCalibratedMotorAbsWindowCounts);
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
    (void)jointIndex;
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
    return out->encoderSpan > 0;
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

bool isAutoCalibrationFeedbackReady(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return false;
    }

    const uint32_t nowMs = millis();

    RemoteSensorData_t sensorData;
    if (!sharedData->canRxQueue ||
        xQueuePeek(sharedData->canRxQueue, &sensorData, 0) != pdTRUE ||
        !sensorData.isValid ||
        (uint32_t)(nowMs - sensorData.timestamp) > kAutoFeedbackReadyCanTimeoutMs) {
        return false;
    }

    ServoAngleData_t servoData;
    if (!sharedData->servoFeedbackQueue ||
        xQueuePeek(sharedData->servoFeedbackQueue, &servoData, 0) != pdTRUE ||
        (uint32_t)(nowMs - servoData.timestamp) > kAutoFeedbackReadyServoTimeoutMs) {
        return false;
    }

    ServoTelemetryData_t telemetry;
    QueueHandle_t telemetryQueue = sharedData->servoTelemetrySnapshotQueue
        ? sharedData->servoTelemetrySnapshotQueue
        : sharedData->servoTelemetryQueue;
    if (!telemetryQueue ||
        xQueuePeek(telemetryQueue, &telemetry, 0) != pdTRUE ||
        (uint32_t)(nowMs - telemetry.timestamp) > kAutoFeedbackReadyTelemetryTimeoutMs) {
        return false;
    }

    uint8_t activeJoints[ENCODER_TOTAL_NUM] = {0};
    const uint8_t activeJointCount =
        buildAutoCalibrationJointListFromActiveScope(sharedData,
                                                     activeJoints,
                                                     ENCODER_TOTAL_NUM);
    if (activeJointCount == 0) {
        return false;
    }

    for (uint8_t i = 0; i < activeJointCount; i++) {
        const uint8_t jointIndex = activeJoints[i];
        if (jointIndex >= ENCODER_TOTAL_NUM) {
            return false;
        }
        const uint32_t bit = (1UL << jointIndex);
        if (!encoderOnlineAndHealthy(sharedData, bit)) {
            return false;
        }

        JointActuatorGroup group;
        if (!getJointActuatorGroup(jointIndex, &group)) {
            return false;
        }
        for (uint8_t actuatorIndex = 0; actuatorIndex < group.count; actuatorIndex++) {
            const uint8_t motorChannel = group.actuators[actuatorIndex].motorChannel;
            if (motorChannel >= SERVO_TOTAL_NUM ||
                servoData.onlineStatus[motorChannel] == 0 ||
                telemetry.onlineStatus[motorChannel] == 0) {
                return false;
            }
        }
    }

    return true;
}

static void syncJointTargetToZero(TaskSharedData_t* sharedData, uint8_t jointIndex)
{
    if (!sharedData || jointIndex >= ENCODER_TOTAL_NUM) {
        return;
    }

    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }

    sharedData->targetAngles[jointIndex] = 0.0f;
    sharedData->joint_command_token++;

    if (lock) {
        xSemaphoreGive(lock);
    }
}

static bool readJointMappedDegFromCalibration(TaskSharedData_t* sharedData,
                                              uint8_t jointIndex,
                                              const JointCalibrationResult& result,
                                              float* outMappedDeg)
{
    if (!result.success ||
        !result.scaled ||
        result.encoderSpan <= 0 ||
        result.totalDeg <= 0.0f ||
        !outMappedDeg) {
        return false;
    }

    int32_t currentEncoder = 0;
    if (!readLatestEncoderOriented(sharedData, jointIndex, &currentEncoder)) {
        return false;
    }

    int32_t delta = calibrationForwardEncoderDelta(result.startEncoder, currentEncoder);
    const int32_t marginCounts = calibrationDegToEncoderCount(10.0f);
    if (delta > result.encoderSpan + marginCounts) {
        delta -= kCalibrationEncoderModulo;
    }

    *outMappedDeg =
        ((float)delta / (float)result.encoderSpan) * result.totalDeg + result.startDeg;
    return true;
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
    params.zeroReturnTimeoutMs = 30000;
    params.windowServoDeltaCounts = 50;
    params.zeroReachAbsTolerance = 200;
    params.followArmDeg = 3.3f;
    params.zeroFollowDeg = 5.0f;
    params.stallActualDeg = 0.75f;
    params.minTightTravelDeg = 5.0f;
    params.startLoadDelta = 25;
    params.tightLoadHigh = 250;
    params.startLoadStableSamples = 3;
    return params;
}

enum AutoCalibState {
    AUTO_CALIB_STATE_IDLE = 0,
    AUTO_CALIB_STATE_INIT,
    AUTO_CALIB_STATE_TIGHTEN,
    AUTO_CALIB_STATE_CLAMP_TIGHT,
    AUTO_CALIB_STATE_RETURN_ZERO,
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
    int8_t tightenEncoderSign;
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
    int32_t returnZeroStartAbs;
    int32_t windowStartEncoder;
    int32_t windowStartAbs;
    uint32_t windowStartMs;
    JointCalibrationResult result;
    bool resultValid;
    bool initStarted;
    bool releaseCommandStarted;
    uint32_t releaseStartMs;
    uint32_t preReleaseHoldStartMs;
    uint8_t releaseAngleStallStable;
    uint8_t releaseReverseStable;
    uint8_t releaseWrongDirectionStable;
    bool zeroMappedCaptured;
    int32_t zeroMappedCaptureAbs;
    float zeroMappedCaptureDeg;
    bool zeroMappedLastValid;
    int32_t zeroMappedLastAbs;
    int32_t zeroMappedLastReturnTravel;
    float zeroMappedLastDeg;
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

static bool publishDualMotorCalibrationTargets(TaskSharedData_t* sharedData,
                                               uint8_t movingMotor,
                                               int32_t movingTargetAbs,
                                               uint8_t holdMotor,
                                               int32_t holdTargetAbs,
                                               const AutoCalibrationParams& params);

static bool releaseSideSwaySecondaryByPresetCounts(TaskSharedData_t* sharedData,
                                                   uint8_t jointIndex,
                                                   uint8_t primaryMotor,
                                                   uint8_t secondaryMotor,
                                                   int32_t primaryHoldAbs,
                                                   const AutoCalibrationParams& params)
{
    int32_t secondaryStartAbs = 0;
    if (!readLatestMotorServoAbs(sharedData, secondaryMotor, &secondaryStartAbs)) {
        setCalibrationDiag(sharedData,
                           CALIB_DIAG_REASON_SERVO_FEEDBACK_TIMEOUT,
                           CALIB_DIAG_PHASE_DUAL_PRELOAD,
                           jointIndex,
                           secondaryMotor,
                           0,
                           0,
                           0,
                           1UL << secondaryMotor);
        FW_LOG_INFO_PRINTF("[AUTO CALIB] SIDE_ADJUST fail no secondary ABS J%02u M%02u\r\n",
                      (unsigned)jointIndex,
                      (unsigned)secondaryMotor);
        return false;
    }

    const int32_t secondaryReleaseAbs =
        motorApplyTendonDelta(secondaryMotor,
                              secondaryStartAbs,
                              -kSideSwaySecondaryPreReleaseCounts);
    if (!isServoCommandInsideLimit(secondaryReleaseAbs)) {
        setCalibrationDiag(sharedData,
                           CALIB_DIAG_REASON_ABS_LIMIT_INVALID,
                           CALIB_DIAG_PHASE_DUAL_PRELOAD,
                           jointIndex,
                           secondaryMotor,
                           secondaryReleaseAbs,
                           -30719,
                           30719,
                           1UL << secondaryMotor);
        FW_LOG_INFO_PRINTF("[AUTO CALIB] SIDE_ADJUST secondary release outside limit J%02u M%02u target=%ld\r\n",
                      (unsigned)jointIndex,
                      (unsigned)secondaryMotor,
                      (long)secondaryReleaseAbs);
        return false;
    }

    FW_LOG_INFO_PRINTF("[AUTO CALIB] SIDE_ADJUST release secondary J%02u secondary=M%02u startAbs=%ld targetAbs=%ld counts=%ld primaryHold=M%02u holdAbs=%ld\r\n",
                  (unsigned)jointIndex,
                  (unsigned)secondaryMotor,
                  (long)secondaryStartAbs,
                  (long)secondaryReleaseAbs,
                  (long)kSideSwaySecondaryPreReleaseCounts,
                  (unsigned)primaryMotor,
                  (long)primaryHoldAbs);

    uint32_t lastTraceMs = 0;
    while (autoCalibrationStillOwnsControl(sharedData)) {
        if (!publishDualMotorCalibrationTargets(sharedData,
                                                secondaryMotor,
                                                secondaryReleaseAbs,
                                                primaryMotor,
                                                primaryHoldAbs,
                                                params)) {
            return false;
        }

        int32_t secondaryAbs = 0;
        if (!readLatestMotorServoAbs(sharedData, secondaryMotor, &secondaryAbs)) {
            setCalibrationDiag(sharedData,
                               CALIB_DIAG_REASON_SERVO_FEEDBACK_TIMEOUT,
                               CALIB_DIAG_PHASE_DUAL_PRELOAD,
                               jointIndex,
                               secondaryMotor,
                               0,
                               0,
                               0,
                               1UL << secondaryMotor);
            return false;
        }

        const int32_t absErr = abs32(secondaryAbs - secondaryReleaseAbs);
        if (absErr <= params.zeroReachAbsTolerance) {
            FW_LOG_INFO_PRINTF("[AUTO CALIB] SIDE_ADJUST secondary released J%02u M%02u targetAbs=%ld currentAbs=%ld absErr=%ld\r\n",
                          (unsigned)jointIndex,
                          (unsigned)secondaryMotor,
                          (long)secondaryReleaseAbs,
                          (long)secondaryAbs,
                          (long)absErr);
            return true;
        }

        const uint32_t nowMs = millis();
        if ((uint32_t)(nowMs - lastTraceMs) >= params.traceMs) {
            lastTraceMs = nowMs;
            FW_LOG_INFO_PRINTF("[AUTO CALIB] SIDE_ADJUST secondary release wait J%02u M%02u targetAbs=%ld currentAbs=%ld absErr=%ld/%ld\r\n",
                          (unsigned)jointIndex,
                          (unsigned)secondaryMotor,
                          (long)secondaryReleaseAbs,
                          (long)secondaryAbs,
                          (long)absErr,
                          (long)params.zeroReachAbsTolerance);
        }

        vTaskDelay(pdMS_TO_TICKS(params.loopMs));
    }

    return false;
}

static bool adjustSideSwayJointToLoad(TaskSharedData_t* sharedData,
                                      uint8_t jointIndex,
                                      const AutoCalibrationParams& params)
{
    uint8_t primaryMotor = 0;
    uint8_t secondaryMotor = 0;
    if (!getDualAntagonistAutoCalibrationMotors(jointIndex, &primaryMotor, &secondaryMotor)) {
        FW_LOG_INFO_PRINTF("[AUTO CALIB] SIDE_ADJUST skip J%02u not dual side-sway\r\n",
                      (unsigned)jointIndex);
        return true;
    }
    const uint32_t sideSwayMotorMask = (1UL << primaryMotor) | (1UL << secondaryMotor);
    const uint32_t unknownMask = sharedData
        ? sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap)
        : sideSwayMotorMask;
    const uint32_t sideSwayUnknownMask = unknownMask & sideSwayMotorMask;
    if (sideSwayUnknownMask != 0) {
        setCalibrationDiag(sharedData,
                           CALIB_DIAG_REASON_TURN_UNKNOWN,
                           CALIB_DIAG_PHASE_DUAL_PRELOAD,
                           jointIndex,
                           (sideSwayUnknownMask & (1UL << primaryMotor)) ? primaryMotor : secondaryMotor,
                           (int32_t)unknownMask,
                           0,
                           0,
                           sideSwayUnknownMask);
        FW_LOG_INFO_PRINTF("[AUTO CALIB] SIDE_ADJUST fail Turn UNKNOWN J%02u mask=0x%08lX primary=M%02u secondary=M%02u\r\n",
                      (unsigned)jointIndex,
                      (unsigned long)sideSwayUnknownMask,
                      (unsigned)primaryMotor,
                      (unsigned)secondaryMotor);
        return false;
    }

    int32_t startAbs = 0;
    if (!readLatestMotorServoAbs(sharedData, primaryMotor, &startAbs)) {
        setCalibrationDiag(sharedData,
                           CALIB_DIAG_REASON_SERVO_FEEDBACK_TIMEOUT,
                           CALIB_DIAG_PHASE_DUAL_PRELOAD,
                           jointIndex,
                           primaryMotor,
                           0,
                           0,
                           0,
                           1UL << primaryMotor);
        FW_LOG_INFO_PRINTF("[AUTO CALIB] SIDE_ADJUST fail no ABS J%02u M%02u\r\n",
                      (unsigned)jointIndex,
                      (unsigned)primaryMotor);
        return false;
    }
    if (!releaseSideSwaySecondaryByPresetCounts(sharedData,
                                                jointIndex,
                                                primaryMotor,
                                                secondaryMotor,
                                                startAbs,
                                                params)) {
        return false;
    }

    int32_t commandAbs = startAbs;
    int32_t commandRemainder = 0;
    uint32_t lastCommandMs = millis();
    uint32_t lastTraceMs = 0;

    FW_LOG_INFO_PRINTF("[AUTO CALIB] SIDE_ADJUST begin J%02u primary=M%02u secondary=M%02u startAbs=%ld shortenSign=%d loadThreshold=%d\r\n",
                  (unsigned)jointIndex,
                  (unsigned)primaryMotor,
                  (unsigned)secondaryMotor,
                  (long)startAbs,
                  (int)getMotorTendonShortenAbsSign(primaryMotor),
                  (int)kSideSwayAdjustLoadThreshold);

    while (autoCalibrationStillOwnsControl(sharedData)) {
        const uint32_t nowMs = millis();
        int32_t currentAbs = 0;
        if (!readLatestMotorServoAbs(sharedData, primaryMotor, &currentAbs)) {
            setCalibrationDiag(sharedData,
                               CALIB_DIAG_REASON_SERVO_FEEDBACK_TIMEOUT,
                               CALIB_DIAG_PHASE_DUAL_PRELOAD,
                               jointIndex,
                               primaryMotor,
                               0,
                               0,
                               0,
                               1UL << primaryMotor);
            return false;
        }

        int16_t loadRaw = 0;
        if (!readLatestMotorServoLoad(sharedData, primaryMotor, &loadRaw)) {
            setCalibrationDiag(sharedData,
                               CALIB_DIAG_REASON_TELEMETRY_TIMEOUT,
                               CALIB_DIAG_PHASE_DUAL_PRELOAD,
                               jointIndex,
                               primaryMotor,
                               0,
                               0,
                               0,
                               1UL << primaryMotor);
            FW_LOG_INFO_PRINTF("[AUTO CALIB] SIDE_ADJUST fail no load J%02u M%02u\r\n",
                          (unsigned)jointIndex,
                          (unsigned)primaryMotor);
            return false;
        }

        const int16_t loadMag = servoLoadMagnitude(loadRaw);
        if (loadMag >= kSideSwayAdjustLoadThreshold) {
            if (!publishMotorCalibrationTarget(sharedData, primaryMotor, currentAbs, params)) {
                return false;
            }
            FW_LOG_INFO_PRINTF("[AUTO CALIB] SIDE_ADJUST reached J%02u M%02u abs=%ld load=%d rawLoad=%d travel=%ld\r\n",
                          (unsigned)jointIndex,
                          (unsigned)primaryMotor,
                          (long)currentAbs,
                          (int)loadMag,
                          (int)loadRaw,
                          (long)abs32(currentAbs - startAbs));
            return true;
        }

        const int32_t travelAbs = abs32(currentAbs - startAbs);
        const uint32_t dtMs = nowMs - lastCommandMs;
        if (dtMs >= params.loopMs) {
            const int32_t numerator =
                (int32_t)dtMs * params.tightenSpeedCountsPerSec + commandRemainder;
            const int32_t stepCounts = numerator / 1000;
            commandRemainder = numerator % 1000;
            if (stepCounts > 0) {
                commandAbs = motorApplyTendonDelta(primaryMotor, commandAbs, stepCounts);
                const int8_t shortenSign = getMotorTendonShortenAbsSign(primaryMotor);
                const int32_t commandLead =
                    (int32_t)shortenSign * (commandAbs - currentAbs);
                if (commandLead > kSideSwayMaxCommandLeadCounts) {
                    commandAbs = motorApplyTendonDelta(primaryMotor,
                                                       currentAbs,
                                                       kSideSwayMaxCommandLeadCounts);
                    commandRemainder = 0;
                }
                if (!isServoCommandInsideLimit(commandAbs)) {
                    setCalibrationDiag(sharedData,
                                       CALIB_DIAG_REASON_ABS_LIMIT_INVALID,
                                       CALIB_DIAG_PHASE_DUAL_PRELOAD,
                                       jointIndex,
                                       primaryMotor,
                                       commandAbs,
                                       -30719,
                                       30719,
                                       1UL << primaryMotor);
                    return false;
                }
                if (!publishMotorCalibrationTarget(sharedData, primaryMotor, commandAbs, params)) {
                    return false;
                }
            }
            lastCommandMs = nowMs;
        }

        if ((uint32_t)(nowMs - lastTraceMs) >= params.traceMs) {
            lastTraceMs = nowMs;
            FW_LOG_INFO_PRINTF("[AUTO CALIB] SIDE_ADJUST trace J%02u M%02u cmd=%ld abs=%ld load=%d/%d travel=%ld\r\n",
                          (unsigned)jointIndex,
                          (unsigned)primaryMotor,
                          (long)commandAbs,
                          (long)currentAbs,
                          (int)loadMag,
                          (int)kSideSwayAdjustLoadThreshold,
                          (long)travelAbs);
        }

        vTaskDelay(pdMS_TO_TICKS(params.loopMs));
    }

    FW_LOG_INFO_PRINTF("[AUTO CALIB] SIDE_ADJUST cancelled J%02u M%02u\r\n",
                  (unsigned)jointIndex,
                  (unsigned)primaryMotor);
    return false;
}

static bool runSideSwayPreAdjust(TaskSharedData_t* sharedData,
                                 const AutoCalibrationParams& params)
{
    if (!sharedData) {
        return false;
    }
    FW_LOG_INFO_PRINTLN("[AUTO CALIB] SIDE_ADJUST sequence begin");
    for (uint8_t i = 0; i < kSideSwayAdjustJointsCount; i++) {
        const uint8_t jointIndex = kSideSwayAdjustJoints[i];
        if (!adjustSideSwayJointToLoad(sharedData, jointIndex, params)) {
            FW_LOG_INFO_PRINTF("[AUTO CALIB] SIDE_ADJUST sequence failed J%02u\r\n",
                          (unsigned)jointIndex);
            return false;
        }
    }
    FW_LOG_INFO_PRINTLN("[AUTO CALIB] SIDE_ADJUST sequence complete");
    return true;
}

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
        if (!publishMotorCalibrationTarget(sharedData, motorChannel, targetAbs, params)) {
            return false;
        }

        int32_t currentAbs = 0;
        if (readLatestMotorServoAbs(sharedData, motorChannel, &currentAbs)) {
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

            const uint32_t nowMs = millis();
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
    const int32_t preloadTargetAbs = clampToServoAbsRangeLocal(
        motorApplyTendonDelta(secondaryMotor,
                              secondaryStartAbs,
                              kDualSecondaryPreloadTargetLeadCounts));
    uint32_t lastTraceMs = 0;

    FW_LOG_INFO_PRINTF("[AUTO CALIB] DUAL_PRELOAD_SECONDARY_BEGIN J%02u secondary=M%02u startAbs=%ld targetAbs=%ld lead=%ld shortenSign=%d primaryHold=M%02u holdAbs=%ld loadThreshold=%d\r\n",
                  jointIndex,
                  secondaryMotor,
                  (long)secondaryStartAbs,
                  (long)preloadTargetAbs,
                  (long)kDualSecondaryPreloadTargetLeadCounts,
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

        if (!publishDualMotorCalibrationTargets(sharedData,
                                                secondaryMotor,
                                                preloadTargetAbs,
                                                primaryMotor,
                                                primaryHoldAbs,
                                                params)) {
            return false;
        }

        if ((uint32_t)(nowMs - lastTraceMs) >= params.traceMs) {
            lastTraceMs = nowMs;
            FW_LOG_INFO_PRINTF("[AUTO CALIB] TRACE_DUAL_PRELOAD_SECONDARY J%02u M%02u target=%ld abs=%ld raw=%u oriented=%ld load=%d/%d primaryHoldAbs=%ld\r\n",
                          jointIndex,
                          secondaryMotor,
                          (long)preloadTargetAbs,
                          (long)currentAbs,
                          (unsigned)currentRaw,
                          (long)currentEncoder,
                          loadMag,
                          loadThreshold,
                          (long)primaryHoldAbs);
        }

        vTaskDelay(pdMS_TO_TICKS(params.loopMs));
    }
    FW_LOG_INFO_PRINTF("[AUTO CALIB] CANCEL DUAL_PRELOAD_SECONDARY J%02u secondary=M%02u primary=M%02u targetAbs=%ld holdAbs=%ld state=%u owner=%u enabled=%u\r\n",
                  jointIndex,
                  secondaryMotor,
                  primaryMotor,
                  (long)preloadTargetAbs,
                  (long)primaryHoldAbs,
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

    FW_LOG_INFO_PRINTF("[AUTO CALIB] DUAL_EXCHANGE_NEG3_BEGIN J%02u secondary=M%02u secondaryAbs=%ld secondaryShortenSign=%d primary=M%02u primaryAbs=%ld primaryShortenSign=%d requiredTravelDeg=%.3f\r\n",
                  jointIndex,
                  secondaryMotor,
                  (long)secondaryStartAbs,
                  (int)secondaryShortenSign,
                  primaryMotor,
                  (long)primaryStartAbs,
                  (int)primaryShortenSign,
                  kReverseMinFollowDeg);

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

            FW_LOG_INFO_PRINTF("[AUTO CALIB] CHECK_DUAL_EXCHANGE_NEG3 J%02u secondary=M%02u abs=%ld primary=M%02u abs=%ld raw=%u oriented=%ld dCount=%ld winCount=%ld winSecAbs=%ld/%ld winPriAbs=%ld/%ld winDeg=%.3f/%.3f totalDeg=%.3f/%.3f load=%d ignored cond sec=%u pri=%u angle=%u angleStable=%u/%u travel=%u\r\n",
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
                          secondaryStillReeling ? 1 : 0,
                          primaryStillPaying ? 1 : 0,
                          angleStalled ? 1 : 0,
                          (unsigned)angleStallStable,
                          (unsigned)kAngleStallStableSamples,
                          enoughEncoderTravel ? 1 : 0);

            limitFound =
                enoughEncoderTravel &&
                angleStallStable >= kAngleStallStableSamples &&
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
            FW_LOG_INFO_PRINTF("[AUTO CALIB] DUAL_EXCHANGE_NEG3_FOUND J%02u secondary=M%02u raw=%u oriented=%ld secondaryAbs=%ld primaryAbs=%ld totalDeg=%.3f load=%d ignored\r\n",
                          jointIndex,
                          secondaryMotor,
                          (unsigned)currentRaw,
                          (long)currentEncoder,
                          (long)currentSecondaryAbs,
                          (long)currentPrimaryAbs,
                          totalAngleDeltaDeg,
                          loadMag);
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
            FW_LOG_INFO_PRINTF("[AUTO CALIB] TRACE_DUAL_EXCHANGE_NEG3 J%02u secondary=M%02u cmd=%ld abs=%ld primary=M%02u cmd=%ld abs=%ld raw=%u oriented=%ld totalDeg=%.3f load=%d ignored\r\n",
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
                          loadMag);
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
                                                  const AutoCalibrationParams& params)
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

    FW_LOG_INFO_PRINTF("[AUTO CALIB] DUAL_BEGIN J%02u primary=M%02u primaryShortenSign=%d secondary=M%02u secondaryShortenSign=%d slack=%ld primaryLoad=%d secondaryLoad=%d startDeg=%.3f tightDeg=dynamic initialEncoder=%ld primaryAbs=%ld secondaryAbs=%ld\r\n",
                  jointIndex,
                  primaryMotor,
                  (int)primaryShortenSign,
                  secondaryMotor,
                  (int)secondaryShortenSign,
                  (long)kDualAutoSlackCounts,
                  primaryTightLoadThreshold,
                  secondaryPreloadLoadThreshold,
                  kAutoCalibrationStartDeg,
                  (long)initialEncoder,
                  (long)primaryAbs,
                  (long)secondaryAbs);

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

    int32_t primary80Encoder = 0;
    int32_t primary80Abs = 0;
    uint16_t primary80Raw = 0;
    int16_t primary80Load = 0;
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

    const int32_t signedSpan = signedEncoderDeltaLocal(neg3Encoder, primary80Encoder);
    const int8_t encoderTravelSign = 1;
    if (signedSpan <= 0) {
        FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL DUAL_FIXED_DIR J%02u defaultDir=%d runtimeDir=%d neg3=%ld pos80=%ld signed=%ld; check encoder direction table\r\n",
                      jointIndex,
                      (int)kDefaultEncoderDirection[jointIndex],
                      (int)g_encoderDirection[jointIndex],
                      (long)neg3Encoder,
                      (long)primary80Encoder,
                      (long)signedSpan);
        setCalibrationDiag(sharedData,
                           CALIB_DIAG_REASON_WRONG_DIRECTION,
                           CALIB_DIAG_PHASE_DUAL_EXCHANGE,
                           jointIndex,
                           0xFF,
                           signedSpan,
                           kDefaultEncoderDirection[jointIndex],
                           g_encoderDirection[jointIndex],
                           (uint32_t)(1UL << jointIndex));
        return false;
    }
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

    FW_LOG_INFO_PRINTF("[AUTO CALIB] DUAL_ENCODER_DIR_FIXED J%02u defaultDir=%d runtimeDir=%d signed=%ld resultStart=%ld resultZero=%ld resultTight=%ld\r\n",
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
    g_encoderOffsetManual[jointIndex] = result.offset;
    syncJointTargetToZero(sharedData, jointIndex);
    clearJointServoMultiTurnUnknown(sharedData, jointIndex);
    setCalibratedMotorAbsLimit(sharedData, primaryMotor, primary80Abs);
    setCalibratedMotorAbsLimit(sharedData, secondaryMotor, secondaryNeg3Abs);

    FW_LOG_INFO_PRINTF("[AUTO CALIB] DUAL_RESULT J%02u neg3Raw=%u neg3=%ld pos80Raw=%u pos80=%ld span=%ld zero=%ld primary80Abs=%ld primaryNeg3Abs=%ld secondaryPreloadRaw=%u secondaryPreloadEncoder=%ld secondaryPreloadAbs=%ld secondaryNeg3Abs=%ld loadPrimary=%d loadSecondaryPreload=%d loadSecondaryNeg3=%d\r\n",
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
                  neg3Load);
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

    FW_LOG_INFO_PRINTF("[AUTO CALIB] AUTO_RANGE_DYNAMIC_BEGIN J%02u startDeg=%.3f zeroDeg=0.000 tightDeg=dynamic totalDeg=dynamic zeroFollowDeg=%.3f angleMax=dynamic\r\n",
                  ctx->jointIndex,
                  kAutoCalibrationStartDeg,
                  params.zeroFollowDeg);
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
                    ctx->tightenEncoderSign = 1;
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
                    ctx->releaseStartMs = 0;
                    ctx->preReleaseHoldStartMs = nowMs;
                    ctx->releaseAngleStallStable = 0;
                    ctx->releaseReverseStable = 0;
                    ctx->releaseWrongDirectionStable = 0;
                    if (!appendJointCalibrationTargetToBatch(batch,
                                                             ctx->jointIndex,
                                                             clampToServoPosLocal(ctx->commandAbs),
                                                             ctx->motionCfg)) {
                        failAutoCalibContextWithDiag(ctx,
                                                     sharedData,
                                                     "pre-release hold target append failed",
                                                     CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED,
                                                     CALIB_DIAG_PHASE_RELEASE,
                                                     ctx->motorChannel,
                                                     ctx->commandAbs,
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
                    FW_LOG_INFO_PRINTF("[AUTO CALIB] PRE_RELEASE_HOLD J%02u holdAbs=%ld holdMs=%lu\r\n",
                                  ctx->jointIndex,
                                  (long)ctx->commandAbs,
                                  (unsigned long)kReleasePreHoldMs);
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

            if (ctx->preReleaseHoldStartMs != 0) {
                const uint32_t holdElapsedMs = nowMs - ctx->preReleaseHoldStartMs;
                if (holdElapsedMs < kReleasePreHoldMs) {
                    const uint32_t holdDtMs = nowMs - ctx->lastCommandMs;
                    if (holdDtMs >= params.loopMs) {
                        if (!appendJointCalibrationTargetToBatch(batch,
                                                                 ctx->jointIndex,
                                                                 clampToServoPosLocal(ctx->commandAbs),
                                                                 ctx->motionCfg)) {
                            failAutoCalibContextWithDiag(ctx,
                                                         sharedData,
                                                         "pre-release hold target append failed",
                                                         CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED,
                                                         CALIB_DIAG_PHASE_RELEASE,
                                                         ctx->motorChannel,
                                                         ctx->commandAbs,
                                                         0,
                                                         0,
                                                         0);
                            return;
                        }
                        ctx->lastCommandMs = nowMs;
                    }
                    return;
                }

                ctx->preReleaseHoldStartMs = 0;
                ctx->windowStartEncoder = currentEncoder;
                ctx->windowStartAbs = currentAbs;
                ctx->windowStartMs = nowMs;
                ctx->lastCommandMs = nowMs;
                ctx->commandRemainder = 0;
                FW_LOG_INFO_PRINTF("[AUTO CALIB] END_PRE_RELEASE_HOLD J%02u holdAbs=%ld currentAbs=%ld elapsed=%lu\r\n",
                              ctx->jointIndex,
                              (long)ctx->commandAbs,
                              (long)currentAbs,
                              (unsigned long)holdElapsedMs);
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
                        ctx->releaseStartMs = nowMs;
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

            const int32_t actualReleaseTendonDelta =
                motorRawAbsToTendonAbs(ctx->motorChannel, currentAbs) -
                motorRawAbsToTendonAbs(ctx->motorChannel, ctx->startServoAbs);
            const int32_t commandReleaseTendonDelta =
                motorRawAbsToTendonAbs(ctx->motorChannel, ctx->commandAbs) -
                motorRawAbsToTendonAbs(ctx->motorChannel, ctx->startServoAbs);
            const int32_t actualReleaseSlackCounts =
                (actualReleaseTendonDelta < 0) ? -actualReleaseTendonDelta : 0;
            const int32_t commandReleaseSlackCounts =
                (commandReleaseTendonDelta < 0) ? -commandReleaseTendonDelta : 0;
            const int32_t maxReleaseSlackCounts =
                (actualReleaseSlackCounts > commandReleaseSlackCounts)
                    ? actualReleaseSlackCounts
                    : commandReleaseSlackCounts;
            if (ctx->releaseCommandStarted &&
                maxReleaseSlackCounts > kAutoCalibrationMaxReverseReleaseCounts) {
                FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL J%02u over-release after max load slack=%ld/%ld actualSlack=%ld commandSlack=%ld actualTendon=%ld commandTendon=%ld startAbs=%ld currentAbs=%ld cmd=%ld loadThreshold=%d\r\n",
                              ctx->jointIndex,
                              (long)maxReleaseSlackCounts,
                              (long)kAutoCalibrationMaxReverseReleaseCounts,
                              (long)actualReleaseSlackCounts,
                              (long)commandReleaseSlackCounts,
                              (long)actualReleaseTendonDelta,
                              (long)commandReleaseTendonDelta,
                              (long)ctx->startServoAbs,
                              (long)currentAbs,
                              (long)ctx->commandAbs,
                              params.tightLoadHigh);
                failAutoCalibContextWithDiag(ctx,
                                             sharedData,
                                             "over-release after max load",
                                             CALIB_DIAG_REASON_OVER_RELEASE,
                                             CALIB_DIAG_PHASE_RELEASE,
                                             ctx->motorChannel,
                                             maxReleaseSlackCounts,
                                             kAutoCalibrationMaxReverseReleaseCounts,
                                             actualReleaseSlackCounts,
                                             (uint32_t)commandReleaseSlackCounts);
                return;
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
                const int8_t tightenSign =
                    (ctx->tightenEncoderSign < 0) ? -1 : 1;
                const int32_t signedReleaseAlongTighten =
                    releaseFromMaxCount * (int32_t)tightenSign;
                const float signedReleaseAlongTightenDeg =
                    calibrationEncoderCountToDeg(signedReleaseAlongTighten);
                const int32_t releaseDirectionThresholdCount =
                    calibrationDegToEncoderCount(kReleaseMinDirectionDeg);
                const int32_t releaseWrongWindowThresholdCount =
                    calibrationDegToEncoderCount(kReleaseWrongWindowDeg);
                const int32_t windowAlongTighten =
                    windowEncoderDelta * (int32_t)tightenSign;
                const float windowAlongTightenDeg =
                    calibrationEncoderCountToDeg(windowAlongTighten);
                const bool releaseOppositeEnough =
                    signedReleaseAlongTighten <= -releaseDirectionThresholdCount;
                const bool releaseWrongCheckArmed =
                    ctx->releaseCommandStarted &&
                    ((uint32_t)(nowMs - ctx->releaseStartMs) >= kReleaseWrongIgnoreMs);
                const bool releaseServoWrongDirection =
                    releaseWrongCheckArmed &&
                    commandReleaseTendonDelta <= -kReleaseServoWrongDirectionCounts &&
                    actualReleaseTendonDelta >= kReleaseServoWrongDirectionCounts;
                const bool releaseWrongDirection =
                    releaseWrongCheckArmed &&
                    servoStillReleasing &&
                    signedReleaseAlongTighten >= releaseDirectionThresholdCount &&
                    windowAlongTighten >= releaseWrongWindowThresholdCount;
                const bool enoughReleaseTravel =
                    releaseFromMaxDeg >= kReverseMinFollowDeg;
                if (releaseOppositeEnough) {
                    if (ctx->releaseReverseStable < 255) {
                        ctx->releaseReverseStable++;
                    }
                } else {
                    ctx->releaseReverseStable = 0;
                }
                if (releaseWrongDirection) {
                    if (ctx->releaseWrongDirectionStable < 255) {
                        ctx->releaseWrongDirectionStable++;
                    }
                } else {
                    ctx->releaseWrongDirectionStable = 0;
                }

                FW_LOG_INFO_PRINTF("[AUTO CALIB] CHECK_RELEASE J%02u abs=%ld raw=%u oriented=%ld releaseCount=%ld releaseDeg=%.3f/%.3f tightenSign=%d alongTighten=%ld alongDeg=%.3f slack=%ld/%ld actualSlack=%ld commandSlack=%ld winCount=%ld winAlong=%ld winAlongDeg=%.3f/%.3f winAbs=%ld/%ld winDeg=%.3f/%.3f load=%d cond servo=%u angle=%u angleStable=%u/%u travel=%u reverse=%u reverseStable=%u/%u wrong=%u wrongArmed=%u wrongStable=%u/%u\r\n",
                              ctx->jointIndex,
                              (long)currentAbs,
                              (unsigned)currentRaw,
                              (long)currentEncoder,
                              (long)releaseFromMaxCount,
                              releaseFromMaxDeg,
                              kReverseMinFollowDeg,
                              (int)tightenSign,
                              (long)signedReleaseAlongTighten,
                              signedReleaseAlongTightenDeg,
                              (long)maxReleaseSlackCounts,
                              (long)kAutoCalibrationMaxReverseReleaseCounts,
                              (long)actualReleaseSlackCounts,
                              (long)commandReleaseSlackCounts,
                              (long)windowEncoderDelta,
                              (long)windowAlongTighten,
                              windowAlongTightenDeg,
                              kReleaseWrongWindowDeg,
                              (long)windowAbsDelta,
                              (long)params.windowServoDeltaCounts,
                              windowAngleDeltaDeg,
                              params.stallActualDeg,
                              loadMag,
                              servoStillReleasing ? 1 : 0,
                              angleStalled ? 1 : 0,
                              (unsigned)ctx->releaseAngleStallStable,
                              (unsigned)kAngleStallStableSamples,
                              enoughReleaseTravel ? 1 : 0,
                              releaseOppositeEnough ? 1 : 0,
                              (unsigned)ctx->releaseReverseStable,
                              (unsigned)kReleaseDirectionStableSamples,
                              releaseWrongDirection ? 1 : 0,
                              releaseWrongCheckArmed ? 1 : 0,
                              (unsigned)ctx->releaseWrongDirectionStable,
                              (unsigned)kReleaseWrongDirectionStableSamples);

                if (releaseServoWrongDirection) {
                    FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL J%02u release servo moved with tighten direction actualTendon=%ld commandTendon=%ld threshold=%ld currentAbs=%ld startAbs=%ld cmd=%ld shortenSign=%d\r\n",
                                  ctx->jointIndex,
                                  (long)actualReleaseTendonDelta,
                                  (long)commandReleaseTendonDelta,
                                  (long)kReleaseServoWrongDirectionCounts,
                                  (long)currentAbs,
                                  (long)ctx->startServoAbs,
                                  (long)ctx->commandAbs,
                                  (int)ctx->directionSign);
                    failAutoCalibContextWithDiag(ctx,
                                                 sharedData,
                                                 "release servo moved with tighten direction",
                                                 CALIB_DIAG_REASON_WRONG_DIRECTION,
                                                 CALIB_DIAG_PHASE_RELEASE,
                                                 ctx->motorChannel,
                                                 actualReleaseTendonDelta,
                                                 commandReleaseTendonDelta,
                                                 kReleaseServoWrongDirectionCounts,
                                                 0);
                    return;
                }

                if (ctx->releaseWrongDirectionStable >= kReleaseWrongDirectionStableSamples) {
                    FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL J%02u release moved with tighten direction count=%ld deg=%.3f tightenSign=%d stable=%u/%u\r\n",
                                  ctx->jointIndex,
                                  (long)signedReleaseAlongTighten,
                                  signedReleaseAlongTightenDeg,
                                  (int)tightenSign,
                                  (unsigned)ctx->releaseWrongDirectionStable,
                                  (unsigned)kReleaseWrongDirectionStableSamples);
                    failAutoCalibContextWithDiag(ctx,
                                                 sharedData,
                                                 "release moved with tighten direction",
                                                 CALIB_DIAG_REASON_WRONG_DIRECTION,
                                                 CALIB_DIAG_PHASE_RELEASE,
                                                 ctx->motorChannel,
                                                 signedReleaseAlongTighten,
                                                 releaseDirectionThresholdCount,
                                                 ctx->releaseWrongDirectionStable,
                                                 0);
                    return;
                }

                if (servoStillReleasing &&
                    ctx->releaseAngleStallStable >= kAngleStallStableSamples &&
                    enoughReleaseTravel &&
                    releaseOppositeEnough &&
                    ctx->releaseReverseStable >= kReleaseDirectionStableSamples) {
                    ctx->zeroFound = true;
                    ctx->zeroEncoder = currentEncoder;
                    ctx->zeroServoAbs = currentAbs;
                    const int32_t signedTightToMax =
                        signedEncoderDeltaLocal(ctx->zeroEncoder, ctx->startEncoder);
                    ctx->encoderTravelSign = 1;
                    if (signedTightToMax <= 0) {
                        FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL J%02u fixed encoder direction result reversed tightToMax=%ld defaultDir=%d taut=%ld tight=%ld\r\n",
                                      ctx->jointIndex,
                                      (long)signedTightToMax,
                                      (int)kDefaultEncoderDirection[ctx->jointIndex],
                                      (long)ctx->zeroEncoder,
                                      (long)ctx->startEncoder);
                        failAutoCalibContextWithDiag(ctx,
                                                     sharedData,
                                                     "fixed encoder direction result reversed",
                                                     CALIB_DIAG_REASON_WRONG_DIRECTION,
                                                     CALIB_DIAG_PHASE_RELEASE,
                                                     ctx->motorChannel,
                                                     signedTightToMax,
                                                     kDefaultEncoderDirection[ctx->jointIndex],
                                                     ctx->startEncoder,
                                                     0);
                        return;
                    }

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
                        FW_LOG_INFO_PRINTF("[AUTO CALIB] FAIL J%02u rejected wrap-like span=%ld start=%ld tight=%ld; check servo tighten direction\r\n",
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

                    FW_LOG_INFO_PRINTF("[AUTO CALIB] ENCODER_DIR_FIXED J%02u defaultDir=%d runtimeDir=%d tightToMax=%ld resultTaut=%ld resultZero=%ld resultTight=%ld\r\n",
                                  ctx->jointIndex,
                                  (int)kDefaultEncoderDirection[ctx->jointIndex],
                                  (int)g_encoderDirection[ctx->jointIndex],
                                  (long)signedTightToMax,
                                  (long)resultTautEncoder,
                                  (long)resultZeroEncoder,
                                  (long)resultTightEncoder);

                    applyDynamicAutoCalibrationConfig(&g_jointCalibConfig[ctx->jointIndex], result);
                    g_jointCalibResult[ctx->jointIndex] = result;
                    g_encoderOffsetManual[ctx->jointIndex] = result.offset;
                    ctx->result = result;
                    ctx->resultValid = true;
                    ctx->targetAbs = currentAbs;
                    ctx->returnZeroStartAbs = tightServoAbs;
                    ctx->zeroMappedCaptured = false;
                    ctx->zeroMappedCaptureAbs = 0;
                    ctx->zeroMappedCaptureDeg = NAN;
                    ctx->zeroMappedLastValid = false;
                    ctx->zeroMappedLastAbs = 0;
                    ctx->zeroMappedLastReturnTravel = 0;
                    ctx->zeroMappedLastDeg = NAN;
                    ctx->state = AUTO_CALIB_STATE_CLAMP_TIGHT;
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
                    FW_LOG_INFO_PRINTF("[AUTO CALIB] CLAMP_TIGHT J%02u targetAbs=%ld zeroTargetAbs=%ld holdMs=%lu speed=%u acc=%u\r\n",
                                  ctx->jointIndex,
                                  (long)ctx->targetAbs,
                                  (long)result.zeroServoAbs,
                                  (unsigned long)kAutoTightClampHoldMs,
                                  ctx->motionCfg.tightenSpeed,
                                  ctx->motionCfg.tightenAcc);

                    appendJointCalibrationTargetToBatch(batch,
                                                        ctx->jointIndex,
                                                        clampToServoPosLocal(ctx->targetAbs),
                                                        ctx->motionCfg);
                    return;
                }

                ctx->windowStartAbs = currentAbs;
                ctx->windowStartEncoder = currentEncoder;
                ctx->windowStartMs = nowMs;
            }
            return;
        }

        case AUTO_CALIB_STATE_CLAMP_TIGHT:
        {
            if (!ctx->resultValid) {
                failAutoCalibContextWithDiag(ctx,
                                             sharedData,
                                             "clamp tight without result",
                                             CALIB_DIAG_REASON_INTERNAL_ERROR,
                                             CALIB_DIAG_PHASE_RETURN_ZERO,
                                             ctx->motorChannel,
                                             0,
                                             0,
                                             0,
                                             0);
                return;
            }
            if (!appendJointCalibrationTargetToBatch(batch,
                                                     ctx->jointIndex,
                                                     clampToServoPosLocal(ctx->targetAbs),
                                                     ctx->motionCfg)) {
                failAutoCalibContextWithDiag(ctx,
                                             sharedData,
                                             "clamp tight target append failed",
                                             CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED,
                                             CALIB_DIAG_PHASE_RETURN_ZERO,
                                             ctx->motorChannel,
                                             ctx->targetAbs,
                                             0,
                                             0,
                                             0);
                return;
            }
            if ((uint32_t)(nowMs - ctx->stateBeginMs) < kAutoTightClampHoldMs) {
                return;
            }

            int32_t clampCurrentAbs = ctx->returnZeroStartAbs;
            if (readLatestServoAbs(sharedData, ctx->bus, ctx->servoId, &clampCurrentAbs)) {
                ctx->returnZeroStartAbs = clampCurrentAbs;
            }
            ctx->targetAbs = ctx->result.zeroServoAbs;
            ctx->zeroMappedLastValid = false;
            ctx->zeroMappedLastAbs = 0;
            ctx->zeroMappedLastReturnTravel = 0;
            ctx->zeroMappedLastDeg = NAN;
            ctx->state = AUTO_CALIB_STATE_RETURN_ZERO;
            ctx->stateBeginMs = nowMs;
            ctx->lastTraceMs = nowMs;
            FW_LOG_INFO_PRINTF("[AUTO CALIB] RETURN_ZERO J%02u targetAbs=%ld speed=%u acc=%u\r\n",
                          ctx->jointIndex,
                          (long)ctx->targetAbs,
                          ctx->motionCfg.tightenSpeed,
                          ctx->motionCfg.tightenAcc);
            FW_LOG_INFO_PRINTF("[AUTO CALIB] BEGIN_RETURN_ZERO J%02u fromClampAbs=%ld targetZeroAbs=%ld delta=%ld\r\n",
                          ctx->jointIndex,
                          (long)ctx->returnZeroStartAbs,
                          (long)ctx->targetAbs,
                          (long)(ctx->targetAbs - ctx->returnZeroStartAbs));
            if (!appendJointCalibrationTargetToBatch(batch,
                                                     ctx->jointIndex,
                                                     clampToServoPosLocal(ctx->targetAbs),
                                                     ctx->motionCfg)) {
                failAutoCalibContextWithDiag(ctx,
                                             sharedData,
                                             "return zero target append failed",
                                             CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED,
                                             CALIB_DIAG_PHASE_RETURN_ZERO,
                                             ctx->motorChannel,
                                             ctx->targetAbs,
                                             0,
                                             0,
                                             0);
            }
            return;
        }
        case AUTO_CALIB_STATE_RETURN_ZERO:
        {
            if (!ctx->resultValid) {
                failAutoCalibContextWithDiag(ctx,
                                             sharedData,
                                             "return zero without result",
                                             CALIB_DIAG_REASON_INTERNAL_ERROR,
                                             CALIB_DIAG_PHASE_RETURN_ZERO,
                                             ctx->motorChannel,
                                             0,
                                             0,
                                             0,
                                             0);
                return;
            }
            if (!appendJointCalibrationTargetToBatch(batch,
                                                     ctx->jointIndex,
                                                     clampToServoPosLocal(ctx->targetAbs),
                                                     ctx->motionCfg)) {
                failAutoCalibContextWithDiag(ctx,
                                             sharedData,
                                             "return zero target append failed",
                                             CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED,
                                             CALIB_DIAG_PHASE_RETURN_ZERO,
                                             ctx->motorChannel,
                                             ctx->targetAbs,
                                             0,
                                             0,
                                             0);
                return;
            }

            int32_t currentAbs = 0;
            const bool absFeedbackValid =
                readLatestServoAbs(sharedData, ctx->bus, ctx->servoId, &currentAbs);
            const int32_t absError = absFeedbackValid
                ? abs32(currentAbs - ctx->targetAbs)
                : 0;
            const int32_t returnTravel = absFeedbackValid
                ? (motorRawAbsToTendonAbs(ctx->motorChannel, currentAbs) -
                   motorRawAbsToTendonAbs(ctx->motorChannel, ctx->returnZeroStartAbs))
                : 0;
            const int32_t requiredReturnTravel =
                kAutoServoZeroPullCounts - kAutoServoZeroPullToleranceCounts;
            const bool returnTravelReached =
                absFeedbackValid && returnTravel >= requiredReturnTravel;
            const int32_t returnZeroTolerance =
                (params.zeroReachAbsTolerance < kAutoServoZeroPullToleranceCounts)
                    ? params.zeroReachAbsTolerance
                    : kAutoServoZeroPullToleranceCounts;
            float zeroMappedDeg = 0.0f;
            const bool encoderMappedValid =
                readJointMappedDegFromCalibration(sharedData,
                                                  ctx->jointIndex,
                                                  ctx->result,
                                                  &zeroMappedDeg);
            const bool mappedZeroReached =
                encoderMappedValid &&
                fabsf(zeroMappedDeg) <= kAutoZeroMappedToleranceDeg;
            bool mappedZeroSeen = false;
            int32_t zeroCaptureAbs = currentAbs;
            int32_t zeroCaptureReturnTravel = returnTravel;
            float zeroCaptureDeg = zeroMappedDeg;
            const char* zeroCaptureReason = "window";
            if (!ctx->zeroMappedCaptured &&
                absFeedbackValid &&
                encoderMappedValid) {
                if (mappedZeroReached) {
                    mappedZeroSeen = true;
                } else if (ctx->zeroMappedLastValid) {
                    const float lastDeg = ctx->zeroMappedLastDeg;
                    const bool signsCrossed =
                        (lastDeg < 0.0f && zeroMappedDeg > 0.0f) ||
                        (lastDeg > 0.0f && zeroMappedDeg < 0.0f);
                    const bool crossingNearZero =
                        fabsf(lastDeg) <= kAutoZeroCrossMaxAbsDeg &&
                        fabsf(zeroMappedDeg) <= kAutoZeroCrossMaxAbsDeg;
                    const float degDelta = zeroMappedDeg - lastDeg;
                    if (signsCrossed && crossingNearZero && fabsf(degDelta) > 0.001f) {
                        float fraction = -lastDeg / degDelta;
                        if (fraction < 0.0f) {
                            fraction = 0.0f;
                        } else if (fraction > 1.0f) {
                            fraction = 1.0f;
                        }
                        zeroCaptureAbs = ctx->zeroMappedLastAbs +
                            (int32_t)lroundf((float)(currentAbs - ctx->zeroMappedLastAbs) * fraction);
                        zeroCaptureReturnTravel = ctx->zeroMappedLastReturnTravel +
                            (int32_t)lroundf((float)(returnTravel - ctx->zeroMappedLastReturnTravel) * fraction);
                        zeroCaptureDeg = 0.0f;
                        zeroCaptureReason = "cross";
                        mappedZeroSeen = true;
                    }
                }
            }
            if (!ctx->zeroMappedCaptured &&
                absFeedbackValid &&
                mappedZeroSeen) {
                const int32_t plannedZeroAbs = ctx->targetAbs;
                ctx->zeroMappedCaptured = true;
                ctx->zeroMappedCaptureAbs = zeroCaptureAbs;
                ctx->zeroMappedCaptureDeg = zeroCaptureDeg;
                FW_LOG_INFO_PRINTF("[AUTO CALIB] ZERO_CAPTURED J%02u reason=%s captureAbs=%ld currentAbs=%ld mappedDeg=%.3f captureDeg=%.3f tol=%.3f returnTravel=%ld targetWas=%ld lastAbs=%ld lastDeg=%.3f\r\n",
                              ctx->jointIndex,
                              zeroCaptureReason,
                              (long)zeroCaptureAbs,
                              (long)currentAbs,
                              zeroMappedDeg,
                              zeroCaptureDeg,
                              kAutoZeroMappedToleranceDeg,
                              (long)zeroCaptureReturnTravel,
                              (long)plannedZeroAbs,
                              ctx->zeroMappedLastValid ? (long)ctx->zeroMappedLastAbs : 0L,
                              ctx->zeroMappedLastValid ? ctx->zeroMappedLastDeg : NAN);
            }
            const bool zeroReached =
                ctx->zeroMappedCaptured &&
                (returnTravelReached ||
                 (absFeedbackValid && absError <= returnZeroTolerance));

            if (zeroReached) {
                syncJointTargetToZero(sharedData, ctx->jointIndex);
                clearJointServoMultiTurnUnknown(sharedData, ctx->jointIndex);
                const int primaryMotorCh = findMotorChannel(ctx->bus, ctx->servoId);
                if (primaryMotorCh >= 0 && primaryMotorCh < SERVO_TOTAL_NUM) {
                    setCalibratedMotorAbsLimit(sharedData,
                                               (uint8_t)primaryMotorCh,
                                               ctx->result.tightServoAbs);
                }
                FW_LOG_INFO_PRINTF("[AUTO CALIB] ZERO_REACHED J%02u capturedAbs=%ld currentAbs=%ld absErr=%ld mappedDeg=%.3f mappedTol=%.3f absReached=%u travelReached=%u\r\n",
                              ctx->jointIndex,
                              (long)ctx->zeroMappedCaptureAbs,
                              (long)currentAbs,
                              (long)absError,
                              ctx->zeroMappedCaptureDeg,
                              kAutoZeroMappedToleranceDeg,
                              (absFeedbackValid && absError <= returnZeroTolerance) ? 1 : 0,
                              returnTravelReached ? 1 : 0);
                ctx->state = AUTO_CALIB_STATE_DONE;
                return;
            }

            if (absFeedbackValid && encoderMappedValid) {
                ctx->zeroMappedLastValid = true;
                ctx->zeroMappedLastAbs = currentAbs;
                ctx->zeroMappedLastReturnTravel = returnTravel;
                ctx->zeroMappedLastDeg = zeroMappedDeg;
            }

            if ((nowMs - ctx->lastTraceMs) >= params.traceMs) {
                ctx->lastTraceMs = nowMs;
                FW_LOG_INFO_PRINTF("[AUTO CALIB] ZERO_WAIT_SERVO J%02u targetAbs=%ld currentAbs=%ld absErr=%ld/%ld returnTravel=%ld/%ld mappedDeg=%.3f/%.3f absReached=%u travelReached=%u mappedReached=%u captured=%u captureAbs=%ld\r\n",
                              ctx->jointIndex,
                              (long)ctx->targetAbs,
                              absFeedbackValid ? (long)currentAbs : 0L,
                              absFeedbackValid ? (long)absError : -1L,
                              (long)returnZeroTolerance,
                              absFeedbackValid ? (long)returnTravel : 0L,
                              (long)requiredReturnTravel,
                              encoderMappedValid ? zeroMappedDeg : NAN,
                              kAutoZeroMappedToleranceDeg,
                              (absFeedbackValid && absError <= returnZeroTolerance) ? 1 : 0,
                              returnTravelReached ? 1 : 0,
                              mappedZeroReached ? 1 : 0,
                              ctx->zeroMappedCaptured ? 1 : 0,
                              ctx->zeroMappedCaptured ? (long)ctx->zeroMappedCaptureAbs : 0L);
            }

            if ((uint32_t)(nowMs - ctx->stateBeginMs) >= params.zeroReturnTimeoutMs) {
                FW_LOG_INFO_PRINTF("[AUTO CALIB] ZERO_TIMEOUT J%02u targetAbs=%ld currentAbs=%ld absErr=%ld/%ld returnTravel=%ld/%ld mappedDeg=%.3f/%.3f absReached=%u travelReached=%u mappedReached=%u captured=%u captureAbs=%ld\r\n",
                              ctx->jointIndex,
                              (long)ctx->targetAbs,
                              absFeedbackValid ? (long)currentAbs : 0L,
                              absFeedbackValid ? (long)absError : -1L,
                              (long)returnZeroTolerance,
                              absFeedbackValid ? (long)returnTravel : 0L,
                              (long)requiredReturnTravel,
                              encoderMappedValid ? zeroMappedDeg : NAN,
                              kAutoZeroMappedToleranceDeg,
                              (absFeedbackValid && absError <= returnZeroTolerance) ? 1 : 0,
                              returnTravelReached ? 1 : 0,
                              mappedZeroReached ? 1 : 0,
                              ctx->zeroMappedCaptured ? 1 : 0,
                              ctx->zeroMappedCaptured ? (long)ctx->zeroMappedCaptureAbs : 0L);
                failAutoCalibContextWithDiag(ctx,
                                             sharedData,
                                             "zero return timeout",
                                             CALIB_DIAG_REASON_ZERO_RETURN_TIMEOUT,
                                             CALIB_DIAG_PHASE_RETURN_ZERO,
                                             ctx->motorChannel,
                                             absFeedbackValid ? absError : -1,
                                             requiredReturnTravel,
                                             encoderMappedValid ? (int32_t)lroundf(zeroMappedDeg * 100.0f) : 0,
                                             0);
            }
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

    uint32_t sideSwayRequiredMask = 0;
    for (uint8_t i = 0; i < kSideSwayAdjustJointsCount; i++) {
        sideSwayRequiredMask |= (1UL << kSideSwayAdjustJoints[i]);
    }
    if ((seenJoints & sideSwayRequiredMask) == sideSwayRequiredMask) {
        FW_LOG_INFO_PRINTLN("[AUTO CALIB] all side-sway dual joints selected; running side-sway pre-adjust");
        if (!runSideSwayPreAdjust(sharedData, params)) {
            FW_LOG_INFO_PRINTLN("[AUTO CALIB] side-sway pre-adjust failed");
            if (!hasCalibrationDiag(sharedData)) {
                setCalibrationDiag(sharedData,
                                   CALIB_DIAG_REASON_SIDE_ADJUST_FAILED,
                                   CALIB_DIAG_PHASE_DUAL_PRELOAD,
                                   0xFF,
                                   0xFF,
                                   seenJoints,
                                   sideSwayRequiredMask,
                                   0,
                                   seenJoints & sideSwayRequiredMask);
            }
            return false;
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
            FW_LOG_INFO_PRINTF("[AUTO CALIB] SEQUENCE_SERIAL begin J%02u (%u/%u) dual M%02u/M%02u\r\n",
                          orderedJoints[i],
                          (unsigned)(i + 1),
                          (unsigned)jointCount,
                          dualPrimaryMotor,
                          dualSecondaryMotor);
            if (!runDualAntagonistAutoCalibrationJoint(sharedData,
                                                       orderedJoints[i],
                                                       params)) {
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

static bool startAutoCalibrationTaskWithMode(TaskSharedData_t* sharedData,
                                             uint8_t mode)
{
    if (!sharedData) {
        return false;
    }
    if (g_autoCalibrationTaskHandle != NULL) {
        FW_LOG_INFO_PRINTLN("[AUTO CALIB] task already running");
        return true;
    }
    clearCalibrationDiag(sharedData);
    g_autoCalibrationTaskMode = mode;

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
    return startAutoCalibrationTaskWithMode(sharedData, AUTO_CAL_TASK_MODE_SEQUENCE);
}

bool startSideSwayAdjustTask(TaskSharedData_t* sharedData)
{
    return startAutoCalibrationTaskWithMode(sharedData, AUTO_CAL_TASK_MODE_SIDE_SWAY_ADJUST);
}

void autoCalibrationTask(void* parameter)
{
    TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;
    const AutoCalibrationParams params = defaultAutoCalibrationParams();
    const uint8_t taskMode = g_autoCalibrationTaskMode;

    if (taskMode == AUTO_CAL_TASK_MODE_SIDE_SWAY_ADJUST) {
        const bool ok = runSideSwayPreAdjust(sharedData, params);
        if (!ok && sharedData && !hasCalibrationDiag(sharedData)) {
            setCalibrationDiag(sharedData,
                               CALIB_DIAG_REASON_SIDE_ADJUST_FAILED,
                               CALIB_DIAG_PHASE_DUAL_PRELOAD,
                               0xFF,
                               0xFF,
                               0,
                               0,
                               0,
                               0);
        }
        postSystemEvent(sharedData, ok ? SYSTEM_EVENT_SIDE_SWAY_ADJUST_DONE : SYSTEM_EVENT_CALIBRATION_FAILED);
        taskRuntimeUnregister(TASK_RUNTIME_AUTO_CALIB);
        g_autoCalibrationTaskHandle = NULL;
        g_autoCalibrationTaskMode = AUTO_CAL_TASK_MODE_SEQUENCE;
        vTaskDelete(NULL);
        return;
    }

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
        FW_LOG_INFO_PRINT("[AUTO CALIB] ACTIVE_SCOPE joints=");
        for (uint8_t i = 0; i < activeJointCount; i++) {
            FW_LOG_INFO_PRINTF("%sJ%02u", (i == 0) ? "" : ",", activeJoints[i]);
        }
        FW_LOG_INFO_PRINTLN("");
    }

    const bool ok = (activeJointCount > 0) &&
                    runAutoCalibrationSequence(sharedData,
                                               activeJoints,
                                               activeJointCount,
                                               params);

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
