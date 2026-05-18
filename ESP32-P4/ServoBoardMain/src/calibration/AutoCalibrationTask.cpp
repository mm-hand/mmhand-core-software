#include "AutoCalibrationTask.h"

#include <math.h>
#include <string.h>

#include "CalibrationMath.h"
#include "CalibrationTask.h"
#include "../hardware/HardwareMap.h"
#include "../system/StateMachineTask.h"

extern volatile uint8_t g_calibrationUIStatus;

static TaskHandle_t g_autoCalibrationTaskHandle = NULL;

const uint8_t kDefaultAutoCalibrationJoints[] = {8,9,10,11};
const uint8_t kDefaultAutoCalibrationJointsCount = sizeof(kDefaultAutoCalibrationJoints) / sizeof(kDefaultAutoCalibrationJoints[0]);
static const uint32_t kAutoFeedbackReadyCanTimeoutMs = 500;
static const uint32_t kAutoFeedbackReadyServoTimeoutMs = 500;
static const uint32_t kAutoFeedbackReadyTelemetryTimeoutMs = 800;
static const uint32_t kAutoInitialFeedbackWaitMs = 1200;
static const int32_t kAutoCalibrationRejectNearFullWrapSpan = kCalibrationEncoderModulo - 1024;
static const float kAutoCalibrationStartDeg = -5.0f;
static const float kAutoCalibrationZeroDeg = 0.0f;
static const float kAutoCalibrationTightDeg = 90.0f;
static const float kAutoCalibrationTotalDeg =
    kAutoCalibrationTightDeg - kAutoCalibrationStartDeg;
static const int32_t kAutoServoZeroPullCounts = 400;
static const int32_t kAutoServoZeroPullToleranceCounts = 40;
static const int32_t kDualAutoSlackCounts = 4800;
static const int32_t kCalibratedMotorAbsWindowCounts = 4096 * 5;

// 0 means use cfg.tightenStep sign. Per-joint overrides are reserved for
// verified hardware exceptions; default automatic tightening remains positive.
static const int8_t kAutoTightenServoDirection[ENCODER_TOTAL_NUM] = {
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0, 0
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
                                       int32_t tightMaxAbs)
{
    if (!sharedData ||
        motorChannel >= SERVO_TOTAL_NUM ||
        !isMotorChannelValid(motorChannel)) {
        return;
    }

    const int32_t maxAbs = clampToServoAbsRangeLocal(tightMaxAbs);
    const int32_t minAbs =
        clampToServoAbsRangeLocal((int32_t)maxAbs - kCalibratedMotorAbsWindowCounts);
    sharedData->motor_abs_limit_min[motorChannel] = minAbs;
    sharedData->motor_abs_limit_max[motorChannel] = maxAbs;
    sharedData->motor_abs_limit_valid_bitmap |= (1UL << motorChannel);
    sharedData->motor_abs_limit_locked_bitmap |= (1UL << motorChannel);
    Serial.printf("[AUTO CALIB] ABS_LIMIT M%02u min=%ld max=%ld window=%ld\r\n",
                  (unsigned)motorChannel,
                  (long)minAbs,
                  (long)maxAbs,
                  (long)kCalibratedMotorAbsWindowCounts);
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

static int32_t getAutoTightenDirectionSign(uint8_t jointIndex, const JointCalibrationConfig& cfg)
{
    if (jointIndex < ENCODER_TOTAL_NUM && kAutoTightenServoDirection[jointIndex] != 0) {
        return kAutoTightenServoDirection[jointIndex] < 0 ? -1 : 1;
    }
    return (cfg.tightenStep < 0) ? -1 : 1;
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
    return (sharedData->multi_turn_unknown_bitmap & (1UL << ch)) == 0;
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
        sharedData->multi_turn_unknown_bitmap &= ~clearMask;
        Serial.printf("[AUTO CALIB] MULTITURN_KNOWN J%02u clearMask=0x%08lX unknown=0x%08lX\r\n",
                      jointIndex,
                      (unsigned long)clearMask,
                      (unsigned long)sharedData->multi_turn_unknown_bitmap);
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
    if (!sharedData->canRxQueue ||
        xQueuePeek(sharedData->canRxQueue, &sensorData, 0) != pdTRUE ||
        !sensorData.isValid ||
        sensorData.errorFlags[jointIndex] != 0) {
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

    Serial.printf("[AUTO CALIB] INIT_FEEDBACK J%02u bus%u id%u ch=%d encoderOk=%u absOk=%u rawOk=%u sensorQ=%u valid=%u err=0x%02X age=%lu servoQ=%u online=%u age=%lu abs=%ld raw=%d\r\n",
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

static bool buildAutoZeroTightCalibrationResult(uint8_t jointIndex,
                                                int32_t tautEncoder,
                                                int32_t zeroEncoder,
                                                int32_t tightEncoder,
                                                int32_t tautServoAbs,
                                                int32_t zeroServoAbs,
                                                int32_t tightServoAbs,
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
    if (startToTightSpan <= 0 || kAutoCalibrationTotalDeg <= 0.0f) {
        return false;
    }

    const float zeroFraction =
        (kAutoCalibrationZeroDeg - kAutoCalibrationStartDeg) /
        kAutoCalibrationTotalDeg;
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
    out->angleMax = kRange100ControlMaxDeg;
    out->startEncoder = tautEncoder;
    out->tightEncoder = tightEncoder;
    out->zeroEncoder = computedZeroEncoder;
    out->encoderSpan = startToTightSpan;
    out->startServoAbs = tautServoAbs;
    out->tightServoAbs = tightServoAbs;
    out->zeroServoAbs = zeroServoAbs;
    out->startDeg = kAutoCalibrationStartDeg;
    out->tightDeg = kAutoCalibrationTightDeg;
    out->totalDeg = kAutoCalibrationTotalDeg;
    out->scaled = true;
    return out->encoderSpan > 0;
}

static void applyAutoCalibrationConfig(JointCalibrationConfig* cfg)
{
    if (!cfg) {
        return;
    }

    cfg->angleScope = kAutoCalibrationTotalDeg;
    cfg->bottomReserved = -kAutoCalibrationStartDeg;
    cfg->topReserved = kAutoCalibrationTightDeg - kRange100ControlMaxDeg;
    cfg->angleReserved = -kAutoCalibrationStartDeg;
}

static uint8_t buildAutoCalibrationJointListFromActiveScope(TaskSharedData_t* sharedData,
                                                            uint8_t* outJoints,
                                                            uint8_t maxCount)
{
    if (!sharedData || !outJoints || maxCount == 0) {
        return 0;
    }

    uint32_t activeMask =
        sharedData->active_joint_mask & ((1UL << ENCODER_TOTAL_NUM) - 1UL);
    if (activeMask == 0 && sharedData->active_motor_mask != 0) {
        const uint32_t activeMotorMask =
            sharedData->active_motor_mask & validMotorChannelMask();
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
            Serial.printf("[AUTO CALIB] ACTIVE_SCOPE inferred joints=0x%08lX from motor=0x%08lX\r\n",
                          (unsigned long)activeMask,
                          (unsigned long)activeMotorMask);
        }
    }

    uint8_t count = 0;
    for (int jointIndex = ENCODER_TOTAL_NUM - 1;
         jointIndex >= 0 && count < maxCount;
         jointIndex--) {
        if ((activeMask & (1UL << jointIndex)) == 0) {
            continue;
        }
        outJoints[count++] = (uint8_t)jointIndex;
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
        if (jointIndex >= ENCODER_TOTAL_NUM || sensorData.errorFlags[jointIndex] != 0) {
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
    sharedData->control_mode = CONTROL_MODE_JOINT;

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
    params.turnNormalizeTimeoutMs = 15000;
    params.windowServoDeltaCounts = 50;
    params.zeroReachAbsTolerance = 200;
    params.followArmDeg = 3.3f;
    params.zeroFollowDeg = 5.0f;
    params.stallActualDeg = 0.5f;
    params.minTightTravelDeg = 5.0f;
    params.startLoadDelta = 25;
    params.tightLoadHigh = 200;
    params.startLoadStableSamples = 3;
    return params;
}

enum AutoCalibState {
    AUTO_CALIB_STATE_IDLE = 0,
    AUTO_CALIB_STATE_INIT,
    AUTO_CALIB_STATE_NORMALIZE,
    AUTO_CALIB_STATE_TIGHTEN,
    AUTO_CALIB_STATE_RETURN_ZERO,
    AUTO_CALIB_STATE_DONE,
    AUTO_CALIB_STATE_FAILED
};

struct JointCalibContext {
    AutoCalibState state;
    uint8_t jointIndex;
    uint8_t bus;
    uint8_t servoId;
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
    int32_t returnZeroStartAbs;
    int32_t windowStartEncoder;
    int32_t windowStartAbs;
    uint32_t windowStartMs;
    JointCalibrationResult result;
    bool resultValid;
    bool initStarted;
    bool releaseCommandStarted;
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
    batch->source = SERVO_TARGET_OWNER_CALIBRATION;
    batch->timestamp = millis();
    return xQueueOverwrite(sharedData->servoTargetQueue, batch) == pdTRUE;
}

static bool appendCalibrationHoldTargetsExcept(TaskSharedData_t* sharedData,
                                               ServoTargetBatch_t* batch,
                                               const uint8_t* jointList,
                                               uint8_t jointCount,
                                               uint8_t activeJoint,
                                               const AutoCalibrationParams& params)
{
    if (!sharedData || !batch || !jointList) {
        return false;
    }

    for (uint8_t i = 0; i < jointCount; i++) {
        const uint8_t jointIndex = jointList[i];
        if (jointIndex == activeJoint || jointIndex >= ENCODER_TOTAL_NUM) {
            continue;
        }

        JointActuatorGroup group;
        if (getJointActuatorGroup(jointIndex, &group)) {
            for (uint8_t actuatorIndex = 0; actuatorIndex < group.count; actuatorIndex++) {
                const uint8_t motorChannel = group.actuators[actuatorIndex].motorChannel;
                int32_t holdAbs = 0;
                if (!readLatestMotorServoAbs(sharedData, motorChannel, &holdAbs)) {
                    Serial.printf("[AUTO CALIB] FAIL hold feedback missing J%02u M%02u\r\n",
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
                    Serial.printf("[AUTO CALIB] FAIL append hold target J%02u M%02u abs=%ld\r\n",
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
                Serial.printf("[AUTO CALIB] FAIL hold feedback missing J%02u bus%u id%u\r\n",
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
                Serial.printf("[AUTO CALIB] FAIL append hold target J%02u abs=%ld\r\n",
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
    // described by jointActuatorGroups, including thumb J16/J17.
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
        Serial.printf("[AUTO CALIB] FAIL motor target outside limit M%02u target=%ld\r\n",
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

static bool waitMotorTargetReached(TaskSharedData_t* sharedData,
                                   uint8_t jointIndex,
                                   const char* phase,
                                   uint8_t motorChannel,
                                   int32_t targetAbs,
                                   const AutoCalibrationParams& params)
{
    const uint32_t startMs = millis();
    uint32_t lastTraceMs = 0;
    while (autoCalibrationStillOwnsControl(sharedData)) {
        if (!publishMotorCalibrationTarget(sharedData, motorChannel, targetAbs, params)) {
            return false;
        }

        int32_t currentAbs = 0;
        if (readLatestMotorServoAbs(sharedData, motorChannel, &currentAbs)) {
            const int32_t absErr = abs32(currentAbs - targetAbs);
            if (absErr <= params.zeroReachAbsTolerance) {
                Serial.printf("[AUTO CALIB] %s_REACHED J%02u M%02u targetAbs=%ld currentAbs=%ld absErr=%ld\r\n",
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
                Serial.printf("[AUTO CALIB] %s_WAIT J%02u M%02u targetAbs=%ld currentAbs=%ld absErr=%ld/%ld\r\n",
                              phase,
                              jointIndex,
                              motorChannel,
                              (long)targetAbs,
                              (long)currentAbs,
                              (long)absErr,
                              (long)params.zeroReachAbsTolerance);
            }
        }

        if ((uint32_t)(millis() - startMs) >= params.zeroReturnTimeoutMs) {
            Serial.printf("[AUTO CALIB] FAIL %s timeout J%02u M%02u targetAbs=%ld\r\n",
                          phase,
                          jointIndex,
                          motorChannel,
                          (long)targetAbs);
            return false;
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
    const uint32_t startMs = millis();
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
            Serial.printf("[AUTO CALIB] DUAL_INIT_WAIT J%02u M%02u=%u M%02u=%u encoder=%u\r\n",
                          jointIndex,
                          primaryMotor,
                          primaryOk ? 1 : 0,
                          secondaryMotor,
                          secondaryOk ? 1 : 0,
                          encoderOk ? 1 : 0);
        }
        if ((uint32_t)(nowMs - startMs) >= kAutoInitialFeedbackWaitMs) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    return false;
}

static bool findDualTightPoint(TaskSharedData_t* sharedData,
                               uint8_t jointIndex,
                               const char* phase,
                               uint8_t motorChannel,
                               int32_t startAbs,
                               int32_t startEncoder,
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

    while (autoCalibrationStillOwnsControl(sharedData)) {
        const uint32_t nowMs = millis();
        uint16_t currentRaw = 0;
        int32_t currentEncoder = 0;
        int32_t currentAbs = 0;
        if (!readLatestEncoderSample(sharedData, jointIndex, &currentRaw, &currentEncoder) ||
            !readLatestMotorServoAbs(sharedData, motorChannel, &currentAbs)) {
            Serial.printf("[AUTO CALIB] FAIL %s lost feedback J%02u M%02u\r\n",
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
        const bool loadHigh = hasLoad && loadMag >= params.tightLoadHigh;
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
            const bool enoughEncoderTravel =
                totalAngleDeltaDeg >= params.zeroFollowDeg;

            Serial.printf("[AUTO CALIB] CHECK_%s J%02u M%02u abs=%ld raw=%u oriented=%ld dCount=%ld winCount=%ld winAbs=%ld/%ld winDeg=%.3f/%.3f totalDeg=%.3f/%.3f load=%d/%d cond servo=%u angle=%u travel=%u load=%u\r\n",
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
                          params.zeroFollowDeg,
                          loadMag,
                          params.tightLoadHigh,
                          servoStillTightening ? 1 : 0,
                          angleStalled ? 1 : 0,
                          enoughEncoderTravel ? 1 : 0,
                          loadHigh ? 1 : 0);

            tightFoundByStall =
                loadHigh && servoStillTightening && angleStalled && enoughEncoderTravel;
            windowStartAbs = currentAbs;
            windowStartEncoder = currentEncoder;
            windowStartMs = nowMs;
        }

        if (loadHigh || tightFoundByStall) {
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
            Serial.printf("[AUTO CALIB] %s_FOUND J%02u M%02u raw=%u oriented=%ld abs=%ld load=%d byLoad=%u byStall=%u totalDeg=%.3f loadThreshold=%d\r\n",
                          phase,
                          jointIndex,
                          motorChannel,
                          (unsigned)currentRaw,
                          (long)currentEncoder,
                          (long)currentAbs,
                          loadMag,
                          loadHigh ? 1 : 0,
                          tightFoundByStall ? 1 : 0,
                          totalAngleDeltaDeg,
                          params.tightLoadHigh);
            return true;
        }

        const uint32_t dtMs = nowMs - lastCommandMs;
        if (dtMs >= params.loopMs) {
            const int32_t numerator =
                (int32_t)dtMs * params.tightenSpeedCountsPerSec + commandRemainder;
            const int32_t stepCounts = numerator / 1000;
            commandRemainder = numerator % 1000;
            if (stepCounts > 0) {
                commandAbs += stepCounts;
                if (!publishMotorCalibrationTarget(sharedData, motorChannel, commandAbs, params)) {
                    return false;
                }
            }
            lastCommandMs = nowMs;
        }

        if ((uint32_t)(nowMs - lastTraceMs) >= params.traceMs) {
            lastTraceMs = nowMs;
            Serial.printf("[AUTO CALIB] TRACE_%s J%02u M%02u cmd=%ld abs=%ld raw=%u oriented=%ld dCount=%ld totalAbs=%ld totalDeg=%.3f load=%d\r\n",
                          phase,
                          jointIndex,
                          motorChannel,
                          (long)commandAbs,
                          (long)currentAbs,
                          (unsigned)currentRaw,
                          (long)currentEncoder,
                          (long)totalEncoderDelta,
                          (long)totalAbsDelta,
                          totalAngleDeltaDeg,
                          loadMag);
        }

        vTaskDelay(pdMS_TO_TICKS(params.loopMs));
    }
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
        Serial.printf("[AUTO CALIB] FAIL DUAL_INIT J%02u M%02u/M%02u missing feedback\r\n",
                      jointIndex,
                      primaryMotor,
                      secondaryMotor);
        return false;
    }

    Serial.printf("[AUTO CALIB] DUAL_BEGIN J%02u primary=M%02u secondary=M%02u slack=%ld loadThreshold=%d initialEncoder=%ld primaryAbs=%ld secondaryAbs=%ld\r\n",
                  jointIndex,
                  primaryMotor,
                  secondaryMotor,
                  (long)kDualAutoSlackCounts,
                  params.tightLoadHigh,
                  (long)initialEncoder,
                  (long)primaryAbs,
                  (long)secondaryAbs);

    const int32_t secondarySlackTarget = secondaryAbs - kDualAutoSlackCounts;
    Serial.printf("[AUTO CALIB] DUAL_SLACK_SECONDARY_BEGIN J%02u M%02u targetAbs=%ld from=%ld\r\n",
                  jointIndex,
                  secondaryMotor,
                  (long)secondarySlackTarget,
                  (long)secondaryAbs);
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
        Serial.printf("[AUTO CALIB] FAIL DUAL_PRIMARY_START J%02u missing feedback\r\n",
                      jointIndex);
        return false;
    }

    int32_t max90Encoder = 0;
    int32_t primaryMaxAbs = 0;
    uint16_t max90Raw = 0;
    int16_t max90Load = 0;
    if (!findDualTightPoint(sharedData,
                            jointIndex,
                            "MAX",
                            primaryMotor,
                            primaryAbs,
                            initialEncoder,
                            params,
                            &primaryMaxAbs,
                            &max90Raw,
                            &max90Encoder,
                            &max90Load)) {
        return false;
    }

    const int32_t primarySlackTarget = primaryMaxAbs - kDualAutoSlackCounts;
    Serial.printf("[AUTO CALIB] DUAL_SLACK_PRIMARY_BEGIN J%02u M%02u targetAbs=%ld from=%ld\r\n",
                  jointIndex,
                  primaryMotor,
                  (long)primarySlackTarget,
                  (long)primaryMaxAbs);
    if (!waitMotorTargetReached(sharedData,
                                jointIndex,
                                "DUAL_SLACK_PRIMARY",
                                primaryMotor,
                                primarySlackTarget,
                                params)) {
        return false;
    }

    int32_t secondaryStartAbs = 0;
    int32_t secondaryStartEncoder = 0;
    if (!readLatestMotorServoAbs(sharedData, secondaryMotor, &secondaryStartAbs) ||
        !readLatestEncoderOriented(sharedData, jointIndex, &secondaryStartEncoder)) {
        Serial.printf("[AUTO CALIB] FAIL DUAL_SECONDARY_START J%02u missing feedback\r\n",
                      jointIndex);
        return false;
    }

    int32_t neg5Encoder = 0;
    int32_t secondaryNeg5Abs = 0;
    uint16_t neg5Raw = 0;
    int16_t neg5Load = 0;
    if (!findDualTightPoint(sharedData,
                            jointIndex,
                            "NEG5",
                            secondaryMotor,
                            secondaryStartAbs,
                            secondaryStartEncoder,
                            params,
                            &secondaryNeg5Abs,
                            &neg5Raw,
                            &neg5Encoder,
                            &neg5Load)) {
        return false;
    }

    const int32_t signedSpan = signedEncoderDeltaLocal(neg5Encoder, max90Encoder);
    const int8_t encoderTravelSign = (signedSpan < 0) ? -1 : 1;
    const int32_t resultStartEncoder =
        encoderForPositiveTravelResult(neg5Encoder, encoderTravelSign);
    const int32_t resultTightEncoder =
        encoderForPositiveTravelResult(max90Encoder, encoderTravelSign);
    const int32_t resultSpan =
        calibrationForwardEncoderDelta(resultStartEncoder, resultTightEncoder);
    if (resultSpan <= 0 || resultSpan >= kAutoCalibrationRejectNearFullWrapSpan) {
        Serial.printf("[AUTO CALIB] FAIL DUAL_RESULT J%02u invalid span=%ld neg5=%ld max=%ld signed=%ld\r\n",
                      jointIndex,
                      (long)resultSpan,
                      (long)neg5Encoder,
                      (long)max90Encoder,
                      (long)signedSpan);
        return false;
    }

    const float zeroFraction =
        (kAutoCalibrationZeroDeg - kAutoCalibrationStartDeg) /
        kAutoCalibrationTotalDeg;
    const int32_t resultZeroEncoder =
        calibrationWrapEncoderCount(
            resultStartEncoder + roundToInt32Local((float)resultSpan * zeroFraction));
    JointCalibrationResult result;
    if (!buildAutoZeroTightCalibrationResult(jointIndex,
                                             resultStartEncoder,
                                             resultZeroEncoder,
                                             resultTightEncoder,
                                             secondaryNeg5Abs,
                                             primaryMaxAbs,
                                             primaryMaxAbs,
                                             &result)) {
        Serial.printf("[AUTO CALIB] FAIL DUAL_RESULT_BUILD J%02u\r\n", jointIndex);
        return false;
    }

    if (encoderTravelSign < 0) {
        const int8_t oldDir = g_encoderDirection[jointIndex];
        const int8_t newDir = (oldDir < 0) ? 1 : -1;
        setRuntimeEncoderDirection(jointIndex, newDir);
        Serial.printf("[AUTO CALIB] DUAL_ENCODER_DIR_AUTO J%02u defaultDir=%d oldDir=%d newDir=%d neg5=%ld max=%ld resultStart=%ld resultZero=%ld resultTight=%ld\r\n",
                      jointIndex,
                      (int)kDefaultEncoderDirection[jointIndex],
                      (int)oldDir,
                      (int)g_encoderDirection[jointIndex],
                      (long)neg5Encoder,
                      (long)max90Encoder,
                      (long)result.startEncoder,
                      (long)result.zeroEncoder,
                      (long)result.tightEncoder);
    }

    applyAutoCalibrationConfig(&g_jointCalibConfig[jointIndex]);
    g_jointCalibResult[jointIndex] = result;
    g_encoderOffsetManual[jointIndex] = result.offset;
    syncJointTargetToZero(sharedData, jointIndex);
    clearJointServoMultiTurnUnknown(sharedData, jointIndex);
    setCalibratedMotorAbsLimit(sharedData, primaryMotor, primaryMaxAbs);
    setCalibratedMotorAbsLimit(sharedData, secondaryMotor, secondaryNeg5Abs);

    Serial.printf("[AUTO CALIB] DUAL_RESULT J%02u neg5Raw=%u neg5=%ld maxRaw=%u max=%ld span=%ld zero=%ld primaryMaxAbs=%ld secondaryNeg5Abs=%ld loadMax=%d loadNeg5=%d\r\n",
                  jointIndex,
                  (unsigned)neg5Raw,
                  (long)result.startEncoder,
                  (unsigned)max90Raw,
                  (long)result.tightEncoder,
                  (long)result.encoderSpan,
                  (long)result.zeroEncoder,
                  (long)primaryMaxAbs,
                  (long)secondaryNeg5Abs,
                  max90Load,
                  neg5Load);

    const int32_t secondaryFinalSlackTarget = secondaryNeg5Abs - kDualAutoSlackCounts;
    Serial.printf("[AUTO CALIB] DUAL_FINAL_SLACK_SECONDARY_BEGIN J%02u M%02u targetAbs=%ld from=%ld\r\n",
                  jointIndex,
                  secondaryMotor,
                  (long)secondaryFinalSlackTarget,
                  (long)secondaryNeg5Abs);
    if (!waitMotorTargetReached(sharedData,
                                jointIndex,
                                "DUAL_FINAL_SLACK_SECONDARY",
                                secondaryMotor,
                                secondaryFinalSlackTarget,
                                params)) {
        return false;
    }

    return true;
}

static void failAutoCalibContext(JointCalibContext* ctx, const char* reason)
{
    if (!ctx || ctx->state == AUTO_CALIB_STATE_FAILED) {
        return;
    }
    Serial.printf("[AUTO CALIB] FAIL J%02u %s\r\n",
                  ctx->jointIndex,
                  reason ? reason : "unknown");
    ctx->state = AUTO_CALIB_STATE_FAILED;
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

    Serial.printf("[AUTO CALIB] AUTO_RANGE J%02u startDeg=%.3f zeroDeg=0.000 tightDeg=%.3f totalDeg=%.3f zeroFollowDeg=%.3f controlMax=%.3f\r\n",
                  ctx->jointIndex,
                  kAutoCalibrationStartDeg,
                  kAutoCalibrationTightDeg,
                  kAutoCalibrationTotalDeg,
                  params.zeroFollowDeg,
                  kRange100ControlMaxDeg);
    Serial.printf("[AUTO CALIB] START J%02u bus%u id%u abs=%ld raw=%u oriented=%ld dir=%d baselineLoad=%d speedCps=%ld\r\n",
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
}

// ============================================================================
// stepAutoCalibrationJoint - 单关节自动标定的并行状态机步进函数
// ============================================================================
// 核心逻辑（非阻塞）：
// 在每一帧定时调度中被调用。它将原先阻塞式的“死循环拉绳”重构成了状态机步进。
// 主要卡点判定条件 (AUTO_CALIB_STATE_TIGHTEN)：
// 1) 舵机在指定的时间窗口内，绝对位置必须移动了一定的距离，否则认为拉紧停滞。
// 2) 电流反馈 (load) 必须达到并稳定超过设定的阈值 (tightLoadHigh)。
// 3) 磁编码器的角度必须在连续多次采样中变化小于容差 (stallActualDeg)，证明机械已经完全抵住死区。
// 只有当以上条件同时满足时，才判定为“成功寻找到物理零位”，进而校准磁编与舵机。
// ============================================================================
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
        Serial.printf("[AUTO CALIB] CANCEL J%02u state=%u owner=%u enabled=%u\r\n",
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
            ctx->directionSign = getAutoTightenDirectionSign(ctx->jointIndex, cfg);
            if (!ctx->initStarted) {
                ctx->initStarted = true;
                ctx->stateBeginMs = nowMs;
                ctx->lastCommandMs = 0;
                ctx->lastTraceMs = 0;
            }

            if (!isJointPrimaryServoMultiTurnKnown(sharedData, ctx->jointIndex)) {
                Serial.printf("[AUTO CALIB] WARN J%02u multi-turn unknown; auto calibration will rebuild zero from current relative abs unknown=0x%08lX\r\n",
                              ctx->jointIndex,
                              (unsigned long)sharedData->multi_turn_unknown_bitmap);
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
                failAutoCalibContext(ctx, "missing initial encoder/servo feedback after wait");
                return;
            }

            const int32_t turn = servoTurnFromAbs(absNow);
            if (turn != 0) {
                Serial.printf("[AUTO CALIB] SKIP_NORMALIZE_TURN J%02u bus%u id%u abs=%ld raw=%d turn=%ld; calibrate from current abs\r\n",
                              ctx->jointIndex,
                              ctx->bus,
                              ctx->servoId,
                              (long)absNow,
                              (int)rawServo,
                              (long)turn);
            }

            if (!initTightenStateFromFeedback(ctx, sharedData, params)) {
                failAutoCalibContext(ctx, "missing feedback after init");
            }
            return;
        }

        case AUTO_CALIB_STATE_NORMALIZE:
        {
            int32_t absNow = 0;
            if (!readLatestServoAbs(sharedData, ctx->bus, ctx->servoId, &absNow)) {
                failAutoCalibContext(ctx, "lost servo feedback during normalize");
                return;
            }
            if (!appendJointCalibrationTargetToBatch(batch,
                                                     ctx->jointIndex,
                                                     clampToServoPosLocal(ctx->targetAbs),
                                                     ctx->motionCfg)) {
                failAutoCalibContext(ctx, "normalize target append failed");
                return;
            }
            if (abs32(absNow - ctx->targetAbs) <= params.zeroReachAbsTolerance) {
                Serial.printf("[AUTO CALIB] NORMALIZE_DONE J%02u abs=%ld targetAbs=%ld\r\n",
                              ctx->jointIndex,
                              (long)absNow,
                              (long)ctx->targetAbs);
                if (!initTightenStateFromFeedback(ctx, sharedData, params)) {
                    failAutoCalibContext(ctx, "missing encoder after turn normalize");
                }
                return;
            }
            if ((uint32_t)(nowMs - ctx->stateBeginMs) >= params.turnNormalizeTimeoutMs) {
                failAutoCalibContext(ctx, "normalize timeout");
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
                failAutoCalibContext(ctx, "lost encoder/servo feedback");
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
                Serial.printf("[AUTO CALIB] TRACE J%02u phase=%s cmd=%ld abs=%ld raw=%u oriented=%ld dCount=%ld totalAbs=%ld totalDeg=%.3f load=%d startFound=%u elapsed=%lu\r\n",
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
                    const bool enoughEncoderTravel =
                        totalAngleDeltaDeg >= params.zeroFollowDeg;

                    Serial.printf("[AUTO CALIB] CHECK_MAX J%02u abs=%ld raw=%u oriented=%ld dCount=%ld winCount=%ld winAbs=%ld/%ld winDeg=%.3f/%.3f totalDeg=%.3f/%.3f load=%d/%d cond servo=%u angle=%u travel=%u load=%u\r\n",
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
                                  params.zeroFollowDeg,
                                  loadMag,
                                  params.tightLoadHigh,
                                  servoStillTightening ? 1 : 0,
                                  angleStalled ? 1 : 0,
                                  enoughEncoderTravel ? 1 : 0,
                                  loadHigh ? 1 : 0);

                    maxFoundByStall =
                        loadHigh && servoStillTightening && angleStalled && enoughEncoderTravel;
                    ctx->windowStartAbs = currentAbs;
                    ctx->windowStartEncoder = currentEncoder;
                    ctx->windowStartMs = nowMs;
                }

                if (loadHigh || maxFoundByStall) {
                    ctx->encoderTravelSign = (totalEncoderDelta < 0) ? -1 : 1;
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
                    Serial.printf("[AUTO CALIB] START_FOUND J%02u maxDeg=%.3f raw=%u oriented=%ld abs=%ld load=%d byLoad=%u byStall=%u totalDeg=%.3f loadThreshold=%d encTravelSign=%d\r\n",
                                  ctx->jointIndex,
                                  kAutoCalibrationTightDeg,
                                  (unsigned)currentRaw,
                                  (long)ctx->startEncoder,
                                  (long)ctx->startServoAbs,
                                  loadMag,
                                  loadHigh ? 1 : 0,
                                  maxFoundByStall ? 1 : 0,
                                  totalAngleDeltaDeg,
                                  params.tightLoadHigh,
                                  (int)ctx->encoderTravelSign);
                    return;
                }

                const uint32_t dtMs = nowMs - ctx->lastCommandMs;
                if (dtMs >= params.loopMs) {
                    const int32_t numerator =
                        (int32_t)dtMs * params.tightenSpeedCountsPerSec + ctx->commandRemainder;
                    const int32_t stepCounts = numerator / 1000;
                    ctx->commandRemainder = numerator % 1000;
                    if (stepCounts > 0) {
                        ctx->commandAbs += ctx->directionSign * stepCounts;
                        if (!isServoCommandInsideLimit(ctx->commandAbs)) {
                            appendJointCalibrationTargetToBatch(batch,
                                                                ctx->jointIndex,
                                                                clampToServoPosLocal(ctx->commandAbs),
                                                                ctx->motionCfg);
                            failAutoCalibContext(ctx, "find max command outside +/-30719");
                            return;
                        }
                        if (!appendJointCalibrationTargetToBatch(batch,
                                                                 ctx->jointIndex,
                                                                 clampToServoPosLocal(ctx->commandAbs),
                                                                 ctx->motionCfg)) {
                            failAutoCalibContext(ctx, "find max target append failed");
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
                    ctx->commandAbs -= ctx->directionSign * stepCounts;
                    if (!isServoCommandInsideLimit(ctx->commandAbs)) {
                        appendJointCalibrationTargetToBatch(batch,
                                                            ctx->jointIndex,
                                                            clampToServoPosLocal(ctx->commandAbs),
                                                            ctx->motionCfg);
                        failAutoCalibContext(ctx, "release command outside +/-30719");
                        return;
                    }
                    if (!appendJointCalibrationTargetToBatch(batch,
                                                             ctx->jointIndex,
                                                             clampToServoPosLocal(ctx->commandAbs),
                                                             ctx->motionCfg)) {
                        failAutoCalibContext(ctx, "release target append failed");
                        return;
                    }
                    if (!ctx->releaseCommandStarted) {
                        ctx->releaseCommandStarted = true;
                        Serial.printf("[AUTO CALIB] BEGIN_RELEASE J%02u fromMaxAbs=%ld firstTargetAbs=%ld step=%ld directionSign=%ld targetDeg=%.3f\r\n",
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
                const int32_t releaseFromMaxCount =
                    calibrationWrapEncoderDelta(currentEncoder, ctx->startEncoder);
                const float releaseFromMaxDeg =
                    fabsf(calibrationEncoderCountToDeg(releaseFromMaxCount));
                const bool enoughReleaseTravel =
                    releaseFromMaxDeg >= params.minTightTravelDeg;

                Serial.printf("[AUTO CALIB] CHECK_RELEASE J%02u abs=%ld raw=%u oriented=%ld releaseCount=%ld releaseDeg=%.3f/%.3f winCount=%ld winAbs=%ld/%ld winDeg=%.3f/%.3f load=%d cond servo=%u angle=%u travel=%u\r\n",
                              ctx->jointIndex,
                              (long)currentAbs,
                              (unsigned)currentRaw,
                              (long)currentEncoder,
                              (long)releaseFromMaxCount,
                              releaseFromMaxDeg,
                              params.minTightTravelDeg,
                              (long)windowEncoderDelta,
                              (long)windowAbsDelta,
                              (long)params.windowServoDeltaCounts,
                              windowAngleDeltaDeg,
                              params.stallActualDeg,
                              loadMag,
                              servoStillReleasing ? 1 : 0,
                              angleStalled ? 1 : 0,
                              enoughReleaseTravel ? 1 : 0);

                if (servoStillReleasing && angleStalled && enoughReleaseTravel) {
                    ctx->zeroFound = true;
                    ctx->zeroEncoder = currentEncoder;
                    ctx->zeroServoAbs = currentAbs;
                    const int32_t signedTightToMax =
                        signedEncoderDeltaLocal(ctx->zeroEncoder, ctx->startEncoder);
                    ctx->encoderTravelSign = (signedTightToMax < 0) ? -1 : 1;

                    const int32_t maxEncoder = ctx->startEncoder;
                    const int32_t maxServoAbs = ctx->startServoAbs;
                    const int32_t tightEncoder = ctx->zeroEncoder;
                    const int32_t tightServoAbs = ctx->zeroServoAbs;
                    if (maxServoAbs == tightServoAbs) {
                        failAutoCalibContext(ctx, "zero find has no servo travel");
                        return;
                    }
                    ctx->directionSign = (maxServoAbs > tightServoAbs) ? 1 : -1;
                    const int32_t resultTautEncoder =
                        encoderForPositiveTravelResult(ctx->zeroEncoder, ctx->encoderTravelSign);
                    const int32_t resultTightEncoder =
                        encoderForPositiveTravelResult(ctx->startEncoder, ctx->encoderTravelSign);
                    const float zeroFraction =
                        (kAutoCalibrationZeroDeg - kAutoCalibrationStartDeg) /
                        kAutoCalibrationTotalDeg;
                    const int32_t resultSpan =
                        calibrationForwardEncoderDelta(resultTautEncoder, resultTightEncoder);
                    const int32_t resultZeroEncoder =
                        calibrationWrapEncoderCount(
                            resultTautEncoder +
                            roundToInt32Local((float)resultSpan * zeroFraction));
                    const int32_t servoZeroAbs =
                        tightServoAbs + (ctx->directionSign * kAutoServoZeroPullCounts);

                    JointCalibrationResult result;
                    if (!buildAutoZeroTightCalibrationResult(ctx->jointIndex,
                                                             resultTautEncoder,
                                                             resultZeroEncoder,
                                                             resultTightEncoder,
                                                             tightServoAbs,
                                                             servoZeroAbs,
                                                             maxServoAbs,
                                                             &result)) {
                        failAutoCalibContext(ctx, "invalid max/release result");
                        return;
                    }
                    if (result.encoderSpan >= kAutoCalibrationRejectNearFullWrapSpan) {
                        Serial.printf("[AUTO CALIB] FAIL J%02u rejected wrap-like span=%ld start=%ld tight=%ld; check servo tighten direction\r\n",
                                      ctx->jointIndex,
                                      (long)result.encoderSpan,
                                      (long)tightEncoder,
                                      (long)maxEncoder);
                        ctx->state = AUTO_CALIB_STATE_FAILED;
                        return;
                    }

                    if (ctx->encoderTravelSign < 0) {
                        const int8_t oldDir = g_encoderDirection[ctx->jointIndex];
                        const int8_t newDir = (oldDir < 0) ? 1 : -1;
                        setRuntimeEncoderDirection(ctx->jointIndex, newDir);
                        Serial.printf("[AUTO CALIB] ENCODER_DIR_AUTO J%02u defaultDir=%d oldDir=%d newDir=%d oldTaut=%ld oldZero=%ld oldTight=%ld resultTaut=%ld resultZero=%ld resultTight=%ld\r\n",
                                      ctx->jointIndex,
                                      (int)kDefaultEncoderDirection[ctx->jointIndex],
                                      (int)oldDir,
                                      (int)g_encoderDirection[ctx->jointIndex],
                                      (long)tightEncoder,
                                      (long)resultZeroEncoder,
                                      (long)maxEncoder,
                                      (long)resultTautEncoder,
                                      (long)resultZeroEncoder,
                                      (long)resultTightEncoder);
                    }

                    applyAutoCalibrationConfig(&g_jointCalibConfig[ctx->jointIndex]);
                    g_jointCalibResult[ctx->jointIndex] = result;
                    g_encoderOffsetManual[ctx->jointIndex] = result.offset;
                    ctx->result = result;
                    ctx->resultValid = true;
                    ctx->targetAbs = result.zeroServoAbs;
                    ctx->returnZeroStartAbs = tightServoAbs;
                    ctx->state = AUTO_CALIB_STATE_RETURN_ZERO;
                    ctx->stateBeginMs = nowMs;

                    Serial.printf("[AUTO CALIB] TIGHT_FOUND J%02u startDeg=%.3f raw=%u encoder=%ld abs=%ld load=%d releaseDeg=%.3f winDeg=%.3f\r\n",
                                  ctx->jointIndex,
                                  kAutoCalibrationStartDeg,
                                  (unsigned)currentRaw,
                                  (long)tightEncoder,
                                  (long)tightServoAbs,
                                  loadMag,
                                  releaseFromMaxDeg,
                                  windowAngleDeltaDeg);
                    Serial.printf("[AUTO CALIB] ZERO_FOUND J%02u zeroDeg=%.3f encoder=%ld servoZero=%ld formula=tightAbs(%ld)+directionSign(%ld)*pullCounts(%ld)\r\n",
                                  ctx->jointIndex,
                                  kAutoCalibrationZeroDeg,
                                  (long)result.zeroEncoder,
                                  (long)result.zeroServoAbs,
                                  (long)tightServoAbs,
                                  (long)ctx->directionSign,
                                  (long)kAutoServoZeroPullCounts);
                    Serial.printf("[AUTO CALIB] RESULT J%02u start=%ld zero=%ld tight=%ld span=%ld servoStart=%ld servoZero=%ld servoTight=%ld\r\n",
                                  ctx->jointIndex,
                                  (long)result.startEncoder,
                                  (long)result.zeroEncoder,
                                  (long)result.tightEncoder,
                                  (long)result.encoderSpan,
                                  (long)result.startServoAbs,
                                  (long)result.zeroServoAbs,
                                  (long)result.tightServoAbs);
                    Serial.printf("[AUTO CALIB] RETURN_ZERO J%02u targetAbs=%ld speed=%u acc=%u\r\n",
                                  ctx->jointIndex,
                                  (long)ctx->targetAbs,
                                  ctx->motionCfg.tightenSpeed,
                                  ctx->motionCfg.tightenAcc);
                    Serial.printf("[AUTO CALIB] BEGIN_RETURN_ZERO J%02u fromTightAbs=%ld targetZeroAbs=%ld delta=%ld\r\n",
                                  ctx->jointIndex,
                                  (long)tightServoAbs,
                                  (long)ctx->targetAbs,
                                  (long)(ctx->targetAbs - tightServoAbs));

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

        case AUTO_CALIB_STATE_RETURN_ZERO:
        {
            if (!ctx->resultValid) {
                failAutoCalibContext(ctx, "return zero without result");
                return;
            }
            if (!appendJointCalibrationTargetToBatch(batch,
                                                     ctx->jointIndex,
                                                     clampToServoPosLocal(ctx->targetAbs),
                                                     ctx->motionCfg)) {
                failAutoCalibContext(ctx, "return zero target append failed");
                return;
            }

            int32_t currentAbs = 0;
            const bool absFeedbackValid =
                readLatestServoAbs(sharedData, ctx->bus, ctx->servoId, &currentAbs);
            const int32_t absError = absFeedbackValid
                ? abs32(currentAbs - ctx->targetAbs)
                : 0;
            const int32_t returnTravel =
                absFeedbackValid ? ((currentAbs - ctx->returnZeroStartAbs) * ctx->directionSign) : 0;
            const int32_t requiredReturnTravel =
                kAutoServoZeroPullCounts - kAutoServoZeroPullToleranceCounts;
            const bool returnTravelReached =
                absFeedbackValid && returnTravel >= requiredReturnTravel;
            const int32_t returnZeroTolerance =
                (params.zeroReachAbsTolerance < kAutoServoZeroPullToleranceCounts)
                    ? params.zeroReachAbsTolerance
                    : kAutoServoZeroPullToleranceCounts;
            const bool zeroReached =
                absFeedbackValid &&
                absError <= returnZeroTolerance &&
                returnTravelReached;
            float zeroMappedDeg = 0.0f;
            const bool encoderMappedValid =
                readJointMappedDegFromCalibration(sharedData,
                                                  ctx->jointIndex,
                                                  ctx->result,
                                                  &zeroMappedDeg);

            if (zeroReached) {
                syncJointTargetToZero(sharedData, ctx->jointIndex);
                clearJointServoMultiTurnUnknown(sharedData, ctx->jointIndex);
                const int primaryMotorCh = findMotorChannel(ctx->bus, ctx->servoId);
                if (primaryMotorCh >= 0 && primaryMotorCh < SERVO_TOTAL_NUM) {
                    setCalibratedMotorAbsLimit(sharedData,
                                               (uint8_t)primaryMotorCh,
                                               ctx->result.tightServoAbs);
                }
                Serial.printf("[AUTO CALIB] ZERO_REACHED J%02u targetAbs=%ld currentAbs=%ld absErr=%ld mappedDeg=%.3f\r\n",
                              ctx->jointIndex,
                              (long)ctx->targetAbs,
                              (long)currentAbs,
                              (long)absError,
                              zeroMappedDeg);
                ctx->state = AUTO_CALIB_STATE_DONE;
                return;
            }

            if ((nowMs - ctx->lastTraceMs) >= params.traceMs) {
                ctx->lastTraceMs = nowMs;
                Serial.printf("[AUTO CALIB] ZERO_WAIT_SERVO J%02u targetAbs=%ld currentAbs=%ld absErr=%ld/%ld returnTravel=%ld/%ld mappedDeg=%.3f absReached=%u travelReached=%u\r\n",
                              ctx->jointIndex,
                              (long)ctx->targetAbs,
                              absFeedbackValid ? (long)currentAbs : 0L,
                              absFeedbackValid ? (long)absError : -1L,
                              (long)returnZeroTolerance,
                              absFeedbackValid ? (long)returnTravel : 0L,
                              (long)requiredReturnTravel,
                              encoderMappedValid ? zeroMappedDeg : NAN,
                              (absFeedbackValid && absError <= returnZeroTolerance) ? 1 : 0,
                              returnTravelReached ? 1 : 0);
            }

            if ((uint32_t)(nowMs - ctx->stateBeginMs) >= params.zeroReturnTimeoutMs) {
                Serial.printf("[AUTO CALIB] ZERO_TIMEOUT J%02u targetAbs=%ld currentAbs=%ld absErr=%ld/%ld returnTravel=%ld/%ld mappedDeg=%.3f absReached=%u travelReached=%u\r\n",
                              ctx->jointIndex,
                              (long)ctx->targetAbs,
                              absFeedbackValid ? (long)currentAbs : 0L,
                              absFeedbackValid ? (long)absError : -1L,
                              (long)returnZeroTolerance,
                              absFeedbackValid ? (long)returnTravel : 0L,
                              (long)requiredReturnTravel,
                              encoderMappedValid ? zeroMappedDeg : NAN,
                              (absFeedbackValid && absError <= returnZeroTolerance) ? 1 : 0,
                              returnTravelReached ? 1 : 0);
                ctx->state = AUTO_CALIB_STATE_FAILED;
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
        return false;
    }

    uint32_t seenJoints = 0;
    uint8_t orderedJoints[ENCODER_TOTAL_NUM] = {0};
    for (uint8_t i = 0; i < jointCount; i++) {
        if (jointList[i] >= ENCODER_TOTAL_NUM) {
            return false;
        }
        const uint32_t bit = (1UL << jointList[i]);
        if ((seenJoints & bit) != 0) {
            Serial.printf("[AUTO CALIB] FAIL duplicate joint J%02u in sequence\r\n",
                          jointList[i]);
            return false;
        }
        seenJoints |= bit;
        orderedJoints[i] = jointList[i];
    }

    for (uint8_t i = 0; i < jointCount; i++) {
        for (uint8_t j = (uint8_t)(i + 1); j < jointCount; j++) {
            if (orderedJoints[j] > orderedJoints[i]) {
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
    for (uint8_t i = 0; i < jointCount; i++) {
        uint8_t dualPrimaryMotor = 0;
        uint8_t dualSecondaryMotor = 0;
        if (getDualAntagonistAutoCalibrationMotors(orderedJoints[i],
                                                   &dualPrimaryMotor,
                                                   &dualSecondaryMotor)) {
            Serial.printf("[AUTO CALIB] SEQUENCE_SERIAL begin J%02u (%u/%u) dual M%02u/M%02u\r\n",
                          orderedJoints[i],
                          (unsigned)(i + 1),
                          (unsigned)jointCount,
                          dualPrimaryMotor,
                          dualSecondaryMotor);
            if (!runDualAntagonistAutoCalibrationJoint(sharedData,
                                                       orderedJoints[i],
                                                       params)) {
                Serial.printf("[AUTO CALIB] SEQUENCE_SERIAL failed J%02u dual\r\n",
                              orderedJoints[i]);
                return false;
            }
            Serial.printf("[AUTO CALIB] SEQUENCE_SERIAL done J%02u dual\r\n",
                          orderedJoints[i]);
            continue;
        }

        JointCalibContext ctx;
        initAutoCalibContext(&ctx, orderedJoints[i]);
        Serial.printf("[AUTO CALIB] SEQUENCE_SERIAL begin J%02u (%u/%u)\r\n",
                      orderedJoints[i],
                      (unsigned)(i + 1),
                      (unsigned)jointCount);

        while (ctx.state != AUTO_CALIB_STATE_DONE &&
               ctx.state != AUTO_CALIB_STATE_FAILED) {
            if (!autoCalibrationStillOwnsControl(sharedData)) {
                Serial.printf("[AUTO CALIB] CANCEL sequence state=%u owner=%u enabled=%u\r\n",
                              sharedData ? (unsigned)sharedData->system_state : 255,
                              sharedData ? (unsigned)sharedData->servo_target_owner : 255,
                              sharedData ? (unsigned)sharedData->control_enabled : 255);
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
                                                    params)) {
                Serial.println("[AUTO CALIB] FAIL sequence append non-active holds");
                return false;
            }

            stepAutoCalibrationJoint(&ctx, sharedData, params, &batch);

            if (!publishCalibrationBatch(sharedData, &batch)) {
                Serial.println("[AUTO CALIB] FAIL sequence publish target batch");
                return false;
            }

            vTaskDelay(pdMS_TO_TICKS(params.loopMs));
        }

        if (ctx.state == AUTO_CALIB_STATE_FAILED) {
            Serial.printf("[AUTO CALIB] SEQUENCE_SERIAL failed J%02u\r\n", orderedJoints[i]);
            return false;
        }

        Serial.printf("[AUTO CALIB] SEQUENCE_SERIAL done J%02u\r\n", orderedJoints[i]);
    }

    return true;
}

bool startAutoCalibrationTask(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return false;
    }
    if (g_autoCalibrationTaskHandle != NULL) {
        Serial.println("[AUTO CALIB] task already running");
        return true;
    }

    const BaseType_t ok = xTaskCreate(autoCalibrationTask,
                                      "autoCalibrationTask",
                                      8192,
                                      sharedData,
                                      TASK_STATE_MACHINE_PRIORITY,
                                      &g_autoCalibrationTaskHandle);
    return ok == pdPASS;
}

void autoCalibrationTask(void* parameter)
{
    TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;
    const AutoCalibrationParams params = defaultAutoCalibrationParams();
    g_calibrationUIStatus = CALIB_STATUS_RUNNING;

    uint8_t activeJoints[ENCODER_TOTAL_NUM] = {0};
    const uint8_t activeJointCount =
        buildAutoCalibrationJointListFromActiveScope(sharedData,
                                                     activeJoints,
                                                     ENCODER_TOTAL_NUM);
    if (activeJointCount == 0) {
        Serial.println("[AUTO CALIB] FAIL active scope has no joints");
    } else {
        Serial.print("[AUTO CALIB] ACTIVE_SCOPE joints=");
        for (uint8_t i = 0; i < activeJointCount; i++) {
            Serial.printf("%sJ%02u", (i == 0) ? "" : ",", activeJoints[i]);
        }
        Serial.println();
    }

    const bool ok = (activeJointCount > 0) &&
                    runAutoCalibrationSequence(sharedData,
                                               activeJoints,
                                               activeJointCount,
                                               params);

    postSystemEvent(sharedData, ok ? SYSTEM_EVENT_CALIBRATION_DONE : SYSTEM_EVENT_CALIBRATION_FAILED);
    g_autoCalibrationTaskHandle = NULL;
    vTaskDelete(NULL);
}
