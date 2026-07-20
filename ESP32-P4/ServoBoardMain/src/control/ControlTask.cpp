#include "ControlTask.h"

#include <math.h>
#include <string.h>

#include "../calibration/CalibrationMath.h"
#include "../calibration/CalibrationTask.h"
#include "../hardware/HardwareMap.h"
#include "../generated/UrdfJointLimits.generated.h"
#include "../shared/SharedSafetyState.h"
#include "../system/FirmwareLog.h"
#include "../system/StateMachineTask.h"
#include "../system/TaskRuntimeMonitor.h"
#include "ControlOutputBuilder.h"
#include "ControlSafety.h"
#include "ControlSolver.h"
#include "DualTendonControl.h"

static ControlSolver g_controlSolver;
static int32_t g_dualReleaseAnchorRawAbs[SERVO_TOTAL_NUM];
static int32_t g_dualReleaseAnchorCalibSignature[SERVO_TOTAL_NUM];
static uint32_t g_dualReleaseAnchorValidBitmap = 0;

void invalidateDualTendonReleaseAnchors(uint32_t motorMask)
{
    g_dualReleaseAnchorValidBitmap &= ~motorMask;
}

static const int32_t kEncoderModulo = 16384;
static const int32_t kEncoderMarginCounts = (kEncoderModulo * 10 + 180) / 360;
static const uint8_t kDebugJointIndices[] = {0, 1, 2, 3};
static const uint8_t kDebugJointCount =
    (uint8_t)(sizeof(kDebugJointIndices) / sizeof(kDebugJointIndices[0]));

static const uint32_t kCanBusOfflineTimeoutMs = 1000;
static const uint32_t kServoTelemetryTimeoutMs = 500;
static const uint16_t kEncoderDisconnectRaw = 0xFFFF;
static const int16_t kControlOverloadLoadThreshold = 800;
static const int16_t kJointControlOverloadLoadThreshold = 2000;
static const uint8_t kControlOverloadLatchCycles = 20;
static const uint8_t kJointControlOverloadLatchCycles = 20;
static const uint16_t kJointControlFeedbackSpeed = 0;
static const int32_t kDirectRunDoneAbsToleranceCounts = 40;
static const uint8_t kDirectRunDoneStableCycles = 5;
static const float kFeedbackRunDoneToleranceDeg = 0.5f;
static const uint8_t kFeedbackRunDoneStableCycles = 10;
static const uint32_t kFeedbackRunTimeoutMs = 8000;
static const int32_t kUpperWorkspaceWarningCounts = 48;
static const uint8_t kUpperWorkspaceWarningStableFrames = 3;
static const float kLocalTestTwoPi = 6.28318530717958647692f;
static const float kLocalTestMinFreqHz = 0.001f;
static const float kLocalTestMaxFreqHz = 100.0f;
static const float kLocalTestMaxAbsAmpDeg = 180.0f;

static float sanitizeLocalTestFreq(float freqHz)
{
    if (!isfinite(freqHz) || freqHz < kLocalTestMinFreqHz) {
        return kLocalTestMinFreqHz;
    }
    if (freqHz > kLocalTestMaxFreqHz) {
        return kLocalTestMaxFreqHz;
    }
    return freqHz;
}

static float sanitizeLocalTestAmp(float ampDeg)
{
    if (!isfinite(ampDeg)) {
        return 0.0f;
    }
    if (ampDeg > kLocalTestMaxAbsAmpDeg) {
        return kLocalTestMaxAbsAmpDeg;
    }
    if (ampDeg < -kLocalTestMaxAbsAmpDeg) {
        return -kLocalTestMaxAbsAmpDeg;
    }
    return ampDeg;
}

// 鍗忚瀛楁浣跨敤 int16 浼犺緭锛屼繚鐣欐渶澶ф鍊?32767 浣滀负寮傚父/鍗犱綅锛屾甯稿€奸檺鍒跺埌 32766銆?
static int16_t clampMappedCountForProtocol(int32_t value)
{
    if (value > 32766) return 32766;
    if (value < -32768) return -32768;
    return (int16_t)value;
}

static int32_t orientEncoderRaw(uint16_t rawValue, int8_t direction)
{
    int32_t oriented = (int32_t)rawValue & (kEncoderModulo - 1);
    if (direction < 0) {
        oriented = (kEncoderModulo - oriented) & (kEncoderModulo - 1);
    }
    return oriented;
}

static int32_t wrapEncoderCount(int32_t value)
{
    value %= kEncoderModulo;
    if (value < 0) value += kEncoderModulo;
    return value;
}

static int32_t forwardEncoderDelta(int32_t from, int32_t to)
{
    return wrapEncoderCount(to - from);
}

// Encoder counts to degrees.
static float convertEncoderCountToDeg(int32_t encoderCount)
{
    return (float)encoderCount * 360.0f / (float)kEncoderModulo;
}

static int32_t degToEncoderCount(float deg)
{
    const float scaled = deg * (float)kEncoderModulo / 360.0f;
    return (int32_t)(scaled + (scaled >= 0.0f ? 0.5f : -0.5f));
}

static float mapCalibratedEncoderToDeg(const JointCalibrationResult& calib, int32_t orientedRaw)
{
    if (calib.scaled && calib.encoderSpan > 0 && calib.totalDeg > 0.0f) {
        int32_t delta = forwardEncoderDelta(calib.startEncoder, orientedRaw);
        if (delta > calib.encoderSpan + kEncoderMarginCounts) {
            delta -= kEncoderModulo;
        }
        return ((float)delta / (float)calib.encoderSpan) * calib.totalDeg + calib.startDeg;
    }

    const int32_t offset = calib.offset;
    int32_t delta = forwardEncoderDelta(offset, orientedRaw);
    int32_t hardMaxFromOffset = forwardEncoderDelta(offset, calib.encoderMax);
    if (hardMaxFromOffset <= 0) {
        hardMaxFromOffset = degToEncoderCount(calib.angleMax);
    }
    if (delta > hardMaxFromOffset + kEncoderMarginCounts) {
        delta -= kEncoderModulo;
    }
    return convertEncoderCountToDeg(delta);
}

static float clampJointTargetDegByCalib(float targetDeg, uint8_t jointIndex)
{
    if (!isfinite(targetDeg)) {
        targetDeg = 0.0f;
    }
    if (jointIndex >= ENCODER_TOTAL_NUM) {
        return 0.0f;
    }

    float minDeg = 0.0f;
    float maxDeg = 0.0f;
    if (g_jointCalibResult[jointIndex].success) {
        const JointCalibrationResult& calibration = g_jointCalibResult[jointIndex];
        if (calibration.mappingVersion >= 2) {
            minDeg = calibration.angleMin;
        }
        maxDeg = calibration.angleMax;
    } else {
        maxDeg =
            g_jointCalibConfig[jointIndex].angleScope -
            g_jointCalibConfig[jointIndex].bottomReserved -
            g_jointCalibConfig[jointIndex].topReserved;
    }

    if (!isfinite(maxDeg) || maxDeg <= 0.0f) {
        return 0.0f;
    }
    if (!isfinite(minDeg) || minDeg >= maxDeg) {
        return 0.0f;
    }
    if (targetDeg < minDeg) {
        return minDeg;
    }
    if (targetDeg > maxDeg) {
        return maxDeg;
    }
    return targetDeg;
}

// Find the motorMap channel for a physical bus/id pair.
static int findMotorChannel(uint8_t bus, uint8_t id)
{
    return findMotorChannelByBusId(bus, id);
}

static uint8_t appendJointActuatorHoldTargets(ServoTargetBatch_t* batch,
                                              uint8_t jointIndex,
                                              const ServoAngleData_t& servoData,
                                              uint16_t speed,
                                              uint8_t acc)
{
    if (!batch) {
        return 0;
    }

    JointActuatorRuntimeTarget targets[MAX_JOINT_ACTUATORS];
    uint8_t targetCount = 0;
    const uint8_t actuatorCount = getJointActuatorCount(jointIndex);
    for (uint8_t i = 0; i < actuatorCount && targetCount < MAX_JOINT_ACTUATORS; i++) {
        JointActuatorItem actuator;
        if (!getJointActuator(jointIndex, i, &actuator) ||
            actuator.motorChannel >= SERVO_TOTAL_NUM ||
            servoData.onlineStatus[actuator.motorChannel] == 0) {
            continue;
        }
        const MotorMapItem& motor = motorMap[actuator.motorChannel];
        JointActuatorRuntimeTarget& target = targets[targetCount++];
        target.motorChannel = actuator.motorChannel;
        target.busIndex = motor.busIndex;
        target.servoId = motor.servoID;
        target.sign = actuator.sign;
        target.targetAbs = clampServoPos(servoData.servoAngles[actuator.motorChannel]);
    }

    return appendJointActuatorRuntimeTargets(batch, targets, targetCount, speed, acc);
}

static uint8_t appendDualTendonTargets(ServoTargetBatch_t* batch,
                                       TaskSharedData_t* sharedData,
                                       uint8_t jointIndex,
                                       float qRefDeg,
                                       float qFbDeg,
                                       const ServoAngleData_t& servoData,
                                       uint16_t speed,
                                       uint8_t acc)
{
    if (!batch || !sharedData) {
        return 0;
    }

    DualTendonMotorTargets tendonTargets;
    memset(&tendonTargets, 0, sizeof(tendonTargets));
    if (!computeDualTendonMotorTargets(jointIndex,
                                       qRefDeg,
                                       qFbDeg,
                                       &servoData,
                                       &tendonTargets)) {
        resetDualTendonController(jointIndex);
        return 0;
    }

    JointActuatorRuntimeTarget runtimeTargets[MAX_JOINT_ACTUATORS];
    uint8_t targetCount = 0;
    for (uint8_t i = 0; i < tendonTargets.count && targetCount < MAX_JOINT_ACTUATORS; i++) {
        const uint8_t motorChannel = tendonTargets.motorChannel[i];
        if (motorChannel >= SERVO_TOTAL_NUM || !isMotorChannelValid(motorChannel)) {
            resetDualTendonController(jointIndex);
            return 0;
        }
        const MotorMapItem& motor = motorMap[motorChannel];
        const uint32_t bit = (1UL << motorChannel);
        uint32_t validBitmap = 0;
        int32_t limitMin = 0;
        int32_t limitMax = 0;
        {
            SharedSafetyGuard guard;
            validBitmap = sharedAtomicLoad(&sharedData->motor_abs_limit_valid_bitmap);
            limitMin = sharedData->motor_abs_limit_min[motorChannel];
            limitMax = sharedData->motor_abs_limit_max[motorChannel];
        }
        // Calibration validity is required to establish the tendon release anchor.
        // LOCK only enables the ABS min/max output clamp; it must not disable
        // feedback control for an otherwise calibrated dual-tendon joint.
        if ((validBitmap & bit) == 0) {
            resetDualTendonController(jointIndex);
            return 0;
        }
        if (limitMin > limitMax) {
            const int32_t tmp = limitMin;
            limitMin = limitMax;
            limitMax = tmp;
        }

        const int32_t calibratedTightAbs =
            (getMotorTendonShortenAbsSign(motorChannel) > 0) ? limitMax : limitMin;
        const int32_t currentRawAbs = servoData.servoAngles[motorChannel];
        if ((g_dualReleaseAnchorValidBitmap & bit) == 0 ||
            g_dualReleaseAnchorCalibSignature[motorChannel] != calibratedTightAbs) {
            g_dualReleaseAnchorRawAbs[motorChannel] = currentRawAbs;
            g_dualReleaseAnchorCalibSignature[motorChannel] = calibratedTightAbs;
            g_dualReleaseAnchorValidBitmap |= bit;
        }

        const int32_t releaseLimitedTarget = clampDualTendonReleaseFromCalib(
            jointIndex,
            motorChannel,
            g_dualReleaseAnchorRawAbs[motorChannel],
            currentRawAbs,
            tendonTargets.targetAbs[i]);
        JointActuatorRuntimeTarget& target = runtimeTargets[targetCount++];
        target.motorChannel = motorChannel;
        target.busIndex = motor.busIndex;
        target.servoId = motor.servoID;
        target.sign = 1;
        // Apply the same LOCK semantics as single-motor feedback: locked
        // channels are clamped to their calibrated ABS range, while unlocked
        // channels keep feedback control and may move beyond that range.
        target.targetAbs = clampServoPosByMotorAbsLimit(sharedData,
                                                        motorChannel,
                                                        releaseLimitedTarget);
    }

    return appendJointActuatorRuntimeTargets(batch, runtimeTargets, targetCount, speed, acc);
}

static int16_t servoLoadMagnitude(int16_t rawLoad)
{
    int16_t mag = rawLoad;
    if (mag < 0) {
        mag = -mag;
    }
    return (int16_t)(mag & 0x03FF);
}

static uint32_t activeOverloadMotorMask(TaskSharedData_t* sharedData,
                                        const ControlCommandSnapshot_t& commandSnapshot)
{
    if (!sharedData) {
        return 0;
    }
    const uint32_t validMask = validMotorChannelMask();
    if (commandSnapshot.controlMode == CONTROL_MODE_DIRECT_MOTOR) {
        return sharedAtomicLoad(&sharedData->active_motor_mask) & validMask;
    }

    uint32_t mask = 0;
    const uint32_t activeJointMask =
        sharedAtomicLoad(&sharedData->active_joint_mask) &
        ((1UL << ENCODER_TOTAL_NUM) - 1UL);
    for (uint8_t jointIndex = 0; jointIndex < ENCODER_TOTAL_NUM; jointIndex++) {
        if ((activeJointMask & (1UL << jointIndex)) == 0) {
            continue;
        }
        const uint8_t actuatorCount = getJointActuatorCount(jointIndex);
        for (uint8_t actuatorIndex = 0; actuatorIndex < actuatorCount; actuatorIndex++) {
            JointActuatorItem actuator;
            if (getJointActuator(jointIndex, actuatorIndex, &actuator) &&
                actuator.motorChannel < SERVO_TOTAL_NUM) {
                mask |= (1UL << actuator.motorChannel);
            }
        }
    }
    return mask & validMask;
}

static bool jointHasOverloadFault(uint8_t jointIndex, uint32_t overloadFaultBitmap)
{
    const uint8_t actuatorCount = getJointActuatorCount(jointIndex);
    for (uint8_t actuatorIndex = 0; actuatorIndex < actuatorCount; actuatorIndex++) {
        JointActuatorItem actuator;
        if (getJointActuator(jointIndex, actuatorIndex, &actuator) &&
            actuator.motorChannel < SERVO_TOTAL_NUM &&
            (overloadFaultBitmap & (1UL << actuator.motorChannel)) != 0) {
            return true;
        }
    }
    return false;
}

static int32_t abs32Local(int32_t value)
{
    return value < 0 ? -value : value;
}

static bool directAbsoluteTargetsReached(TaskSharedData_t* sharedData,
                                         const ServoAngleData_t& servoData,
                                         const ControlCommandSnapshot_t& commandSnapshot)
{
    if (!sharedData ||
        commandSnapshot.controlMode != CONTROL_MODE_DIRECT_MOTOR ||
        commandSnapshot.motorDirectCommandSource != MOTOR_DIRECT_SOURCE_ABSOLUTE) {
        return false;
    }

    const uint32_t requiredMask =
        sharedAtomicLoad(&sharedData->active_motor_mask) & validMotorChannelMask();
    if (requiredMask == 0) {
        return false;
    }

    bool checkedAny = false;
    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        if (((requiredMask >> ch) & 0x1UL) == 0) {
            continue;
        }
        checkedAny = true;
        if (servoData.onlineStatus[ch] == 0) {
            return false;
        }
        const int32_t targetAbs =
            (int32_t)clampServoPos(commandSnapshot.motorTargetRaw[ch]);
        if (abs32Local(servoData.servoAngles[ch] - targetAbs) > kDirectRunDoneAbsToleranceCounts) {
            return false;
        }
    }
    return checkedAny;
}

static bool feedbackJointTargetsReached(TaskSharedData_t* sharedData,
                                        const MappedAngleData_t& mappedData,
                                        const float* targetDegs,
                                        const float* actualDegs)
{
    if (!sharedData || !targetDegs || !actualDegs || !mappedData.isValid) {
        return false;
    }

    const uint32_t requiredMask =
        sharedAtomicLoad(&sharedData->active_joint_mask) &
        ((1UL << ENCODER_TOTAL_NUM) - 1UL);
    if (requiredMask == 0) {
        return false;
    }

    bool checkedAny = false;
    for (uint8_t jointIndex = 0; jointIndex < ENCODER_TOTAL_NUM; jointIndex++) {
        if (((requiredMask >> jointIndex) & 0x1UL) == 0) {
            continue;
        }
        checkedAny = true;
        if (mappedData.validFlags[jointIndex] == 0) {
            return false;
        }
        if (fabsf(actualDegs[jointIndex] - targetDegs[jointIndex]) > kFeedbackRunDoneToleranceDeg) {
            return false;
        }
    }

    return checkedAny;
}

static bool readControlCommandSnapshot(TaskSharedData_t* sharedData, ControlCommandSnapshot_t* out)
{
// Read host commands once per control cycle to avoid mixed partial updates.
    if (!sharedData || !out) {
        return false;
    }

    SemaphoreHandle_t lock = sharedData->commandStateMutex ? sharedData->commandStateMutex : sharedData->targetAnglesMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(2)) != pdTRUE) {
        return false;
    }

    memcpy(out->targetAngles, sharedData->targetAngles, sizeof(out->targetAngles));
    memcpy(out->motorTargetRaw, sharedData->motorTargetRaw, sizeof(out->motorTargetRaw));
    memcpy(out->motorSweepTargetRaw, sharedData->motorSweepTargetRaw, sizeof(out->motorSweepTargetRaw));
    out->motorCommandToken = sharedData->motor_command_token;
    out->motorSweepCommandToken = sharedData->motor_sweep_command_token;
    out->jointCommandToken = sharedData->joint_command_token;
    out->motorDirectCommandGeneration = sharedData->motor_direct_command_generation;
    out->motorDirectCommandSource = sharedData->motor_direct_command_source;
    out->controlMode = sharedData->control_mode;
    out->sineTestEnabled = sharedData->sine_test_enabled;
    out->sineTestRequestToken = sharedData->sine_test_request_token;
    out->sineTestStartMs = sharedData->sine_test_start_ms;
    out->sineTestJointMask = sharedData->sine_test_joint_mask;
    out->sineTestHoldMs = sharedData->sine_test_hold_ms;
    memcpy(out->sineTestCenterDeg, sharedData->sine_test_center_deg, sizeof(out->sineTestCenterDeg));
    memcpy(out->sineTestAmpDeg, sharedData->sine_test_amp_deg, sizeof(out->sineTestAmpDeg));
    memcpy(out->sineTestFreqHz, sharedData->sine_test_freq_hz, sizeof(out->sineTestFreqHz));
    memcpy(out->sineTestPhaseRad, sharedData->sine_test_phase_rad, sizeof(out->sineTestPhaseRad));
    if (lock) {
        xSemaphoreGive(lock);
    }
    return true;
}

// ============================================================================
// ControlTask - 鏍稿績瀹炴椂鎺у埗浠诲姟
// ============================================================================
// 杩愯棰戠巼锛氶€氬父涓?100Hz (10ms 鍛ㄦ湡)
// 鑱岃矗锛?// 1) 鏀堕泦鎬荤嚎浼犳潵鐨勪己鏈嶇數鏈虹粷瀵逛綅缃笌纾佺紪鐮佸櫒瑙掑害銆?// 2) 璇诲彇涓婁綅鏈轰笅鍙戠殑鍛戒护蹇収 (ControlCommandSnapshot_t)銆?// 3) 鏍规嵁涓嶅悓妯″紡 (CONTROL_MODE_JOINT 闂幆 / CONTROL_MODE_DIRECT_MOTOR 鐩存帶) 鎵ц鍒嗗彂銆?// 4) 鎵ц瀹夊叏妫€鏌ワ紙濡傛€ュ仠銆佽繃杞姐€佸弽鍚戦噴鏀句繚鎶ゃ€佸伐浣滅┖闂撮檺浣嶏級銆?// 5) 璋冪敤鍙岀幆 PID 姹傝В鍣?(ControlSolver) 鑾峰緱鐩爣鑴夊啿锛屽苟缁勮 Batch 浜ょ粰搴曞眰銆?// ============================================================================
void controlTask(void* parameter)
{
    TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;
    if (!sharedData) {
        vTaskDelete(NULL);
        return;
    }

    g_controlSolver.begin();

    float localTargets[ENCODER_TOTAL_NUM] = {0.0f};
    float magAngles[ENCODER_TOTAL_NUM] = {0.0f};
    int32_t absolutePositionRaw[ENCODER_TOTAL_NUM] = {0};
    int32_t absolutePositionTendon[ENCODER_TOTAL_NUM] = {0};
    int32_t outPulsesTendon[ENCODER_TOTAL_NUM] = {0};

    RemoteSensorData_t sensorData;
    MappedAngleData_t mappedData;
    uint32_t overloadFaultLatched = 0;
    uint8_t overloadHighCount[SERVO_TOTAL_NUM] = {0};
    uint8_t upperWorkspaceHighCount[ENCODER_TOTAL_NUM] = {0};
    uint32_t upperWorkspaceWarningBitmap = 0;
    uint32_t lastOverloadFaultResetToken =
        sharedAtomicLoad(&sharedData->overload_fault_reset_token);
    {
        SharedSafetyGuard guard;
        sharedAtomicStore(&sharedData->overload_fault_bitmap, (uint32_t)0);
        for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
            sharedData->overload_fault_load[ch] = 0;
        }
    }
    ControlCommandSnapshot_t commandSnapshot;
    memset(&commandSnapshot, 0, sizeof(commandSnapshot));
    readControlCommandSnapshot(sharedData, &commandSnapshot);

    TickType_t lastWakeTime = xTaskGetTickCount();
    TickType_t lastPidComputeTick = lastWakeTime;
    const TickType_t solverPeriodTicks = pdMS_TO_TICKS(10);
    bool prevMotorOnline[SERVO_TOTAL_NUM] = {false};
    bool hotplugHoldMotor[SERVO_TOTAL_NUM] = {false};
    uint32_t lastMotorCommandToken = commandSnapshot.motorCommandToken;
    uint32_t lastMotorSweepCommandToken = commandSnapshot.motorSweepCommandToken;
    uint32_t lastJointCommandToken = commandSnapshot.jointCommandToken;
    uint8_t lastMotorDirectSource = commandSnapshot.motorDirectCommandSource;
    int16_t prevSweepRawTarget[SERVO_TOTAL_NUM] = {0};
    int32_t sweepExpandedTarget[SERVO_TOTAL_NUM] = {0};
    bool sweepAnchorValid[SERVO_TOTAL_NUM] = {false};
    uint8_t directRunDoneStableCount = 0;
    uint8_t feedbackRunDoneStableCount = 0;
    uint32_t feedbackRunToken = 0;
    uint32_t feedbackRunStartMs = 0;
    float feedbackRunTargetDeg[ENCODER_TOTAL_NUM] = {0.0f};
    bool feedbackRunTracking = false;
    bool feedbackRunDoneEventQueued = false;
    bool feedbackRunTimeoutEventQueued = false;
    uint32_t lastSineTestRequestToken = commandSnapshot.sineTestRequestToken;
    bool sineTestActive = false;
    uint32_t sineTestStartMs = 0;

    while (1)
    {
        taskRuntimeMarkLoopStart(TASK_RUNTIME_CONTROL);
        if (!readControlCommandSnapshot(sharedData, &commandSnapshot)) {
            taskRuntimeMarkLoopEnd(TASK_RUNTIME_CONTROL);
            const TickType_t nextWakeTime = lastWakeTime + solverPeriodTicks;
            const bool periodOverrun = (int32_t)(xTaskGetTickCount() - nextWakeTime) >= 0;
            taskRuntimeMarkPeriodResult(TASK_RUNTIME_CONTROL, periodOverrun);
            vTaskDelayUntil(&lastWakeTime, solverPeriodTicks);
            if (periodOverrun) vTaskDelay(1);
            continue;
        }

        const uint32_t motorCommandToken = commandSnapshot.motorCommandToken;
        if (motorCommandToken != lastMotorCommandToken)
        {
            memset(hotplugHoldMotor, 0, sizeof(hotplugHoldMotor));
            lastMotorCommandToken = motorCommandToken;
        }

        const uint32_t motorSweepCommandToken = commandSnapshot.motorSweepCommandToken;
        if (motorSweepCommandToken != lastMotorSweepCommandToken)
        {
            memset(hotplugHoldMotor, 0, sizeof(hotplugHoldMotor));
            lastMotorSweepCommandToken = motorSweepCommandToken;
        }

        const uint32_t jointCommandToken = commandSnapshot.jointCommandToken;
        if (jointCommandToken != lastJointCommandToken)
        {
            memset(hotplugHoldMotor, 0, sizeof(hotplugHoldMotor));
            feedbackRunDoneStableCount = 0;
            lastJointCommandToken = jointCommandToken;
        }

        const uint8_t motorDirectSource = commandSnapshot.motorDirectCommandSource;
        if (motorDirectSource != lastMotorDirectSource)
        {
            memset(sweepAnchorValid, 0, sizeof(sweepAnchorValid));
            lastMotorDirectSource = motorDirectSource;
        }

        const uint32_t overloadFaultResetToken =
            sharedAtomicLoad(&sharedData->overload_fault_reset_token);
        if (overloadFaultResetToken != lastOverloadFaultResetToken)
        {
            overloadFaultLatched = 0;
            memset(overloadHighCount, 0, sizeof(overloadHighCount));
            {
                SharedSafetyGuard guard;
                sharedAtomicStore(&sharedData->overload_fault_bitmap, (uint32_t)0);
                for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
                    sharedData->overload_fault_load[ch] = 0;
                }
            }
            lastOverloadFaultResetToken = overloadFaultResetToken;
        }

        ServoTargetBatch_t targetBatch;
        memset(&targetBatch, 0, sizeof(targetBatch));
        targetBatch.timestamp = millis();
        targetBatch.source = SERVO_TARGET_OWNER_CONTROL;
        int16_t jointCmdPos[ENCODER_TOTAL_NUM] = {0};
        uint8_t jointCmdValid[ENCODER_TOTAL_NUM] = {0};

        ServoAngleData_t servoData;
        memset(&servoData, 0, sizeof(servoData));
        servoData.timestamp = millis();

        if (sharedData->servoFeedbackQueue) {
            xQueuePeek(sharedData->servoFeedbackQueue, &servoData, 0);
        }

        ServoTelemetryData_t telemetryData;
        memset(&telemetryData, 0, sizeof(telemetryData));
        bool telemetryFresh = false;
        if (sharedData->servoTelemetrySnapshotQueue &&
            xQueuePeek(sharedData->servoTelemetrySnapshotQueue, &telemetryData, 0) == pdTRUE) {
            telemetryFresh =
                (uint32_t)(millis() - telemetryData.timestamp) <= kServoTelemetryTimeoutMs;
        }

        for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++)
        {
            const bool onlineNow = (servoData.onlineStatus[ch] != 0);
            if (onlineNow && !prevMotorOnline[ch]) {
                hotplugHoldMotor[ch] = true;
                sweepAnchorValid[ch] = false;
            }
            if (!onlineNow) {
                sweepAnchorValid[ch] = false;
            }
            prevMotorOnline[ch] = onlineNow;
        }

        sharedAtomicFetchOr(&sharedData->overload_fault_bitmap, overloadFaultLatched);
        overloadFaultLatched = sharedAtomicLoad(&sharedData->overload_fault_bitmap);

        g_dualReleaseAnchorValidBitmap &=
            sharedAtomicLoad(&sharedData->motor_abs_limit_valid_bitmap);

        memset(absolutePositionRaw, 0, sizeof(absolutePositionRaw));
        memset(absolutePositionTendon, 0, sizeof(absolutePositionTendon));
        for (uint8_t jointIndex = 0; jointIndex < ENCODER_TOTAL_NUM; jointIndex++)
        {
            const uint8_t bus = jointMap[jointIndex].busIndex;
            const uint8_t id = jointMap[jointIndex].servoID;
            const int ch = findMotorChannel(bus, id);
            if (ch >= 0 && servoData.onlineStatus[ch]) {
                const int32_t rawAbs = servoData.servoAngles[ch];
                absolutePositionRaw[jointIndex] = rawAbs;
                absolutePositionTendon[jointIndex] =
                    motorRawAbsToTendonAbs((uint8_t)ch, rawAbs);
            }
        }

        sharedAtomicStore(&sharedData->joint16_dual_feedback_fault, (uint8_t)0);

        bool canBusOnline = false;
        if (xQueuePeek(sharedData->canRxQueue, &sensorData, 0) == pdTRUE && sensorData.isValid) {
            const uint32_t nowMs = millis();
            canBusOnline = (nowMs - sensorData.timestamp) <= kCanBusOfflineTimeoutMs;
        }

        memset(&mappedData, 0, sizeof(mappedData));
        mappedData.timestamp = millis();
        mappedData.isValid = canBusOnline;

        if (canBusOnline)
        {
            uint32_t encoderOnlineMask = 0;
            uint32_t encoderFaultMask = 0;
            {
                SharedSafetyGuard guard;
                encoderOnlineMask = sharedAtomicLoad(&sharedData->encoder_online_bitmap);
                encoderFaultMask = sharedAtomicLoad(&sharedData->encoder_fault_bitmap);
            }
            for (int i = 0; i < ENCODER_TOTAL_NUM; i++)
            {
                const uint32_t bit = (1UL << i);
                if (((encoderOnlineMask & bit) == 0) ||
                    ((encoderFaultMask & bit) != 0) ||
                    sensorData.encoderValues[i] == kEncoderDisconnectRaw) {
                    mappedData.validFlags[i] = 0;
                    magAngles[i] = 0.0f;
                    upperWorkspaceHighCount[i] = 0;
                    upperWorkspaceWarningBitmap &= ~bit;
                    continue;
                }

                const int32_t orientedRaw = orientEncoderRaw(sensorData.encoderValues[i], g_encoderDirection[i]);
                if (!g_jointCalibResult[i].success) {
                    mappedData.angleValues[i] = 0;
                    mappedData.validFlags[i] = 0;
                    magAngles[i] = 0.0f;
                    upperWorkspaceHighCount[i] = 0;
                    upperWorkspaceWarningBitmap &= ~bit;
                    continue;
                }

                const JointCalibrationResult& calib = g_jointCalibResult[i];
                float mappedDeg = 0.0f;
                if (calib.mappingVersion >= 2) {
                    EncoderUrdfMapResult mapping;
                    const bool mapped =
                        calib.mappingVersion == kManualScaledUrdfFeedbackMappingVersion
                        ? mapManualScaledEncoderToUrdf(
                              (uint8_t)i,
                              calib,
                              g_manualEncoderReference[i],
                              sensorData.encoderValues[i],
                              g_encoderDirection[i],
                              &mapping)
                        : mapJointCalibrationEncoderToUrdf(
                              (uint8_t)i,
                              calib,
                              orientedRaw,
                              &mapping);
                    if (!mapped) {
                        mappedData.angleValues[i] = 0;
                        mappedData.outsideCounts[i] = 0;
                        mappedData.validFlags[i] = 0;
                        magAngles[i] = 0.0f;
                        upperWorkspaceHighCount[i] = 0;
                        upperWorkspaceWarningBitmap &= ~bit;
                        continue;
                    }
                    mappedDeg = mapping.urdfDeg;
                    int32_t outsideCounts = mapping.outsideCounts;
                    if (outsideCounts > 32767) outsideCounts = 32767;
                    if (outsideCounts < -32768) outsideCounts = -32768;
                    mappedData.outsideCounts[i] = (int16_t)outsideCounts;
                    if (outsideCounts > kUpperWorkspaceWarningCounts) {
                        if (upperWorkspaceHighCount[i] < 255) {
                            upperWorkspaceHighCount[i]++;
                        }
                        if (upperWorkspaceHighCount[i] >=
                                kUpperWorkspaceWarningStableFrames &&
                            (upperWorkspaceWarningBitmap & bit) == 0) {
                            upperWorkspaceWarningBitmap |= bit;
                            FW_LOG_WARN_PRINTF(
                                "[CONTROL] UPPER_WORKSPACE_WARN J%02u outside=%ld threshold=%ld raw=%u mapped=%.3f max=%.3f\r\n",
                                (unsigned)i,
                                (long)outsideCounts,
                                (long)kUpperWorkspaceWarningCounts,
                                (unsigned)sensorData.encoderValues[i],
                                mappedDeg,
                                calib.currentMaxDeg);
                        }
                    } else {
                        if ((upperWorkspaceWarningBitmap & bit) != 0) {
                            FW_LOG_INFO_PRINTF(
                                "[CONTROL] UPPER_WORKSPACE_RECOVER J%02u outside=%ld\r\n",
                                (unsigned)i,
                                (long)outsideCounts);
                        }
                        upperWorkspaceHighCount[i] = 0;
                        upperWorkspaceWarningBitmap &= ~bit;
                    }
                } else {
                    mappedDeg = mapCalibratedEncoderToDeg(calib, orientedRaw);
                    mappedData.outsideCounts[i] = 0;
                }
                const int32_t mappedCount = degToEncoderCount(mappedDeg);
                mappedData.angleValues[i] = clampMappedCountForProtocol(mappedCount);
                mappedData.validFlags[i] = 1;
                magAngles[i] = mappedDeg;
            }
        }

        if (sharedData->mappedAngleQueue) {
            xQueueOverwrite(sharedData->mappedAngleQueue, &mappedData);
        }

        memcpy(localTargets, commandSnapshot.targetAngles, sizeof(localTargets));
        const bool sineTestEnabled =
            commandSnapshot.sineTestEnabled != 0 &&
            commandSnapshot.controlMode == CONTROL_MODE_JOINT;
        if (sineTestEnabled)
        {
            const uint32_t sineTestToken = commandSnapshot.sineTestRequestToken;
            if (!sineTestActive || sineTestToken != lastSineTestRequestToken)
            {
                sineTestStartMs = commandSnapshot.sineTestStartMs != 0
                                      ? commandSnapshot.sineTestStartMs
                                      : millis();
                sineTestActive = true;
                lastSineTestRequestToken = sineTestToken;
                feedbackRunDoneStableCount = 0;
            }

            const float elapsedSec =
                (float)((uint32_t)(millis() - sineTestStartMs)) * 0.001f;
            const float waveSec =
                elapsedSec > ((float)commandSnapshot.sineTestHoldMs * 0.001f)
                    ? elapsedSec - ((float)commandSnapshot.sineTestHoldMs * 0.001f)
                    : 0.0f;
            const uint32_t activeJointMask = commandSnapshot.sineTestJointMask;
            for (uint8_t jointIndex = 0; jointIndex < ENCODER_TOTAL_NUM; jointIndex++)
            {
                if ((activeJointMask & (1UL << jointIndex)) != 0)
                {
                    const float freqHz = sanitizeLocalTestFreq(commandSnapshot.sineTestFreqHz[jointIndex]);
                    const float ampDeg = sanitizeLocalTestAmp(commandSnapshot.sineTestAmpDeg[jointIndex]);
                    const float phaseRad = commandSnapshot.sineTestPhaseRad[jointIndex];
                    localTargets[jointIndex] =
                        commandSnapshot.sineTestCenterDeg[jointIndex] +
                        ampDeg * sinf(kLocalTestTwoPi * freqHz * waveSec + phaseRad);
                }
            }
        }
        else
        {
            sineTestActive = false;
        }
        const bool hostControlEnabled = (sharedData->control_enabled != 0);
        const uint8_t systemState = sharedData->system_state;
        const uint8_t targetOwner = sharedData->servo_target_owner;
        const bool controlOwnerActive = (targetOwner == SERVO_TARGET_OWNER_CONTROL);
        const bool feedbackRunningState = (systemState == SYSTEM_STATE_RUN_FEEDBACK);
        const bool directRunningState = (systemState == SYSTEM_STATE_RUN_DIRECT);
        const bool feedbackOutputState =
            feedbackRunningState && commandSnapshot.controlMode == CONTROL_MODE_JOINT;
        const bool directOutputState =
            directRunningState && commandSnapshot.controlMode == CONTROL_MODE_DIRECT_MOTOR;
        const bool controlEnabled = hostControlEnabled &&
                                    controlOwnerActive &&
                                    (feedbackOutputState || directOutputState);
        const bool normalOutputAllowed = controlEnabled;

        if (normalOutputAllowed && telemetryFresh)
        {
            const uint32_t overloadMotorMask =
                activeOverloadMotorMask(sharedData, commandSnapshot);
            for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++)
            {
                const uint32_t bit = 1UL << ch;
                if (((overloadMotorMask & bit) == 0) ||
                    telemetryData.onlineStatus[ch] == 0)
                {
                    overloadHighCount[ch] = 0;
                    continue;
                }

                const bool jointControlMode =
                    (commandSnapshot.controlMode == CONTROL_MODE_JOINT);
                const int16_t overloadThreshold = jointControlMode
                                                      ? kJointControlOverloadLoadThreshold
                                                      : kControlOverloadLoadThreshold;
                const uint8_t overloadLatchCycles = jointControlMode
                                                        ? kJointControlOverloadLatchCycles
                                                        : kControlOverloadLatchCycles;
                const int16_t loadMag = servoLoadMagnitude(telemetryData.load[ch]);
                if (loadMag >= overloadThreshold) {
                    if (overloadHighCount[ch] < 255) {
                        overloadHighCount[ch]++;
                    }
                } else {
                    overloadHighCount[ch] = 0;
                }

                if (overloadHighCount[ch] >= overloadLatchCycles &&
                    (overloadFaultLatched & bit) == 0)
                {
                    overloadFaultLatched |= bit;
                    {
                        SharedSafetyGuard guard;
                        sharedData->overload_fault_load[ch] = telemetryData.load[ch];
                    }
                    FW_LOG_INFO_PRINTF("[CONTROL] OVERLOAD_HOLD M%02u load=%d threshold=%d cycles=%u mode=%u\r\n",
                                  (unsigned)ch,
                                  loadMag,
                                  overloadThreshold,
                                  (unsigned)overloadLatchCycles,
                                  (unsigned)commandSnapshot.controlMode);
                }
            }
        }
        else
        {
            memset(overloadHighCount, 0, sizeof(overloadHighCount));
        }
        sharedAtomicFetchOr(&sharedData->overload_fault_bitmap, overloadFaultLatched);
        overloadFaultLatched = sharedAtomicLoad(&sharedData->overload_fault_bitmap);

        if (commandSnapshot.controlMode == CONTROL_MODE_JOINT) {
            for (uint8_t jointIndex = 0; jointIndex < ENCODER_TOTAL_NUM; jointIndex++) {
                localTargets[jointIndex] = clampJointTargetDegByCalib(localTargets[jointIndex], jointIndex);
            }
        }

        const bool feedbackOneShotActive =
            feedbackRunningState &&
            commandSnapshot.controlMode == CONTROL_MODE_JOINT &&
            !sineTestActive;
        if (feedbackOneShotActive) {
            // Snapshot the clamped target for this command generation. Completion
            // must be judged against one stable target, not a later UI update.
            if (!feedbackRunTracking || feedbackRunToken != jointCommandToken) {
                memcpy(feedbackRunTargetDeg, localTargets, sizeof(feedbackRunTargetDeg));
                feedbackRunToken = jointCommandToken;
                feedbackRunStartMs = millis();
                feedbackRunDoneStableCount = 0;
                feedbackRunTracking = true;
                feedbackRunDoneEventQueued = false;
                feedbackRunTimeoutEventQueued = false;
            }
        } else {
            feedbackRunTracking = false;
            feedbackRunDoneStableCount = 0;
            feedbackRunDoneEventQueued = false;
            feedbackRunTimeoutEventQueued = false;
        }

        // Only the CONTROL owner may construct servo targets. Angle mapping,
        // telemetry and debug publication above continue during calibration,
        // but they must not compete with the calibration producer for the
        // single-slot servoTargetQueue.
        const bool controlTargetBuildAllowed = controlEnabled;
        if (!controlTargetBuildAllowed || !feedbackOutputState) {
            for (uint8_t jointIndex = 0; jointIndex < ENCODER_TOTAL_NUM; jointIndex++) {
                if (isDualTendonJoint(jointIndex)) {
                    resetDualTendonController(jointIndex);
                }
            }
        }
        if (controlTargetBuildAllowed) {
            const TickType_t pidComputeTick = xTaskGetTickCount();
            const TickType_t elapsedPidTicks = pidComputeTick - lastPidComputeTick;
            lastPidComputeTick = pidComputeTick;
            float pidDtSeconds =
                (float)elapsedPidTicks * (float)portTICK_PERIOD_MS * 0.001f;
            // Use the nominal period on first entry or after a scheduler stall.
            // This prevents a stale interval from creating an I/D impulse.
            if (!isfinite(pidDtSeconds) || pidDtSeconds < 0.001f || pidDtSeconds > 0.100f) {
                pidDtSeconds = 0.010f;
            }
            // PID uses tendon-space ABS so a positive correction has the same
            // "shorten tendon" meaning on motors with opposite raw ABS directions.
            // Single-motor output converts the final tendon-space target back to raw.
            g_controlSolver.compute(localTargets,
                                    magAngles,
                                    absolutePositionTendon,
                                    outPulsesTendon,
                                    pidDtSeconds);
            if (commandSnapshot.controlMode == CONTROL_MODE_DIRECT_MOTOR)
            {
                const uint32_t activeMotorMask =
                    sharedAtomicLoad(&sharedData->active_motor_mask) &
                    validMotorChannelMask();
                for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++)
                {
                    if (((activeMotorMask >> ch) & 0x1UL) == 0) {
                        continue;
                    }
                    const uint8_t bus = motorMap[ch].busIndex;
                    const uint8_t id = motorMap[ch].servoID;
                    if (bus >= NUM_BUSES) {
                        continue;
                    }

                    int16_t targetPos = clampServoPos(servoData.servoAngles[ch]);
                    if (!hotplugHoldMotor[ch]) {
                        if (motorDirectSource == MOTOR_DIRECT_SOURCE_SWEEP) {
                            const int16_t rawTarget =
                                clampServoSingleTurnRaw(commandSnapshot.motorSweepTargetRaw[ch]);
                            if (!sweepAnchorValid[ch]) {
                                sweepExpandedTarget[ch] =
                                    anchorSingleTurnSweepToCurrent(servoData.servoAngles[ch], rawTarget);
                                prevSweepRawTarget[ch] = rawTarget;
                                sweepAnchorValid[ch] = true;
                            } else if (rawTarget != prevSweepRawTarget[ch]) {
                                const int32_t d = (int32_t)rawTarget - (int32_t)prevSweepRawTarget[ch];
                                sweepExpandedTarget[ch] += wrapServoRawDelta(d);
                                prevSweepRawTarget[ch] = rawTarget;
                            }
                            targetPos = clampServoPos(sweepExpandedTarget[ch]);
                            sweepExpandedTarget[ch] = targetPos;
                        } else if (motorDirectSource == MOTOR_DIRECT_SOURCE_ABSOLUTE) {
                            sweepAnchorValid[ch] = false;
                            targetPos = clampServoPos(
                                (int32_t)commandSnapshot.motorTargetRaw[ch]);
                        } else {
                            sweepAnchorValid[ch] = false;
                            targetPos = expandSingleTurnTargetNearCurrent(
                                servoData.servoAngles[ch],
                                clampServoSingleTurnRaw(commandSnapshot.motorTargetRaw[ch])
                            );
                        }
                    } else {
                        sweepAnchorValid[ch] = false;
                    }
                    appendServoTarget(&targetBatch, bus, id, targetPos,
                                      SERVO_TARGET_SPEED_DEFAULT, SERVO_TARGET_ACC_DEFAULT);
                }
            }
            else
            {
            for (uint8_t jointIndex = 0; jointIndex < ENCODER_TOTAL_NUM; jointIndex++)
            {
                const uint8_t bus = jointMap[jointIndex].busIndex;
                const uint8_t id = jointMap[jointIndex].servoID;
                const int jointMotorCh = findMotorChannel(bus, id);
                const bool servoOnline = (jointMotorCh >= 0) && (servoData.onlineStatus[jointMotorCh] != 0);
                bool overloadFaultActive = jointHasOverloadFault(jointIndex, overloadFaultLatched);

                bool emergency =
                    (!normalOutputAllowed) ||
                    shouldEmergencyStop(canBusOnline,
                                        sensorData,
                                        mappedData,
                                        jointIndex,
                                        ((sharedAtomicLoad(&sharedData->encoder_fault_bitmap) &
                                          (1UL << jointIndex)) != 0));
                emergency = emergency || overloadFaultActive;
                const bool hotplugHoldActive =
                    (jointMotorCh >= 0) &&
                    (jointMotorCh < SERVO_TOTAL_NUM) &&
                    hotplugHoldMotor[jointMotorCh];
                const bool jointActive =
                    (sharedAtomicLoad(&sharedData->active_joint_mask) &
                     (1UL << jointIndex)) != 0;

                if (bus >= NUM_BUSES) {
                    continue;
                }

                if (!jointActive)
                {
                    if (isDualTendonJoint(jointIndex)) {
                        resetDualTendonController(jointIndex);
                    }
                    jointCmdPos[jointIndex] = clampServoPos(absolutePositionRaw[jointIndex]);
                    jointCmdValid[jointIndex] = 0;
                    continue;
                }

                if (emergency)
                {
                    if (isDualTendonJoint(jointIndex)) {
                        resetDualTendonController(jointIndex);
                    }
                    const int16_t holdPos = clampServoPos(absolutePositionRaw[jointIndex]);
                    appendJointActuatorHoldTargets(&targetBatch, jointIndex, servoData,
                                                   SERVO_TARGET_SPEED_DEFAULT, SERVO_TARGET_ACC_DEFAULT);
                    jointCmdPos[jointIndex] = holdPos;
                    jointCmdValid[jointIndex] = servoOnline ? 1 : 0;
                    continue;
                }

                if (hotplugHoldActive)
                {
                    if (isDualTendonJoint(jointIndex)) {
                        resetDualTendonController(jointIndex);
                    }
                    const int16_t holdPos = clampServoPos(absolutePositionRaw[jointIndex]);
                    appendJointActuatorHoldTargets(&targetBatch, jointIndex, servoData,
                                                   SERVO_TARGET_SPEED_DEFAULT, SERVO_TARGET_ACC_DEFAULT);
                    jointCmdPos[jointIndex] = holdPos;
                    jointCmdValid[jointIndex] = servoOnline ? 1 : 0;
                    continue;
                }

                int16_t targetPos = clampServoPos(absolutePositionRaw[jointIndex]);

                // Only active joints follow PID and emit servo targets.
                const bool dualTendonJoint = isDualTendonJoint(jointIndex);
                if (jointActive && dualTendonJoint) {
                    const uint8_t appended =
                        appendDualTendonTargets(&targetBatch,
                                                sharedData,
                                                jointIndex,
                                                localTargets[jointIndex],
                                                magAngles[jointIndex],
                                                servoData,
                                                kJointControlFeedbackSpeed,
                                                SERVO_TARGET_ACC_DEFAULT);
                    if (appended == 0) {
                        appendJointActuatorHoldTargets(&targetBatch, jointIndex, servoData,
                                                       SERVO_TARGET_SPEED_DEFAULT, SERVO_TARGET_ACC_DEFAULT);
                    }
                    jointCmdPos[jointIndex] = targetPos;
                    jointCmdValid[jointIndex] = (appended > 0) ? 1 : 0;
                    continue;
                }

                if (jointActive) {
                    if (jointMotorCh >= 0 && jointMotorCh < SERVO_TOTAL_NUM) {
                        const uint8_t motorCh = (uint8_t)jointMotorCh;
                        const int32_t targetRawAbs =
                            motorTendonAbsToRawAbs(motorCh, outPulsesTendon[jointIndex]);
                        targetPos = clampServoPosByMotorAbsLimit(sharedData,
                                                                 motorCh,
                                                                 targetRawAbs);
                    } else {
                        targetPos = clampServoPos(outPulsesTendon[jointIndex]);
                    }
                }

                if (dualTendonJoint) {
                    appendJointActuatorHoldTargets(&targetBatch, jointIndex, servoData,
                                                   SERVO_TARGET_SPEED_DEFAULT, SERVO_TARGET_ACC_DEFAULT);
                } else {
                    appendJointActuatorTargets(&targetBatch, jointIndex, targetPos,
                                               jointActive ? kJointControlFeedbackSpeed : SERVO_TARGET_SPEED_DEFAULT,
                                               SERVO_TARGET_ACC_DEFAULT);
                }
                jointCmdPos[jointIndex] = targetPos;
                jointCmdValid[jointIndex] = 1;
            }
            }
        }

        if (sharedData->jointDebugQueue)
        {
            for (uint8_t di = 0; di < kDebugJointCount; di++)
            {
                const uint8_t jointIndex = kDebugJointIndices[di];
                JointDebugData_t debugData;
                memset(&debugData, 0, sizeof(debugData));
                debugData.jointIndex = jointIndex;
                debugData.timestamp = millis();
                debugData.valid = (mappedData.validFlags[jointIndex] != 0) ? 1 : 0;
                debugData.targetDeg = localTargets[jointIndex];
                debugData.magActualDeg = magAngles[jointIndex];
                debugData.loop1Output = g_controlSolver.getPidOutput(jointIndex, 0);
                debugData.loop2Actual = (float)absolutePositionTendon[jointIndex];
                debugData.loop2Output = g_controlSolver.getPidOutput(jointIndex, 1);
                debugData.cmdTargetPos = jointCmdPos[jointIndex];
                debugData.cmdValid = jointCmdValid[jointIndex];

                if (xQueueSend(sharedData->jointDebugQueue, &debugData, 0) != pdTRUE) {
                    JointDebugData_t drop;
                    xQueueReceive(sharedData->jointDebugQueue, &drop, 0);
                    xQueueSend(sharedData->jointDebugQueue, &debugData, 0);
                }
            }
        }

        const uint8_t publishOwner =
            sharedAtomicLoad(&sharedData->servo_target_owner);
        const uint8_t publishState =
            sharedAtomicLoad(&sharedData->system_state);
        const bool publishControlEnabled =
            sharedAtomicLoad(&sharedData->control_enabled) != 0;
        const bool publishStateMatchesMode =
            (publishState == SYSTEM_STATE_RUN_FEEDBACK &&
             commandSnapshot.controlMode == CONTROL_MODE_JOINT) ||
            (publishState == SYSTEM_STATE_RUN_DIRECT &&
             commandSnapshot.controlMode == CONTROL_MODE_DIRECT_MOTOR);
        const bool controlTargetPublishAllowed =
            publishControlEnabled &&
            publishOwner == SERVO_TARGET_OWNER_CONTROL &&
            publishStateMatchesMode;
        if (sharedData->servoTargetQueue &&
            targetBatch.count > 0 &&
            controlTargetPublishAllowed) {
            xQueueOverwrite(sharedData->servoTargetQueue, &targetBatch);
        }

        if (directRunningState &&
            commandSnapshot.controlMode == CONTROL_MODE_DIRECT_MOTOR &&
            commandSnapshot.motorDirectCommandSource == MOTOR_DIRECT_SOURCE_ABSOLUTE) {
            if (directAbsoluteTargetsReached(sharedData, servoData, commandSnapshot)) {
                if (directRunDoneStableCount < 255) {
                    directRunDoneStableCount++;
                }
                if (directRunDoneStableCount >= kDirectRunDoneStableCycles) {
                    postSystemEventForToken(
                        sharedData,
                        SYSTEM_EVENT_RUN_DIRECT_DONE,
                        commandSnapshot.motorDirectCommandGeneration);
                    directRunDoneStableCount = 0;
                }
            } else {
                directRunDoneStableCount = 0;
            }
        } else {
            directRunDoneStableCount = 0;
        }

        if (feedbackOneShotActive && feedbackRunTracking) {
            const bool targetsReached = feedbackJointTargetsReached(
                sharedData,
                mappedData,
                feedbackRunTargetDeg,
                magAngles
            );
            if (targetsReached) {
                if (feedbackRunDoneStableCount < 255) {
                    feedbackRunDoneStableCount++;
                }
                if (feedbackRunDoneStableCount >= kFeedbackRunDoneStableCycles &&
                    !feedbackRunDoneEventQueued) {
                    // Queue-full is transient. Keep the completion pending and retry
                    // instead of dropping it and leaving RUN_FEEDBACK latched.
                    feedbackRunDoneEventQueued = postSystemEventForToken(
                        sharedData,
                        SYSTEM_EVENT_RUN_FEEDBACK_DONE,
                        feedbackRunToken
                    );
                }
            } else {
                feedbackRunDoneStableCount = 0;
            }

            const uint32_t elapsedMs = (uint32_t)(millis() - feedbackRunStartMs);
            if (!targetsReached &&
                !feedbackRunDoneEventQueued &&
                elapsedMs >= kFeedbackRunTimeoutMs &&
                !feedbackRunTimeoutEventQueued) {
                feedbackRunTimeoutEventQueued = postSystemEventForToken(
                    sharedData,
                    SYSTEM_EVENT_RUN_FEEDBACK_TIMEOUT,
                    feedbackRunToken
                );
            }
        }

        taskRuntimeMarkLoopEnd(TASK_RUNTIME_CONTROL);
        const TickType_t nextWakeTime = lastWakeTime + solverPeriodTicks;
        const bool periodOverrun = (int32_t)(xTaskGetTickCount() - nextWakeTime) >= 0;
        taskRuntimeMarkPeriodResult(TASK_RUNTIME_CONTROL, periodOverrun);
        vTaskDelayUntil(&lastWakeTime, solverPeriodTicks);
        if (periodOverrun) vTaskDelay(1);
    }
}
