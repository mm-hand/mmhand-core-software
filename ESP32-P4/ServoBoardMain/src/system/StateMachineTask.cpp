#include "StateMachineTask.h"

#include <math.h>
#include <string.h>

#include "../calibration/AutoCalibrationTask.h"
#include "../calibration/CalibrationMath.h"
#include "../calibration/CalibrationTask.h"
#include "../calibration/RopeInitTask.h"
#include "../generated/UrdfJointLimits.generated.h"
#include "../hardware/HardwareMap.h"
#include "../persistence/ManualCalibrationNvsStore.h"
#include "../shared/SharedSafetyState.h"
#include "FirmwareLog.h"
#include "TaskRuntimeMonitor.h"

extern volatile uint8_t g_calibrationStatus;
extern volatile uint8_t g_calibrationAckStatus;
extern volatile uint8_t g_calibrationAckRepeatRemaining;

static const uint32_t kStateCanOfflineTimeoutMs = 1000;
static const uint32_t kStateServoFeedbackTimeoutMs = 500;
static const int32_t kReadyEnterOutsideToleranceCounts = 24;
static const int32_t kReadyExitOutsideToleranceCounts = 48;
static const uint8_t kCalibrationAckRepeatCount = 20;
static uint32_t collectSystemFaults(TaskSharedData_t* sharedData);
static bool activeJointsInControlWorkspace(TaskSharedData_t* sharedData);
static uint32_t readyServoRequiredMask(TaskSharedData_t* sharedData);

static bool manualReferenceGateReady(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return false;
    }
    const uint32_t activeMask =
        sharedAtomicLoad(&sharedData->active_joint_mask) &
        kManualEncoderReferenceFullMask;
    const uint32_t validMask =
        sharedAtomicLoad(&sharedData->manual_reference_valid_bitmap) &
        kManualEncoderReferenceFullMask;
    return activeMask != 0 &&
           sharedData->manual_reference_nvs_valid != 0 &&
           (validMask & activeMask) == activeMask &&
           manualEncoderReferencesCoverActiveMask(activeMask);
}

static void logManualReferenceReject(TaskSharedData_t* sharedData,
                                     const char* eventName)
{
    static uint32_t s_lastLogMs = 0;
    static uint32_t s_lastMask = 0;
    const uint32_t nowMs = millis();
    const uint32_t validMask = sharedData
        ? sharedAtomicLoad(&sharedData->manual_reference_valid_bitmap) : 0;
    if (validMask == s_lastMask &&
        (uint32_t)(nowMs - s_lastLogMs) <= 1000) {
        return;
    }
    s_lastMask = validMask;
    s_lastLogMs = nowMs;
    const uint32_t requiredMask = sharedData
        ? (sharedAtomicLoad(&sharedData->active_joint_mask) &
           kManualEncoderReferenceFullMask)
        : 0;
    FW_LOG_INFO_PRINTF(
        "[STATE] reject %s: Manual encoder reference incomplete "
        "required=0x%08lX valid=0x%08lX missing=0x%08lX nvs=%u\r\n",
        eventName ? eventName : "MOTION",
        (unsigned long)requiredMask,
        (unsigned long)validMask,
        (unsigned long)(requiredMask & ~validMask),
        sharedData && sharedData->manual_reference_nvs_valid ? 1U : 0U);
}

static void setCalibrationStatus(uint8_t status, bool emitAck)
{
    g_calibrationStatus = status;
    if (status == CALIB_STATUS_IDLE) {
        g_calibrationAckStatus = CALIB_STATUS_IDLE;
        g_calibrationAckRepeatRemaining = 0;
        return;
    }
    if (emitAck) {
        g_calibrationAckStatus = status;
        g_calibrationAckRepeatRemaining = kCalibrationAckRepeatCount;
    }
}

bool postSystemEvent(TaskSharedData_t* sharedData, uint8_t event)
{
    return postSystemEventForToken(sharedData, event, 0);
}

bool requestEmergencyStop(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return false;
    }

    // Latch STOP independently of the normal event queue, then notify the
    // servo task immediately. Both tokens are monotonic and cannot be dropped
    // when the normal command queue is full.
    sharedAtomicAddFetch(&sharedData->emergency_stop_request_token, (uint32_t)1);
    sharedAtomicAddFetch(&sharedData->servo_emergency_stop_token, (uint32_t)1);
    taskRuntimeMarkStopRequested();
    return true;
}

bool postSystemEventForToken(TaskSharedData_t* sharedData, uint8_t event, uint32_t token)
{
    if (event == SYSTEM_EVENT_STOP) {
        return requestEmergencyStop(sharedData);
    }
    if (!sharedData || !sharedData->stateEventQueue) {
        return false;
    }

    SystemEvent_t msg;
    msg.event = event;
    msg.timestamp = millis();
    msg.token = token;
    return xQueueSend(sharedData->stateEventQueue, &msg, 0) == pdTRUE;
}

static void requestMultiTurnNvsSave(TaskSharedData_t* sharedData, uint8_t reason)
{
    if (!sharedData) {
        return;
    }
#if !MULTITURN_NVS_PERSISTENCE_ENABLE
    (void)reason;
    return;
#else
    sharedData->multiturn_nvs_save_reason = reason;
    sharedAtomicAddFetch(&sharedData->multiturn_nvs_save_request_token, (uint32_t)1);
#endif
}

static bool shouldSaveMultiTurnOnInterrupt(uint8_t state)
{
    return state == SYSTEM_STATE_RUN_DIRECT ||
           state == SYSTEM_STATE_RUN_FEEDBACK ||
           state == SYSTEM_STATE_CALIBRATION_RUNNING ||
           state == SYSTEM_STATE_ROPE_INIT_RUNNING ||
           state == SYSTEM_STATE_PAUSED;
}

static void setSystemState(TaskSharedData_t* sharedData, uint8_t state)
{
    if (!sharedData) {
        return;
    }
    if (sharedData->system_state != state) {
        sharedData->system_state = state;
        sharedData->state_transition_count++;
    }
}

static void resetCommandState(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return;
    }

    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }

    for (uint8_t i = 0; i < ENCODER_TOTAL_NUM; i++) {
        sharedData->targetAngles[i] = 0.0f;
    }
    for (uint8_t i = 0; i < SERVO_TOTAL_NUM; i++) {
        sharedData->motorTargetRaw[i] = 0;
        sharedData->motorSweepTargetRaw[i] = 0;
        sharedData->pendingMotorTargetRaw[i] = 0;
        sharedData->pendingMotorSweepTargetRaw[i] = 0;
    }
    sharedData->motor_command_token++;
    sharedData->motor_sweep_command_token++;
    sharedData->joint_command_token++;
    sharedData->motor_direct_command_generation++;
    sharedData->motor_direct_command_source = MOTOR_DIRECT_SOURCE_NONE;
    sharedData->pending_direct_generation = sharedData->motor_direct_command_generation;
    sharedData->applied_direct_generation = sharedData->motor_direct_command_generation;
    sharedData->pending_direct_source = MOTOR_DIRECT_SOURCE_NONE;
    sharedData->pending_direct_valid = 0;
    sharedData->pending_direct_motor_mask = 0;
    sharedData->pending_direct_joint_mask = 0;
    sharedData->pending_direct_require_can = 0;
    sharedData->direct_reject_reason = DIRECT_REJECT_NONE;
    sharedData->control_mode = CONTROL_MODE_JOINT;
    sharedData->sine_test_enabled = 0;
    sharedData->sine_test_request_token++;
    sharedData->sine_test_start_ms = 0;
    sharedData->sine_test_joint_mask = 0;
    sharedData->sine_test_hold_ms = 0;
    memset(sharedData->sine_test_center_deg, 0, sizeof(sharedData->sine_test_center_deg));
    memset(sharedData->sine_test_amp_deg, 0, sizeof(sharedData->sine_test_amp_deg));
    memset(sharedData->sine_test_freq_hz, 0, sizeof(sharedData->sine_test_freq_hz));
    memset(sharedData->sine_test_phase_rad, 0, sizeof(sharedData->sine_test_phase_rad));

    if (lock) {
        xSemaphoreGive(lock);
    }
}

static bool setCommandModeJoint(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return false;
    }

    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        return false;
    }
    sharedData->control_mode = CONTROL_MODE_JOINT;
    sharedData->motor_direct_command_source = MOTOR_DIRECT_SOURCE_NONE;
    if (lock) {
        xSemaphoreGive(lock);
    }
    return true;
}

static void setCommandModeDirect(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return;
    }

    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    sharedData->control_mode = CONTROL_MODE_DIRECT_MOTOR;
    if (lock) {
        xSemaphoreGive(lock);
    }
}

static void clearPendingDirectCommand(TaskSharedData_t* sharedData,
                                      uint8_t rejectReason)
{
    if (!sharedData) {
        return;
    }
    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    sharedData->pending_direct_valid = 0;
    sharedData->pending_direct_source = MOTOR_DIRECT_SOURCE_NONE;
    sharedData->pending_direct_motor_mask = 0;
    sharedData->pending_direct_joint_mask = 0;
    sharedData->pending_direct_require_can = 0;
    sharedData->direct_reject_reason = rejectReason;
    if (lock) {
        xSemaphoreGive(lock);
    }
}

static bool pendingDirectCommandMatches(TaskSharedData_t* sharedData,
                                        uint32_t generation)
{
    if (!sharedData) {
        return false;
    }
    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        return false;
    }
    const bool matches =
        sharedData->pending_direct_valid != 0 &&
        sharedData->pending_direct_generation == generation;
    if (lock) {
        xSemaphoreGive(lock);
    }
    return matches;
}

static uint32_t pendingDirectMotorMask(TaskSharedData_t* sharedData,
                                       uint32_t generation)
{
    if (!sharedData) {
        return 0;
    }
    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        return 0;
    }
    const uint32_t motorMask =
        (sharedData->pending_direct_valid != 0 &&
         sharedData->pending_direct_generation == generation)
            ? sharedData->pending_direct_motor_mask
            : 0;
    if (lock) {
        xSemaphoreGive(lock);
    }
    return motorMask & validMotorChannelMask();
}

static void rejectPendingDirectCommand(TaskSharedData_t* sharedData,
                                       uint32_t generation,
                                       uint8_t rejectReason)
{
    if (!sharedData) {
        return;
    }
    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    if (sharedData->pending_direct_valid != 0 &&
        sharedData->pending_direct_generation == generation) {
        sharedData->pending_direct_valid = 0;
        sharedData->pending_direct_source = MOTOR_DIRECT_SOURCE_NONE;
        sharedData->pending_direct_motor_mask = 0;
        sharedData->pending_direct_joint_mask = 0;
        sharedData->pending_direct_require_can = 0;
        sharedData->direct_reject_reason = rejectReason;
    }
    if (lock) {
        xSemaphoreGive(lock);
    }
}

static bool promotePendingDirectCommand(TaskSharedData_t* sharedData,
                                        uint32_t generation)
{
    if (!sharedData) {
        return false;
    }
    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        return false;
    }
    if (sharedData->pending_direct_valid == 0 ||
        sharedData->pending_direct_generation != generation) {
        if (lock) {
            xSemaphoreGive(lock);
        }
        return false;
    }

    const uint8_t source = sharedData->pending_direct_source;
    if (source == MOTOR_DIRECT_SOURCE_SWEEP) {
        memcpy(sharedData->motorSweepTargetRaw,
               sharedData->pendingMotorSweepTargetRaw,
               sizeof(sharedData->motorSweepTargetRaw));
        sharedData->motor_sweep_command_token++;
    } else {
        memcpy(sharedData->motorTargetRaw,
               sharedData->pendingMotorTargetRaw,
               sizeof(sharedData->motorTargetRaw));
        sharedData->motor_command_token++;
    }
    sharedData->motor_direct_command_generation = generation;
    sharedData->motor_direct_command_source = source;
    sharedData->applied_direct_generation = generation;
    {
        SharedSafetyGuard guard;
        sharedAtomicStore(&sharedData->active_motor_mask,
                          sharedData->pending_direct_motor_mask & validMotorChannelMask());
        sharedAtomicStore(&sharedData->active_joint_mask,
                          sharedData->pending_direct_joint_mask &
                          ((1UL << ENCODER_TOTAL_NUM) - 1UL));
        sharedAtomicStore(&sharedData->require_can_feedback,
                          (uint8_t)(sharedData->pending_direct_require_can ? 1U : 0U));
    }
    sharedData->pending_direct_valid = 0;
    sharedData->pending_direct_source = MOTOR_DIRECT_SOURCE_NONE;
    sharedData->pending_direct_motor_mask = 0;
    sharedData->pending_direct_joint_mask = 0;
    sharedData->pending_direct_require_can = 0;
    sharedData->direct_reject_reason = DIRECT_REJECT_NONE;
    sharedData->control_mode = CONTROL_MODE_DIRECT_MOTOR;
    if (lock) {
        xSemaphoreGive(lock);
    }
    return true;
}

static void resetServoTargetQueue(TaskSharedData_t* sharedData)
{
    if (sharedData && sharedData->servoTargetQueue) {
        xQueueReset(sharedData->servoTargetQueue);
    }
}

static void setFeedbackAdmissionResult(TaskSharedData_t* sharedData,
                                       uint32_t requestSequence,
                                       uint8_t status,
                                       uint8_t rejectReason)
{
    if (!sharedData || requestSequence == 0) {
        return;
    }
    if (sharedAtomicLoad(&sharedData->feedback_admission_sequence) != requestSequence) {
        return;
    }
    sharedAtomicStore(&sharedData->feedback_admission_status, status);
    sharedAtomicStore(&sharedData->feedback_reject_reason, rejectReason);
}

static void requestServoEmergencyTorqueOff(TaskSharedData_t* sharedData)
{
    if (sharedData) {
        sharedAtomicAddFetch(&sharedData->servo_emergency_stop_token, (uint32_t)1);
    }
}

static bool canAcceptStart(uint8_t state)
{
    return state == SYSTEM_STATE_UNCONTROLLABLE ||
           state == SYSTEM_STATE_IDLE ||
           state == SYSTEM_STATE_READY ||
           state == SYSTEM_STATE_PAUSED;
}

static bool canAcceptRunDirect(uint8_t state)
{
    return state == SYSTEM_STATE_IDLE ||
           state == SYSTEM_STATE_READY ||
           state == SYSTEM_STATE_RUN_DIRECT;
}

static bool canAcceptRunFeedback(uint8_t state)
{
    return state == SYSTEM_STATE_READY;
}

static bool canAcceptCalibrate(uint8_t state)
{
    // Before the selected joints have Auto results, the feedback state can
    // legitimately remain UNCONTROLLABLE. Manual coverage is checked before
    // this state gate, so calibration must be allowed to create those results.
    return state == SYSTEM_STATE_UNCONTROLLABLE ||
           state == SYSTEM_STATE_IDLE ||
           state == SYSTEM_STATE_READY;
}

static bool activeMotorsMultiTurnKnown(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return false;
    }
    const uint32_t activeMask = readyServoRequiredMask(sharedData);
    return activeMask != 0 &&
           ((sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap) & activeMask) == 0);
}

static bool activeMotorsAbsLimitsCalibrated(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return false;
    }
    const uint32_t activeMask = readyServoRequiredMask(sharedData);
    if (activeMask == 0) {
        return false;
    }
    const uint32_t validLimitMask =
        sharedAtomicLoad(&sharedData->motor_abs_limit_valid_bitmap);
    const uint32_t missingMask = activeMask & ~validLimitMask;
    if (missingMask == 0) {
        return true;
    }

    static uint32_t s_lastMissingLimitLogMs = 0;
    static uint32_t s_lastMissingLimitMask = 0;
    const uint32_t nowMs = millis();
    if (missingMask != s_lastMissingLimitMask ||
        (uint32_t)(nowMs - s_lastMissingLimitLogMs) > 1000) {
        FW_LOG_INFO_PRINTF("[STATE] feedback ready rejected: uncalibrated ABS limit active=0x%08lX missing=0x%08lX valid=0x%08lX\r\n",
                      (unsigned long)activeMask,
                      (unsigned long)missingMask,
                      (unsigned long)validLimitMask);
        s_lastMissingLimitMask = missingMask;
        s_lastMissingLimitLogMs = nowMs;
    }
    return false;
}

static void logFeedbackReadyReject(TaskSharedData_t* sharedData,
                                   uint8_t reason,
                                   uint32_t a,
                                   uint32_t b,
                                   int32_t c,
                                   int32_t d)
{
    static uint32_t s_lastLogMs = 0;
    static uint8_t s_lastReason = 0;
    static uint32_t s_lastA = 0;
    static uint32_t s_lastB = 0;
    static int32_t s_lastC = 0;
    static int32_t s_lastD = 0;
    const uint32_t nowMs = millis();
    if (reason == s_lastReason &&
        a == s_lastA &&
        b == s_lastB &&
        c == s_lastC &&
        d == s_lastD &&
        (uint32_t)(nowMs - s_lastLogMs) <= 1000) {
        return;
    }
    s_lastReason = reason;
    s_lastA = a;
    s_lastB = b;
    s_lastC = c;
    s_lastD = d;
    s_lastLogMs = nowMs;

    switch (reason) {
        case 1:
            FW_LOG_INFO_PRINTF("[STATE] feedback ready rejected: joint mode/CAN gate inactive motor=0x%08lX joint=0x%08lX mode=%ld require_can=%ld\r\n",
                          (unsigned long)a,
                          (unsigned long)b,
                          (long)c,
                          (long)d);
            break;
        case 2:
            FW_LOG_INFO_PRINTF("[STATE] feedback ready rejected: multi-turn unknown active=0x%08lX unknown=0x%08lX\r\n",
                          (unsigned long)a,
                          (unsigned long)b);
            break;
        case 3:
            FW_LOG_INFO_PRINTF("[STATE] feedback ready rejected: active joint mask is zero motor=0x%08lX\r\n",
                          (unsigned long)a);
            break;
        case 4:
            FW_LOG_INFO_PRINTF("[STATE] feedback ready rejected: mapped angle unavailable code=%lu age_ms=%ld\r\n",
                          (unsigned long)a,
                          (long)c);
            break;
        case 5:
            FW_LOG_INFO_PRINTF("[STATE] feedback ready rejected: joint calibration missing J%02lu active=0x%08lX\r\n",
                          (unsigned long)a,
                          (unsigned long)b);
            break;
        case 6:
            FW_LOG_INFO_PRINTF("[STATE] feedback ready rejected: mapped angle invalid J%02lu valid_flags=0x%08lX\r\n",
                          (unsigned long)a,
                          (unsigned long)b);
            break;
        case 7:
            FW_LOG_INFO_PRINTF("[STATE] feedback ready rejected: joint below workspace J%02lu preserve=%ld outside=%ld tolerance=%ld\r\n",
                          (unsigned long)a,
                          (long)((int32_t)b),
                          (long)c,
                          (long)d);
            break;
        case 8:
            FW_LOG_INFO_PRINTF("[STATE] feedback ready rejected: calibration/manual mismatch J%02lu mapping=%lu urdf_crc=0x%08lX manual_entry_crc=0x%08lX\r\n",
                          (unsigned long)a,
                          (unsigned long)b,
                          (unsigned long)(uint32_t)c,
                          (unsigned long)(uint32_t)d);
            break;
        case 9:
            FW_LOG_INFO_PRINTF("[STATE] feedback ready rejected: encoder/CAN unavailable offline=0x%08lX fault=0x%08lX sensor_fresh=%ld can_link=%ld\r\n",
                          (unsigned long)a,
                          (unsigned long)b,
                          (long)c,
                          (long)d);
            break;
        case 10:
            FW_LOG_INFO_PRINTF("[STATE] feedback ready rejected: system fault=0x%08lX\r\n",
                          (unsigned long)a);
            break;
        default:
            FW_LOG_INFO_PRINTLN("[STATE] feedback ready rejected: unknown reason");
            break;
    }
}

static void logMultiTurnUnknownReject(TaskSharedData_t* sharedData, const char* eventName)
{
    if (!sharedData) {
        return;
    }
    FW_LOG_INFO_PRINTF("[STATE] reject %s: multi-turn unknown active=0x%08lX unknown=0x%08lX\r\n",
                  eventName ? eventName : "RUN",
                  (unsigned long)readyServoRequiredMask(sharedData),
                  (unsigned long)sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap));
}

static void enterFaultHold(TaskSharedData_t* sharedData, uint32_t faultBits)
{
    if (!sharedData) {
        return;
    }
    const uint8_t interruptedState = sharedData->system_state;
    clearPendingDirectCommand(sharedData, DIRECT_REJECT_FAULT);
    resetServoTargetQueue(sharedData);
    sharedAtomicFetchOr(&sharedData->system_fault_latched_bitmap, faultBits);
    const uint32_t latchedFaults =
        sharedAtomicLoad(&sharedData->system_fault_latched_bitmap);
    sharedAtomicFetchOr(&sharedData->system_fault_bitmap, latchedFaults);
    sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
    sharedData->control_enabled = 0;
    if (interruptedState == SYSTEM_STATE_CALIBRATION_RUNNING) {
        setCalibrationStatus(CALIB_STATUS_FAILED, true);
        sharedAtomicStore(&sharedData->calibration_fault_latched, (uint8_t)1);
    }
    if (interruptedState == SYSTEM_STATE_ROPE_INIT_RUNNING) {
        sharedData->rope_init_status = ROPE_INIT_STATUS_FAILED;
        sharedData->rope_init_reason = ROPE_INIT_REASON_SYSTEM_FAULT;
        sharedData->rope_init_timestamp = millis();
    }
    requestServoEmergencyTorqueOff(sharedData);
    setSystemState(sharedData, SYSTEM_STATE_UNCONTROLLABLE);
}

static void resetControlProtectionFaults(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return;
    }
    sharedAtomicStore(&sharedData->joint16_dual_feedback_fault, (uint8_t)0);
    {
        SharedSafetyGuard guard;
        sharedAtomicStore(&sharedData->overload_fault_bitmap, (uint32_t)0);
        for (uint8_t i = 0; i < SERVO_TOTAL_NUM; i++) {
            sharedData->overload_fault_load[i] = 0;
        }
    }
    sharedAtomicAddFetch(&sharedData->overload_fault_reset_token, (uint32_t)1);

}

static uint32_t readyServoRequiredMask(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return 0;
    }
    uint32_t requiredMask = sharedAtomicLoad(&sharedData->active_motor_mask);
    if (sharedData->system_state == SYSTEM_STATE_CALIBRATION_RUNNING) {
        requiredMask |= sharedAtomicLoad(&sharedData->calibration_aux_motor_mask);
    }
    return requiredMask & validMotorChannelMask();
}

static bool readyRequiresCan(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return false;
    }
    if (sharedData->control_mode == CONTROL_MODE_DIRECT_MOTOR) {
        return false;
    }
    return sharedAtomicLoad(&sharedData->require_can_feedback) != 0;
}

static bool requiredServosOnline(TaskSharedData_t* sharedData, const ServoAngleData_t& servoSnapshot)
{
    const uint32_t requiredMask = readyServoRequiredMask(sharedData);
    if (requiredMask == 0) {
        static uint32_t s_lastZeroMaskLogMs = 0;
        const uint32_t nowMs = millis();
        if ((uint32_t)(nowMs - s_lastZeroMaskLogMs) > 1000) {
            FW_LOG_INFO_PRINTLN("[STATE] servo ready rejected: active motor mask is zero");
            s_lastZeroMaskLogMs = nowMs;
        }
        return false;
    }

    uint32_t missingMask = 0;
    for (uint8_t i = 0; i < SERVO_TOTAL_NUM; i++) {
        if (((requiredMask >> i) & 0x1UL) != 0 && servoSnapshot.onlineStatus[i] == 0) {
            missingMask |= (1UL << i);
        }
    }
    if (missingMask == 0) {
        return true;
    }

    static uint32_t s_lastMissingServoLogMs = 0;
    static uint32_t s_lastMissingServoMask = 0;
    const uint32_t nowMs = millis();
    if (missingMask != s_lastMissingServoMask ||
        (uint32_t)(nowMs - s_lastMissingServoLogMs) > 1000) {
        FW_LOG_INFO_PRINTF("[STATE] servo offline activeMask=0x%08lX missing=0x%08lX\r\n",
                      (unsigned long)requiredMask,
                      (unsigned long)missingMask);
        s_lastMissingServoMask = missingMask;
        s_lastMissingServoLogMs = nowMs;
    }
    return false;
}

static bool directMotorMaskReady(TaskSharedData_t* sharedData,
                                 uint32_t requiredMask)
{
    if (!sharedData) {
        return false;
    }
    requiredMask &= validMotorChannelMask();
    if (requiredMask == 0) {
        return false;
    }

    ServoAngleData_t servoSnapshot;
    const uint32_t nowMs = millis();
    if (!sharedData->servoFeedbackQueue ||
        xQueuePeek(sharedData->servoFeedbackQueue, &servoSnapshot, 0) != pdTRUE ||
        (uint32_t)(nowMs - servoSnapshot.timestamp) > kStateServoFeedbackTimeoutMs) {
        return false;
    }
    for (uint8_t motor = 0; motor < SERVO_TOTAL_NUM; motor++) {
        const uint32_t bit = (1UL << motor);
        if ((requiredMask & bit) != 0 && servoSnapshot.onlineStatus[motor] == 0) {
            return false;
        }
    }
    if ((sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap) & requiredMask) != 0) {
        return false;
    }

    // Servo/CAN/multi-turn readiness is evaluated against the pending scope
    // above. Keep only faults that are independent of the previous scope.
    const uint32_t scopeDependentFaults =
        SYSTEM_FAULT_CAN_OFFLINE |
        SYSTEM_FAULT_SERVO_OFFLINE |
        SYSTEM_FAULT_MULTITURN_UNKNOWN;
    const uint32_t blockingFaults =
        (sharedAtomicLoad(&sharedData->system_fault_bitmap) |
         sharedAtomicLoad(&sharedData->system_fault_latched_bitmap)) &
        ~scopeDependentFaults;
    return blockingFaults == 0;
}

static bool isControllableEntryReady(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return false;
    }

    // IDLE means direct-motor hardware is usable. Manual/Auto/CAN belong to
    // the feedback READY gate and must not block recovery or direct control.
    return directMotorMaskReady(sharedData, readyServoRequiredMask(sharedData));
}

// ============================================================================
// activeJointsInControlWorkspace - 关节工作空间准入校验
// ============================================================================
// 该函数是判断系统能否�?IDLE 自动晋升�?READY 状态的关键�?// 1) 遍历当前启用�?active_joint_mask（支持动态任意数量的关节）�?// 2) 检查这些被选中的关节是否都已经成功完成过标定（g_jointCalibResult[i].success == true）�?// 3) 读取磁编码器当前的角度值，校验其是否落在宽容度范围�?(-0.8° �?max+0.8°)�?// 只有当所有选中关节都处于“安全可控范围内”时，系统才允许进入 READY�?// ============================================================================
static bool activeJointsInControlWorkspace(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return false;
    }
    if (!manualReferenceGateReady(sharedData)) {
        logManualReferenceReject(sharedData, "READY");
        return false;
    }
    if (!readyRequiresCan(sharedData)) {
        logFeedbackReadyReject(sharedData,
                               1,
                               sharedAtomicLoad(&sharedData->active_motor_mask),
                               sharedAtomicLoad(&sharedData->active_joint_mask),
                               sharedData->control_mode,
                               sharedAtomicLoad(&sharedData->require_can_feedback));
        return false;
    }
    if (!activeMotorsMultiTurnKnown(sharedData)) {
        logFeedbackReadyReject(sharedData,
                               2,
                               readyServoRequiredMask(sharedData),
                               sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap),
                               0,
                               0);
        return false;
    }
    if (!activeMotorsAbsLimitsCalibrated(sharedData)) {
        return false;
    }

    const uint32_t activeMask =
        sharedAtomicLoad(&sharedData->active_joint_mask) &
        ((1UL << ENCODER_TOTAL_NUM) - 1UL);
    if (activeMask == 0) {
        logFeedbackReadyReject(sharedData,
                               3,
                               sharedAtomicLoad(&sharedData->active_motor_mask),
                               0,
                               0,
                               0);
        return false;
    }

    RemoteSensorData_t sensorData;
    const uint32_t encoderOnlineMask =
        sharedAtomicLoad(&sharedData->encoder_online_bitmap);
    const uint32_t encoderFaultMask =
        sharedAtomicLoad(&sharedData->encoder_fault_bitmap);
    const uint8_t canLinkOnline =
        sharedAtomicLoad(&sharedData->can_link_online);
    const uint32_t nowMs = millis();
    const bool sensorFresh =
        sharedData->canRxQueue &&
        xQueuePeek(sharedData->canRxQueue, &sensorData, 0) == pdTRUE &&
        sensorData.isValid &&
        (uint32_t)(nowMs - sensorData.timestamp) <= kStateCanOfflineTimeoutMs;
    if (!sensorFresh || canLinkOnline == 0 ||
        (activeMask & ~encoderOnlineMask) != 0 ||
        (activeMask & encoderFaultMask) != 0) {
        logFeedbackReadyReject(sharedData,
                               9,
                               activeMask & ~encoderOnlineMask,
                               activeMask & encoderFaultMask,
                               sensorFresh ? 1 : 0,
                               canLinkOnline);
        return false;
    }
    const uint32_t feedbackFaults = collectSystemFaults(sharedData);
    if (feedbackFaults != 0) {
        logFeedbackReadyReject(sharedData, 10, feedbackFaults, 0, 0, 0);
        return false;
    }

    MappedAngleData_t mappedData;
    if (!sharedData->mappedAngleQueue) {
        logFeedbackReadyReject(sharedData, 4, 0, 0, 0, 0);
        return false;
    }
    if (xQueuePeek(sharedData->mappedAngleQueue, &mappedData, 0) != pdTRUE) {
        logFeedbackReadyReject(sharedData, 4, 1, 0, 0, 0);
        return false;
    }
    const uint32_t mappedAgeMs = nowMs - mappedData.timestamp;
    if (!mappedData.isValid) {
        logFeedbackReadyReject(sharedData, 4, 2, 0, (int32_t)mappedAgeMs, 0);
        return false;
    }
    if (mappedAgeMs > kStateCanOfflineTimeoutMs) {
        logFeedbackReadyReject(sharedData, 4, 3, 0, (int32_t)mappedAgeMs, 0);
        return false;
    }

    const uint8_t currentState = sharedData->system_state;
    const bool preserveReady =
        currentState == SYSTEM_STATE_READY ||
        currentState == SYSTEM_STATE_RUN_FEEDBACK ||
        (currentState == SYSTEM_STATE_PAUSED &&
         sharedData->paused_resume_state == SYSTEM_STATE_RUN_FEEDBACK);
    const int32_t outsideToleranceCounts = preserveReady
        ? kReadyExitOutsideToleranceCounts
        : kReadyEnterOutsideToleranceCounts;

    for (uint8_t calibJoint = 0; calibJoint < ENCODER_TOTAL_NUM; calibJoint++) {
        if ((activeMask & (1UL << calibJoint)) == 0) {
            continue;
        }
        const JointCalibrationResult& calib = g_jointCalibResult[calibJoint];
        if (!calib.success) {
            logFeedbackReadyReject(sharedData, 5, calibJoint, activeMask, 0, 0);
            return false;
        }
        const uint32_t manualEntryCrc =
            manualEncoderReferenceEntryCrc(
                calibJoint, g_manualEncoderReference[calibJoint]);
        if (calib.mappingVersion != kManualScaledUrdfFeedbackMappingVersion ||
            calib.urdfConfigCrc != kManualScaledUrdfConfigCrc ||
            calib.manualReferenceCrc != manualEntryCrc) {
            logFeedbackReadyReject(sharedData,
                                   8,
                                   calibJoint,
                                   calib.mappingVersion,
                                   (int32_t)calib.urdfConfigCrc,
                                   (int32_t)manualEntryCrc);
            return false;
        }
        if (mappedData.validFlags[calibJoint] == 0) {
            uint32_t validMask = 0;
            for (uint8_t i = 0; i < ENCODER_TOTAL_NUM; i++) {
                if (mappedData.validFlags[i] != 0) {
                    validMask |= (1UL << i);
                }
            }
            logFeedbackReadyReject(sharedData, 6, calibJoint, validMask, 0, 0);
            return false;
        }
        const int32_t outsideCounts = mappedData.outsideCounts[calibJoint];
        if (outsideCounts < -outsideToleranceCounts) {
            logFeedbackReadyReject(sharedData,
                                   7,
                                   calibJoint,
                                   preserveReady ? 1U : 0U,
                                   outsideCounts,
                                   outsideToleranceCounts);
            return false;
        }
    }

    return true;
}

static uint8_t idleOrReadyState(TaskSharedData_t* sharedData)
{
    if (!isControllableEntryReady(sharedData)) {
        return SYSTEM_STATE_UNCONTROLLABLE;
    }
    return activeJointsInControlWorkspace(sharedData) ? SYSTEM_STATE_READY : SYSTEM_STATE_IDLE;
}

static void updateReadyStateIfIdle(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return;
    }
    if (sharedData->control_enabled != 0 || sharedData->servo_target_owner != SERVO_TARGET_OWNER_NONE) {
        return;
    }
    const uint8_t state = sharedData->system_state;
    if (state == SYSTEM_STATE_IDLE ||
        state == SYSTEM_STATE_READY) {
        setSystemState(sharedData, idleOrReadyState(sharedData));
    }
}

static bool feedbackCompletionMatchesCurrentCommand(TaskSharedData_t* sharedData,
                                                    const SystemEvent_t& event)
{
    return sharedData &&
           (event.token == 0 || event.token == sharedData->joint_command_token);
}

static void releaseFeedbackRun(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return;
    }
    // Leave the servo's last accepted position in place for a stable hold.
    // Only the feedback task's ownership is released so another joint scope
    // can be selected immediately.
    sharedData->control_enabled = 0;
    sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
    setSystemState(sharedData, idleOrReadyState(sharedData));
    requestMultiTurnNvsSave(sharedData, MULTITURN_NVS_SAVE_REASON_RUN_DONE);
}

static void handleSystemEvent(TaskSharedData_t* sharedData, const SystemEvent_t& event)
{
    if (!sharedData) {
        return;
    }

    switch (event.event)
    {
        case SYSTEM_EVENT_START:
            if (!canAcceptStart(sharedData->system_state)) {
                FW_LOG_INFO_PRINTF("[STATE] reject START in state=%u\r\n",
                              (unsigned)sharedData->system_state);
                break;
            }

            if (sharedData->system_state == SYSTEM_STATE_PAUSED) {
                const uint8_t resumeState = sharedData->paused_resume_state;
                const bool resumeReady =
                    (resumeState == SYSTEM_STATE_RUN_DIRECT)
                        ? isControllableEntryReady(sharedData)
                        : ((resumeState == SYSTEM_STATE_RUN_FEEDBACK) &&
                           activeJointsInControlWorkspace(sharedData));
                if (!resumeReady) {
                    sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
                    sharedData->control_enabled = 0;
                    setSystemState(sharedData, idleOrReadyState(sharedData));
                    break;
                }
                resetServoTargetQueue(sharedData);
                sharedData->servo_target_owner = SERVO_TARGET_OWNER_CONTROL;
                sharedData->control_enabled = 1;
                if (resumeState == SYSTEM_STATE_RUN_DIRECT) {
                    setCommandModeDirect(sharedData);
                } else if (resumeState == SYSTEM_STATE_RUN_FEEDBACK) {
                    setCommandModeJoint(sharedData);
                }
                setSystemState(sharedData,
                               (resumeState == SYSTEM_STATE_RUN_DIRECT ||
                                resumeState == SYSTEM_STATE_RUN_FEEDBACK)
                                   ? resumeState
                                   : idleOrReadyState(sharedData));
                break;
            }

            resetServoTargetQueue(sharedData);
            sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
            sharedData->control_enabled = 0;
            setSystemState(sharedData, idleOrReadyState(sharedData));
            break;

        case SYSTEM_EVENT_STOP:
            // Discard motion events queued before STOP. New commands must be
            // issued after the state has visibly returned to UNCONTROLLABLE.
            if (shouldSaveMultiTurnOnInterrupt(sharedData->system_state)) {
                requestMultiTurnNvsSave(sharedData, MULTITURN_NVS_SAVE_REASON_RUN_DONE);
            }
            if (sharedData->stateEventQueue) {
                xQueueReset(sharedData->stateEventQueue);
            }
            clearPendingDirectCommand(sharedData, DIRECT_REJECT_STATE);
            resetServoTargetQueue(sharedData);
            sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
            sharedData->control_enabled = 0;
            if (sharedData->system_state == SYSTEM_STATE_ROPE_INIT_RUNNING) {
                sharedData->rope_init_status = ROPE_INIT_STATUS_FAILED;
                sharedData->rope_init_reason = ROPE_INIT_REASON_CANCELLED;
                sharedData->rope_init_timestamp = millis();
            }
            if (sharedData->system_state == SYSTEM_STATE_CALIBRATION_RUNNING) {
                setCalibrationStatus(CALIB_STATUS_IDLE, false);
            }
            setSystemState(sharedData, SYSTEM_STATE_UNCONTROLLABLE);
            break;

        case SYSTEM_EVENT_RESET:
            if (shouldSaveMultiTurnOnInterrupt(sharedData->system_state)) {
                requestMultiTurnNvsSave(sharedData, MULTITURN_NVS_SAVE_REASON_RUN_DONE);
            }
            resetCommandState(sharedData);
            sharedData->control_enabled = 0;
            sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
            resetServoTargetQueue(sharedData);
            requestServoEmergencyTorqueOff(sharedData);
            resetControlProtectionFaults(sharedData);
            sharedAtomicStore(&sharedData->calibration_fault_latched, (uint8_t)0);
            sharedAtomicStore(&sharedData->system_fault_latched_bitmap, (uint32_t)0);
            sharedAtomicStore(&sharedData->system_fault_bitmap, (uint32_t)0);
            setCalibrationStatus(CALIB_STATUS_IDLE, false);
            setSystemState(sharedData, SYSTEM_STATE_UNCONTROLLABLE);
            break;

        case SYSTEM_EVENT_RUN_DIRECT:
            if (!pendingDirectCommandMatches(sharedData, event.token)) {
                FW_LOG_INFO_PRINTF("[STATE] ignore stale RUN_DIRECT generation=%lu pending=%lu valid=%u\r\n",
                                   (unsigned long)event.token,
                                   (unsigned long)sharedData->pending_direct_generation,
                                   (unsigned)sharedData->pending_direct_valid);
                break;
            }
            if (!canAcceptRunDirect(sharedData->system_state)) {
                FW_LOG_INFO_PRINTF("[STATE] reject RUN_DIRECT in state=%u\r\n",
                              (unsigned)sharedData->system_state);
                rejectPendingDirectCommand(sharedData,
                                           event.token,
                                           DIRECT_REJECT_STATE);
                break;
            }
            {
                const uint32_t pendingMotorMask =
                    pendingDirectMotorMask(sharedData, event.token);
                if ((sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap) &
                     pendingMotorMask) != 0) {
                    FW_LOG_INFO_PRINTF("[STATE] reject RUN_DIRECT generation=%lu: multi-turn unknown motor=0x%08lX unknown=0x%08lX\r\n",
                                       (unsigned long)event.token,
                                       (unsigned long)pendingMotorMask,
                                       (unsigned long)sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap));
                    rejectPendingDirectCommand(sharedData,
                                               event.token,
                                               DIRECT_REJECT_MULTITURN);
                    break;
                }
                if (!directMotorMaskReady(sharedData, pendingMotorMask)) {
                    FW_LOG_INFO_PRINTF("[STATE] reject RUN_DIRECT generation=%lu: pending hardware not ready motor=0x%08lX\r\n",
                                       (unsigned long)event.token,
                                       (unsigned long)pendingMotorMask);
                    rejectPendingDirectCommand(sharedData,
                                               event.token,
                                               DIRECT_REJECT_HARDWARE);
                    break;
                }
            }
            if (!promotePendingDirectCommand(sharedData, event.token)) {
                rejectPendingDirectCommand(sharedData,
                                           event.token,
                                           DIRECT_REJECT_STALE);
                break;
            }
            sharedAtomicFetchAnd(&sharedData->system_fault_bitmap,
                                 (uint32_t)~(SYSTEM_FAULT_CAN_OFFLINE |
                                             SYSTEM_FAULT_SERVO_OFFLINE |
                                             SYSTEM_FAULT_MULTITURN_UNKNOWN));
            sharedAtomicFetchAnd(&sharedData->system_fault_latched_bitmap,
                                 (uint32_t)~(SYSTEM_FAULT_CAN_OFFLINE |
                                             SYSTEM_FAULT_SERVO_OFFLINE |
                                             SYSTEM_FAULT_MULTITURN_UNKNOWN));
            if (sharedData->system_state != SYSTEM_STATE_RUN_DIRECT) {
                resetServoTargetQueue(sharedData);
            }
            sharedData->servo_target_owner = SERVO_TARGET_OWNER_CONTROL;
            sharedData->control_enabled = 1;
            setSystemState(sharedData, SYSTEM_STATE_RUN_DIRECT);
            break;

        case SYSTEM_EVENT_RUN_FEEDBACK:
            if (!manualReferenceGateReady(sharedData)) {
                logManualReferenceReject(sharedData, "RUN_FEEDBACK");
                setFeedbackAdmissionResult(sharedData, event.token,
                                           FEEDBACK_ADMISSION_REJECTED,
                                           FEEDBACK_REJECT_MANUAL_REFERENCE);
                break;
            }
            // Every desktop joint target is paired with RUN_FEEDBACK. A status
            // packet can be one feedback cycle behind P4, so a repeated request
            // while this run already owns the output must be harmless. Do not
            // reset the queue here: ControlTask will consume the newer joint
            // command token on its next cycle.
            if (sharedData->system_state == SYSTEM_STATE_RUN_FEEDBACK) {
                setFeedbackAdmissionResult(sharedData, event.token,
                                           FEEDBACK_ADMISSION_ACCEPTED,
                                           FEEDBACK_REJECT_NONE);
                break;
            }
            if (!canAcceptRunFeedback(sharedData->system_state)) {
                FW_LOG_INFO_PRINTF("[STATE] reject RUN_FEEDBACK in state=%u\r\n",
                              (unsigned)sharedData->system_state);
                setFeedbackAdmissionResult(sharedData, event.token,
                                           FEEDBACK_ADMISSION_REJECTED,
                                           FEEDBACK_REJECT_STATE);
                break;
            }
            if (!activeJointsInControlWorkspace(sharedData)) {
                FW_LOG_INFO_PRINTLN("[STATE] reject RUN_FEEDBACK: feedback gate changed");
                setFeedbackAdmissionResult(sharedData, event.token,
                                           FEEDBACK_ADMISSION_REJECTED,
                                           FEEDBACK_REJECT_WORKSPACE);
                break;
            }
            if (!activeMotorsMultiTurnKnown(sharedData)) {
                logMultiTurnUnknownReject(sharedData, "RUN_FEEDBACK");
                setFeedbackAdmissionResult(sharedData, event.token,
                                           FEEDBACK_ADMISSION_REJECTED,
                                           FEEDBACK_REJECT_MULTITURN);
                break;
            }
            if (!activeMotorsAbsLimitsCalibrated(sharedData)) {
                FW_LOG_INFO_PRINTLN("[STATE] reject RUN_FEEDBACK: active joints are not calibrated");
                setFeedbackAdmissionResult(sharedData, event.token,
                                           FEEDBACK_ADMISSION_REJECTED,
                                           FEEDBACK_REJECT_ABS_LIMIT);
                break;
            }
            if (!setCommandModeJoint(sharedData)) {
                FW_LOG_INFO_PRINTLN("[STATE] reject RUN_FEEDBACK: command mode mutex busy");
                setFeedbackAdmissionResult(sharedData, event.token,
                                           FEEDBACK_ADMISSION_REJECTED,
                                           FEEDBACK_REJECT_MODE_SWITCH);
                break;
            }
            resetServoTargetQueue(sharedData);
            sharedData->servo_target_owner = SERVO_TARGET_OWNER_CONTROL;
            sharedData->control_enabled = 1;
            setSystemState(sharedData, SYSTEM_STATE_RUN_FEEDBACK);
            setFeedbackAdmissionResult(sharedData, event.token,
                                       FEEDBACK_ADMISSION_ACCEPTED,
                                       FEEDBACK_REJECT_NONE);
            break;

        case SYSTEM_EVENT_RUN_DIRECT_DONE:
            if (sharedData->system_state != SYSTEM_STATE_RUN_DIRECT) {
                FW_LOG_INFO_PRINTF("[STATE] ignore RUN_DIRECT_DONE in state=%u\r\n",
                              (unsigned)sharedData->system_state);
                break;
            }
            if (event.token != sharedData->applied_direct_generation) {
                FW_LOG_INFO_PRINTF("[STATE] ignore stale RUN_DIRECT_DONE generation=%lu applied=%lu\r\n",
                                   (unsigned long)event.token,
                                   (unsigned long)sharedData->applied_direct_generation);
                break;
            }
            sharedData->control_enabled = 0;
            sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
            setSystemState(sharedData, idleOrReadyState(sharedData));
            requestMultiTurnNvsSave(sharedData, MULTITURN_NVS_SAVE_REASON_RUN_DONE);
            break;

        case SYSTEM_EVENT_RUN_FEEDBACK_DONE:
            if (sharedData->system_state != SYSTEM_STATE_RUN_FEEDBACK) {
                FW_LOG_INFO_PRINTF("[STATE] ignore RUN_FEEDBACK_DONE in state=%u\r\n",
                              (unsigned)sharedData->system_state);
                break;
            }
            if (!feedbackCompletionMatchesCurrentCommand(sharedData, event)) {
                FW_LOG_INFO_PRINTF("[STATE] ignore stale RUN_FEEDBACK_DONE token=%lu current=%lu\r\n",
                              (unsigned long)event.token,
                              (unsigned long)sharedData->joint_command_token);
                break;
            }
            releaseFeedbackRun(sharedData);
            break;

        case SYSTEM_EVENT_RUN_FEEDBACK_TIMEOUT:
            if (sharedData->system_state != SYSTEM_STATE_RUN_FEEDBACK) {
                FW_LOG_INFO_PRINTF("[STATE] ignore RUN_FEEDBACK_TIMEOUT in state=%u\r\n",
                              (unsigned)sharedData->system_state);
                break;
            }
            if (!feedbackCompletionMatchesCurrentCommand(sharedData, event)) {
                FW_LOG_INFO_PRINTF("[STATE] ignore stale RUN_FEEDBACK_TIMEOUT token=%lu current=%lu\r\n",
                              (unsigned long)event.token,
                              (unsigned long)sharedData->joint_command_token);
                break;
            }
            FW_LOG_INFO_PRINTF("[STATE] RUN_FEEDBACK timeout token=%lu; releasing control ownership\r\n",
                          (unsigned long)event.token);
            releaseFeedbackRun(sharedData);
            break;

        case SYSTEM_EVENT_PAUSE:
            if (sharedData->system_state != SYSTEM_STATE_RUN_DIRECT &&
                sharedData->system_state != SYSTEM_STATE_RUN_FEEDBACK) {
                FW_LOG_INFO_PRINTF("[STATE] reject PAUSE in state=%u\r\n",
                              (unsigned)sharedData->system_state);
                break;
            }
            sharedData->paused_resume_state = sharedData->system_state;
            sharedData->control_enabled = 0;
            sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
            requestMultiTurnNvsSave(sharedData, MULTITURN_NVS_SAVE_REASON_RUN_DONE);
            requestServoEmergencyTorqueOff(sharedData);
            setSystemState(sharedData, SYSTEM_STATE_PAUSED);
            break;

        case SYSTEM_EVENT_CALIBRATE:
        {
            if (!manualReferenceGateReady(sharedData)) {
                logManualReferenceReject(sharedData, "CALIBRATE");
                setCalibrationStatus(CALIB_STATUS_FAILED, true);
                break;
            }
            if (!canAcceptCalibrate(sharedData->system_state)) {
                FW_LOG_INFO_PRINTF("[STATE] reject CALIBRATE in state=%u\r\n",
                              (unsigned)sharedData->system_state);
                break;
            }
            resetServoTargetQueue(sharedData);
            sharedData->servo_target_owner = SERVO_TARGET_OWNER_CALIBRATION;
            sharedData->control_enabled = 0;
            sharedAtomicStore(&sharedData->calibration_fault_latched, (uint8_t)0);
            setCommandModeJoint(sharedData);
            setCalibrationStatus(CALIB_STATUS_RUNNING, true);
            setSystemState(sharedData, SYSTEM_STATE_CALIBRATION_RUNNING);
            if (!startAutoCalibrationTask(sharedData)) {
                CalibrationDiagData_t diag;
                memset(&diag, 0, sizeof(diag));
                diag.valid = 1;
                diag.reason = CALIB_DIAG_REASON_TASK_START_FAILED;
                diag.phase = CALIB_DIAG_PHASE_SEQUENCE;
                diag.joint = 0xFF;
                diag.motor = 0xFF;
                diag.timestamp = millis();
                {
                    SharedSafetyGuard guard;
                    diag.sequence = sharedData->calibration_diag.sequence + 1;
                    sharedData->calibration_diag = diag;
                }
                setCalibrationStatus(CALIB_STATUS_FAILED, true);
                sharedAtomicStore(&sharedData->calibration_fault_latched, (uint8_t)1);
                requestServoEmergencyTorqueOff(sharedData);
                setSystemState(sharedData, SYSTEM_STATE_UNCONTROLLABLE);
            }
            break;
        }

        case SYSTEM_EVENT_ROPE_INIT:
        {
            sharedData->rope_init_sequence++;
            sharedData->rope_init_joint_mask =
                sharedAtomicLoad(&sharedData->active_joint_mask) &
                kManualEncoderReferenceFullMask;
            sharedData->rope_init_timestamp = millis();
            const uint8_t preflightReason =
                ropeInitActiveScopeRejectReason(sharedData);
            if (preflightReason != ROPE_INIT_REASON_NONE) {
                sharedData->rope_init_status = ROPE_INIT_STATUS_FAILED;
                sharedData->rope_init_reason = preflightReason;
                sharedData->rope_init_timestamp = millis();
                if (preflightReason == ROPE_INIT_REASON_MANUAL_REQUIRED) {
                    logManualReferenceReject(sharedData, "ROPE_INIT");
                } else {
                    FW_LOG_INFO_PRINTF("[STATE] reject ROPE_INIT preflight reason=%u\r\n",
                                       (unsigned)preflightReason);
                }
                break;
            }
            if (sharedData->system_state != SYSTEM_STATE_IDLE &&
                sharedData->system_state != SYSTEM_STATE_READY) {
                FW_LOG_INFO_PRINTF("[STATE] reject ROPE_INIT in state=%u\r\n",
                                   (unsigned)sharedData->system_state);
                sharedData->rope_init_status = ROPE_INIT_STATUS_FAILED;
                sharedData->rope_init_reason = ROPE_INIT_REASON_CANCELLED;
                sharedData->rope_init_timestamp = millis();
                break;
            }
            if (!activeMotorsMultiTurnKnown(sharedData)) {
                logMultiTurnUnknownReject(sharedData, "ROPE_INIT");
                sharedData->rope_init_status = ROPE_INIT_STATUS_FAILED;
                sharedData->rope_init_reason = ROPE_INIT_REASON_MULTITURN_UNKNOWN;
                sharedData->rope_init_timestamp = millis();
                break;
            }
            resetServoTargetQueue(sharedData);
            sharedData->servo_target_owner = SERVO_TARGET_OWNER_CALIBRATION;
            sharedData->control_enabled = 0;
            sharedData->rope_init_status = ROPE_INIT_STATUS_RUNNING;
            sharedData->rope_init_reason = ROPE_INIT_REASON_NONE;
            sharedData->rope_init_timestamp = millis();
            setCommandModeDirect(sharedData);
            setSystemState(sharedData, SYSTEM_STATE_ROPE_INIT_RUNNING);
            if (!startRopeInitTask(sharedData)) {
                sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
                sharedData->rope_init_status = ROPE_INIT_STATUS_FAILED;
                sharedData->rope_init_reason = ROPE_INIT_REASON_TASK_START_FAILED;
                sharedData->rope_init_timestamp = millis();
                setSystemState(sharedData, idleOrReadyState(sharedData));
            }
            break;
        }

        case SYSTEM_EVENT_ROPE_INIT_DONE:
            if (sharedData->system_state != SYSTEM_STATE_ROPE_INIT_RUNNING) {
                break;
            }
            resetServoTargetQueue(sharedData);
            sharedData->control_enabled = 0;
            sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
            setCommandModeJoint(sharedData);
            sharedData->rope_init_status = ROPE_INIT_STATUS_SUCCESS;
            sharedData->rope_init_reason = ROPE_INIT_REASON_NONE;
            sharedData->rope_init_timestamp = millis();
            setSystemState(sharedData, idleOrReadyState(sharedData));
            requestMultiTurnNvsSave(sharedData,
                                    MULTITURN_NVS_SAVE_REASON_RUN_DONE);
            break;

        case SYSTEM_EVENT_ROPE_INIT_FAILED:
            if (sharedData->system_state != SYSTEM_STATE_ROPE_INIT_RUNNING) {
                break;
            }
            resetServoTargetQueue(sharedData);
            sharedData->control_enabled = 0;
            sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
            setCommandModeJoint(sharedData);
            if (sharedData->rope_init_status != ROPE_INIT_STATUS_FAILED) {
                sharedData->rope_init_status = ROPE_INIT_STATUS_FAILED;
                sharedData->rope_init_reason = ROPE_INIT_REASON_CANCELLED;
                sharedData->rope_init_timestamp = millis();
            }
            requestServoEmergencyTorqueOff(sharedData);
            setSystemState(sharedData, idleOrReadyState(sharedData));
            requestMultiTurnNvsSave(sharedData,
                                    MULTITURN_NVS_SAVE_REASON_RUN_DONE);
            break;

        case SYSTEM_EVENT_CALIBRATION_DONE:
            if (sharedData->system_state != SYSTEM_STATE_CALIBRATION_RUNNING) {
                FW_LOG_INFO_PRINTF("[STATE] ignore CALIB_DONE in state=%u\r\n",
                              (unsigned)sharedData->system_state);
                break;
            }
            sharedData->control_enabled = 0;
            sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
            sharedAtomicStore(&sharedData->calibration_fault_latched, (uint8_t)0);
            sharedAtomicFetchAnd(&sharedData->multi_turn_unknown_bitmap,
                                 ~readyServoRequiredMask(sharedData));
            setCalibrationStatus(
                sharedAtomicLoad(&sharedData->calibration_warning_bitmap) != 0
                    ? CALIB_STATUS_WARN
                    : CALIB_STATUS_SUCCESS,
                true);
            setSystemState(sharedData, idleOrReadyState(sharedData));
            requestMultiTurnNvsSave(sharedData, MULTITURN_NVS_SAVE_REASON_CALIB_DONE);
            break;

        case SYSTEM_EVENT_REEVALUATE_READY:
            if (sharedData->control_enabled == 0 &&
                sharedData->servo_target_owner == SERVO_TARGET_OWNER_NONE &&
                (sharedData->system_state == SYSTEM_STATE_UNCONTROLLABLE ||
                 sharedData->system_state == SYSTEM_STATE_IDLE ||
                 sharedData->system_state == SYSTEM_STATE_READY)) {
                const uint8_t previousState = sharedData->system_state;
                const uint8_t nextState = idleOrReadyState(sharedData);
                setSystemState(sharedData, nextState);
                FW_LOG_INFO_PRINTF("[STATE] READY reevaluate previous=%u next=%u motor=0x%08lX joint=0x%08lX\r\n",
                              (unsigned)previousState,
                              (unsigned)nextState,
                              (unsigned long)sharedAtomicLoad(&sharedData->active_motor_mask),
                              (unsigned long)sharedAtomicLoad(&sharedData->active_joint_mask));
            }
            break;

        case SYSTEM_EVENT_CALIBRATION_FAILED:
            if (sharedData->system_state != SYSTEM_STATE_CALIBRATION_RUNNING) {
                FW_LOG_INFO_PRINTF("[STATE] ignore CALIB_FAILED in state=%u\r\n",
                              (unsigned)sharedData->system_state);
                break;
            }
            sharedData->control_enabled = 0;
            sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
            sharedAtomicStore(&sharedData->calibration_fault_latched, (uint8_t)1);
            setCalibrationStatus(CALIB_STATUS_FAILED, true);
            requestServoEmergencyTorqueOff(sharedData);
            setSystemState(sharedData, SYSTEM_STATE_UNCONTROLLABLE);
            break;

        default:
            break;
    }
}

static uint32_t collectSystemFaults(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return 0;
    }

    const uint32_t nowMs = millis();
    uint32_t faults = 0;

    uint32_t activeJointMask = 0;
    uint32_t encoderOnlineMask = 0;
    uint32_t encoderFaultMask = 0;
    uint8_t canLinkOnline = 0;
    {
        SharedSafetyGuard guard;
        activeJointMask = sharedAtomicLoad(&sharedData->active_joint_mask) &
                          ((1UL << ENCODER_TOTAL_NUM) - 1UL);
        encoderOnlineMask = sharedAtomicLoad(&sharedData->encoder_online_bitmap);
        encoderFaultMask = sharedAtomicLoad(&sharedData->encoder_fault_bitmap);
        canLinkOnline = sharedAtomicLoad(&sharedData->can_link_online);
    }

    RemoteSensorData_t sensorData;
    const bool motionRequiresCan =
        readyRequiresCan(sharedData) ||
        sharedData->system_state == SYSTEM_STATE_ROPE_INIT_RUNNING;
    if (motionRequiresCan &&
        (xQueuePeek(sharedData->canRxQueue, &sensorData, 0) != pdTRUE ||
        !sensorData.isValid ||
        (uint32_t)(nowMs - sensorData.timestamp) > kStateCanOfflineTimeoutMs ||
        canLinkOnline == 0 ||
        ((activeJointMask & ~encoderOnlineMask) != 0) ||
        ((activeJointMask & encoderFaultMask) != 0))) {
        faults |= SYSTEM_FAULT_CAN_OFFLINE;
    }

    ServoAngleData_t servoSnapshot;
    const bool servoFeedbackFresh =
        sharedData->servoFeedbackQueue &&
        xQueuePeek(sharedData->servoFeedbackQueue, &servoSnapshot, 0) == pdTRUE &&
        (uint32_t)(nowMs - servoSnapshot.timestamp) <= kStateServoFeedbackTimeoutMs;
    if (servoFeedbackFresh) {
        if (!requiredServosOnline(sharedData, servoSnapshot)) {
            faults |= SYSTEM_FAULT_SERVO_OFFLINE;
        }
    } else {
        faults |= SYSTEM_FAULT_SERVO_OFFLINE;
    }

    // Temporary single-finger test mode: do not promote the joint16 dual-servo
    // consistency check to a system fault. Otherwise a setup without the J16
    // primary/secondary motors cannot enter READY. The control task still keeps
    // its local J16 hold behavior when that joint is actually used.
    if (sharedAtomicLoad(&sharedData->overload_fault_bitmap) != 0) {
        faults |= SYSTEM_FAULT_OVERLOAD;
    }

    if (sharedAtomicLoad(&sharedData->calibration_fault_latched) != 0) {
        faults |= SYSTEM_FAULT_CALIBRATION;
    }
    if ((sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap) &
         readyServoRequiredMask(sharedData)) != 0) {
        faults |= SYSTEM_FAULT_MULTITURN_UNKNOWN;
    }
    faults |= sharedAtomicLoad(&sharedData->system_fault_latched_bitmap);

    return faults;
}

static uint32_t blockingFaultsForState(uint8_t state, uint32_t faults)
{
    switch (state) {
        case SYSTEM_STATE_RUN_DIRECT:
            return faults & ~SYSTEM_FAULT_CAN_OFFLINE;
        case SYSTEM_STATE_RUN_FEEDBACK:
        case SYSTEM_STATE_ROPE_INIT_RUNNING:
            return faults;
        case SYSTEM_STATE_CALIBRATION_RUNNING:
            // Auto calibration intentionally reconstructs the multi-turn
            // reference, so unknown turns are not an abort condition here.
            return faults & ~SYSTEM_FAULT_MULTITURN_UNKNOWN;
        default:
            return 0;
    }
}

static bool stateOwnsMotionResources(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return false;
    }
    return sharedData->control_enabled != 0 ||
           sharedData->servo_target_owner != SERVO_TARGET_OWNER_NONE ||
           sharedData->system_state == SYSTEM_STATE_RUN_DIRECT ||
           sharedData->system_state == SYSTEM_STATE_RUN_FEEDBACK ||
           sharedData->system_state == SYSTEM_STATE_CALIBRATION_RUNNING ||
           sharedData->system_state == SYSTEM_STATE_ROPE_INIT_RUNNING;
}

// ============================================================================
// StateMachineTask - 系统顶层大状态机任务
// ============================================================================
// 运行频率：通常�?100Hz (10ms 周期)
// 职责�?// 1) 负责全局安全状态流转：UNCONTROLLABLE (不可�?异常) -> IDLE (空闲可用)
//    -> READY (各关节在有效工作空间内，准备就绪) -> RUN_FEEDBACK / RUN_DIRECT�?// 2) 处理外部发来的指令事件（�?SYSTEM_EVENT_START、SYSTEM_EVENT_CALIBRATE）�?// 3) 不断收集底层的错误状�?(collectSystemFaults)，一旦发现断联、过载等致命错误�?//    立即强制退�?UNCONTROLLABLE 状态并触发紧急急停 (Emergency Hold)�?// ============================================================================
void stateMachineTask(void* parameter)
{
    TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;
    if (!sharedData) {
        vTaskDelete(NULL);
        return;
    }

    setSystemState(sharedData, SYSTEM_STATE_UNCONTROLLABLE);

    uint32_t handledEmergencyStopToken = 0;

    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t periodTicks = pdMS_TO_TICKS(10);

    for (;;)
    {
        taskRuntimeMarkLoopStart(TASK_RUNTIME_STATE_MACHINE);
        const uint32_t emergencyStopToken =
            sharedAtomicLoad(&sharedData->emergency_stop_request_token);
        if (emergencyStopToken != handledEmergencyStopToken) {
            handledEmergencyStopToken = emergencyStopToken;
            SystemEvent_t stopEvent;
            stopEvent.event = SYSTEM_EVENT_STOP;
            stopEvent.timestamp = millis();
            stopEvent.token = emergencyStopToken;
            handleSystemEvent(sharedData, stopEvent);
        }

        SystemEvent_t event;
        while (xQueueReceive(sharedData->stateEventQueue, &event, 0) == pdTRUE) {
            handleSystemEvent(sharedData, event);
        }

        const uint32_t faults = collectSystemFaults(sharedData);
        sharedAtomicStore(&sharedData->system_fault_bitmap, faults);

        const uint32_t blockingFaults =
            blockingFaultsForState(sharedData->system_state, faults);
        if (stateOwnsMotionResources(sharedData) && blockingFaults != 0) {
            enterFaultHold(sharedData, blockingFaults);
        } else if (!stateOwnsMotionResources(sharedData)) {
            updateReadyStateIfIdle(sharedData);
        }

        taskRuntimeMarkLoopEnd(TASK_RUNTIME_STATE_MACHINE);
        const TickType_t nextWakeTime = lastWakeTime + periodTicks;
        const bool periodOverrun = (int32_t)(xTaskGetTickCount() - nextWakeTime) >= 0;
        taskRuntimeMarkPeriodResult(TASK_RUNTIME_STATE_MACHINE, periodOverrun);
        vTaskDelayUntil(&lastWakeTime, periodTicks);
        if (periodOverrun) vTaskDelay(1);
    }
}
