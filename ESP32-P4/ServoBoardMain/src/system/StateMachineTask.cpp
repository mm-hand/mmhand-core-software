#include "StateMachineTask.h"

#include <math.h>
#include <string.h>

#include "../calibration/AutoCalibrationTask.h"
#include "../calibration/CalibrationMath.h"
#include "../calibration/CalibrationTask.h"
#include "../hardware/HardwareMap.h"
#include "../shared/SharedSafetyState.h"
#include "FirmwareLog.h"
#include "TaskRuntimeMonitor.h"

extern volatile uint8_t g_calibrationStatus;
extern volatile uint8_t g_calibrationAckStatus;
extern volatile uint8_t g_calibrationAckRepeatRemaining;

static const uint32_t kStateCanOfflineTimeoutMs = 1000;
static const uint32_t kStateServoFeedbackTimeoutMs = 500;
static const float kWorkspaceToleranceDeg = 1.0f;
static const uint8_t kCalibrationAckRepeatCount = 20;
#ifndef BOOT_AUTO_CALIBRATION_ENABLE
#define BOOT_AUTO_CALIBRATION_ENABLE 0
#endif
static const uint32_t kBootAutoCalibrationDelayMs = 1500;

static uint32_t collectSystemFaults(TaskSharedData_t* sharedData);
static bool activeJointsInControlWorkspace(TaskSharedData_t* sharedData);
static uint32_t readyServoRequiredMask(TaskSharedData_t* sharedData);

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
    }
    sharedData->motor_command_token++;
    sharedData->motor_sweep_command_token++;
    sharedData->joint_command_token++;
    sharedData->motor_direct_command_generation++;
    sharedData->motor_direct_command_source = MOTOR_DIRECT_SOURCE_NONE;
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

static void setCommandModeJoint(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return;
    }

    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    sharedData->control_mode = CONTROL_MODE_JOINT;
    sharedData->motor_direct_command_source = MOTOR_DIRECT_SOURCE_NONE;
    if (lock) {
        xSemaphoreGive(lock);
    }
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

static void resetServoTargetQueue(TaskSharedData_t* sharedData)
{
    if (sharedData && sharedData->servoTargetQueue) {
        xQueueReset(sharedData->servoTargetQueue);
    }
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
           state == SYSTEM_STATE_READY;
}

static bool canAcceptRunFeedback(uint8_t state)
{
    return state == SYSTEM_STATE_READY;
}

static bool canAcceptCalibrate(uint8_t state)
{
    return state == SYSTEM_STATE_IDLE ||
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
            FW_LOG_INFO_PRINTF("[STATE] feedback ready rejected: joint outside workspace J%02lu angle=%ld min=%ld max=%ld\r\n",
                          (unsigned long)a,
                          (long)((int32_t)b),
                          (long)c,
                          (long)d);
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
    resetServoTargetQueue(sharedData);
    sharedAtomicFetchOr(&sharedData->system_fault_latched_bitmap, faultBits);
    const uint32_t latchedFaults =
        sharedAtomicLoad(&sharedData->system_fault_latched_bitmap);
    sharedAtomicFetchOr(&sharedData->system_fault_bitmap, latchedFaults);
    sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
    sharedData->control_enabled = 0;
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
    sharedAtomicAddFetch(&sharedData->reverse_release_fault_reset_token, (uint32_t)1);
}

static uint32_t readyServoRequiredMask(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return 0;
    }
    return sharedAtomicLoad(&sharedData->active_motor_mask) & validMotorChannelMask();
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

static bool isControllableEntryReady(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return false;
    }

    const uint32_t nowMs = millis();

    uint32_t activeJointMaskSnapshot = 0;
    uint32_t encoderOnlineMaskSnapshot = 0;
    uint32_t encoderFaultMaskSnapshot = 0;
    uint8_t canLinkOnlineSnapshot = 0;
    {
        SharedSafetyGuard guard;
        activeJointMaskSnapshot = sharedAtomicLoad(&sharedData->active_joint_mask);
        encoderOnlineMaskSnapshot = sharedAtomicLoad(&sharedData->encoder_online_bitmap);
        encoderFaultMaskSnapshot = sharedAtomicLoad(&sharedData->encoder_fault_bitmap);
        canLinkOnlineSnapshot = sharedAtomicLoad(&sharedData->can_link_online);
    }

    RemoteSensorData_t sensorData;
    bool canReady =
        sharedData->canRxQueue &&
        xQueuePeek(sharedData->canRxQueue, &sensorData, 0) == pdTRUE &&
        sensorData.isValid &&
        (uint32_t)(nowMs - sensorData.timestamp) <= kStateCanOfflineTimeoutMs &&
        canLinkOnlineSnapshot != 0;
    if (canReady && readyRequiresCan(sharedData)) {
        const uint32_t activeJointMask =
            activeJointMaskSnapshot &
            ((1UL << ENCODER_TOTAL_NUM) - 1UL);
        const uint32_t offlineMask =
            activeJointMask & ~encoderOnlineMaskSnapshot;
        const uint32_t faultMask =
            activeJointMask & encoderFaultMaskSnapshot;
        if (offlineMask != 0 || faultMask != 0) {
            canReady = false;
        }
    }
    if (!readyRequiresCan(sharedData)) {
        canReady = true;
    }

    ServoAngleData_t servoSnapshot;
    bool servoReady =
        sharedData->servoFeedbackQueue &&
        xQueuePeek(sharedData->servoFeedbackQueue, &servoSnapshot, 0) == pdTRUE &&
        (uint32_t)(nowMs - servoSnapshot.timestamp) <= kStateServoFeedbackTimeoutMs;
    if (servoReady) {
        servoReady = requiredServosOnline(sharedData, servoSnapshot);
    }

    // This is the gate from UNCONTROLLABLE to IDLE(controllable). READY is
    // reserved as a later pre-feedback state and is not entered automatically.
    return canReady &&
           servoReady &&
           sharedAtomicLoad(&sharedData->system_fault_bitmap) == 0 &&
           sharedAtomicLoad(&sharedData->system_fault_latched_bitmap) == 0;
}

// ============================================================================
// activeJointsInControlWorkspace - 鍏宠妭宸ヤ綔绌洪棿鍑嗗叆鏍￠獙
// ============================================================================
// 璇ュ嚱鏁版槸鍒ゆ柇绯荤粺鑳藉惁浠?IDLE 鑷姩鏅嬪崌鍒?READY 鐘舵€佺殑鍏抽敭锛?// 1) 閬嶅巻褰撳墠鍚敤鐨?active_joint_mask锛堟敮鎸佸姩鎬佷换鎰忔暟閲忕殑鍏宠妭锛夈€?// 2) 妫€鏌ヨ繖浜涜閫変腑鐨勫叧鑺傛槸鍚﹂兘宸茬粡鎴愬姛瀹屾垚杩囨爣瀹氾紙g_jointCalibResult[i].success == true锛夈€?// 3) 璇诲彇纾佺紪鐮佸櫒褰撳墠鐨勮搴﹀€硷紝鏍￠獙鍏舵槸鍚﹁惤鍦ㄥ瀹瑰害鑼冨洿鍐?(-0.8掳 鍒?max+0.8掳)銆?// 鍙湁褰撴墍鏈夐€変腑鍏宠妭閮藉浜庘€滃畨鍏ㄥ彲鎺ц寖鍥村唴鈥濇椂锛岀郴缁熸墠鍏佽杩涘叆 READY銆?// ============================================================================
static bool activeJointsInControlWorkspace(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
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

    MappedAngleData_t mappedData;
    const uint32_t nowMs = millis();
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

    for (uint8_t calibJoint = 0; calibJoint < ENCODER_TOTAL_NUM; calibJoint++) {
        if ((activeMask & (1UL << calibJoint)) == 0) {
            continue;
        }
        const JointCalibrationResult& calib = g_jointCalibResult[calibJoint];
        if (!calib.success) {
            logFeedbackReadyReject(sharedData, 5, calibJoint, activeMask, 0, 0);
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
        const int32_t angleCount = mappedData.angleValues[calibJoint];
        const float maxDeg = (isfinite(calib.angleMax) && calib.angleMax > 0.0f)
            ? calib.angleMax
            : kRange100ControlMaxDeg;
        const int32_t maxCount = calibrationDegToEncoderCount(maxDeg);
        const int32_t toleranceCount = calibrationDegToEncoderCount(kWorkspaceToleranceDeg);
        if (angleCount < -toleranceCount || angleCount > maxCount + toleranceCount) {
            logFeedbackReadyReject(sharedData,
                                   7,
                                   calibJoint,
                                   (uint32_t)angleCount,
                                   -toleranceCount,
                                   maxCount + toleranceCount);
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
                if (resumeState == SYSTEM_STATE_RUN_FEEDBACK && !activeJointsInControlWorkspace(sharedData)) {
                    sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
                    sharedData->control_enabled = 0;
                    setSystemState(sharedData, idleOrReadyState(sharedData));
                    break;
                }
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
            resetServoTargetQueue(sharedData);
            sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
            sharedData->control_enabled = 0;
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
            if (!canAcceptRunDirect(sharedData->system_state)) {
                FW_LOG_INFO_PRINTF("[STATE] reject RUN_DIRECT in state=%u\r\n",
                              (unsigned)sharedData->system_state);
                break;
            }
            if (!activeMotorsMultiTurnKnown(sharedData)) {
                logMultiTurnUnknownReject(sharedData, "RUN_DIRECT");
                break;
            }
            resetServoTargetQueue(sharedData);
            sharedData->servo_target_owner = SERVO_TARGET_OWNER_CONTROL;
            sharedData->control_enabled = 1;
            setCommandModeDirect(sharedData);
            setSystemState(sharedData, SYSTEM_STATE_RUN_DIRECT);
            break;

        case SYSTEM_EVENT_RUN_FEEDBACK:
            if (!canAcceptRunFeedback(sharedData->system_state)) {
                FW_LOG_INFO_PRINTF("[STATE] reject RUN_FEEDBACK in state=%u\r\n",
                              (unsigned)sharedData->system_state);
                break;
            }
            if (!activeMotorsMultiTurnKnown(sharedData)) {
                logMultiTurnUnknownReject(sharedData, "RUN_FEEDBACK");
                break;
            }
            if (!activeMotorsAbsLimitsCalibrated(sharedData)) {
                FW_LOG_INFO_PRINTLN("[STATE] reject RUN_FEEDBACK: active joints are not calibrated");
                break;
            }
            resetServoTargetQueue(sharedData);
            sharedData->servo_target_owner = SERVO_TARGET_OWNER_CONTROL;
            sharedData->control_enabled = 1;
            setCommandModeJoint(sharedData);
            setSystemState(sharedData, SYSTEM_STATE_RUN_FEEDBACK);
            break;

        case SYSTEM_EVENT_RUN_DIRECT_DONE:
            if (sharedData->system_state != SYSTEM_STATE_RUN_DIRECT) {
                FW_LOG_INFO_PRINTF("[STATE] ignore RUN_DIRECT_DONE in state=%u\r\n",
                              (unsigned)sharedData->system_state);
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
            setCalibrationStatus(CALIB_STATUS_SUCCESS, true);
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
    if (readyRequiresCan(sharedData) &&
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
    if (sharedAtomicLoad(&sharedData->reverse_release_fault_bitmap) != 0) {
        faults |= SYSTEM_FAULT_RELEASE_GUARD;
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

// ============================================================================
// StateMachineTask - 绯荤粺椤跺眰澶х姸鎬佹満浠诲姟
// ============================================================================
// 杩愯棰戠巼锛氶€氬父涓?100Hz (10ms 鍛ㄦ湡)
// 鑱岃矗锛?// 1) 璐熻矗鍏ㄥ眬瀹夊叏鐘舵€佹祦杞細UNCONTROLLABLE (涓嶅彲鎺?寮傚父) -> IDLE (绌洪棽鍙敤)
//    -> READY (鍚勫叧鑺傚湪鏈夋晥宸ヤ綔绌洪棿鍐咃紝鍑嗗灏辩华) -> RUN_FEEDBACK / RUN_DIRECT銆?// 2) 澶勭悊澶栭儴鍙戞潵鐨勬寚浠や簨浠讹紙濡?SYSTEM_EVENT_START銆丼YSTEM_EVENT_CALIBRATE锛夈€?// 3) 涓嶆柇鏀堕泦搴曞眰鐨勯敊璇姸鎬?(collectSystemFaults)锛屼竴鏃﹀彂鐜版柇鑱斻€佽繃杞界瓑鑷村懡閿欒锛?//    绔嬪嵆寮哄埗閫€鍥?UNCONTROLLABLE 鐘舵€佸苟瑙﹀彂绱ф€ユ€ュ仠 (Emergency Hold)銆?// ============================================================================
void stateMachineTask(void* parameter)
{
    TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;
    if (!sharedData) {
        vTaskDelete(NULL);
        return;
    }

    setSystemState(sharedData, SYSTEM_STATE_UNCONTROLLABLE);
    bool bootAutoCalibrationRequested = false;
    uint32_t handledEmergencyStopToken = 0;
    const uint32_t bootStateBeginMs = millis();
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

#if BOOT_AUTO_CALIBRATION_ENABLE
        if (!bootAutoCalibrationRequested &&
            sharedData->system_state == SYSTEM_STATE_IDLE &&
            sharedData->control_enabled == 0 &&
            sharedData->servo_target_owner == SERVO_TARGET_OWNER_NONE &&
            (uint32_t)(millis() - bootStateBeginMs) >= kBootAutoCalibrationDelayMs &&
            isAutoCalibrationFeedbackReady(sharedData)) {
            bootAutoCalibrationRequested = true;
            FW_LOG_INFO_PRINTLN("[STATE] boot feedback ready; posting AUTO CALIBRATE");
            postSystemEvent(sharedData, SYSTEM_EVENT_CALIBRATE);
        }
#endif

        if (sharedData->control_enabled != 0) {
            if (faults != 0) {
                enterFaultHold(sharedData, faults);
            }
        } else {
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
