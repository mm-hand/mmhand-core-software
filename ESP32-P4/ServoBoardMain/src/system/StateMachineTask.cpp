#include "StateMachineTask.h"

#include <math.h>
#include <string.h>

#include "../calibration/AutoCalibrationTask.h"
#include "../calibration/CalibrationMath.h"
#include "../calibration/CalibrationTask.h"
#include "../hardware/HardwareMap.h"

extern volatile uint8_t g_calibrationUIStatus;

static const uint32_t kStateCanOfflineTimeoutMs = 300;
static const uint32_t kStateServoFeedbackTimeoutMs = 500;
static const float kWorkspaceToleranceDeg = 1.0f;
#ifndef BOOT_AUTO_CALIBRATION_ENABLE
#define BOOT_AUTO_CALIBRATION_ENABLE 0
#endif
static const uint32_t kBootAutoCalibrationDelayMs = 1500;

static uint32_t collectSystemFaults(TaskSharedData_t* sharedData);
static bool activeJointsInControlWorkspace(TaskSharedData_t* sharedData);
static uint32_t readyServoRequiredMask(TaskSharedData_t* sharedData);

bool postSystemEvent(TaskSharedData_t* sharedData, uint8_t event)
{
    if (!sharedData || !sharedData->stateEventQueue) {
        return false;
    }

    SystemEvent_t msg;
    msg.event = event;
    msg.timestamp = millis();
    return xQueueSend(sharedData->stateEventQueue, &msg, 0) == pdTRUE;
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

static void requestServoEmergencyHold(TaskSharedData_t* sharedData)
{
    if (sharedData) {
        sharedData->servo_emergency_stop_token++;
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
           ((sharedData->multi_turn_unknown_bitmap & activeMask) == 0);
}

static void logMultiTurnUnknownReject(TaskSharedData_t* sharedData, const char* eventName)
{
    if (!sharedData) {
        return;
    }
    Serial.printf("[STATE] reject %s: multi-turn unknown active=0x%08lX unknown=0x%08lX\r\n",
                  eventName ? eventName : "RUN",
                  (unsigned long)readyServoRequiredMask(sharedData),
                  (unsigned long)sharedData->multi_turn_unknown_bitmap);
}

static void enterFaultHold(TaskSharedData_t* sharedData, uint32_t faultBits)
{
    if (!sharedData) {
        return;
    }
    resetServoTargetQueue(sharedData);
    sharedData->system_fault_latched_bitmap |= faultBits;
    sharedData->system_fault_bitmap |= sharedData->system_fault_latched_bitmap;
    sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
    sharedData->control_enabled = 0;
    requestServoEmergencyHold(sharedData);
    setSystemState(sharedData, SYSTEM_STATE_UNCONTROLLABLE);
}

static void resetControlProtectionFaults(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return;
    }
    sharedData->joint16_dual_feedback_fault = 0;
    sharedData->overload_fault_reset_token++;
    sharedData->reverse_release_fault_reset_token++;
}

static uint32_t readyServoRequiredMask(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return 0;
    }
    return sharedData->active_motor_mask & validMotorChannelMask();
}

static bool readyRequiresCan(TaskSharedData_t* sharedData)
{
    return sharedData && sharedData->require_can_feedback != 0;
}

static bool requiredServosOnline(TaskSharedData_t* sharedData, const ServoAngleData_t& servoSnapshot)
{
    const uint32_t requiredMask = readyServoRequiredMask(sharedData);
    if (requiredMask == 0) {
        static uint32_t s_lastZeroMaskLogMs = 0;
        const uint32_t nowMs = millis();
        if ((uint32_t)(nowMs - s_lastZeroMaskLogMs) > 1000) {
            Serial.println("[STATE] servo ready rejected: active motor mask is zero");
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
        Serial.printf("[STATE] servo offline activeMask=0x%08lX missing=0x%08lX\r\n",
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

    RemoteSensorData_t sensorData;
    bool canReady =
        sharedData->canRxQueue &&
        xQueuePeek(sharedData->canRxQueue, &sensorData, 0) == pdTRUE &&
        sensorData.isValid &&
        (uint32_t)(nowMs - sensorData.timestamp) <= kStateCanOfflineTimeoutMs;
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

    // TODO: Add a calibration-complete bitmap here. READY should require every
    // This is the gate from UNCONTROLLABLE to IDLE(controllable). READY is
    // reserved as a later pre-feedback state and is not entered automatically.
    return canReady &&
           servoReady &&
           sharedData->system_fault_bitmap == 0 &&
           sharedData->system_fault_latched_bitmap == 0;
}

// ============================================================================
// activeJointsInControlWorkspace - 关节工作空间准入校验
// ============================================================================
// 该函数是判断系统能否从 IDLE 自动晋升到 READY 状态的关键：
// 1) 遍历当前启用的 active_joint_mask（支持动态任意数量的关节）。
// 2) 检查这些被选中的关节是否都已经成功完成过标定（g_jointCalibResult[i].success == true）。
// 3) 读取磁编码器当前的角度值，校验其是否落在宽容度范围内 (-0.8° 到 max+0.8°)。
// 只有当所有选中关节都处于“安全可控范围内”时，系统才允许进入 READY。
// ============================================================================
static bool activeJointsInControlWorkspace(TaskSharedData_t* sharedData)
{
    if (!sharedData || !readyRequiresCan(sharedData)) {
        return false;
    }
    if (!activeMotorsMultiTurnKnown(sharedData)) {
        return false;
    }

    const uint32_t activeMask = sharedData->active_joint_mask & ((1UL << ENCODER_TOTAL_NUM) - 1UL);
    if (activeMask == 0) {
        return false;
    }

    MappedAngleData_t mappedData;
    if (!sharedData->mappedAngleQueue ||
        xQueuePeek(sharedData->mappedAngleQueue, &mappedData, 0) != pdTRUE ||
        !mappedData.isValid ||
        (uint32_t)(millis() - mappedData.timestamp) > kStateCanOfflineTimeoutMs) {
        return false;
    }

    for (uint8_t calibJoint = 0; calibJoint < ENCODER_TOTAL_NUM; calibJoint++) {
        if ((activeMask & (1UL << calibJoint)) == 0) {
            continue;
        }
        const JointCalibrationResult& calib = g_jointCalibResult[calibJoint];
        if (!calib.success || mappedData.validFlags[calibJoint] == 0) {
            return false;
        }
        const int32_t angleCount = mappedData.angleValues[calibJoint];
        const float maxDeg = (isfinite(calib.angleMax) && calib.angleMax > 0.0f)
            ? calib.angleMax
            : kRange100ControlMaxDeg;
        const int32_t maxCount = calibrationDegToEncoderCount(maxDeg);
        const int32_t toleranceCount = calibrationDegToEncoderCount(kWorkspaceToleranceDeg);
        if (angleCount < -toleranceCount || angleCount > maxCount + toleranceCount) {
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

static void handleSystemEvent(TaskSharedData_t* sharedData, const SystemEvent_t& event)
{
    if (!sharedData) {
        return;
    }

    switch (event.event)
    {
        case SYSTEM_EVENT_START:
            if (!canAcceptStart(sharedData->system_state)) {
                Serial.printf("[STATE] reject START in state=%u\r\n",
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
            resetServoTargetQueue(sharedData);
            sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
            sharedData->control_enabled = 0;
            requestServoEmergencyHold(sharedData);
            setSystemState(sharedData, SYSTEM_STATE_UNCONTROLLABLE);
            break;

        case SYSTEM_EVENT_RESET:
            resetCommandState(sharedData);
            sharedData->control_enabled = 0;
            sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
            resetServoTargetQueue(sharedData);
            requestServoEmergencyHold(sharedData);
            resetControlProtectionFaults(sharedData);
            sharedData->calibration_fault_latched = 0;
            sharedData->system_fault_latched_bitmap = 0;
            sharedData->system_fault_bitmap = 0;
            g_calibrationUIStatus = CALIB_STATUS_IDLE;
            setSystemState(sharedData, SYSTEM_STATE_UNCONTROLLABLE);
            break;

        case SYSTEM_EVENT_RUN_DIRECT:
            if (!canAcceptRunDirect(sharedData->system_state)) {
                Serial.printf("[STATE] reject RUN_DIRECT in state=%u\r\n",
                              (unsigned)sharedData->system_state);
                break;
            }
            if (!activeMotorsMultiTurnKnown(sharedData)) {
                Serial.printf("[STATE] allow RUN_DIRECT recovery with multi-turn unknown active=0x%08lX unknown=0x%08lX\r\n",
                              (unsigned long)readyServoRequiredMask(sharedData),
                              (unsigned long)sharedData->multi_turn_unknown_bitmap);
            }
            resetServoTargetQueue(sharedData);
            sharedData->servo_target_owner = SERVO_TARGET_OWNER_CONTROL;
            sharedData->control_enabled = 1;
            setCommandModeDirect(sharedData);
            setSystemState(sharedData, SYSTEM_STATE_RUN_DIRECT);
            break;

        case SYSTEM_EVENT_RUN_FEEDBACK:
            if (!canAcceptRunFeedback(sharedData->system_state)) {
                Serial.printf("[STATE] reject RUN_FEEDBACK in state=%u\r\n",
                              (unsigned)sharedData->system_state);
                break;
            }
            if (!activeMotorsMultiTurnKnown(sharedData)) {
                logMultiTurnUnknownReject(sharedData, "RUN_FEEDBACK");
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
                Serial.printf("[STATE] ignore RUN_DIRECT_DONE in state=%u\r\n",
                              (unsigned)sharedData->system_state);
                break;
            }
            sharedData->control_enabled = 0;
            sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
            setSystemState(sharedData, idleOrReadyState(sharedData));
            break;

        case SYSTEM_EVENT_RUN_FEEDBACK_DONE:
            if (sharedData->system_state != SYSTEM_STATE_RUN_FEEDBACK) {
                Serial.printf("[STATE] ignore RUN_FEEDBACK_DONE in state=%u\r\n",
                              (unsigned)sharedData->system_state);
                break;
            }
            sharedData->control_enabled = 0;
            sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
            setSystemState(sharedData, idleOrReadyState(sharedData));
            break;

        case SYSTEM_EVENT_PAUSE:
            if (sharedData->system_state != SYSTEM_STATE_RUN_DIRECT &&
                sharedData->system_state != SYSTEM_STATE_RUN_FEEDBACK) {
                Serial.printf("[STATE] reject PAUSE in state=%u\r\n",
                              (unsigned)sharedData->system_state);
                break;
            }
            sharedData->paused_resume_state = sharedData->system_state;
            sharedData->control_enabled = 0;
            sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
            requestServoEmergencyHold(sharedData);
            setSystemState(sharedData, SYSTEM_STATE_PAUSED);
            break;

        case SYSTEM_EVENT_CALIBRATE:
        {
            if (!canAcceptCalibrate(sharedData->system_state)) {
                Serial.printf("[STATE] reject CALIBRATE in state=%u\r\n",
                              (unsigned)sharedData->system_state);
                break;
            }
            resetServoTargetQueue(sharedData);
            sharedData->servo_target_owner = SERVO_TARGET_OWNER_CALIBRATION;
            sharedData->control_enabled = 0;
            sharedData->calibration_fault_latched = 0;
            setCommandModeJoint(sharedData);
            g_calibrationUIStatus = CALIB_STATUS_RUNNING;
            setSystemState(sharedData, SYSTEM_STATE_CALIBRATION_RUNNING);
            if (!startAutoCalibrationTask(sharedData)) {
                g_calibrationUIStatus = CALIB_STATUS_FAILED;
                sharedData->calibration_fault_latched = 1;
                requestServoEmergencyHold(sharedData);
                setSystemState(sharedData, SYSTEM_STATE_UNCONTROLLABLE);
            }
            break;
        }

        case SYSTEM_EVENT_CALIBRATION_DONE:
            if (sharedData->system_state != SYSTEM_STATE_CALIBRATION_RUNNING) {
                Serial.printf("[STATE] ignore CALIB_DONE in state=%u\r\n",
                              (unsigned)sharedData->system_state);
                break;
            }
            sharedData->control_enabled = 0;
            sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
            sharedData->calibration_fault_latched = 0;
            sharedData->multi_turn_unknown_bitmap &= ~readyServoRequiredMask(sharedData);
            g_calibrationUIStatus = CALIB_STATUS_SUCCESS;
            setSystemState(sharedData, idleOrReadyState(sharedData));
            break;

        case SYSTEM_EVENT_CALIBRATION_FAILED:
            if (sharedData->system_state != SYSTEM_STATE_CALIBRATION_RUNNING) {
                Serial.printf("[STATE] ignore CALIB_FAILED in state=%u\r\n",
                              (unsigned)sharedData->system_state);
                break;
            }
            sharedData->control_enabled = 0;
            sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
            sharedData->calibration_fault_latched = 1;
            g_calibrationUIStatus = CALIB_STATUS_FAILED;
            requestServoEmergencyHold(sharedData);
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

    RemoteSensorData_t sensorData;
    if (readyRequiresCan(sharedData) &&
        (xQueuePeek(sharedData->canRxQueue, &sensorData, 0) != pdTRUE ||
        !sensorData.isValid ||
        (uint32_t)(nowMs - sensorData.timestamp) > kStateCanOfflineTimeoutMs)) {
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
    if (sharedData->overload_fault_bitmap != 0) {
        faults |= SYSTEM_FAULT_OVERLOAD;
    }
    if (sharedData->reverse_release_fault_bitmap != 0) {
        faults |= SYSTEM_FAULT_RELEASE_GUARD;
    }
    if (sharedData->calibration_fault_latched != 0) {
        faults |= SYSTEM_FAULT_CALIBRATION;
    }
    faults |= sharedData->system_fault_latched_bitmap;

    return faults;
}

// ============================================================================
// StateMachineTask - 系统顶层大状态机任务
// ============================================================================
// 运行频率：通常为 100Hz (10ms 周期)
// 职责：
// 1) 负责全局安全状态流转：UNCONTROLLABLE (不可控/异常) -> IDLE (空闲可用)
//    -> READY (各关节在有效工作空间内，准备就绪) -> RUN_FEEDBACK / RUN_DIRECT。
// 2) 处理外部发来的指令事件（如 SYSTEM_EVENT_START、SYSTEM_EVENT_CALIBRATE）。
// 3) 不断收集底层的错误状态 (collectSystemFaults)，一旦发现断联、过载等致命错误，
//    立即强制退回 UNCONTROLLABLE 状态并触发紧急急停 (Emergency Hold)。
// ============================================================================
void stateMachineTask(void* parameter)
{
    TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;
    if (!sharedData) {
        vTaskDelete(NULL);
        return;
    }

    setSystemState(sharedData, SYSTEM_STATE_UNCONTROLLABLE);
    bool bootAutoCalibrationRequested = false;
    const uint32_t bootStateBeginMs = millis();

    for (;;)
    {
        SystemEvent_t event;
        while (xQueueReceive(sharedData->stateEventQueue, &event, 0) == pdTRUE) {
            handleSystemEvent(sharedData, event);
        }

        const uint32_t faults = collectSystemFaults(sharedData);
        sharedData->system_fault_bitmap = faults;

#if BOOT_AUTO_CALIBRATION_ENABLE
        if (!bootAutoCalibrationRequested &&
            sharedData->system_state == SYSTEM_STATE_IDLE &&
            sharedData->control_enabled == 0 &&
            sharedData->servo_target_owner == SERVO_TARGET_OWNER_NONE &&
            (uint32_t)(millis() - bootStateBeginMs) >= kBootAutoCalibrationDelayMs &&
            isAutoCalibrationFeedbackReady(sharedData)) {
            bootAutoCalibrationRequested = true;
            Serial.println("[STATE] boot feedback ready; posting AUTO CALIBRATE");
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

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
