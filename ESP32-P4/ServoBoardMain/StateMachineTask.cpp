#include "StateMachineTask.h"

#include <string.h>

#include "CalibrationTask.h"

extern volatile uint8_t g_calibrationUIStatus;

static const uint32_t kStateCanOfflineTimeoutMs = 300;
static const uint32_t kStateServoFeedbackTimeoutMs = 500;

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

static void applyTargetAngles(TaskSharedData_t* sharedData, const float* angles, uint8_t count)
{
    if (!sharedData || !angles) {
        return;
    }
    if (count > ENCODER_TOTAL_NUM) {
        count = ENCODER_TOTAL_NUM;
    }
    if (xSemaphoreTake(sharedData->targetAnglesMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        for (uint8_t i = 0; i < count; i++) {
            sharedData->targetAngles[i] = angles[i];
        }
        xSemaphoreGive(sharedData->targetAnglesMutex);
    }
}

static void clearTargetAngles(TaskSharedData_t* sharedData)
{
    float zeroAngles[ENCODER_TOTAL_NUM] = {0.0f};
    applyTargetAngles(sharedData, zeroAngles, ENCODER_TOTAL_NUM);
}

static void clearMotorTargets(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return;
    }
    for (uint8_t i = 0; i < SERVO_TOTAL_NUM; i++) {
        sharedData->motorTargetRaw[i] = 0;
        sharedData->motorSweepTargetRaw[i] = 0;
    }
}

static void resetControlAndFaultTokens(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return;
    }
    sharedData->joint16_dual_feedback_fault = 0;
    sharedData->overload_fault_reset_token++;
    sharedData->reverse_release_fault_reset_token++;
}

static void handleSystemEvent(TaskSharedData_t* sharedData, const SystemEvent_t& event)
{
    if (!sharedData) {
        return;
    }

    switch (event.event)
    {
        case SYSTEM_EVENT_START:
            sharedData->control_enabled = 1;
            resetControlAndFaultTokens(sharedData);
            setSystemState(sharedData, SYSTEM_STATE_RUNNING);
            break;

        case SYSTEM_EVENT_STOP:
            sharedData->control_enabled = 0;
            setSystemState(sharedData, SYSTEM_STATE_STOPPED);
            break;

        case SYSTEM_EVENT_RESET:
            clearTargetAngles(sharedData);
            clearMotorTargets(sharedData);
            sharedData->motor_command_token++;
            sharedData->motor_sweep_command_token++;
            sharedData->joint_command_token++;
            sharedData->motor_direct_command_generation++;
            sharedData->motor_direct_command_source = MOTOR_DIRECT_SOURCE_NONE;
            sharedData->control_enabled = 0;
            sharedData->control_mode = CONTROL_MODE_JOINT;
            resetControlAndFaultTokens(sharedData);
            sharedData->system_fault_bitmap = 0;
            setSystemState(sharedData, SYSTEM_STATE_IDLE);
            break;

        case SYSTEM_EVENT_CALIBRATE:
            sharedData->control_enabled = 0;
            sharedData->control_mode = CONTROL_MODE_JOINT;
            g_calibrationUIStatus = CALIB_STATUS_IDLE;
            setSystemState(sharedData, SYSTEM_STATE_CALIBRATION_IDLE);
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
    if (xQueuePeek(sharedData->canRxQueue, &sensorData, 0) != pdTRUE ||
        !sensorData.isValid ||
        (uint32_t)(nowMs - sensorData.timestamp) > kStateCanOfflineTimeoutMs) {
        faults |= SYSTEM_FAULT_CAN_OFFLINE;
    }

    ServoAngleData_t servoSnapshot;
    const bool servoFeedbackFresh =
        sharedData->servoFeedbackQueue &&
        xQueuePeek(sharedData->servoFeedbackQueue, &servoSnapshot, 0) == pdTRUE &&
        (uint32_t)(nowMs - servoSnapshot.timestamp) <= kStateServoFeedbackTimeoutMs;
    if (servoFeedbackFresh) {
        for (uint8_t i = 0; i < SERVO_TOTAL_NUM; i++) {
            if (servoSnapshot.onlineStatus[i] == 0) {
                faults |= SYSTEM_FAULT_SERVO_OFFLINE;
                break;
            }
        }
    } else {
        faults |= SYSTEM_FAULT_SERVO_OFFLINE;
    }

    if (sharedData->joint16_dual_feedback_fault != 0) {
        faults |= SYSTEM_FAULT_JOINT16_DUAL;
    }
    if (sharedData->overload_fault_bitmap != 0) {
        faults |= SYSTEM_FAULT_OVERLOAD;
    }
    if (sharedData->reverse_release_fault_bitmap != 0) {
        faults |= SYSTEM_FAULT_RELEASE_GUARD;
    }

    return faults;
}

void stateMachineTask(void* parameter)
{
    TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;
    if (!sharedData) {
        vTaskDelete(NULL);
        return;
    }

    setSystemState(sharedData, SYSTEM_STATE_IDLE);

    for (;;)
    {
        SystemEvent_t event;
        while (xQueueReceive(sharedData->stateEventQueue, &event, 0) == pdTRUE) {
            handleSystemEvent(sharedData, event);
        }

        const uint32_t faults = collectSystemFaults(sharedData);
        sharedData->system_fault_bitmap = faults;

        if (sharedData->control_enabled != 0) {
            setSystemState(sharedData, faults ? SYSTEM_STATE_FAULT_HOLD : SYSTEM_STATE_RUNNING);
        } else if (sharedData->system_state == SYSTEM_STATE_BOOTING) {
            setSystemState(sharedData, SYSTEM_STATE_IDLE);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
