#include "UpperCommCommandRouter.h"

#include <math.h>
#include <string.h>

#include "../../hardware/HardwareMap.h"
#include "../../system/StateMachineTask.h"

// Clamp a host absolute motor command to the firmware multi-turn range.
int16_t clampServoAbsCommand(int32_t value)
{
    if (value > 30719) return 30719;
    if (value < -30719) return -30719;
    return (int16_t)value;
}

void upperApplyTargetAngles(TaskSharedData_t* sharedData, const float* angles, uint8_t count)
{
    if (!sharedData || !angles) {
        return;
    }
    if (count > ENCODER_TOTAL_NUM) count = ENCODER_TOTAL_NUM;
    SemaphoreHandle_t lock = sharedData->commandStateMutex ? sharedData->commandStateMutex : sharedData->targetAnglesMutex;
    if (xSemaphoreTake(lock, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        // Write the complete joint target set while the command mutex is held.
        for (uint8_t i = 0; i < count; i++) {
            sharedData->targetAngles[i] = angles[i];
        }
        sharedData->joint_command_token++;
        xSemaphoreGive(lock);
    }
}

void upperClearTargetAngles(TaskSharedData_t* sharedData)
{
    float zeroAngles[ENCODER_TOTAL_NUM] = {0.0f};
    upperApplyTargetAngles(sharedData, zeroAngles, ENCODER_TOTAL_NUM);
}

void upperApplyMotorTargets(TaskSharedData_t* sharedData,
                            const int32_t* targets,
                            uint8_t count,
                            uint8_t source)
{
    if (!sharedData || !targets) {
        return;
    }
    if (count > SERVO_TOTAL_NUM) {
        count = SERVO_TOTAL_NUM;
    }
    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    // Store direct motor targets in motorMap channel order.
    for (uint8_t i = 0; i < count; i++) {
        sharedData->motorTargetRaw[i] = targets[i];
    }
    sharedData->motor_command_token++;
    sharedData->motor_direct_command_generation++;
    sharedData->motor_direct_command_source = source;
    if (lock) {
        xSemaphoreGive(lock);
    }
    postSystemEvent(sharedData, SYSTEM_EVENT_RUN_DIRECT);
}

void upperApplyMotorSweepTargets(TaskSharedData_t* sharedData, const int32_t* targets, uint8_t count)
{
    if (!sharedData || !targets) {
        return;
    }
    if (count > SERVO_TOTAL_NUM) {
        count = SERVO_TOTAL_NUM;
    }
    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    // Store sweep targets as single-turn raw positions in motorMap order.
    for (uint8_t i = 0; i < count; i++) {
        sharedData->motorSweepTargetRaw[i] = targets[i];
    }
    sharedData->motor_sweep_command_token++;
    sharedData->motor_direct_command_generation++;
    sharedData->motor_direct_command_source = MOTOR_DIRECT_SOURCE_SWEEP;
    if (lock) {
        xSemaphoreGive(lock);
    }
    postSystemEvent(sharedData, SYSTEM_EVENT_RUN_DIRECT);
}

void upperApplyTendonGuardConfig(TaskSharedData_t* sharedData,
                                 const uint8_t* enabled,
                                 const int8_t* sign,
                                 const int16_t* x1Abs,
                                 uint8_t count)
{
    if (!sharedData || !enabled || !sign || !x1Abs) {
        return;
    }
    if (count > ENCODER_TOTAL_NUM) {
        count = ENCODER_TOTAL_NUM;
    }
    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    // Store tendon guard config in joint order.
    for (uint8_t i = 0; i < count; i++) {
        sharedData->tendon_guard_enabled[i] = enabled[i] ? 1 : 0;
        sharedData->tendon_guard_sign[i] = (sign[i] < 0) ? -1 : 1;
        sharedData->tendon_guard_x1_abs[i] = clampServoAbsCommand(x1Abs[i]);
    }
    if (lock) {
        xSemaphoreGive(lock);
    }
}

void upperCacheCalibZeroRaw(TaskSharedData_t* sharedData, const float* values, uint8_t count)
{
    if (!sharedData || !values) {
        return;
    }
    if (count > ENCODER_TOTAL_NUM) {
        count = ENCODER_TOTAL_NUM;
    }
    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    // Cache the host-provided raw calibration values.
    for (uint8_t i = 0; i < count; i++) {
        float v = values[i];
        if (!isfinite(v)) {
            v = 0.0f;
        }
        sharedData->calib_zero_raw_cache[i] = (int32_t)lroundf(v);
    }
    for (uint8_t i = count; i < ENCODER_TOTAL_NUM; i++) {
        sharedData->calib_zero_raw_cache[i] = 0;
    }
    sharedData->calib_zero_raw_valid = 1;
    if (lock) {
        xSemaphoreGive(lock);
    }
}

bool upperApplyActiveScope(TaskSharedData_t* sharedData,
                           uint32_t activeMotorMask,
                           uint32_t activeJointMask,
                           uint8_t requireCanFeedback)
{
    if (!sharedData) {
        return false;
    }
    const uint8_t state = sharedData->system_state;
    if (sharedData->servo_target_owner != SERVO_TARGET_OWNER_NONE ||
        sharedData->control_enabled != 0 ||
        (state != SYSTEM_STATE_IDLE &&
         state != SYSTEM_STATE_READY &&
         state != SYSTEM_STATE_UNCONTROLLABLE)) {
        Serial.printf("[UPPER] ACTIVE_SCOPE rejected state=%u owner=%u enabled=%u\r\n",
                      (unsigned)state,
                      (unsigned)sharedData->servo_target_owner,
                      (unsigned)sharedData->control_enabled);
        return false;
    }
    const uint32_t motorMaskLimit = validMotorChannelMask();
    const uint32_t jointMaskLimit = (1UL << ENCODER_TOTAL_NUM) - 1UL;
    sharedData->active_motor_mask = activeMotorMask & motorMaskLimit;
    sharedData->active_joint_mask = activeJointMask & jointMaskLimit;
    sharedData->require_can_feedback = requireCanFeedback ? 1 : 0;

    const uint32_t scopeFaultMask = SYSTEM_FAULT_CAN_OFFLINE | SYSTEM_FAULT_SERVO_OFFLINE;
    sharedData->system_fault_bitmap &= ~scopeFaultMask;
    sharedData->system_fault_latched_bitmap &= ~scopeFaultMask;

    Serial.printf("[UPPER] ACTIVE_SCOPE motor=0x%08lX joint=0x%08lX can=%u\r\n",
                  (unsigned long)sharedData->active_motor_mask,
                  (unsigned long)sharedData->active_joint_mask,
                  (unsigned)sharedData->require_can_feedback);
    return true;
}
