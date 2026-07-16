#include "UpperCommCommandRouter.h"

#include <math.h>
#include <string.h>

#include "../../hardware/HardwareMap.h"
#include "../../shared/SharedSafetyState.h"
#include "../../system/FirmwareLog.h"
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
        sharedData->control_mode = CONTROL_MODE_JOINT;
        sharedData->motor_direct_command_source = MOTOR_DIRECT_SOURCE_NONE;
        sharedData->joint_command_token++;
        xSemaphoreGive(lock);
    }
}

void upperClearTargetAngles(TaskSharedData_t* sharedData)
{
    float zeroAngles[ENCODER_TOTAL_NUM] = {0.0f};
    upperApplyTargetAngles(sharedData, zeroAngles, ENCODER_TOTAL_NUM);
}

static void cancelStagedDirectCommand(TaskSharedData_t* sharedData,
                                      uint32_t generation,
                                      uint8_t reason)
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
        sharedData->direct_reject_reason = reason;
    }
    if (lock) {
        xSemaphoreGive(lock);
    }
}

bool upperApplyMotorTargets(TaskSharedData_t* sharedData,
                            const int32_t* targets,
                            uint8_t count,
                            uint8_t source)
{
    if (!sharedData || !targets) {
        return false;
    }
    if (count > SERVO_TOTAL_NUM) {
        count = SERVO_TOTAL_NUM;
    }
    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        return false;
    }
    memcpy(sharedData->pendingMotorTargetRaw,
           sharedData->motorTargetRaw,
           sizeof(sharedData->pendingMotorTargetRaw));
    for (uint8_t i = 0; i < count; i++) {
        sharedData->pendingMotorTargetRaw[i] = clampServoAbsCommand(targets[i]);
    }
    const uint32_t generation = sharedData->pending_direct_generation + 1U;
    sharedData->pending_direct_generation = generation;
    sharedData->pending_direct_source = source;
    sharedData->pending_direct_motor_mask =
        sharedAtomicLoad(&sharedData->active_motor_mask) & validMotorChannelMask();
    sharedData->pending_direct_joint_mask =
        sharedAtomicLoad(&sharedData->active_joint_mask) &
        ((1UL << ENCODER_TOTAL_NUM) - 1UL);
    sharedData->pending_direct_require_can = 0;
    sharedData->pending_direct_valid = 1;
    sharedData->direct_reject_reason = DIRECT_REJECT_NONE;
    if (lock) {
        xSemaphoreGive(lock);
    }
    if (!postSystemEventForToken(sharedData, SYSTEM_EVENT_RUN_DIRECT, generation)) {
        cancelStagedDirectCommand(sharedData, generation, DIRECT_REJECT_EVENT_QUEUE);
        return false;
    }
    return true;
}

bool upperApplySingleMotorTarget(TaskSharedData_t* sharedData,
                                 uint8_t motorIndex,
                                 int32_t target,
                                 uint8_t source)
{
    if (!sharedData || motorIndex >= SERVO_TOTAL_NUM) {
        return false;
    }
    const uint32_t motorMask = (1UL << motorIndex) & validMotorChannelMask();
    if (motorMask == 0) {
        return false;
    }
    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        return false;
    }
    if (sharedData->pending_direct_valid != 0 &&
        sharedData->pending_direct_source != MOTOR_DIRECT_SOURCE_SWEEP) {
        // Keep other edits from the newest not-yet-applied direct packet.
    } else {
        memcpy(sharedData->pendingMotorTargetRaw,
               sharedData->motorTargetRaw,
               sizeof(sharedData->pendingMotorTargetRaw));
    }
    sharedData->pendingMotorTargetRaw[motorIndex] = clampServoAbsCommand(target);
    const uint32_t generation = sharedData->pending_direct_generation + 1U;
    sharedData->pending_direct_generation = generation;
    sharedData->pending_direct_source = source;
    sharedData->pending_direct_motor_mask = motorMask;
    sharedData->pending_direct_joint_mask = 0;
    sharedData->pending_direct_require_can = 0;
    sharedData->pending_direct_valid = 1;
    sharedData->direct_reject_reason = DIRECT_REJECT_NONE;
    if (lock) {
        xSemaphoreGive(lock);
    }
    FW_LOG_INFO_PRINTF("[UPPER] SINGLE_MOTOR_TARGET M%02u target=%ld mask=0x%08lX\r\n",
                  (unsigned)motorIndex,
                  (long)clampServoAbsCommand(target),
                  (unsigned long)motorMask);
    if (!postSystemEventForToken(sharedData, SYSTEM_EVENT_RUN_DIRECT, generation)) {
        cancelStagedDirectCommand(sharedData, generation, DIRECT_REJECT_EVENT_QUEUE);
        return false;
    }
    return true;
}

bool upperApplyMotorSweepTargets(TaskSharedData_t* sharedData, const int32_t* targets, uint8_t count)
{
    if (!sharedData || !targets) {
        return false;
    }
    if (count > SERVO_TOTAL_NUM) {
        count = SERVO_TOTAL_NUM;
    }
    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        return false;
    }
    memcpy(sharedData->pendingMotorSweepTargetRaw,
           sharedData->motorSweepTargetRaw,
           sizeof(sharedData->pendingMotorSweepTargetRaw));
    for (uint8_t i = 0; i < count; i++) {
        sharedData->pendingMotorSweepTargetRaw[i] = targets[i];
    }
    const uint32_t generation = sharedData->pending_direct_generation + 1U;
    sharedData->pending_direct_generation = generation;
    sharedData->pending_direct_source = MOTOR_DIRECT_SOURCE_SWEEP;
    sharedData->pending_direct_motor_mask =
        sharedAtomicLoad(&sharedData->active_motor_mask) & validMotorChannelMask();
    sharedData->pending_direct_joint_mask =
        sharedAtomicLoad(&sharedData->active_joint_mask) &
        ((1UL << ENCODER_TOTAL_NUM) - 1UL);
    sharedData->pending_direct_require_can = 0;
    sharedData->pending_direct_valid = 1;
    sharedData->direct_reject_reason = DIRECT_REJECT_NONE;
    if (lock) {
        xSemaphoreGive(lock);
    }
    if (!postSystemEventForToken(sharedData, SYSTEM_EVENT_RUN_DIRECT, generation)) {
        cancelStagedDirectCommand(sharedData, generation, DIRECT_REJECT_EVENT_QUEUE);
        return false;
    }
    return true;
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
        FW_LOG_INFO_PRINTF("[UPPER] ACTIVE_SCOPE rejected state=%u owner=%u enabled=%u\r\n",
                      (unsigned)state,
                      (unsigned)sharedData->servo_target_owner,
                      (unsigned)sharedData->control_enabled);
        return false;
    }
    const uint32_t motorMaskLimit = validMotorChannelMask();
    const uint32_t jointMaskLimit = (1UL << ENCODER_TOTAL_NUM) - 1UL;

    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        FW_LOG_INFO_PRINTLN("[UPPER] ACTIVE_SCOPE rejected: command mutex busy");
        return false;
    }

    const uint32_t appliedMotorMask = activeMotorMask & motorMaskLimit;
    const uint32_t appliedJointMask = activeJointMask & jointMaskLimit;
    {
        SharedSafetyGuard guard;
        sharedAtomicStore(&sharedData->active_motor_mask, appliedMotorMask);
        sharedAtomicStore(&sharedData->active_joint_mask, appliedJointMask);
        sharedAtomicStore(&sharedData->require_can_feedback,
                          (uint8_t)(requireCanFeedback ? 1 : 0));
    }
    sharedData->control_mode = requireCanFeedback ? CONTROL_MODE_JOINT : CONTROL_MODE_DIRECT_MOTOR;
    if (requireCanFeedback) {
        sharedData->motor_direct_command_source = MOTOR_DIRECT_SOURCE_NONE;
    }

    if (lock) {
        xSemaphoreGive(lock);
    }

    const uint32_t scopeFaultMask = SYSTEM_FAULT_CAN_OFFLINE | SYSTEM_FAULT_SERVO_OFFLINE;
    sharedAtomicFetchAnd(&sharedData->system_fault_bitmap, ~scopeFaultMask);
    sharedAtomicFetchAnd(&sharedData->system_fault_latched_bitmap, ~scopeFaultMask);

    FW_LOG_INFO_PRINTF("[UPPER] ACTIVE_SCOPE motor=0x%08lX joint=0x%08lX can=%u\r\n",
                  (unsigned long)appliedMotorMask,
                  (unsigned long)appliedJointMask,
                  (unsigned)(requireCanFeedback ? 1 : 0));
    return true;
}
