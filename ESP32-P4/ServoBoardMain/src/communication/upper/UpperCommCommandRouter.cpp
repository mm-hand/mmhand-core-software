/**
 * @file UpperCommCommandRouter.cpp
 * @brief 把已解析的上位机控制命令写入共享命令邮箱。
 */
#include "UpperCommCommandRouter.h"

#include <math.h>
#include <string.h>

#include "../../hardware/HardwareMap.h"
#include "../../platform/AtomicValue.h"
#include "../../shared/SharedSafetyState.h"
#include "../../system/FirmwareLog.h"
#include "../../system/StateMachineTask.h"

// Clamp a host absolute motor command to the firmware multi-turn range.
int32_t clampServoAbsCommand(int32_t value)
{
    // 上位机传入的是 P4 逻辑多圈 ABS。每台舵机的实际行程校验统一在
    // ServoCommunicationTask 中结合该电机自己的 logicalOffset 完成。
    return value;
}

// A direct command is staged before the state machine accepts RUN_DIRECT.
// If queuing that event fails, discard only the matching staged generation so
// a later command is never cancelled accidentally.
static void cancelStagedDirectCommand(TaskSharedData_t* sharedData,
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

void upperApplyTargetAnglesMasked(TaskSharedData_t* sharedData,
                                  const float* angles,
                                  uint8_t count,
                                  uint32_t jointMask)
{
    if (!sharedData || !angles) {
        return;
    }
    if (count > ENCODER_TOTAL_NUM) count = ENCODER_TOTAL_NUM;
    const uint32_t appliedMask =
        jointMask & sharedAtomicLoad(&sharedData->active_joint_mask) &
        ((1UL << ENCODER_TOTAL_NUM) - 1UL);
    if (appliedMask == 0) {
        FW_LOG_INFO_PRINTF(
            "[UPPER] ANGLE_MASK rejected requested=0x%08lX active=0x%08lX\r\n",
            (unsigned long)jointMask,
            (unsigned long)sharedAtomicLoad(&sharedData->active_joint_mask));
        return;
    }
    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (xSemaphoreTake(lock, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        const uint32_t previousMask = sharedData->feedback_command_joint_mask;
        // Write the complete joint target set while the command mutex is held.
        for (uint8_t i = 0; i < count; i++) {
            sharedData->targetAngles[i] = angles[i];
        }
        sharedData->control_mode = CONTROL_MODE_JOINT;
        sharedData->motor_direct_command_source = MOTOR_DIRECT_SOURCE_NONE;
        sharedData->feedback_command_joint_mask = appliedMask;
        p4::platform::atomicAddFetch(&sharedData->joint_command_token,
                                     (uint32_t)1);
        xSemaphoreGive(lock);
        if (previousMask != appliedMask) {
            FW_LOG_INFO_PRINTF(
                "[UPPER] ANGLE_MASK applied=0x%08lX previous=0x%08lX\r\n",
                (unsigned long)appliedMask,
                (unsigned long)previousMask);
        }
    }
}

void upperApplyTargetAngles(TaskSharedData_t* sharedData,
                            const float* angles,
                            uint8_t count)
{
    const uint32_t activeMask = sharedData
        ? sharedAtomicLoad(&sharedData->active_joint_mask)
        : 0;
    upperApplyTargetAnglesMasked(sharedData, angles, count, activeMask);
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
         state != SYSTEM_STATE_RECOVERY_REQUIRED &&
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

    // Recheck after taking the command lock. A queued RUN_DIRECT event may
    // otherwise take ownership between the optimistic check above and this
    // scope update.
    const uint8_t lockedState = sharedData->system_state;
    if (sharedData->servo_target_owner != SERVO_TARGET_OWNER_NONE ||
        sharedData->control_enabled != 0 ||
        (lockedState != SYSTEM_STATE_IDLE &&
         lockedState != SYSTEM_STATE_READY &&
         lockedState != SYSTEM_STATE_RECOVERY_REQUIRED &&
         lockedState != SYSTEM_STATE_UNCONTROLLABLE)) {
        if (lock) {
            xSemaphoreGive(lock);
        }
        FW_LOG_INFO_PRINTF("[UPPER] ACTIVE_SCOPE rejected after lock state=%u owner=%u enabled=%u\r\n",
                           (unsigned)lockedState,
                           (unsigned)sharedData->servo_target_owner,
                           (unsigned)sharedData->control_enabled);
        return false;
    }

    const uint32_t appliedMotorMask = activeMotorMask & motorMaskLimit;
    const uint32_t appliedJointMask = activeJointMask & jointMaskLimit;
    const bool cancelledPendingDirect = sharedData->pending_direct_valid != 0;
    const uint32_t cancelledGeneration = sharedData->pending_direct_generation;
    if (cancelledPendingDirect) {
        sharedData->pending_direct_valid = 0;
        sharedData->pending_direct_source = MOTOR_DIRECT_SOURCE_NONE;
        sharedData->pending_direct_motor_mask = 0;
        sharedData->pending_direct_joint_mask = 0;
        sharedData->pending_direct_require_can = 0;
        sharedData->direct_reject_reason = DIRECT_REJECT_STALE;
    }
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
    if (cancelledPendingDirect) {
        FW_LOG_INFO_PRINTF("[UPPER] ACTIVE_SCOPE cancelled pending RUN_DIRECT generation=%lu\r\n",
                           (unsigned long)cancelledGeneration);
    }
    return true;
}
