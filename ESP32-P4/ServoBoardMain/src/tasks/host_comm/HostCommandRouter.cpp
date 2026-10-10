/**
 * @file HostCommandRouter.cpp
 * @brief 把已解析的上位机控制命令写入共享命令邮箱。
 */
#include "HostCommandRouter.h"
#include "../../platform/time/Clock.h"

#include <math.h>
#include <string.h>

#include "../../config/topology/HardwareMap.h"
#include "../../data/SafetyTypes.h"
#include "../../platform/rtos/AtomicValue.h"
#include "../../platform/rtos/SharedSafetyState.h"
#include "../../platform/log/FirmwareLog.h"
#include "../../runtime/ports/SystemEventPort.h"
#include "../../runtime/ports/DirectTargetResult.h"

// RUN_DIRECT 被状态机接受前，直控目标只存在 pending 邮箱中。
// 若事件入队失败，仅撤销同 generation 的目标，避免误删随后到达的新命令。
static void cancelStagedDirectCommand(SystemRuntimeContext* sharedData,
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
        rejectOutstandingDirectTarget(sharedData);
        sharedData->pending_direct_token = 0;
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

// 提交一帧关节角目标；jointMask 与当前 active scope 取交集后才会参与 PID。
bool upperApplyTargetAnglesMasked(SystemRuntimeContext* sharedData,
                                  const float* angles,
                                  uint8_t count,
                                  uint32_t jointMask)
{
    if (!sharedData || !angles) {
        return false;
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
        return false;
    }
    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (xSemaphoreTake(lock, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        const uint32_t previousMask = sharedData->feedback_command_joint_mask;
        // 在同一把命令锁内写完整角度数组和 token，MotionControlTask 不会读到半帧目标。
        for (uint8_t i = 0; i < count; i++) {
            sharedData->targetAngles[i] = angles[i];
        }
        rejectOutstandingDirectTarget(sharedData);
        sharedData->direct_release_generation = 0;
        sharedData->control_mode = CONTROL_MODE_JOINT;
        sharedData->motor_direct_command_source = MOTOR_DIRECT_SOURCE_NONE;
        sharedData->feedback_command_joint_mask = appliedMask;
        sharedData->joint_command_accepted_ms =
            p4::platform::time::monotonicMillis();
        p4::platform::rtos::atomicAddFetch(&sharedData->joint_command_token,
                                     (uint32_t)1);
        xSemaphoreGive(lock);
        // 目标帧已经完整写入命令邮箱；立即唤醒控制任务，不改变 10 ms 的看门狗周期。
        sharedData->motionControlWake.notify(p4::runtime::wake::kMotionTarget);
        if (previousMask != appliedMask) {
            FW_LOG_INFO_PRINTF(
                "[UPPER] ANGLE_MASK applied=0x%08lX previous=0x%08lX\r\n",
                (unsigned long)appliedMask,
                (unsigned long)previousMask);
        }
        return true;
    }
    return false;
}

bool upperApplyTargetAngles(SystemRuntimeContext* sharedData,
                            const float* angles,
                            uint8_t count)
{
    const uint32_t activeMask = sharedData
        ? sharedAtomicLoad(&sharedData->active_joint_mask)
        : 0;
    return upperApplyTargetAnglesMasked(sharedData, angles, count, activeMask);
}

// 所有直控使用同一暂存、准入和执行路径；token 只关联可选的总线回执。
bool upperApplyMotorTargets(SystemRuntimeContext* sharedData,
                            const int32_t* targets,
                            uint8_t count,
                            uint32_t token,
                            uint32_t requestedMotorMask,
                            uint16_t speed,
                            uint8_t acc)
{
    if (!sharedData || !targets || count == 0 || count > SERVO_TOTAL_NUM) return false;
    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) return false;
    const uint32_t activeMask =
        sharedAtomicLoad(&sharedData->active_motor_mask) & validMotorChannelMask();
    const uint32_t mask = requestedMotorMask & validMotorChannelMask();
    const bool rejected = requestedMotorMask != mask || mask == 0 ||
        (mask & ~activeMask) != 0 ||
        sharedData->direct_release_generation != 0 ||
        (token != 0 && token == sharedData->direct_target_request_token) ||
        sharedData->servo_rebase_zero_request_token !=
            sharedData->servo_rebase_zero_result_token ||
        sharedData->restore_multi_turn_request_token !=
            sharedData->restore_multi_turn_result_token ||
        sharedData->servo_reset_zero_request_token !=
            sharedData->servo_reset_zero_result_token;
    if (rejected) {
        if (lock) xSemaphoreGive(lock);
        return false;
    }

    rejectOutstandingDirectTarget(sharedData);
    memcpy(sharedData->pendingMotorTargetRaw, sharedData->motorTargetRaw,
           sizeof(sharedData->pendingMotorTargetRaw));
    for (uint8_t i = 0; i < count; ++i) {
        if ((mask & (1UL << i)) != 0) sharedData->pendingMotorTargetRaw[i] = targets[i];
    }
    uint32_t generation = sharedData->pending_direct_generation + 1U;
    if (generation == 0) generation = 1;
    sharedData->pending_direct_generation = generation;
    sharedData->pending_direct_source = MOTOR_DIRECT_SOURCE_ABSOLUTE;
    sharedData->pending_direct_motor_mask = mask;
    sharedData->pending_direct_joint_mask =
        sharedAtomicLoad(&sharedData->active_joint_mask) &
        ((1UL << ENCODER_TOTAL_NUM) - 1UL);
    sharedData->pending_direct_require_can = 0;
    sharedData->pending_direct_token = token;
    sharedData->pending_direct_speed = speed;
    sharedData->pending_direct_acc = acc;
    {
        SharedSafetyGuard guard;
        sharedData->direct_target_request_generation = generation;
        sharedData->direct_target_request_token = token;
        sharedData->direct_target_request_mask = mask;
    }
    sharedData->pending_direct_valid = 1;
    sharedData->direct_reject_reason = DIRECT_REJECT_NONE;
    if (lock) xSemaphoreGive(lock);
    if (!postSystemEventForToken(sharedData, SYSTEM_EVENT_RUN_DIRECT, generation)) {
        cancelStagedDirectCommand(sharedData, generation, DIRECT_REJECT_EVENT_QUEUE);
        return false;
    }
    return true;
}

bool upperApplySingleMotorTarget(SystemRuntimeContext* sharedData,
                                 uint8_t motorIndex,
                                 int32_t target)
{
    if (!sharedData || motorIndex >= SERVO_TOTAL_NUM) return false;
    int32_t targets[SERVO_TOTAL_NUM] = {};
    targets[motorIndex] = target;
    return upperApplyMotorTargets(sharedData, targets, SERVO_TOTAL_NUM, 0,
        1UL << motorIndex, SERVO_TARGET_SPEED_DEFAULT, SERVO_TARGET_ACC_DEFAULT);
}

bool upperReleaseMotorDirect(SystemRuntimeContext* sharedData, uint32_t token)
{
    if (!sharedData) return false;
    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) return false;
    const uint32_t generation = sharedData->applied_direct_generation;
    const bool tokenMatches = token == 0 ||
        (token == sharedData->applied_direct_token &&
         sharedData->direct_target_result_generation == generation &&
         sharedData->direct_target_result_token == token &&
         sharedData->direct_target_status == DIRECT_TARGET_APPLY_APPLIED);
    const uint8_t currentState = sharedAtomicLoad(&sharedData->system_state);
    const bool alreadyCompleted = generation != 0 && tokenMatches &&
        (currentState == SYSTEM_STATE_IDLE || currentState == SYSTEM_STATE_READY ||
         currentState == SYSTEM_STATE_RECOVERY_REQUIRED) &&
        sharedAtomicLoad(&sharedData->servo_target_owner) == SERVO_TARGET_OWNER_NONE &&
        sharedAtomicLoad(&sharedData->control_enabled) == 0 &&
        sharedAtomicLoad(&sharedData->emergency_stop_latched) == 0 &&
        sharedData->pending_direct_valid == 0 &&
        sharedData->applied_direct_motor_mask != 0 &&
        sharedData->direct_target_result_generation == generation &&
        sharedData->direct_target_status == DIRECT_TARGET_APPLY_APPLIED;
    if (alreadyCompleted) {
        if (lock) xSemaphoreGive(lock);
        return true;
    }
    const bool accepted = generation != 0 && tokenMatches &&
        sharedAtomicLoad(&sharedData->system_state) == SYSTEM_STATE_RUN_DIRECT &&
        sharedAtomicLoad(&sharedData->servo_target_owner) == SERVO_TARGET_OWNER_CONTROL &&
        sharedData->motor_direct_command_source == MOTOR_DIRECT_SOURCE_ABSOLUTE &&
        sharedData->pending_direct_valid == 0 &&
        sharedData->direct_release_generation == 0;
    if (accepted) sharedData->direct_release_generation = generation;
    if (lock) xSemaphoreGive(lock);
    if (!accepted) return false;
    if (postSystemEventForToken(sharedData, SYSTEM_EVENT_RUN_DIRECT_DONE, generation)) {
        return true;
    }
    if (!lock || xSemaphoreTake(lock, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (sharedData->direct_release_generation == generation) {
            sharedData->direct_release_generation = 0;
        }
        if (lock) xSemaphoreGive(lock);
    }
    return false;
}

// Active scope 仅能在无 owner、控制关闭的静止态修改，并在持锁后复核一次状态。
bool upperApplyActiveScope(SystemRuntimeContext* sharedData,
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

    // 取锁后再次检查：第一次检查与 scope 写入之间，排队中的 RUN_DIRECT 可能已取得 owner。
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
        rejectOutstandingDirectTarget(sharedData);
        sharedData->pending_direct_token = 0;
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
    rejectOutstandingDirectTarget(sharedData);
    sharedData->direct_release_generation = 0;
    sharedData->applied_direct_motor_mask = 0;
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
