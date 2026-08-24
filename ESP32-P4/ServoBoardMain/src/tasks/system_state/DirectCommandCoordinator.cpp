/**
 * @file DirectCommandCoordinator.cpp
 * @brief 实现直控命令在 pending 与 applied 两个阶段之间的安全切换。
 */
#include "DirectCommandCoordinator.h"

#include <Arduino.h>
#include <string.h>

#include "../../config/topology/HardwareMap.h"
#include "../../platform/rtos/AtomicValue.h"
#include "../../platform/rtos/SharedSafetyState.h"
#include "../../platform/time/Clock.h"
#include "../../safety/SafetyEventReporter.h"
#include "../../safety/SafetyRuleRegistry.h"

namespace mmhand::system {

void resetCommandState(SystemRuntimeContext* sharedData)
{
    if (!sharedData) {
        return;
    }

    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }

    // RESET 同时失效全部命令 token，随后到达的旧 DONE 事件会被识别为过期。
    for (uint8_t i = 0; i < ENCODER_TOTAL_NUM; i++) {
        sharedData->targetAngles[i] = 0.0f;
    }
    for (uint8_t i = 0; i < SERVO_TOTAL_NUM; i++) {
        sharedData->motorTargetRaw[i] = 0;
        sharedData->motorSweepTargetRaw[i] = 0;
        sharedData->pendingMotorTargetRaw[i] = 0;
        sharedData->pendingMotorSweepTargetRaw[i] = 0;
    }
    p4::platform::rtos::atomicAddFetch(&sharedData->motor_command_token,
                                 (uint32_t)1);
    p4::platform::rtos::atomicAddFetch(&sharedData->motor_sweep_command_token,
                                 (uint32_t)1);
    p4::platform::rtos::atomicAddFetch(&sharedData->joint_command_token,
                                 (uint32_t)1);
    sharedData->joint_command_accepted_ms =
        p4::platform::time::monotonicMillis();
    p4::platform::rtos::atomicAddFetch(
        &sharedData->motor_direct_command_generation,
        (uint32_t)1);
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
    sharedAtomicStore(&sharedData->feedback_recovery_pending, (uint8_t)0);
    sharedAtomicStore(&sharedData->feedback_recovery_request_sequence,
                      (uint32_t)0);

    if (lock) {
        xSemaphoreGive(lock);
    }
}

bool setCommandModeJoint(SystemRuntimeContext* sharedData)
{
    if (!sharedData) {
        return false;
    }

    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        return false;
    }
    // 清除直控来源，避免 MotionControlTask 返回负反馈后仍按旧直控 generation 判定。
    sharedData->control_mode = CONTROL_MODE_JOINT;
    sharedData->motor_direct_command_source = MOTOR_DIRECT_SOURCE_NONE;
    if (lock) {
        xSemaphoreGive(lock);
    }
    return true;
}

void setCommandModeDirect(SystemRuntimeContext* sharedData)
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

void clearPendingDirectCommand(SystemRuntimeContext* sharedData,
                               uint8_t rejectReason)
{
    // 只清 staged 命令，不覆盖已经生效并正在执行的 motorTargetRaw。
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

bool pendingDirectCommandMatches(SystemRuntimeContext* sharedData,
                                 uint32_t generation)
{
    if (!sharedData) {
        return false;
    }
    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        return false;
    }
    // generation 是单调递增代际号，防止状态机处理排队事件时接受更早的目标。
    const bool matches =
        sharedData->pending_direct_valid != 0 &&
        sharedData->pending_direct_generation == generation;
    if (lock) {
        xSemaphoreGive(lock);
    }
    return matches;
}

uint32_t pendingDirectMotorMask(SystemRuntimeContext* sharedData,
                                uint32_t generation)
{
    if (!sharedData) {
        return 0;
    }
    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        return 0;
    }
    // 只有同代际命令可暴露 scope，过期事件看到的 mask 必须为 0。
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

void rejectPendingDirectCommand(SystemRuntimeContext* sharedData,
                                uint32_t generation,
                                uint8_t rejectReason)
{
    // generation 必须匹配，旧事件不能拒绝后来写入的新直控命令。
    if (!sharedData) {
        return;
    }
    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    bool rejectedCurrentGeneration = false;
    if (sharedData->pending_direct_valid != 0 &&
        sharedData->pending_direct_generation == generation) {
        sharedData->pending_direct_valid = 0;
        sharedData->pending_direct_source = MOTOR_DIRECT_SOURCE_NONE;
        sharedData->pending_direct_motor_mask = 0;
        sharedData->pending_direct_joint_mask = 0;
        sharedData->pending_direct_require_can = 0;
        sharedData->direct_reject_reason = rejectReason;
        rejectedCurrentGeneration = true;
    }
    if (lock) {
        xSemaphoreGive(lock);
    }

    // 只登记真正命中当前 generation 的拒绝；过期状态事件不能污染诊断历史。
    if (rejectedCurrentGeneration) {
        const SafetyRuleId ruleId =
            p4::safety::ruleForDirectRejectReason(rejectReason);
        if (ruleId != SafetyRuleId::None) {
            p4::safety::reportSafetyRuleEvent(sharedData,
                                              ruleId,
                                              (int32_t)generation,
                                              (int32_t)rejectReason);
        }
    }
}

bool promotePendingDirectCommand(SystemRuntimeContext* sharedData,
                                 uint32_t generation)
{
    // 状态/在线/多圈门禁通过后，才把 pending 命令及其 active scope
    // 一次性提升为 MotionControlTask 可见的正式直控命令。
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
    // Sweep 与 ABS 使用不同目标邮箱/token，但共享同一代际和状态机门禁。
    if (source == MOTOR_DIRECT_SOURCE_SWEEP) {
        memcpy(sharedData->motorSweepTargetRaw,
               sharedData->pendingMotorSweepTargetRaw,
               sizeof(sharedData->motorSweepTargetRaw));
        p4::platform::rtos::atomicAddFetch(
            &sharedData->motor_sweep_command_token,
            (uint32_t)1);
    } else {
        memcpy(sharedData->motorTargetRaw,
               sharedData->pendingMotorTargetRaw,
               sizeof(sharedData->motorTargetRaw));
        p4::platform::rtos::atomicAddFetch(&sharedData->motor_command_token,
                                     (uint32_t)1);
    }
    sharedData->motor_direct_command_generation = generation;
    sharedData->motor_direct_command_source = source;
    sharedData->applied_direct_generation = generation;
    {
        // 目标与 active scope 在同一命令锁内生效，MotionControlTask 下一帧看到完整版本。
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
} // namespace mmhand::system
