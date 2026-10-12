/**
 * @file ControlCommandSnapshotReader.cpp
 * @brief 实现控制命令的一致快照读取。
 */
#include "ControlCommandSnapshotReader.h"

#include <string.h>

namespace p4::control {

bool readCommandSnapshot(SystemRuntimeContext* runtime,
                         ControlCommandSnapshot_t* snapshot)
{
    if (!runtime || !snapshot) {
        return false;
    }

    SemaphoreHandle_t lock = runtime->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(2)) != pdTRUE) {
        return false;
    }

    memcpy(snapshot->targetAngles,
           runtime->targetAngles,
           sizeof(snapshot->targetAngles));
    memcpy(snapshot->motorTargetRaw,
           runtime->motorTargetRaw,
           sizeof(snapshot->motorTargetRaw));
    snapshot->motorCommandToken = runtime->motor_command_token;
    snapshot->directToken = runtime->applied_direct_token;
    snapshot->directMotorMask = runtime->applied_direct_motor_mask;
    snapshot->directSpeed = runtime->applied_direct_speed;
    snapshot->directAcc = runtime->applied_direct_acc;
    snapshot->jointCommandToken = runtime->joint_command_token;
    snapshot->jointCommandAcceptedMs = runtime->joint_command_accepted_ms;
    snapshot->feedbackCommandJointMask = runtime->feedback_command_joint_mask;
    snapshot->motorDirectCommandGeneration =
        runtime->motor_direct_command_generation;
    snapshot->motorDirectCommandSource = runtime->motor_direct_command_source;
    snapshot->controlMode = runtime->control_mode;

    if (lock) {
        xSemaphoreGive(lock);
    }
    return true;
}

} // namespace p4::control
