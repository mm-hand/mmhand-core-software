/**
 * @file ControlRuntimePolicy.cpp
 * @brief 实现控制命令一致快照、硬过载作用域与运行完成判定。
 */
#include "ControlRuntimePolicy.h"

#include <math.h>
#include <string.h>

#include "../config/ControlConfig.h"
#include "../hardware/HardwareMap.h"
#include "../shared/SharedSafetyState.h"
#include "ControlOutputBuilder.h"

namespace p4::control {

using namespace mmhand::config::control;

bool readCommandSnapshot(TaskSharedData_t* sharedData,
                         ControlCommandSnapshot_t* snapshot)
{
    if (!sharedData || !snapshot) {
        return false;
    }

    // 目标数组、模式、mask 和 token 属于同一命令版本，必须在同一临界区复制。
    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(2)) != pdTRUE) {
        return false;
    }

    memcpy(snapshot->targetAngles,
           sharedData->targetAngles,
           sizeof(snapshot->targetAngles));
    memcpy(snapshot->motorTargetRaw,
           sharedData->motorTargetRaw,
           sizeof(snapshot->motorTargetRaw));
    memcpy(snapshot->motorSweepTargetRaw,
           sharedData->motorSweepTargetRaw,
           sizeof(snapshot->motorSweepTargetRaw));
    snapshot->motorCommandToken = sharedData->motor_command_token;
    snapshot->motorSweepCommandToken =
        sharedData->motor_sweep_command_token;
    snapshot->jointCommandToken = sharedData->joint_command_token;
    snapshot->feedbackCommandJointMask =
        sharedData->feedback_command_joint_mask;
    snapshot->motorDirectCommandGeneration =
        sharedData->motor_direct_command_generation;
    snapshot->motorDirectCommandSource =
        sharedData->motor_direct_command_source;
    snapshot->controlMode = sharedData->control_mode;

    if (lock) {
        xSemaphoreGive(lock);
    }
    return true;
}

uint32_t activeOverloadMotorMask(
    TaskSharedData_t* sharedData,
    const ControlCommandSnapshot_t& commandSnapshot)
{
    if (!sharedData) {
        return 0;
    }

    const uint32_t validMask = validMotorChannelMask();
    // 直控直接使用 active motor；关节闭环则由 HardwareMap 展开为执行器集合。
    if (commandSnapshot.controlMode == CONTROL_MODE_DIRECT_MOTOR) {
        return sharedAtomicLoad(&sharedData->active_motor_mask) & validMask;
    }

    uint32_t mask = 0;
    const uint32_t activeJointMask =
        sharedAtomicLoad(&sharedData->active_joint_mask) &
        ((1UL << ENCODER_TOTAL_NUM) - 1UL);
    for (uint8_t jointIndex = 0;
         jointIndex < ENCODER_TOTAL_NUM;
         ++jointIndex) {
        if ((activeJointMask & (1UL << jointIndex)) == 0) {
            continue;
        }
        const uint8_t actuatorCount =
            getJointActuatorCount(jointIndex);
        for (uint8_t actuatorIndex = 0;
             actuatorIndex < actuatorCount;
             ++actuatorIndex) {
            JointActuatorItem actuator;
            if (getJointActuator(
                    jointIndex, actuatorIndex, &actuator) &&
                actuator.motorChannel < SERVO_TOTAL_NUM) {
                mask |= (1UL << actuator.motorChannel);
            }
        }
    }
    return mask & validMask;
}

bool jointHasOverloadFault(uint8_t jointIndex,
                           uint32_t overloadFaultBitmap)
{
    const uint8_t actuatorCount =
        getJointActuatorCount(jointIndex);
    for (uint8_t actuatorIndex = 0;
         actuatorIndex < actuatorCount;
         ++actuatorIndex) {
        JointActuatorItem actuator;
        if (getJointActuator(
                jointIndex, actuatorIndex, &actuator) &&
            actuator.motorChannel < SERVO_TOTAL_NUM &&
            (overloadFaultBitmap &
             (1UL << actuator.motorChannel)) != 0) {
            return true;
        }
    }
    return false;
}

bool directTargetsReached(
    TaskSharedData_t* sharedData,
    const ServoAngleData_t& servoData,
    const ControlCommandSnapshot_t& commandSnapshot)
{
    if (!sharedData ||
        commandSnapshot.controlMode != CONTROL_MODE_DIRECT_MOTOR ||
        commandSnapshot.motorDirectCommandSource !=
            MOTOR_DIRECT_SOURCE_ABSOLUTE) {
        return false;
    }

    // 只有 ABS 直控具有确定的“到位”语义；Sweep 是持续编辑，不自动判定完成。
    const uint32_t requiredMask =
        sharedAtomicLoad(&sharedData->active_motor_mask) &
        validMotorChannelMask();
    if (requiredMask == 0) {
        return false;
    }

    bool checkedAny = false;
    for (uint8_t channel = 0;
         channel < SERVO_TOTAL_NUM;
         ++channel) {
        if ((requiredMask & (1UL << channel)) == 0) {
            continue;
        }
        checkedAny = true;
        if (servoData.onlineStatus[channel] == 0) {
            return false;
        }
        const int32_t targetAbs =
            static_cast<int32_t>(
                clampServoPos(
                    commandSnapshot.motorTargetRaw[channel]));
        const int32_t error =
            servoData.servoAngles[channel] - targetAbs;
        if ((error < 0 ? -error : error) >
            kDirectDoneAbsToleranceCounts) {
            return false;
        }
    }
    return checkedAny;
}

bool feedbackTargetsReached(
    TaskSharedData_t* sharedData,
    const MappedAngleData_t& mappedData,
    const float* targetDegs,
    const float* actualDegs,
    uint32_t commandJointMask)
{
    if (!sharedData || !targetDegs || !actualDegs ||
        !mappedData.isValid) {
        return false;
    }

    // 使用本帧 command mask 与 active scope 的交集，未参与本次命令的关节不阻塞 DONE。
    const uint32_t requiredMask =
        commandJointMask &
        sharedAtomicLoad(&sharedData->active_joint_mask) &
        ((1UL << ENCODER_TOTAL_NUM) - 1UL);
    if (requiredMask == 0) {
        return false;
    }

    bool checkedAny = false;
    for (uint8_t jointIndex = 0;
         jointIndex < ENCODER_TOTAL_NUM;
         ++jointIndex) {
        if ((requiredMask & (1UL << jointIndex)) == 0) {
            continue;
        }
        checkedAny = true;
        if (mappedData.validFlags[jointIndex] == 0 ||
            fabsf(actualDegs[jointIndex] -
                  targetDegs[jointIndex]) >
                kFeedbackDoneToleranceDeg) {
            return false;
        }
    }
    return checkedAny;
}

} // namespace p4::control
