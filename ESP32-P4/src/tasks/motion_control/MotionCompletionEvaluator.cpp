/**
 * @file MotionCompletionEvaluator.cpp
 * @brief 实现直控 ABS 与关节角命令的完成窗口判断。
 */
#include "MotionCompletionEvaluator.h"

#include <math.h>

#include "../../config/ControlConfig.h"
#include "../../config/topology/HardwareMap.h"
#include "../../system/rtos/SharedSafetyState.h"
#include "ServoTargetBuilder.h"

namespace p4::control {

using namespace mmhand::config::control;

bool directTargetsReached(SystemRuntimeContext* runtime,
                          const ServoAngleData_t& servoData,
                          const ControlCommandSnapshot_t& command)
{
    if (!runtime || command.controlMode != CONTROL_MODE_DIRECT_MOTOR ||
        command.motorDirectCommandSource != MOTOR_DIRECT_SOURCE_ABSOLUTE) {
        return false;
    }

    const uint32_t requiredMask =
        command.directMotorMask & validMotorChannelMask();
    if (requiredMask == 0) {
        return false;
    }

    bool checkedAny = false;
    for (uint8_t motor = 0; motor < SERVO_TOTAL_NUM; ++motor) {
        const uint32_t bit = 1UL << motor;
        if ((requiredMask & bit) == 0) {
            continue;
        }
        checkedAny = true;
        if (servoData.onlineStatus[motor] == 0) {
            return false;
        }
        const int32_t targetAbs = command.motorTargetRaw[motor];
        const int32_t error = servoData.servoAngles[motor] - targetAbs;
        if ((error < 0 ? -error : error) > kDirectDoneAbsToleranceCounts) {
            return false;
        }
    }
    return checkedAny;
}

bool feedbackTargetsReached(SystemRuntimeContext* runtime,
                            const MappedAngleData_t& mappedData,
                            const float* targetDegs,
                            const float* actualDegs,
                            uint32_t commandJointMask)
{
    if (!runtime || !targetDegs || !actualDegs || !mappedData.isValid) {
        return false;
    }

    const uint32_t requiredMask =
        commandJointMask & sharedAtomicLoad(&runtime->active_joint_mask) &
        ((1UL << ENCODER_TOTAL_NUM) - 1UL);
    if (requiredMask == 0) {
        return false;
    }

    bool checkedAny = false;
    for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; ++joint) {
        if ((requiredMask & (1UL << joint)) == 0) {
            continue;
        }
        checkedAny = true;
        if (mappedData.validFlags[joint] == 0 ||
            fabsf(actualDegs[joint] - targetDegs[joint]) >
                kFeedbackDoneToleranceDeg) {
            return false;
        }
    }
    return checkedAny;
}

} // namespace p4::control
