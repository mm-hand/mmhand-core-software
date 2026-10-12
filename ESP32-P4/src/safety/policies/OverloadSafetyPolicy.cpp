/**
 * @file OverloadSafetyPolicy.cpp
 * @brief 实现过载安全规则；所有阈值统一读取 ControlConfig。
 */
#include "OverloadSafetyPolicy.h"

#include "../../config/ControlConfig.h"
#include "../../config/topology/HardwareMap.h"
#include "../../hal/servo/ServoValue.h"
#include "../../system/rtos/SharedSafetyState.h"

namespace p4::safety {

using namespace mmhand::config::control;

uint32_t overloadActiveMotorMask(SystemRuntimeContext* runtime,
                                 const ControlCommandSnapshot_t& command)
{
    if (!runtime) {
        return 0;
    }
    const uint32_t validMask = validMotorChannelMask();
    if (command.controlMode == CONTROL_MODE_DIRECT_MOTOR) {
        return command.directMotorMask & validMask;
    }

    uint32_t motorMask = 0;
    const uint32_t jointMask = sharedAtomicLoad(&runtime->active_joint_mask) &
        ((1UL << ENCODER_TOTAL_NUM) - 1UL);
    for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; ++joint) {
        if ((jointMask & (1UL << joint)) == 0) {
            continue;
        }
        const uint8_t count = getJointActuatorCount(joint);
        for (uint8_t index = 0; index < count; ++index) {
            JointActuatorItem actuator = {};
            if (getJointActuator(joint, index, &actuator) &&
                actuator.motorChannel < SERVO_TOTAL_NUM) {
                motorMask |= 1UL << actuator.motorChannel;
            }
        }
    }
    return motorMask & validMask;
}

OverloadSampleDecision evaluateOverloadSample(uint8_t controlMode,
                                              int16_t signedLoad,
                                              uint8_t previousHighCount)
{
    OverloadSampleDecision decision = {};
    const bool feedbackMode = controlMode == CONTROL_MODE_JOINT;
    decision.threshold = feedbackMode ? kFeedbackOverloadLoadThreshold
                                      : kDirectOverloadLoadThreshold;
    decision.latchCycles = feedbackMode ? kFeedbackOverloadLatchCycles
                                        : kDirectOverloadLatchCycles;
    decision.thresholdExceeded =
        mmhand::servo::loadMagnitude(signedLoad) >= decision.threshold;
    decision.nextHighCount = decision.thresholdExceeded
        ? (previousHighCount < 255 ? previousHighCount + 1 : 255)
        : 0;
    decision.shouldLatch = decision.nextHighCount >= decision.latchCycles;
    return decision;
}

bool jointHasOverloadFault(uint8_t jointIndex, uint32_t overloadFaultBitmap)
{
    const uint8_t count = getJointActuatorCount(jointIndex);
    for (uint8_t index = 0; index < count; ++index) {
        JointActuatorItem actuator = {};
        if (getJointActuator(jointIndex, index, &actuator) &&
            actuator.motorChannel < SERVO_TOTAL_NUM &&
            (overloadFaultBitmap & (1UL << actuator.motorChannel)) != 0) {
            return true;
        }
    }
    return false;
}

} // namespace p4::safety
