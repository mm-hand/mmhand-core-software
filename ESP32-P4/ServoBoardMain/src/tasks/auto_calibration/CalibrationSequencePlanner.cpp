/**
 * @file CalibrationSequencePlanner.cpp
 * @brief 实现自动标定关节选择、执行顺序和执行器依赖规划。
 */
#include "CalibrationSequencePlanner.h"

#include "../../config/topology/HardwareMap.h"
#include "../../platform/rtos/SharedSafetyState.h"
#include "../../platform/log/FirmwareLog.h"

uint8_t autoCalibrationSequenceRank(uint8_t jointIndex)
{
    if (jointIndex < 16) return jointIndex;
    switch (jointIndex) {
        case 20: return 16;
        case 16: return 17;
        case 17: return 18;
        case 18: return 19;
        case 19: return 20;
        default: return jointIndex;
    }
}

uint32_t calibrationJointMotorMask(uint8_t jointIndex)
{
    if (jointIndex >= ENCODER_TOTAL_NUM) return 0;
    uint32_t motorMask = 0;
    JointActuatorGroup group = {};
    if (getJointActuatorGroup(jointIndex, &group)) {
        for (uint8_t i = 0; i < group.count; i++) {
            const uint8_t motor = group.actuators[i].motorChannel;
            if (motor < SERVO_TOTAL_NUM && isMotorChannelValid(motor)) {
                motorMask |= 1UL << motor;
            }
        }
    } else {
        const int motor = findPrimaryMotorChannelForJoint(jointIndex);
        if (motor >= 0 && motor < SERVO_TOTAL_NUM) motorMask |= 1UL << motor;
    }
    return motorMask & validMotorChannelMask();
}

uint32_t buildCalibrationJointMask(const uint8_t* joints, uint8_t jointCount)
{
    uint32_t mask = 0;
    if (!joints) return 0;
    for (uint8_t i = 0; i < jointCount; i++) {
        if (joints[i] < ENCODER_TOTAL_NUM) mask |= 1UL << joints[i];
    }
    return mask;
}

uint32_t buildCalibrationMotorMask(const uint8_t* joints, uint8_t jointCount)
{
    uint32_t mask = 0;
    if (!joints) return 0;
    for (uint8_t i = 0; i < jointCount; i++) {
        mask |= calibrationJointMotorMask(joints[i]);
    }
    return mask & validMotorChannelMask();
}

uint8_t buildAutoCalibrationJointListFromActiveScope(SystemRuntimeContext* sharedData,
                                                     uint8_t* outJoints,
                                                     uint8_t maxCount)
{
    if (!sharedData || !outJoints || maxCount == 0) return 0;
    uint32_t activeMask = sharedAtomicLoad(&sharedData->active_joint_mask) &
                          ((1UL << ENCODER_TOTAL_NUM) - 1UL);
    const uint32_t selectedMotorMask =
        sharedAtomicLoad(&sharedData->active_motor_mask) & validMotorChannelMask();

    if (activeMask == 0 && selectedMotorMask != 0) {
        for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; joint++) {
            if ((calibrationJointMotorMask(joint) & selectedMotorMask) != 0) {
                activeMask |= 1UL << joint;
            }
        }
        if (activeMask != 0) {
            FW_LOG_INFO_PRINTF(
                "[AUTO CALIB] ACTIVE_SCOPE inferred joints=0x%08lX from motor=0x%08lX\r\n",
                (unsigned long)activeMask,
                (unsigned long)selectedMotorMask);
        }
    }

    uint8_t count = 0;
    for (uint8_t joint = 0;
         joint < ENCODER_TOTAL_NUM && count < maxCount;
         joint++) {
        if ((activeMask & (1UL << joint)) != 0) outJoints[count++] = joint;
    }
    return count;
}
