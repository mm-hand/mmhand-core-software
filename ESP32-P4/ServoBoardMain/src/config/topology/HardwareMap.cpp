/**
 * @file HardwareMap.cpp
 * @brief 实现 21 关节、27 舵机身份及双腱执行器映射。
 */
#include "HardwareMap.h"

#include "../../generated/HardwareBindings.generated.h"

bool isMotorMapItemValid(const MotorMapItem& item)
{
    return item.servoID <= MAX_SERVO_ID;
}

bool isMotorChannelValid(uint8_t motorChannel)
{
    return motorChannel < SERVO_TOTAL_NUM && isMotorMapItemValid(motorMap[motorChannel]);
}

uint32_t validMotorChannelMask()
{
    uint32_t mask = 0;
    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        if (isMotorChannelValid(ch)) {
            mask |= (1UL << ch);
        }
    }
    return mask;
}

int8_t getMotorTendonShortenAbsSign(uint8_t motorChannel)
{
    if (motorChannel >= SERVO_TOTAL_NUM) {
        return 1;
    }
    return (kMotorTendonShortenAbsSign[motorChannel] < 0) ? -1 : 1;
}

int32_t motorRawAbsToTendonAbs(uint8_t motorChannel, int32_t rawAbs)
{
    return (int32_t)getMotorTendonShortenAbsSign(motorChannel) * rawAbs;
}

int32_t motorTendonAbsToRawAbs(uint8_t motorChannel, int32_t tendonAbs)
{
    return (int32_t)getMotorTendonShortenAbsSign(motorChannel) * tendonAbs;
}

int32_t motorRawDeltaFromTendonDelta(uint8_t motorChannel, int32_t tendonDelta)
{
    return (int32_t)getMotorTendonShortenAbsSign(motorChannel) * tendonDelta;
}

int32_t motorApplyTendonDelta(uint8_t motorChannel, int32_t rawAbs, int32_t tendonDelta)
{
    return rawAbs + motorRawDeltaFromTendonDelta(motorChannel, tendonDelta);
}

uint8_t getJointActuatorCount(uint8_t jointIndex)
{
    if (jointIndex >= ENCODER_TOTAL_NUM) {
        return 0;
    }
    uint8_t count = jointActuatorGroups[jointIndex].count;
    if (count > MAX_JOINT_ACTUATORS) {
        count = MAX_JOINT_ACTUATORS;
    }
    return count;
}

bool getJointActuator(uint8_t jointIndex, uint8_t actuatorIndex, JointActuatorItem* out)
{
    if (!out || jointIndex >= ENCODER_TOTAL_NUM || actuatorIndex >= getJointActuatorCount(jointIndex)) {
        return false;
    }
    const JointActuatorItem item = jointActuatorGroups[jointIndex].actuators[actuatorIndex];
    if (!isMotorChannelValid(item.motorChannel)) {
        return false;
    }
    *out = item;
    return true;
}

bool getJointActuatorGroup(uint8_t jointIndex, JointActuatorGroup* out)
{
    if (!out || jointIndex >= ENCODER_TOTAL_NUM) {
        return false;
    }

    out->count = 0;
    for (uint8_t i = 0; i < MAX_JOINT_ACTUATORS; i++) {
        out->actuators[i].motorChannel = 0xFF;
        out->actuators[i].sign = 1;
    }

    const uint8_t configuredCount = getJointActuatorCount(jointIndex);
    for (uint8_t i = 0; i < configuredCount && out->count < MAX_JOINT_ACTUATORS; i++) {
        JointActuatorItem item;
        if (getJointActuator(jointIndex, i, &item)) {
            out->actuators[out->count++] = item;
        }
    }
    return out->count > 0;
}

int findPrimaryMotorChannelForJoint(uint8_t jointIndex)
{
    JointActuatorItem item;
    if (!getJointActuator(jointIndex, 0, &item)) {
        return -1;
    }
    return (int)item.motorChannel;
}

int findMotorChannelByServoId(uint8_t servoID)
{
    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        if (isMotorChannelValid(ch) && motorMap[ch].servoID == servoID) {
            return (int)ch;
        }
    }
    return -1;
}
