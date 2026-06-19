#include "HardwareMap.h"

JointMapItem jointMap[ENCODER_TOTAL_NUM] = {
    {0, 1}, {0, 2}, {0, 3}, {0, 4}, {0, 5}, {0, 6}, {0, 7}, {0, 8}, {0, 9},
    {1, 1}, {1, 2}, {1, 3}, {1, 4}, {1, 5}, {1, 6}, {1, 7}, {1, 8}, {1, 9},
    {2, 1}, {2, 2}, {2, 3}
};

MotorMapItem motorMap[SERVO_TOTAL_NUM] = {
    {0, 1}, {0, 2}, {0, 3}, {0, 4}, {0, 5}, {0, 6}, {0, 7}, {0, 8}, {0, 9},
    {1, 1}, {1, 2}, {1, 3}, {1, 4}, {1, 5}, {1, 6}, {1, 7}, {1, 8}, {1, 9},
    {2, 1}, {2, 2}, {2, 3}, {2, 4}, {2, 5}, {2, 6}, {2, 7}, {2, 8}, {2, 9}
};

#define ACT_NONE {0xFF, 1, 0, 0}
#define ACT_PRIMARY(ch) {ch, 1, 0, 0}
#define GROUP1(ch) {1, {ACT_PRIMARY(ch), ACT_NONE}}

JointActuatorGroup jointActuatorGroups[ENCODER_TOTAL_NUM] = {
    GROUP1(0),
    GROUP1(1),
    GROUP1(2),
    GROUP1(3),
    GROUP1(4),
    GROUP1(5),
    GROUP1(6),
    GROUP1(7),
    GROUP1(8),
    GROUP1(9),
    GROUP1(10),
    GROUP1(11),
    GROUP1(12),
    GROUP1(13),
    GROUP1(14),
    GROUP1(15),
    GROUP1(16),
    GROUP1(17),
    GROUP1(18),
    GROUP1(19),
    GROUP1(20)
};

#undef GROUP1
#undef ACT_PRIMARY
#undef ACT_NONE

bool isMotorMapItemValid(const MotorMapItem *item)
{
    return (item != nullptr) &&
           (item->busIndex < NUM_BUSES) &&
           (item->servoID != 0U) &&
           (item->servoID <= MAX_SERVO_ID);
}

bool isMotorChannelValid(uint8_t motorChannel)
{
    return (motorChannel < SERVO_TOTAL_NUM) && isMotorMapItemValid(&motorMap[motorChannel]);
}

uint32_t validMotorChannelMask(void)
{
    uint32_t mask = 0U;
    for (uint8_t ch = 0U; ch < SERVO_TOTAL_NUM; ch++) {
        if (isMotorChannelValid(ch)) {
            mask |= (1UL << ch);
        }
    }
    return mask;
}

uint8_t getJointActuatorCount(uint8_t jointIndex)
{
    if (jointIndex >= ENCODER_TOTAL_NUM) {
        return 0U;
    }
    uint8_t count = jointActuatorGroups[jointIndex].count;
    if (count > MAX_JOINT_ACTUATORS) {
        count = MAX_JOINT_ACTUATORS;
    }
    return count;
}

bool getJointActuator(uint8_t jointIndex, uint8_t actuatorIndex, JointActuatorItem *out)
{
    if ((out == nullptr) ||
        (jointIndex >= ENCODER_TOTAL_NUM) ||
        (actuatorIndex >= getJointActuatorCount(jointIndex))) {
        return false;
    }

    const JointActuatorItem item = jointActuatorGroups[jointIndex].actuators[actuatorIndex];
    if (!isMotorChannelValid(item.motorChannel)) {
        return false;
    }
    *out = item;
    return true;
}

bool getJointActuatorGroup(uint8_t jointIndex, JointActuatorGroup *out)
{
    if ((out == nullptr) || (jointIndex >= ENCODER_TOTAL_NUM)) {
        return false;
    }

    out->count = 0U;
    for (uint8_t i = 0U; i < MAX_JOINT_ACTUATORS; i++) {
        out->actuators[i].motorChannel = 0xFFU;
        out->actuators[i].sign = 1;
        out->actuators[i].offset = 0;
        out->actuators[i].tensionBias = 0;
    }

    const uint8_t configuredCount = getJointActuatorCount(jointIndex);
    for (uint8_t i = 0U; (i < configuredCount) && (out->count < MAX_JOINT_ACTUATORS); i++) {
        JointActuatorItem item = {};
        if (getJointActuator(jointIndex, i, &item)) {
            out->actuators[out->count++] = item;
        }
    }
    return out->count > 0U;
}

int findPrimaryMotorChannelForJoint(uint8_t jointIndex)
{
    JointActuatorItem item = {};
    if (!getJointActuator(jointIndex, 0U, &item)) {
        return -1;
    }
    return static_cast<int>(item.motorChannel);
}

int findMotorChannelByBusId(uint8_t busIndex, uint8_t servoID)
{
    for (uint8_t ch = 0U; ch < SERVO_TOTAL_NUM; ch++) {
        if (isMotorChannelValid(ch) &&
            motorMap[ch].busIndex == busIndex &&
            motorMap[ch].servoID == servoID) {
            return static_cast<int>(ch);
        }
    }
    return -1;
}
