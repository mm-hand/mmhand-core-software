#include "HardwareMap.h"

static const MotorMapItem kInvalidMotor = {MOTOR_MAP_INVALID_BUS, 0};

#if HAND_PROFILE == HAND_PROFILE_V2
static const MotorMapItem kExtraV2Motors[5] = {
    {0, 23}, {0, 24}, {1, 25}, {2, 26}, {3, 23}
};
#else
static const MotorMapItem kExtraV2Motors[5] = {
    kInvalidMotor, kInvalidMotor, kInvalidMotor, kInvalidMotor, kInvalidMotor
};
#endif

// Primary joint -> servo map. Keep this table for legacy code paths and for the
// primary actuator of each joint. Multi-actuator ownership lives in
// jointActuatorGroups below.
JointMapItem jointMap[ENCODER_TOTAL_NUM] = {
    {0, 1},  {0, 2},  {0, 3},  {0, 4},
    {0, 5},  {0, 6},  {1, 7},  {2, 8},
    {1, 9},  {1, 10}, {1, 11}, {1, 12},
    {2, 13}, {2, 14}, {2, 15}, {2, 16},
    {3, 17}, {3, 19}, {3, 20}, {3, 21}, {3, 22}
};

// Motor channel -> physical bus/id. M22-M26 are reserved for Hand 2.0.
MotorMapItem motorMap[SERVO_TOTAL_NUM] = {
    {0, 1},  {0, 2},  {0, 3},  {0, 4},
    {0, 5},  {0, 6},  {1, 7},  {2, 8},
    {1, 9},  {1, 10}, {1, 11}, {1, 12},
    {2, 13}, {2, 14}, {2, 15}, {2, 16},
    {3, 17}, {3, 18}, {3, 19}, {3, 20},
    {3, 21}, {3, 22},
    kExtraV2Motors[0], kExtraV2Motors[1], kExtraV2Motors[2],
    kExtraV2Motors[3], kExtraV2Motors[4]
};

// Motor channel -> tendon shortening direction in servo absolute counts.
// +1: increasing ABS shortens the tendon.
// -1: decreasing ABS shortens the tendon.
static const int8_t kMotorTendonShortenAbsSign[SERVO_TOTAL_NUM] = {
    1, 1, 1, 1,
    1, 1, 1, 1,
    1, 1, 1, 1,
   -1, 1, -1, 1,
    1, 1, 1, 1,
    1, 1,
    1, 1, 1, -1, 1
};

#define ACT_NONE {0xFF, 1, 0, 0}
#define ACT_PRIMARY(ch) {ch, 1, 0, 0}
#define ACT_SECONDARY_SAME(ch) {ch, 1, 0, 0}
#define ACT_SECONDARY_OPPOSE(ch) {ch, -1, 0, 0}
#define GROUP1(ch) {1, {ACT_PRIMARY(ch), ACT_NONE}}
#define GROUP2(primaryCh, secondaryAct) {2, {ACT_PRIMARY(primaryCh), secondaryAct}}

JointActuatorGroup jointActuatorGroups[ENCODER_TOTAL_NUM] = {
#if HAND_PROFILE == HAND_PROFILE_V2
    GROUP2(0, ACT_SECONDARY_OPPOSE(22)),
#else
    GROUP1(0),
#endif
    GROUP1(1),
    GROUP1(2),
    GROUP1(3),
#if HAND_PROFILE == HAND_PROFILE_V2
    GROUP2(4, ACT_SECONDARY_OPPOSE(23)),
#else
    GROUP1(4),
#endif
    GROUP1(5),
    GROUP1(6),
    GROUP1(7),
#if HAND_PROFILE == HAND_PROFILE_V2
    GROUP2(8, ACT_SECONDARY_OPPOSE(24)),
#else
    GROUP1(8),
#endif
    GROUP1(9),
    GROUP1(10),
    GROUP1(11),
#if HAND_PROFILE == HAND_PROFILE_V2
    GROUP2(12, ACT_SECONDARY_OPPOSE(25)),
#else
    GROUP1(12),
#endif
    GROUP1(13),
    GROUP1(14),
    GROUP1(15),
    GROUP2(16, ACT_SECONDARY_OPPOSE(17)),
#if HAND_PROFILE == HAND_PROFILE_V2
    GROUP2(18, ACT_SECONDARY_OPPOSE(26)),
#else
    GROUP1(18),
#endif
    GROUP1(19),
    GROUP1(20),
    GROUP1(21)
};

#undef GROUP2
#undef GROUP1
#undef ACT_SECONDARY_OPPOSE
#undef ACT_SECONDARY_SAME
#undef ACT_PRIMARY
#undef ACT_NONE

bool isMotorMapItemValid(const MotorMapItem& item)
{
    return item.busIndex < NUM_BUSES && item.servoID != 0 && item.servoID <= MAX_SERVO_ID;
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
        out->actuators[i].offset = 0;
        out->actuators[i].tensionBias = 0;
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

int findMotorChannelByBusId(uint8_t busIndex, uint8_t servoID)
{
    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        if (isMotorChannelValid(ch) &&
            motorMap[ch].busIndex == busIndex &&
            motorMap[ch].servoID == servoID) {
            return (int)ch;
        }
    }
    return -1;
}
