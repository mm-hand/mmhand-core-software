#include "HardwareMap.h"

// Primary joint -> servo map. Keep this table for legacy code paths and for the
// primary actuator of each joint. Multi-actuator ownership lives in
// jointActuatorGroups below.
JointMapItem jointMap[ENCODER_TOTAL_NUM] = {
    {0, 0},  {0, 2},  {0, 3},  {0, 4},
    {0, 5},  {0, 7},  {1, 8},  {2, 9},
    {1, 10}, {1, 12}, {1, 13}, {1, 14},
    {2, 15}, {2, 17}, {2, 18}, {2, 19},
    {3, 20}, {3, 22}, {3, 23}, {3, 24}, {3, 25}
};

// Motor channel -> physical bus/id. Servo IDs are zero-based and match Mxx.
MotorMapItem motorMap[SERVO_TOTAL_NUM] = {
    {0, 0},  {0, 1},  {0, 2},  {0, 3},  {0, 4},
    {0, 5},  {0, 6},  {0, 7},  {1, 8},  {2, 9},
    {1, 10}, {1, 11}, {1, 12}, {1, 13}, {1, 14},
    {2, 15}, {2, 16}, {2, 17}, {2, 18}, {2, 19},
    {3, 20}, {3, 21}, {3, 22}, {3, 23}, {3, 24},
    {3, 25}, {3, 26}
};

// Motor channel -> tendon shortening direction in servo absolute counts.
// +1: increasing ABS shortens the tendon.
// -1: decreasing ABS shortens the tendon.
static constexpr int8_t kMotorTendonShortenAbsSign[] = {
    1, 1, 1, 1,
    1, 1, 1, 1,
    1, 1, 1, 1,
    1, 1, 1, 1,
    1, 1, 1, 1,
    1, 1, 1, 1,
    1, 1, 1
};

static_assert(
    sizeof(kMotorTendonShortenAbsSign) / sizeof(kMotorTendonShortenAbsSign[0]) == SERVO_TOTAL_NUM,
    "motor tendon sign count must match SERVO_TOTAL_NUM"
);

#define ACT_NONE {0xFF, 1, 0, 0}
#define ACT_PRIMARY(ch) {ch, 1, 0, 0}
#define ACT_SECONDARY_SAME(ch) {ch, 1, 0, 0}
#define ACT_SECONDARY_OPPOSE(ch) {ch, -1, 0, 0}
#define GROUP1(ch) {1, {ACT_PRIMARY(ch), ACT_NONE}}
#define GROUP2(primaryCh, secondaryAct) {2, {ACT_PRIMARY(primaryCh), secondaryAct}}

JointActuatorGroup jointActuatorGroups[ENCODER_TOTAL_NUM] = {
#if HAND_PROFILE == HAND_PROFILE_V2
    GROUP2(0, ACT_SECONDARY_OPPOSE(1)),
#else
    GROUP1(0),
#endif
    GROUP1(2),
    GROUP1(3),
    GROUP1(4),
#if HAND_PROFILE == HAND_PROFILE_V2
    GROUP2(5, ACT_SECONDARY_OPPOSE(6)),
#else
    GROUP1(5),
#endif
    GROUP1(7),
    GROUP1(8),
    GROUP1(9),
#if HAND_PROFILE == HAND_PROFILE_V2
    GROUP2(10, ACT_SECONDARY_OPPOSE(11)),
#else
    GROUP1(10),
#endif
    GROUP1(12),
    GROUP1(13),
    GROUP1(14),
#if HAND_PROFILE == HAND_PROFILE_V2
    GROUP2(15, ACT_SECONDARY_OPPOSE(16)),
#else
    GROUP1(15),
#endif
    GROUP1(17),
    GROUP1(18),
    GROUP1(19),
    // Thumb mapping: J16 -> M20/M21, J17 -> M22, J18 -> M23,
    // J19 -> M24, J20 -> M25/M26.
    GROUP2(20, ACT_SECONDARY_OPPOSE(21)),
    GROUP1(22),
    GROUP1(23),
    GROUP1(24),
    GROUP2(25, ACT_SECONDARY_OPPOSE(26))
};

#undef GROUP2
#undef GROUP1
#undef ACT_SECONDARY_OPPOSE
#undef ACT_SECONDARY_SAME
#undef ACT_PRIMARY
#undef ACT_NONE

bool isMotorMapItemValid(const MotorMapItem& item)
{
    return item.busIndex < NUM_BUSES && item.servoID <= MAX_SERVO_ID;
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
