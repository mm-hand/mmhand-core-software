/**
 * @file HardwareMap.cpp
 * @brief 实现 21 关节、27 舵机身份及双腱执行器映射。
 */
#include "HardwareMap.h"

// 关节到主舵机映射供单腱查询使用；双腱完整所有权见 jointActuatorGroups。
JointMapItem jointMap[ENCODER_TOTAL_NUM] = {
    {0},  {2},  {3},  {4},
    {5},  {7},  {8},  {9},
    {10}, {12}, {13}, {14},
    {15}, {17}, {18}, {19},
    {20}, {22}, {23}, {24}, {25}
};

// Mxx 到全手唯一舵机 ID 的身份映射；物理 BUS 不属于静态拓扑。
MotorMapItem motorMap[SERVO_TOTAL_NUM] = {
    {0},  {1},  {2},  {3},  {4},  {5},  {6},  {7},  {8},
    {9},  {10}, {11}, {12}, {13}, {14}, {15}, {16}, {17},
    {18}, {19}, {20}, {21}, {22}, {23}, {24}, {25}, {26}
};

// 舵机 ABS 到腱绳收紧坐标的方向：+1 表示 ABS 增大收绳，-1 表示减小收绳。
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
#define ACT_SECONDARY_OPPOSE(ch) {ch, -1, 0, 0}
#define GROUP1(ch) {1, {ACT_PRIMARY(ch), ACT_NONE}}
#define GROUP2(primaryCh, secondaryAct) {2, {ACT_PRIMARY(primaryCh), secondaryAct}}

JointActuatorGroup jointActuatorGroups[ENCODER_TOTAL_NUM] = {
    GROUP2(0, ACT_SECONDARY_OPPOSE(1)),
    GROUP1(2),
    GROUP1(3),
    GROUP1(4),
    GROUP2(5, ACT_SECONDARY_OPPOSE(6)),
    GROUP1(7),
    GROUP1(8),
    GROUP1(9),
    GROUP2(10, ACT_SECONDARY_OPPOSE(11)),
    GROUP1(12),
    GROUP1(13),
    GROUP1(14),
    GROUP2(15, ACT_SECONDARY_OPPOSE(16)),
    GROUP1(17),
    GROUP1(18),
    GROUP1(19),
    // 拇指映射：J16->M20/M21，J17->M22，J18->M23，J19->M24，J20->M25/M26。
    GROUP2(20, ACT_SECONDARY_OPPOSE(21)),
    GROUP1(22),
    GROUP1(23),
    GROUP1(24),
    GROUP2(25, ACT_SECONDARY_OPPOSE(26))
};

#undef GROUP2
#undef GROUP1
#undef ACT_SECONDARY_OPPOSE
#undef ACT_PRIMARY
#undef ACT_NONE

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

int findMotorChannelByServoId(uint8_t servoID)
{
    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        if (isMotorChannelValid(ch) && motorMap[ch].servoID == servoID) {
            return (int)ch;
        }
    }
    return -1;
}
