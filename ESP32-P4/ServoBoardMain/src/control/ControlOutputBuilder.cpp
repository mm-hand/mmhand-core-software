/**
 * @file ControlOutputBuilder.cpp
 * @brief 把关节控制结果组装为统一舵机目标批次。
 */
#include "ControlOutputBuilder.h"
#include "../hardware/HardwareMap.h"
#include "../shared/SharedSafetyState.h"

static const int32_t kServoSingleTurnModulo = 4096;
static const int32_t kServoSingleTurnHalf = kServoSingleTurnModulo / 2;

// 统一保留 logical ABS 的 int32 坐标；真实硬件范围由逐电机动态边界处理。
int32_t clampServoPos(int32_t value)
{
    // P4 内部逻辑 ABS 使用 int32_t，不再在控制层按固定七圈截断。
    return value;
}

int32_t clampServoPosByMotorAbsLimit(TaskSharedData_t* sharedData,
                                    uint8_t motorChannel,
                                    int32_t value)
{
    // 限位数组由标定和状态机代码共享，先在安全保护下拷贝，
    // 再在本地执行限制。
    int32_t clamped = value;
    if (sharedData && motorChannel < SERVO_TOTAL_NUM) {
        const uint32_t bit = (1UL << motorChannel);
        uint32_t validBitmap = 0;
        uint32_t lockedBitmap = 0;
        int32_t lo = 0;
        int32_t hi = 0;
        {
            SharedSafetyGuard guard;
            validBitmap = sharedAtomicLoad(&sharedData->motor_abs_limit_valid_bitmap);
            lockedBitmap = sharedAtomicLoad(&sharedData->motor_abs_limit_locked_bitmap);
            lo = sharedData->motor_abs_limit_min[motorChannel];
            hi = sharedData->motor_abs_limit_max[motorChannel];
        }
        if (((validBitmap & bit) != 0) && ((lockedBitmap & bit) != 0)) {
            if (lo > hi) {
                const int32_t tmp = lo;
                lo = hi;
                hi = tmp;
            }
            if (clamped < lo) clamped = lo;
            if (clamped > hi) clamped = hi;
        }
    }
    // LOCK/LUT 限位之后，再限制到该台舵机当前上电会话的动态逻辑行程。
    if (sharedData && motorChannel < SERVO_TOTAL_NUM) {
        const uint32_t bit = (1UL << motorChannel);
        uint32_t rangeValidBitmap = 0;
        int32_t sessionLo = 0;
        int32_t sessionHi = 0;
        {
            SharedSafetyGuard guard;
            rangeValidBitmap = sharedAtomicLoad(
                &sharedData->servo_logical_range_valid_bitmap);
            sessionLo = sharedData->servo_logical_range_min[motorChannel];
            sessionHi = sharedData->servo_logical_range_max[motorChannel];
        }
        if ((rangeValidBitmap & bit) != 0 && sessionLo <= sessionHi) {
            if (clamped < sessionLo) clamped = sessionLo;
            if (clamped > sessionHi) clamped = sessionHi;
        }
    }
    return clamped;
}

// 将单圈 raw 舵机位置限制到 0..4095。
int16_t clampServoSingleTurnRaw(int32_t value)
{
    if (value > 4095) return 4095;
    if (value < 0) return 0;
    return (int16_t)value;
}

// 将单圈目标展开为最近的等价多圈位置。
int32_t expandSingleTurnTargetNearCurrent(int32_t currentAbsPos, int16_t singleTurnRaw)
{
    const int32_t baseTarget = (int32_t)clampServoSingleTurnRaw(singleTurnRaw);
    int32_t bestTarget = baseTarget;
    int32_t bestDistance = 0x7FFFFFFF;

    // 检查当前圈以及前后一圈；在这个小窗口内，单圈目标
    // 有且只有一个距离当前反馈最近的等价位置。
    const int32_t currentTurn = currentAbsPos / kServoSingleTurnModulo;
    for (int32_t turnOffset = -1; turnOffset <= 1; ++turnOffset)
    {
        const int32_t candidate = baseTarget + (currentTurn + turnOffset) * kServoSingleTurnModulo;
        const int32_t distance = (candidate >= currentAbsPos)
            ? (candidate - currentAbsPos)
            : (currentAbsPos - candidate);
        if (distance < bestDistance)
        {
            bestDistance = distance;
            bestTarget = candidate;
        }
    }

    return bestTarget;
}

// 将 sweep 模式锚定到当前多圈位置。
int32_t anchorSingleTurnSweepToCurrent(int32_t currentAbsPos, int16_t singleTurnRaw)
{
    return (int32_t)expandSingleTurnTargetNearCurrent(currentAbsPos, singleTurnRaw);
}

// 将单圈 raw 差值归一化为最短路径。
int32_t wrapServoRawDelta(int32_t delta)
{
    while (delta > kServoSingleTurnHalf) delta -= kServoSingleTurnModulo;
    while (delta < -kServoSingleTurnHalf) delta += kServoSingleTurnModulo;
    return delta;
}

// 为一个逻辑关节目标生成物理电机目标。双腱拮抗关节需要
// 显式的双电机控制器，因此这个旧辅助函数只展开主执行器。
uint8_t buildJointActuatorRuntimeTargets(uint8_t jointIndex,
                                         int32_t primaryTargetAbs,
                                         JointActuatorRuntimeTarget* outTargets,
                                         uint8_t outCapacity)
{
    if (!outTargets || outCapacity == 0 || jointIndex >= ENCODER_TOTAL_NUM) {
        return 0;
    }

    uint8_t outCount = 0;
    const uint8_t actuatorCount = getJointActuatorCount(jointIndex);
    // 符号相反的双电机关节属于拮抗机构，第二个电机不能用
    // 简单 offset 镜像生成，因此交给双腱控制器处理。
    const bool dualAntagonist =
        actuatorCount >= 2 &&
        outCapacity > 0;
    int8_t primarySign = 1;
    int8_t secondarySign = 1;
    if (dualAntagonist) {
        JointActuatorItem primary;
        JointActuatorItem secondary;
        if (getJointActuator(jointIndex, 0, &primary) &&
            getJointActuator(jointIndex, 1, &secondary)) {
            primarySign = primary.sign;
            secondarySign = secondary.sign;
        }
    }
    const uint8_t maxActuatorCount =
        (actuatorCount >= 2 && primarySign != secondarySign) ? 1 : actuatorCount;
    for (uint8_t i = 0; i < maxActuatorCount && outCount < outCapacity; i++) {
        // HardwareMap 负责 bus/id/channel 映射；这里仅跳过
        // 无法安全寻址的条目。
        JointActuatorItem actuator;
        if (!getJointActuator(jointIndex, i, &actuator)) {
            continue;
        }
        if (actuator.motorChannel >= SERVO_TOTAL_NUM ||
            !isMotorChannelValid(actuator.motorChannel)) {
            continue;
        }

        JointActuatorRuntimeTarget& target = outTargets[outCount++];
        target.motorChannel = actuator.motorChannel;
        target.sign = actuator.sign;
        target.targetAbs = clampServoPos(primaryTargetAbs + actuator.offset);
    }
    return outCount;
}

// 向 batch 追加一个舵机目标。
bool appendServoTarget(ServoTargetBatch_t* batch,
                       uint8_t motorChannel,
                       int32_t position,
                       uint16_t speed,
                       uint8_t acc)
{
    // 舵机目标队列使用固定大小 batch，即使调用方传入过多执行器，
    // 也不能写出 SERVO_TARGET_BATCH_MAX。
    if (!batch || batch->count >= SERVO_TARGET_BATCH_MAX ||
        !isMotorChannelValid(motorChannel)) {
        return false;
    }
    ServoTargetCommand_t& cmd = batch->commands[batch->count++];
    cmd.motorChannel = motorChannel;
    cmd.position = position;
    cmd.speed = speed;
    cmd.acc = acc;
    return true;
}

uint8_t appendJointActuatorTargets(ServoTargetBatch_t* batch,
                                   uint8_t jointIndex,
                                   int32_t primaryTargetAbs,
                                   uint16_t speed,
                                   uint8_t acc)
{
    if (!batch) {
        return 0;
    }

    JointActuatorRuntimeTarget targets[MAX_JOINT_ACTUATORS];
    const uint8_t targetCount =
        buildJointActuatorRuntimeTargets(jointIndex, primaryTargetAbs, targets, MAX_JOINT_ACTUATORS);

    uint8_t appended = 0;
    for (uint8_t i = 0; i < targetCount; i++) {
        // 统计成功追加的数量，调用方可据此回退到保持当前位置。
        if (appendServoTarget(batch,
                              targets[i].motorChannel,
                              targets[i].targetAbs,
                              speed,
                              acc)) {
            appended++;
        }
    }
    return appended;
}

uint8_t appendJointActuatorRuntimeTargets(ServoTargetBatch_t* batch,
                                          const JointActuatorRuntimeTarget* targets,
                                          uint8_t targetCount,
                                          uint16_t speed,
                                          uint8_t acc)
{
    if (!batch || !targets) {
        return 0;
    }

    uint8_t appended = 0;
    for (uint8_t i = 0; i < targetCount; i++) {
        // 运行时目标已包含 offset 和方向相关决策。
        if (appendServoTarget(batch,
                              targets[i].motorChannel,
                              targets[i].targetAbs,
                              speed,
                              acc)) {
            appended++;
        }
    }
    return appended;
}
