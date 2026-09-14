/**
 * @file ServoTargetBuilder.cpp
 * @brief 把关节控制结果组装为统一舵机目标批次。
 */
#include "ServoTargetBuilder.h"
#include "../../config/topology/HardwareMap.h"
#include "../../platform/rtos/SharedSafetyState.h"

int32_t clampServoPosToSessionRange(SystemRuntimeContext* sharedData,
                                    uint8_t motorChannel,
                                    int32_t value)
{
    int32_t clamped = value;
    // LUT 是负反馈工作区的唯一业务边界；这里仅保留掉电恢复后重建的
    // 会话级 logical ABS 硬边界，防止向舵机发送当前会话不可达的圈数。
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

// 向 batch 追加一个舵机目标。
bool appendServoTarget(ServoTargetBatch_t* batch,
                       uint8_t motorChannel,
                       int32_t position,
                       uint16_t speed,
                       uint8_t acc)
{
    // 舵机目标队列使用固定大小 batch，即使调用方传入过多执行器，
    // 也不能写出 SERVO_TARGET_BATCH_MAX。
    if (!batch) {
        return false;
    }
    if (batch->buildFailed != 0 ||
        batch->count >= SERVO_TARGET_BATCH_MAX ||
        !isMotorChannelValid(motorChannel)) {
        // 批次采用事务语义：一条命令构建失败后，之前已追加的命令也不得单独发布。
        batch->buildFailed = 1;
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

    if (jointIndex >= ENCODER_TOTAL_NUM) {
        batch->buildFailed = 1;
        return 0;
    }

    const uint8_t actuatorCount = getJointActuatorCount(jointIndex);
    int8_t primarySign = 1;
    int8_t secondarySign = 1;
    if (actuatorCount >= 2) {
        JointActuatorItem primary;
        JointActuatorItem secondary;
        if (getJointActuator(jointIndex, 0, &primary) &&
            getJointActuator(jointIndex, 1, &secondary)) {
            primarySign = primary.sign;
            secondarySign = secondary.sign;
        }
    }
    // 符号相反的双腱关节由专用控制器生成两路目标，通用链路只追加主执行器。
    const uint8_t appendCount =
        (actuatorCount >= 2 && primarySign != secondarySign) ? 1 : actuatorCount;
    uint8_t appended = 0;
    for (uint8_t i = 0; i < appendCount; ++i) {
        JointActuatorItem actuator;
        if (!getJointActuator(jointIndex, i, &actuator) ||
            !isMotorChannelValid(actuator.motorChannel)) {
            batch->buildFailed = 1;
            return 0;
        }
        if (appendServoTarget(batch, actuator.motorChannel,
                              primaryTargetAbs + actuator.offset, speed, acc)) {
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
        if (batch) {
            batch->buildFailed = 1;
        }
        return 0;
    }

    uint8_t appended = 0;
    for (uint8_t i = 0; i < targetCount; i++) {
        // 运行时目标已包含 offset 和方向相关决策。
        if (!appendServoTarget(batch,
                               targets[i].motorChannel,
                               targets[i].targetAbs,
                               speed,
                               acc)) {
            return 0;
        }
        appended++;
    }
    return appended;
}
