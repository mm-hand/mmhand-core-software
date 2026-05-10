#include "ControlOutputBuilder.h"

static const int32_t kServoSingleTurnModulo = 4096;
static const int32_t kServoSingleTurnHalf = kServoSingleTurnModulo / 2;

// 飞特舵机扩展位置安全范围，避免下发超出固件/协议可接受的多圈值。
int16_t clampServoPos(int32_t value)
{
    if (value > 30719) return 30719;
    if (value < -30719) return -30719;
    return (int16_t)value;
}

// 单圈 raw 位置只允许 0..4095，主要用于上位机滑条输入。
int16_t clampServoSingleTurnRaw(int32_t value)
{
    if (value > 4095) return 4095;
    if (value < 0) return 0;
    return (int16_t)value;
}

// 将单圈目标展开到最接近当前多圈位置的等价位置，避免滑条跨 0/4095 时绕远路。
int16_t expandSingleTurnTargetNearCurrent(int32_t currentAbsPos, int16_t singleTurnRaw)
{
    const int32_t baseTarget = (int32_t)clampServoSingleTurnRaw(singleTurnRaw);
    int32_t bestTarget = baseTarget;
    int32_t bestDistance = 0x7FFFFFFF;

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

    return clampServoPos(bestTarget);
}

// sweep 模式第一次收到目标时以当前位置建立锚点，后续再按增量连续展开。
int32_t anchorSingleTurnSweepToCurrent(int32_t currentAbsPos, int16_t singleTurnRaw)
{
    return (int32_t)expandSingleTurnTargetNearCurrent(currentAbsPos, singleTurnRaw);
}

// 单圈 raw 增量归一化到半圈范围，保证 sweep 跨边界时走最短方向。
int32_t wrapServoRawDelta(int32_t delta)
{
    while (delta > kServoSingleTurnHalf) delta -= kServoSingleTurnModulo;
    while (delta < -kServoSingleTurnHalf) delta += kServoSingleTurnModulo;
    return delta;
}

// 向 batch 追加一个舵机目标。调用方负责设置 batch.source 和 timestamp。
bool appendServoTarget(ServoTargetBatch_t* batch,
                       uint8_t busIndex,
                       uint8_t servoId,
                       int16_t position,
                       uint16_t speed,
                       uint8_t acc)
{
    if (!batch || batch->count >= SERVO_TARGET_BATCH_MAX) {
        return false;
    }
    ServoTargetCommand_t& cmd = batch->commands[batch->count++];
    cmd.busIndex = busIndex;
    cmd.servoId = servoId;
    cmd.position = position;
    cmd.speed = speed;
    cmd.acc = acc;
    return true;
}
