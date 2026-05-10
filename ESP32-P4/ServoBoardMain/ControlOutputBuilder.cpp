#include "ControlOutputBuilder.h"

static const int32_t kServoSingleTurnModulo = 4096;
static const int32_t kServoSingleTurnHalf = kServoSingleTurnModulo / 2;

int16_t clampServoPos(int32_t value)
{
    if (value > 30719) return 30719;
    if (value < -30719) return -30719;
    return (int16_t)value;
}

int16_t clampServoSingleTurnRaw(int32_t value)
{
    if (value > 4095) return 4095;
    if (value < 0) return 0;
    return (int16_t)value;
}

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

int32_t anchorSingleTurnSweepToCurrent(int32_t currentAbsPos, int16_t singleTurnRaw)
{
    return (int32_t)expandSingleTurnTargetNearCurrent(currentAbsPos, singleTurnRaw);
}

int32_t wrapServoRawDelta(int32_t delta)
{
    while (delta > kServoSingleTurnHalf) delta -= kServoSingleTurnModulo;
    while (delta < -kServoSingleTurnHalf) delta += kServoSingleTurnModulo;
    return delta;
}

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
