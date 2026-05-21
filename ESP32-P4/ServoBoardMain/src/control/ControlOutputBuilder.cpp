#include "ControlOutputBuilder.h"
#include "../hardware/HardwareMap.h"

static const int32_t kServoSingleTurnModulo = 4096;
static const int32_t kServoSingleTurnHalf = kServoSingleTurnModulo / 2;

// Clamp to the signed multi-turn range accepted by the servo protocol.
int16_t clampServoPos(int32_t value)
{
    if (value > 30719) return 30719;
    if (value < -30719) return -30719;
    return (int16_t)value;
}

int16_t clampServoPosByMotorAbsLimit(TaskSharedData_t* sharedData,
                                     uint8_t motorChannel,
                                     int32_t value)
{
    int32_t clamped = value;
    if (sharedData && motorChannel < SERVO_TOTAL_NUM) {
        const uint32_t bit = (1UL << motorChannel);
        if (((sharedData->motor_abs_limit_valid_bitmap & bit) != 0) &&
            ((sharedData->motor_abs_limit_locked_bitmap & bit) != 0)) {
            int32_t lo = sharedData->motor_abs_limit_min[motorChannel];
            int32_t hi = sharedData->motor_abs_limit_max[motorChannel];
            if (lo > hi) {
                const int32_t tmp = lo;
                lo = hi;
                hi = tmp;
            }
            if (clamped < lo) clamped = lo;
            if (clamped > hi) clamped = hi;
        }
    }
    return clampServoPos(clamped);
}

// Clamp a single-turn raw servo position to 0..4095.
int16_t clampServoSingleTurnRaw(int32_t value)
{
    if (value > 4095) return 4095;
    if (value < 0) return 0;
    return (int16_t)value;
}

// Expand a single-turn target to the nearest equivalent multi-turn position.
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

// Anchor sweep mode to the current multi-turn position.
int32_t anchorSingleTurnSweepToCurrent(int32_t currentAbsPos, int16_t singleTurnRaw)
{
    return (int32_t)expandSingleTurnTargetNearCurrent(currentAbsPos, singleTurnRaw);
}

// Normalize single-turn raw delta to the shortest path.
int32_t wrapServoRawDelta(int32_t delta)
{
    while (delta > kServoSingleTurnHalf) delta -= kServoSingleTurnModulo;
    while (delta < -kServoSingleTurnHalf) delta += kServoSingleTurnModulo;
    return delta;
}

// Build physical motor targets for one logical joint target. Dual-tendon
// antagonist joints need an explicit two-motor controller, so this legacy helper
// only expands the primary actuator for them.
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
        JointActuatorItem actuator;
        if (!getJointActuator(jointIndex, i, &actuator)) {
            continue;
        }
        if (actuator.motorChannel >= SERVO_TOTAL_NUM ||
            !isMotorChannelValid(actuator.motorChannel)) {
            continue;
        }

        const MotorMapItem& motor = motorMap[actuator.motorChannel];

        JointActuatorRuntimeTarget& target = outTargets[outCount++];
        target.motorChannel = actuator.motorChannel;
        target.busIndex = motor.busIndex;
        target.servoId = motor.servoID;
        target.sign = actuator.sign;
        target.targetAbs = clampServoPos(primaryTargetAbs + actuator.offset);
    }
    return outCount;
}

// Append one servo target to a batch.
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
        if (appendServoTarget(batch,
                              targets[i].busIndex,
                              targets[i].servoId,
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
        if (appendServoTarget(batch,
                              targets[i].busIndex,
                              targets[i].servoId,
                              targets[i].targetAbs,
                              speed,
                              acc)) {
            appended++;
        }
    }
    return appended;
}
