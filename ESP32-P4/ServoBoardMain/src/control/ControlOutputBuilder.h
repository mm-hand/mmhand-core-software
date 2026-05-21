#ifndef CONTROL_OUTPUT_BUILDER_H
#define CONTROL_OUTPUT_BUILDER_H

#include <Arduino.h>
#include "../shared/TaskSharedData.h"

// Runtime target expanded from one logical joint command to one physical motor.
// Antagonist pairs are controlled by DualTendonControl and should provide both
// motor targets explicitly instead of deriving a mirrored secondary target.
struct JointActuatorRuntimeTarget {
    uint8_t motorChannel;
    uint8_t busIndex;
    uint8_t servoId;
    int8_t sign;
    int16_t targetAbs;
};

// ControlOutputBuilder turns controller outputs into servo-bus targets:
// - all absolute positions are clamped to the servo multi-turn command range;
// - single-turn raw targets can be expanded around a current absolute anchor;
// - joint actuator helpers hide V1/V2 one-motor/two-motor differences from PID.
int16_t clampServoPos(int32_t value);
int16_t clampServoPosByMotorAbsLimit(TaskSharedData_t* sharedData,
                                     uint8_t motorChannel,
                                     int32_t value);
int16_t clampServoSingleTurnRaw(int32_t value);
int16_t expandSingleTurnTargetNearCurrent(int32_t currentAbsPos, int16_t singleTurnRaw);
int32_t anchorSingleTurnSweepToCurrent(int32_t currentAbsPos, int16_t singleTurnRaw);
int32_t wrapServoRawDelta(int32_t delta);

uint8_t buildJointActuatorRuntimeTargets(uint8_t jointIndex,
                                         int32_t primaryTargetAbs,
                                         JointActuatorRuntimeTarget* outTargets,
                                         uint8_t outCapacity);

bool appendServoTarget(ServoTargetBatch_t* batch,
                       uint8_t busIndex,
                       uint8_t servoId,
                       int16_t position,
                       uint16_t speed,
                       uint8_t acc);

uint8_t appendJointActuatorTargets(ServoTargetBatch_t* batch,
                                   uint8_t jointIndex,
                                   int32_t primaryTargetAbs,
                                   uint16_t speed,
                                   uint8_t acc);

uint8_t appendJointActuatorRuntimeTargets(ServoTargetBatch_t* batch,
                                          const JointActuatorRuntimeTarget* targets,
                                          uint8_t targetCount,
                                          uint16_t speed,
                                          uint8_t acc);

#endif // CONTROL_OUTPUT_BUILDER_H
