#ifndef CONTROL_OUTPUT_BUILDER_H
#define CONTROL_OUTPUT_BUILDER_H

#include <Arduino.h>
#include "TaskSharedData.h"

int16_t clampServoPos(int32_t value);
int16_t clampServoSingleTurnRaw(int32_t value);
int16_t expandSingleTurnTargetNearCurrent(int32_t currentAbsPos, int16_t singleTurnRaw);
int32_t anchorSingleTurnSweepToCurrent(int32_t currentAbsPos, int16_t singleTurnRaw);
int32_t wrapServoRawDelta(int32_t delta);
bool appendServoTarget(ServoTargetBatch_t* batch,
                       uint8_t busIndex,
                       uint8_t servoId,
                       int16_t position,
                       uint16_t speed,
                       uint8_t acc);

#endif // CONTROL_OUTPUT_BUILDER_H
