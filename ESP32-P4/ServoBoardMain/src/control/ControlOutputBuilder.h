#ifndef CONTROL_OUTPUT_BUILDER_H
#define CONTROL_OUTPUT_BUILDER_H

#include <Arduino.h>
#include "../shared/TaskSharedData.h"

// ControlOutputBuilder 负责把控制结果整理成舵机通信层可执行的目标：
// - 所有输出都限制在飞特舵机多圈安全范围内；
// - single-turn 目标会按当前绝对位置展开到最近的多圈位置；
// - appendServoTarget 只填充 batch，不直接访问舵机总线。
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
