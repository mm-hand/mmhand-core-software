#ifndef UPPER_COMM_COMMAND_ROUTER_H
#define UPPER_COMM_COMMAND_ROUTER_H

#include <Arduino.h>
#include "TaskSharedData.h"

int16_t clampServoAbsCommand(int32_t value);
void upperApplyTargetAngles(TaskSharedData_t* sharedData, const float* angles, uint8_t count);
void upperClearTargetAngles(TaskSharedData_t* sharedData);
void upperApplyMotorTargets(TaskSharedData_t* sharedData,
                            const int32_t* targets,
                            uint8_t count,
                            uint8_t source);
void upperApplyMotorSweepTargets(TaskSharedData_t* sharedData, const int32_t* targets, uint8_t count);
void upperApplyTendonGuardConfig(TaskSharedData_t* sharedData,
                                 const uint8_t* enabled,
                                 const int8_t* sign,
                                 const int16_t* x1Abs,
                                 uint8_t count);
void upperCacheCalibZeroRaw(TaskSharedData_t* sharedData, const float* values, uint8_t count);

#endif // UPPER_COMM_COMMAND_ROUTER_H
