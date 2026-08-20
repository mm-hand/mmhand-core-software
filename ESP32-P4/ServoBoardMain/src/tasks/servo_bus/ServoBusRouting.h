/**
 * @file ServoBusRouting.h
 * @brief 声明 ServoBusTask 私有的 Mxx 路由解析、快照发布与总线初始化工具。
 */
#ifndef P4_TASKS_SERVO_BUS_SERVO_BUS_ROUTING_H
#define P4_TASKS_SERVO_BUS_SERVO_BUS_ROUTING_H

#include <stdint.h>

#include "../../config/Config.h"
#include "ServoRouteTable.h"

class FeetechServoDriver;
struct SystemRuntimeContext;

FeetechServoDriver* resolveMotorDriver(uint8_t motorChannel,
                                      ServoRoute* outRoute,
                                      bool controllable);
void publishServoRouteSnapshot(SystemRuntimeContext* sharedData);
bool addServoReadTarget(uint8_t busIds[NUM_BUSES][MAX_SERVOS_PER_BUS],
                        uint8_t busCounts[NUM_BUSES],
                        uint8_t bus,
                        uint8_t id);
bool restoreServoMultiTurnFromHost(SystemRuntimeContext* sharedData,
                                   uint32_t motorMask,
                                   const int32_t* motorAbs,
                                   bool clearRecoveryTransaction,
                                   uint32_t* restoredMask,
                                   uint32_t* rejectedMask);
void beginServoBuses();

#endif // P4_TASKS_SERVO_BUS_SERVO_BUS_ROUTING_H
