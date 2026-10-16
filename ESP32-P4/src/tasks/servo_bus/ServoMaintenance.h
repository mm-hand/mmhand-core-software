/** @file ServoMaintenance.h
 * @brief 声明串行舵机维护请求与普通命令互斥接口。
 */
#pragma once
#include <stdint.h>
struct SystemRuntimeContext;
// Request: big-endian token/session, then operation/bus/old/new.
// Operations: capability=0, begin=1, end=2, probe=3, rename=4.
bool submitServoMaintenance(const uint8_t* payload);
bool servoMaintenanceBlocksCommands();
bool stepServoMaintenance(SystemRuntimeContext* context, uint32_t nowMs);
void publishServoMaintenance();
void cancelServoMaintenance();
uint32_t takeServoMaintenanceChangedMask();
