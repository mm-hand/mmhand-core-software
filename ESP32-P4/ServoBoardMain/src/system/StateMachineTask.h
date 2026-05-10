#ifndef STATE_MACHINE_TASK_H
#define STATE_MACHINE_TASK_H

#include <Arduino.h>
#include "../shared/TaskSharedData.h"

// StateMachineTask 是系统运行状态的唯一权威：
// - 上位机 START/STOP/RESET/CALIBRATE 只能通过 postSystemEvent 投递事件；
// - control_enabled、servo_target_owner 和 system_state 在本任务内集中维护；
// - 故障位图在本任务内周期性汇总，并决定是否进入 FAULT_HOLD。

// 向状态机投递事件，失败通常表示队列未创建或队列已满。
bool postSystemEvent(TaskSharedData_t* sharedData, uint8_t event);
// 状态机 FreeRTOS 任务入口。
void stateMachineTask(void* parameter);

#endif // STATE_MACHINE_TASK_H
