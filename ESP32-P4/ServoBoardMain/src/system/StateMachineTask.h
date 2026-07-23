#ifndef STATE_MACHINE_TASK_H
#define STATE_MACHINE_TASK_H

#include <Arduino.h>
#include "../shared/TaskSharedData.h"

// StateMachineTask 是 system_state、control_enabled 和 servo_target_owner
// 的唯一仲裁者。上位机和控制任务只能投递事件，不能自行切换系统状态。
// 运行中故障按 E-STOP 处理：撤销输出所有权、通知舵机急停并退回 UNCONTROLLABLE。

// 投递无 token 的普通事件；队列满时返回 false。
bool postSystemEvent(TaskSharedData_t* sharedData, uint8_t event);
// 投递带命令代次的事件，防止旧命令的 DONE/TIMEOUT 释放新命令。
bool postSystemEventForToken(TaskSharedData_t* sharedData, uint8_t event, uint32_t token);
// E-STOP 不走普通队列，使用单调 token 保证队列满时也不会丢失。
bool requestEmergencyStop(TaskSharedData_t* sharedData);
// 10 ms 状态机任务入口。
void stateMachineTask(void* parameter);

#endif // STATE_MACHINE_TASK_H
