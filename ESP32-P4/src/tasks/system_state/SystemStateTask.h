/**
 * @file SystemStateTask.h
 * @brief 声明 P4 总状态机任务、系统事件入口和紧急停止接口。
 */
#ifndef P4_TASKS_STATE_MACHINE_TASK_H
#define P4_TASKS_STATE_MACHINE_TASK_H

#include <Arduino.h>

// SystemStateTask 是 system_state、control_enabled 和 servo_target_owner
// 的唯一仲裁者。上位机和控制任务只能投递事件，不能自行切换系统状态。
// 运行中故障按 E-STOP 处理：撤销输出所有权、通知舵机急停并退回 UNCONTROLLABLE。

// 10 ms 状态机任务入口。
void systemStateTask(void* parameter);

#endif // P4_TASKS_STATE_MACHINE_TASK_H
