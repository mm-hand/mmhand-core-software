/**
 * @file ServoBusTask.h
 * @brief 声明舵机总线通信、反馈发布和多圈恢复任务入口。
 */
#ifndef P4_TASKS_SERVO_TASK_H
#define P4_TASKS_SERVO_TASK_H

#include <Arduino.h>

// 只有该任务可以调度 Feetech UART；控制与标定任务只能向统一目标队列提交命令。
void servoBusTask(void* parameter);

#endif // P4_TASKS_SERVO_TASK_H
