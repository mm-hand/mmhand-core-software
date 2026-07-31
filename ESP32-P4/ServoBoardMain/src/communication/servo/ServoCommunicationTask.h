/**
 * @file ServoCommunicationTask.h
 * @brief 声明舵机总线通信、反馈发布和多圈恢复任务入口。
 */
#ifndef SERVO_COMMUNICATION_TASK_H
#define SERVO_COMMUNICATION_TASK_H

#include <Arduino.h>
#include "../../shared/TaskSharedData.h"

// 只有该任务可以调度 Feetech UART；控制与标定任务只能向统一目标队列提交命令。
void servoCommunicationTask(void* parameter);

#endif // SERVO_COMMUNICATION_TASK_H
