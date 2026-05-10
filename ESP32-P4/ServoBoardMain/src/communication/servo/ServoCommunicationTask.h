#ifndef SERVO_COMMUNICATION_TASK_H
#define SERVO_COMMUNICATION_TASK_H

#include <Arduino.h>
#include "../../shared/TaskSharedData.h"

// ServoCommunicationTask 是唯一允许直接访问飞特舵机 UART/FTServo API 的任务。
// 其他层只能通过 servoTargetQueue 下发目标，通过反馈/遥测队列读取结果。
void servoCommunicationTask(void* parameter);

#endif // SERVO_COMMUNICATION_TASK_H
