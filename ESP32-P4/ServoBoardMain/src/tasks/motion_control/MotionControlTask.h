/**
 * @file MotionControlTask.h
 * @brief 声明 100 Hz 关节反馈与舵机直控任务入口。
 */
#ifndef P4_TASKS_CONTROL_TASK_H
#define P4_TASKS_CONTROL_TASK_H

#include <Arduino.h>

// MotionControlTask 是实时控制层入口：
// - 周期读取 ControlCommandSnapshot_t 和底层反馈；
// - 在 joint 模式下执行 PID，在 direct motor 模式下透传/展开舵机目标；
// - 只向 servoTargetQueue 写 batch，不直接访问舵机 UART。
void motionControlTask(void* parameter);

#endif // P4_TASKS_CONTROL_TASK_H
