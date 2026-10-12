/**
 * @file S3CanRxTask.h
 * @brief 声明 S3 CAN 传感器接入的 FreeRTOS 任务入口。
 * @details Task 层不拥有协议常量和重组状态，实际业务由 SensorIngressService 执行。
 */
#ifndef P4_TASKS_CAN_TASK_H
#define P4_TASKS_CAN_TASK_H


/** @brief FreeRTOS 任务入口；parameter 必须指向已装配完成的 SystemRuntimeContext。 */
void s3CanRxTask(void* parameter);

#endif // P4_TASKS_CAN_TASK_H
