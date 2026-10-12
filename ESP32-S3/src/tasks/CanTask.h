#ifndef CAN_TASK_H
#define CAN_TASK_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// CAN任务句柄，由SystemTasks创建。
extern TaskHandle_t xCanTask;

/** @brief 维护TWAI总线，并周期发送磁编快照及错误详情。 */
void Task_CanBus(void* pvParameters);

#endif // CAN_TASK_H
