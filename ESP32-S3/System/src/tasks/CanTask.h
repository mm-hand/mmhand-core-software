#ifndef CAN_TASK_H
#define CAN_TASK_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// 声明外部可访问的任务句柄
extern TaskHandle_t xCanTask;

// CAN任务函数声明
void Task_CanBus(void *pvParameters);

#endif // CAN_TASK_H