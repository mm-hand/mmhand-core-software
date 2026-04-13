#ifndef ENCODERS_TASK_H
#define ENCODERS_TASK_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// 声明外部可访问的任务句柄
extern TaskHandle_t xEncTask;

// 编码器任务函数声明
void Task_Encoders(void* pvParameters);

#endif // ENCODERS_TASK_H