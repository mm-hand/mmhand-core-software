#ifndef TACTILE_TASK_H
#define TACTILE_TASK_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// 声明外部可访问的任务句柄
extern TaskHandle_t xTacTask;

// 触觉任务函数声明
void Task_Tactile(void* pvParameters);

#endif // TACTILE_TASK_H