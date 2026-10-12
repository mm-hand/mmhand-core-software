#ifndef ENCODERS_TASK_H
#define ENCODERS_TASK_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// 磁编采集任务句柄，由SystemTasks创建。
extern TaskHandle_t xEncTask;

/** @brief 以2ms周期采集21路磁编，并覆盖写入最新值队列。 */
void Task_Encoders(void* pvParameters);

#endif // ENCODERS_TASK_H
