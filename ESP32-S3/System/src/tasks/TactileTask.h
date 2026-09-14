#ifndef TACTILE_TASK_H
#define TACTILE_TASK_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// 触觉采集任务句柄；关闭ENABLE_TACTILE_TASK时保持为NULL。
extern TaskHandle_t xTacTask;

/** @brief 约33ms周期采集15个触觉模块并上报摘要/全量数据。 */
void Task_Tactile(void* pvParameters);

#endif // TACTILE_TASK_H
