#ifndef SYSTEM_TASKS_H
#define SYSTEM_TASKS_H

#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// 长度为1的覆盖队列：EncTask写入、CanTask读取，始终只保留最新磁编快照。
extern QueueHandle_t xQueueEncoderData;

/** @brief 创建磁编覆盖队列，并启动全部FreeRTOS任务。 */
void startSystemTasks();

#endif
