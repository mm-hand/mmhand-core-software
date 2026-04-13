#ifndef SYSTEM_TASKS_H
#define SYSTEM_TASKS_H

#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// 队列声明
extern QueueHandle_t xQueueEncoderData;

// 启动系统任务
void startSystemTasks();

#endif