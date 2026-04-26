#ifndef SYSTEM_TASKS_H
#define SYSTEM_TASKS_H

#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

extern QueueHandle_t xQueueEncoderData;
extern QueueHandle_t xQueueTactileData;

void startSystemTasks();

#endif
