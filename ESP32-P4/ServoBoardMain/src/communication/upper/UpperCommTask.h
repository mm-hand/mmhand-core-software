/**
 * @file UpperCommTask.h
 * @brief 声明上位机串口通信任务入口。
 */
#ifndef UPPER_COMM_TASK_H
#define UPPER_COMM_TASK_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "../../shared/TaskSharedData.h"

// 解析上位机命令、路由共享控制状态，并周期上报传感器、舵机、故障和 ACK。
void upperCommunicationTask(void* parameter);

#endif
