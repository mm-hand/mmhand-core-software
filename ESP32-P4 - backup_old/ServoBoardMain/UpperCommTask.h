#ifndef UPPER_COMM_TASK_H
#define UPPER_COMM_TASK_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "TaskSharedData.h"

// UpperCommTask 模块职责：
// 1) 解析上位机串口命令并更新共享控制状态；
// 2) 汇总并上报传感器/舵机/调试数据；
// 3) 发送协议 ACK 与故障位图心跳。
void taskUpperComm(void *parameter);

#endif
