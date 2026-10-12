/**
 * @file HostCommTask.h
 * @brief 声明上位机串口通信任务入口。
 */
#ifndef P4_TASKS_UPPER_COMM_TASK_H
#define P4_TASKS_UPPER_COMM_TASK_H

#include <Arduino.h>

// 解析上位机命令、路由共享控制状态，并周期上报传感器、舵机、故障和 ACK。
void hostCommTask(void* parameter);

#endif // P4_TASKS_UPPER_COMM_TASK_H
