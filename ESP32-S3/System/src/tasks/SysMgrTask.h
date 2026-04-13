#ifndef SYS_MGR_TASK_H
#define SYS_MGR_TASK_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// 声明外部可访问的任务句柄
extern TaskHandle_t xSysMgrTask;

// 系统管理任务函数声明
void Task_SysMgr(void *pvParameters);

#endif // SYS_MGR_TASK_H