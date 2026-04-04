#ifndef SYSTEM_TASK_H
#define SYSTEM_TASK_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// SystemTask 头文件职责：
// 1) 声明系统初始化与主循环入口；
// 2) 暴露跨任务共享的标定状态标志；
// 3) 汇总系统层依赖的任务与求解器头文件。

// 应用层头文件
#include "TaskSharedData.h"
#include "UpperCommTask.h"
#include "CanCommTask.h"
#include "AngleSolver.h"          // 引入 AngleSolver（含 taskSolver 声明）
#include "ServoBusManager.h"      // 引入舵机总线管理器

// 全局标志：用于上位机标定流程的一次性状态回传（跨任务共享）。
extern volatile uint8_t g_calibrationUIStatus;

// 系统初始化入口（替代 Arduino setup）：
// 完成共享资源初始化并创建各业务任务。
void System_Init();

// 系统主循环入口（替代 Arduino loop）：
// 仅保留轻量调度节拍，业务逻辑由任务执行。
void System_Loop();

#endif // SYSTEM_TASK_H
