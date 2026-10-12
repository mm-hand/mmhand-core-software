/**
 * @file RuntimeMonitorTask.h
 * @brief 声明运行时监控 FreeRTOS 任务入口，并导出稳定监控端口。
 */
#pragma once


/** 1 Hz 更新任务栈高水位和双核 idle 比例。 */
void runtimeMonitorTask(void* parameter);
