/**
 * @file RuntimeMonitorService.h
 * @brief 声明 FreeRTOS 任务周期、执行时间、栈水位和双核空闲率采集服务。
 * @details 各业务模块通过 runtime/metrics/TaskRuntimeMetrics.h 打点，本服务只负责汇总和刷新诊断快照。
 */
#ifndef P4_MODULES_DIAGNOSTICS_RUNTIME_MONITOR_SERVICE_H
#define P4_MODULES_DIAGNOSTICS_RUNTIME_MONITOR_SERVICE_H

#include <stdint.h>

/** 采集一次任务栈与双核空闲率；周期由 RuntimeMonitorTask 决定。 */
void stepRuntimeMonitorService(uint32_t nowMs);

#endif // P4_MODULES_DIAGNOSTICS_RUNTIME_MONITOR_SERVICE_H
