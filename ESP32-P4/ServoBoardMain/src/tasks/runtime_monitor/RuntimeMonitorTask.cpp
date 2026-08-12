/**
 * @file RuntimeMonitorTask.cpp
 * @brief 提供运行时诊断服务的 FreeRTOS 薄任务入口。
 * @details 本文件不持有统计数据，实际采集由 RuntimeMonitorService 完成。
 */
#include "RuntimeMonitorTask.h"

#include <Arduino.h>

#include "RuntimeMonitorService.h"
#include "../../platform/time/Clock.h"
#include "../../runtime/metrics/TaskRuntimeMetrics.h"

void runtimeMonitorTask(void* parameter)
{
    (void)parameter;
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t periodTicks = pdMS_TO_TICKS(1000);
    for (;;) {
        taskRuntimeMarkLoopStart(TASK_RUNTIME_MONITOR);
        stepRuntimeMonitorService(p4::platform::time::monotonicMillis());
        taskRuntimeMarkLoopEnd(TASK_RUNTIME_MONITOR);
        const TickType_t nextWakeTime = lastWakeTime + periodTicks;
        const bool periodOverrun =
            (int32_t)(xTaskGetTickCount() - nextWakeTime) >= 0;
        taskRuntimeMarkPeriodResult(TASK_RUNTIME_MONITOR, periodOverrun);
        vTaskDelayUntil(&lastWakeTime, periodTicks);
        if (periodOverrun) {
            vTaskDelay(1);
        }
    }
    vTaskDelete(NULL);
}
