/**
 * @file RuntimeMonitorTask.cpp
 * @brief 提供运行时诊断服务的 FreeRTOS 薄任务入口。
 * @details 本文件不持有统计数据，实际采集由 RuntimeMonitorService 完成。
 */
#include "RuntimeMonitorTask.h"

#include <Arduino.h>

#include "RuntimeMonitorService.h"
#include "../../system/rtos/PeriodicDeadline.h"
#include "../../system/time/Clock.h"
#include "../../config/RuntimeMonitorConfig.h"
#include "../../system/TaskRuntimeMetrics.h"

void runtimeMonitorTask(void* parameter)
{
    (void)parameter;
    const TickType_t periodTicks = pdMS_TO_TICKS(kTaskRuntimeWindowMs);
    p4::platform::rtos::PeriodicDeadline deadline(periodTicks);
    for (;;) {
        const TickType_t dispatchTicks = xTaskGetTickCount();
        const bool periodicDue = deadline.due(dispatchTicks);
        if (!periodicDue) {
            vTaskDelay(deadline.remaining(dispatchTicks));
            continue;
        }
        deadline.advancePast(dispatchTicks);
        const TickType_t loopBegin = xTaskGetTickCount();
        taskRuntimeMarkLoopStart(TASK_RUNTIME_MONITOR);
        taskRuntimeMarkCycle(TASK_RUNTIME_MONITOR, true, false);
        stepRuntimeMonitorService(p4::platform::time::monotonicMillis());
        taskRuntimeFinishCycle(TASK_RUNTIME_MONITOR, true, loopBegin,
                               periodTicks, deadline);
    }
    vTaskDelete(NULL);
}
