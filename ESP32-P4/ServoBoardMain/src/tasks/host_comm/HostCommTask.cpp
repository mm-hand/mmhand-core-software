/**
 * @file HostCommTask.cpp
 * @brief 驱动 200 Hz 上位机通信服务，拥有周期等待与截止时间统计。
 * @details 本文件不解析协议；全部收发行为委托给 modules/communication 服务。
 */
#include "HostCommTask.h"
#include <Arduino.h>

#include "HostCommService.h"
#include "../../platform/time/Clock.h"
#include "../../runtime/metrics/TaskRuntimeMetrics.h"

void hostCommTask(void* parameter)
{
    SystemRuntimeContext* sharedData = static_cast<SystemRuntimeContext*>(parameter);
    if (!initHostCommService(sharedData)) {
        vTaskDelete(NULL);
        return;
    }

    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t periodTicks = pdMS_TO_TICKS(5);
    for (;;) {
        taskRuntimeMarkLoopStart(TASK_RUNTIME_HOST_COMM);
        stepHostCommService(sharedData,
                             p4::platform::time::monotonicMillis());
        taskRuntimeMarkLoopEnd(TASK_RUNTIME_HOST_COMM);
        const TickType_t nextWakeTime = lastWakeTime + periodTicks;
        const bool periodOverrun =
            (int32_t)(xTaskGetTickCount() - nextWakeTime) >= 0;
        taskRuntimeMarkPeriodResult(TASK_RUNTIME_HOST_COMM, periodOverrun);
        vTaskDelayUntil(&lastWakeTime, periodTicks);
        if (periodOverrun) {
            vTaskDelay(1);
        }
    }
    vTaskDelete(NULL);
}
