/**
 * @file SystemStateTask.cpp
 * @brief 驱动 100 Hz 顶层状态服务，拥有周期等待与截止时间统计。
 * @details 本文件不维护状态规则；全部事件与状态推进委托给 modules/state 编排器。
 */
#include "SystemStateTask.h"
#include <Arduino.h>

#include "SystemStateCoordinator.h"
#include "../../platform/time/Clock.h"
#include "../../runtime/adapters/ServiceContextAdapter.h"
#include "../../runtime/metrics/TaskRuntimeMetrics.h"

void systemStateTask(void* parameter)
{
    SystemRuntimeContext* sharedData = static_cast<SystemRuntimeContext*>(parameter);
    SystemStateCoordinatorContext context = {};
    if (!initSystemStateCoordinator(&context, sharedData)) {
        vTaskDelete(NULL);
        return;
    }

    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t periodTicks = pdMS_TO_TICKS(10);
    for (;;) {
        taskRuntimeMarkLoopStart(TASK_RUNTIME_SYSTEM_STATE);
        const uint32_t nowMs = p4::platform::time::monotonicMillis();
        const StateServiceInputSnapshot input =
            captureStateServiceInput(sharedData, nowMs);
        const ServiceOutputPort output =
            makeSystemServiceOutputPort(sharedData);
        stepSystemStateCoordinator(&context, sharedData, input, output);
        taskRuntimeMarkLoopEnd(TASK_RUNTIME_SYSTEM_STATE);

        const TickType_t nextWakeTime = lastWakeTime + periodTicks;
        const bool periodOverrun =
            (int32_t)(xTaskGetTickCount() - nextWakeTime) >= 0;
        taskRuntimeMarkPeriodResult(TASK_RUNTIME_SYSTEM_STATE, periodOverrun);
        vTaskDelayUntil(&lastWakeTime, periodTicks);
        if (periodOverrun) {
            vTaskDelay(1);
        }
    }
    vTaskDelete(NULL);
}
