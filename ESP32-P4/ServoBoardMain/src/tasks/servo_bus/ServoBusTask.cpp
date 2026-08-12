/**
 * @file ServoBusTask.cpp
 * @brief 驱动舵机运行服务，拥有周期等待与截止时间统计。
 * @details 本文件不访问飞特 SDK，四条舵机总线所有权属于 ServoRuntimeService。
 */
#include "ServoBusTask.h"

#include <Arduino.h>

#include "ServoBusService.h"
#include "../../platform/time/Clock.h"
#include "../../runtime/adapters/ServiceContextAdapter.h"
#include "../../runtime/metrics/TaskRuntimeMetrics.h"

void servoBusTask(void* parameter)
{
    SystemRuntimeContext* sharedData = static_cast<SystemRuntimeContext*>(parameter);
    if (!sharedData ||
        !initServoBusService(sharedData,
                                p4::platform::time::monotonicMillis())) {
        vTaskDelete(NULL);
        return;
    }

    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t periodTicks = pdMS_TO_TICKS(10);
    for (;;) {
        taskRuntimeMarkLoopStart(TASK_RUNTIME_SERVO_BUS);
        const uint32_t nowMs = p4::platform::time::monotonicMillis();
        const ServoServiceInputSnapshot input =
            captureServoServiceInput(sharedData, nowMs);
        const ServiceOutputPort output =
            makeSystemServiceOutputPort(sharedData);
        stepServoBusService(sharedData, input, output);
        taskRuntimeMarkLoopEnd(TASK_RUNTIME_SERVO_BUS);

        const TickType_t nextWakeTime = lastWakeTime + periodTicks;
        const bool periodOverrun =
            (int32_t)(xTaskGetTickCount() - nextWakeTime) >= 0;
        taskRuntimeMarkPeriodResult(TASK_RUNTIME_SERVO_BUS, periodOverrun);
        vTaskDelayUntil(&lastWakeTime, periodTicks);
        if (periodOverrun) {
            // 离线 BUS 探测可能超过周期，至少让出一个 tick 给状态和通信任务。
            vTaskDelay(1);
        }
    }
    vTaskDelete(NULL);
}
