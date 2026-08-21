/**
 * @file ServoBusTask.cpp
 * @brief 驱动舵机运行服务，拥有周期等待与截止时间统计。
 * @details 本文件不访问飞特 SDK，四条舵机总线所有权属于 ServoRuntimeService。
 */
#include "ServoBusTask.h"

#include <Arduino.h>

#include "ServoBusService.h"
#include "../../config/ServoConfig.h"
#include "../../platform/time/Clock.h"
#include "../../runtime/SystemRuntimeContext.h"
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

    sharedData->servoBusWake.bind(xTaskGetCurrentTaskHandle());
    // 服务任务保持 10 ms 兜底唤醒；总线服务每轮读取两条 BUS，因此每条
    // BUS 的完整位置/速度/负载反馈周期为 ServoConfig 定义的 20 ms。
    const TickType_t periodTicks = pdMS_TO_TICKS(
        mmhand::config::servo::kServoBusServicePeriodMs);
    for (;;) {
        const TickType_t loopBegin = xTaskGetTickCount();
        taskRuntimeMarkLoopStart(TASK_RUNTIME_SERVO_BUS);
        const uint32_t nowMs = p4::platform::time::monotonicMillis();
        const ServoServiceInputSnapshot input =
            captureServoServiceInput(sharedData, nowMs);
        const ServiceOutputPort output =
            makeSystemServiceOutputPort(sharedData);
        stepServoBusService(sharedData, input, output);
        taskRuntimeMarkLoopEnd(TASK_RUNTIME_SERVO_BUS);

        const bool periodOverrun =
            (xTaskGetTickCount() - loopBegin) >= periodTicks;
        taskRuntimeMarkPeriodResult(TASK_RUNTIME_SERVO_BUS, periodOverrun);
        if (periodOverrun) {
            // 离线 BUS 探测可能超过周期，至少让出一个 tick 给状态和通信任务。
            vTaskDelay(1);
        } else {
            (void)sharedData->servoBusWake.wait(periodTicks);
        }
    }
    vTaskDelete(NULL);
}
