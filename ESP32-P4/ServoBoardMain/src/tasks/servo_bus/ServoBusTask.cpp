/**
 * @file ServoBusTask.cpp
 * @brief 驱动舵机运行服务，拥有周期等待与截止时间统计。
 * @details 本文件不访问飞特 SDK，四条舵机总线所有权属于 ServoRuntimeService。
 */
#include "ServoBusTask.h"

#include <Arduino.h>

#include "ServoBusService.h"
#include "../../config/ServoConfig.h"
#include "../../platform/rtos/PeriodicDeadline.h"
#include "../../platform/rtos/TaskSignal.h"
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
    p4::platform::rtos::PeriodicDeadline deadline(periodTicks);
    uint32_t wakeReasons = 0;
    for (;;) {
        const TickType_t dispatchTicks = xTaskGetTickCount();
        const bool periodicDue = deadline.due(dispatchTicks);
        if (!periodicDue && wakeReasons == 0U) {
            wakeReasons = sharedData->servoBusWake.wait(
                deadline.remaining(dispatchTicks));
            continue;
        }
        if (periodicDue) deadline.advancePast(dispatchTicks);

        const TickType_t loopBegin = xTaskGetTickCount();
        taskRuntimeMarkLoopStart(TASK_RUNTIME_SERVO_BUS);
        taskRuntimeMarkCycle(TASK_RUNTIME_SERVO_BUS,
                             periodicDue,
                             wakeReasons != 0U);
        const uint32_t nowMs = p4::platform::time::monotonicMillis();
        const ServoServiceInputSnapshot input =
            captureServoServiceInput(sharedData, nowMs);
        const ServiceOutputPort output =
            makeSystemServiceOutputPort(sharedData);
        const ServoBusStepRequest request = {wakeReasons, periodicDue};
        wakeReasons = 0;
        stepServoBusService(sharedData, input, output, request);
        taskRuntimeMarkLoopEnd(TASK_RUNTIME_SERVO_BUS);

        if (periodicDue) {
            const TickType_t loopEnd = xTaskGetTickCount();
            const bool periodOverrun =
                (loopEnd - loopBegin) >= periodTicks || deadline.due(loopEnd);
            taskRuntimeMarkPeriodResult(TASK_RUNTIME_SERVO_BUS, periodOverrun);
            if (periodOverrun) {
                // 丢弃已经错过的截止点，不连续补做旧反馈周期。
                if (deadline.due(loopEnd)) deadline.advancePast(loopEnd);
                vTaskDelay(1);
            }
        }
    }
    vTaskDelete(NULL);
}
