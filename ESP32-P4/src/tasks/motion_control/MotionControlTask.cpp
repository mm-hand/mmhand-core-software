/**
 * @file MotionControlTask.cpp
 * @brief 驱动 100 Hz 控制服务，拥有周期等待与截止时间统计。
 * @details 任务只负责 init/step 调度；控制公式和命令发布位于 MotionControlService。
 */
#include "MotionControlTask.h"

#include <Arduino.h>

#include "MotionControlService.h"
#include "../../system/rtos/PeriodicDeadline.h"
#include "../../system/time/Clock.h"
#include "../../system/SystemRuntimeContext.h"
#include "../../system/adapters/ServiceContextAdapter.h"
#include "../../system/TaskRuntimeMetrics.h"

void motionControlTask(void* parameter)
{
    SystemRuntimeContext* sharedData = static_cast<SystemRuntimeContext*>(parameter);
    if (!sharedData ||
        !initMotionControlService(sharedData,
                            p4::platform::time::monotonicMillis())) {
        vTaskDelete(NULL);
        return;
    }

    sharedData->motionControlWake.bind(xTaskGetCurrentTaskHandle());
    const TickType_t periodTicks = pdMS_TO_TICKS(10);
    p4::platform::rtos::PeriodicDeadline deadline(periodTicks);
    uint32_t wakeReasons = 0;
    for (;;) {
        const TickType_t dispatchTicks = xTaskGetTickCount();
        const bool periodicDue = deadline.due(dispatchTicks);
        // 目标或授权变化会提前唤醒；没有业务事件时按绝对周期等待。
        if (!periodicDue && wakeReasons == 0U) {
            wakeReasons = sharedData->motionControlWake.wait(
                deadline.remaining(dispatchTicks));
            continue;
        }
        if (periodicDue) deadline.advancePast(dispatchTicks);
        const TickType_t loopBegin = xTaskGetTickCount();
        taskRuntimeMarkLoopStart(TASK_RUNTIME_MOTION_CONTROL);
        taskRuntimeMarkCycle(TASK_RUNTIME_MOTION_CONTROL, periodicDue, wakeReasons != 0U);
        wakeReasons = 0;
        const uint32_t nowMs = p4::platform::time::monotonicMillis();
        const ControlServiceInputSnapshot input =
            captureControlServiceInput(sharedData, nowMs);
        const ServiceOutputPort output =
            makeSystemServiceOutputPort(sharedData);
        stepMotionControlService(sharedData, input, output);
        taskRuntimeFinishCycle(TASK_RUNTIME_MOTION_CONTROL, periodicDue, loopBegin,
                               periodTicks, deadline);
    }
    vTaskDelete(NULL);
}
