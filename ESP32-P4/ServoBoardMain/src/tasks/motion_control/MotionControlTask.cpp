/**
 * @file MotionControlTask.cpp
 * @brief 驱动 100 Hz 控制服务，拥有周期等待与截止时间统计。
 * @details 任务只负责 init/step 调度；控制公式和命令发布位于 MotionControlService。
 */
#include "MotionControlTask.h"

#include <Arduino.h>

#include "MotionControlService.h"
#include "../../platform/time/Clock.h"
#include "../../runtime/SystemRuntimeContext.h"
#include "../../runtime/adapters/ServiceContextAdapter.h"
#include "../../runtime/metrics/TaskRuntimeMetrics.h"

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
    for (;;) {
        const TickType_t loopBegin = xTaskGetTickCount();
        taskRuntimeMarkLoopStart(TASK_RUNTIME_MOTION_CONTROL);
        const uint32_t nowMs = p4::platform::time::monotonicMillis();
        const ControlServiceInputSnapshot input =
            captureControlServiceInput(sharedData, nowMs);
        const ServiceOutputPort output =
            makeSystemServiceOutputPort(sharedData);
        stepMotionControlService(sharedData, input, output);
        taskRuntimeMarkLoopEnd(TASK_RUNTIME_MOTION_CONTROL);

        const bool periodOverrun =
            (xTaskGetTickCount() - loopBegin) >= periodTicks;
        taskRuntimeMarkPeriodResult(TASK_RUNTIME_MOTION_CONTROL, periodOverrun);
        if (periodOverrun) {
            vTaskDelay(1);
        } else {
            (void)sharedData->motionControlWake.wait(periodTicks);
        }
    }
    vTaskDelete(NULL);
}
