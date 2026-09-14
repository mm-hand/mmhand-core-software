/**
 * @file SystemStateTask.cpp
 * @brief 驱动 100 Hz 顶层状态服务，拥有周期等待与截止时间统计。
 * @details 本文件不维护状态规则；全部事件与状态推进委托给 modules/state 编排器。
 */
#include "SystemStateTask.h"
#include <Arduino.h>

#include "SystemStateCoordinator.h"
#include "../../platform/rtos/TaskSignal.h"
#include "../../platform/time/Clock.h"
#include "../../runtime/SystemRuntimeContext.h"
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

    sharedData->systemStateWake.bind(xTaskGetCurrentTaskHandle());
    const TickType_t periodTicks = pdMS_TO_TICKS(10);
    for (;;) {
        const TickType_t loopBegin = xTaskGetTickCount();
        const uint8_t stateBefore = sharedData->system_state;
        const uint8_t ownerBefore = sharedData->servo_target_owner;
        const uint8_t controlBefore = sharedData->control_enabled;
        taskRuntimeMarkLoopStart(TASK_RUNTIME_SYSTEM_STATE);
        const uint32_t nowMs = p4::platform::time::monotonicMillis();
        const StateServiceInputSnapshot input =
            captureStateServiceInput(sharedData, nowMs);
        const ServiceOutputPort output =
            makeSystemServiceOutputPort(sharedData);
        stepSystemStateCoordinator(&context, sharedData, input, output);
        // 状态机授予/释放运动所有权后立即唤醒控制任务，避免首次目标额外等待一个 10 ms 周期。
        if (stateBefore != sharedData->system_state ||
            ownerBefore != sharedData->servo_target_owner ||
            controlBefore != sharedData->control_enabled) {
            sharedData->motionControlWake.notify(
                p4::runtime::wake::kMotionStateChanged);
        }
        taskRuntimeMarkLoopEnd(TASK_RUNTIME_SYSTEM_STATE);

        const bool periodOverrun =
            (xTaskGetTickCount() - loopBegin) >= periodTicks;
        taskRuntimeMarkPeriodResult(TASK_RUNTIME_SYSTEM_STATE, periodOverrun);
        if (periodOverrun) {
            vTaskDelay(1);
        } else {
            (void)sharedData->systemStateWake.wait(periodTicks);
        }
    }
    vTaskDelete(NULL);
}
