/**
 * @file S3CanRxTask.cpp
 * @brief 驱动 1 kHz S3 CAN 接入服务并统计周期超限。
 * @details 本文件不处理协议和业务状态，全部工作委托给 SensorIngressService。
 */
#include "S3CanRxTask.h"

#include <Arduino.h>

#include "S3CanRxService.h"
#include "../../system/rtos/PeriodicDeadline.h"
#include "../../system/time/Clock.h"
#include "../../system/SystemRuntimeContext.h"
#include "../../system/TaskRuntimeMetrics.h"

void s3CanRxTask(void* parameter)
{
    SystemRuntimeContext* sharedData = static_cast<SystemRuntimeContext*>(parameter);
    if (!sharedData ||
        !initS3CanRxService(sharedData,
                                  p4::platform::time::monotonicMillis())) {
        vTaskDelete(NULL);
        return;
    }
    const TickType_t periodTicks = pdMS_TO_TICKS(1);
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
        taskRuntimeMarkLoopStart(TASK_RUNTIME_S3_CAN_RX);
        taskRuntimeMarkCycle(TASK_RUNTIME_S3_CAN_RX, true, false);
        stepS3CanRxService(sharedData,
                                 p4::platform::time::monotonicMillis());
        taskRuntimeFinishCycle(TASK_RUNTIME_S3_CAN_RX, true, loopBegin,
                               periodTicks, deadline);
    }
    vTaskDelete(NULL);
}
