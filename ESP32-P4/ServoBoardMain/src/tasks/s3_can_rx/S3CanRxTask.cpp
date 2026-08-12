/**
 * @file S3CanRxTask.cpp
 * @brief 驱动 1 kHz S3 CAN 接入服务并统计周期超限。
 * @details 本文件不处理协议和业务状态，全部工作委托给 SensorIngressService。
 */
#include "S3CanRxTask.h"

#include <Arduino.h>

#include "S3CanRxService.h"
#include "../../platform/time/Clock.h"
#include "../../runtime/metrics/TaskRuntimeMetrics.h"

void s3CanRxTask(void* parameter)
{
    SystemRuntimeContext* sharedData = static_cast<SystemRuntimeContext*>(parameter);
    if (!sharedData ||
        !initS3CanRxService(sharedData,
                                  p4::platform::time::monotonicMillis())) {
        vTaskDelete(NULL);
        return;
    }
    for (;;) {
        const TickType_t loopStartTicks = xTaskGetTickCount();
        taskRuntimeMarkLoopStart(TASK_RUNTIME_S3_CAN_RX);
        stepS3CanRxService(sharedData,
                                 p4::platform::time::monotonicMillis());
        taskRuntimeMarkLoopEnd(TASK_RUNTIME_S3_CAN_RX);
        const bool periodOverrun =
            (TickType_t)(xTaskGetTickCount() - loopStartTicks) >=
            pdMS_TO_TICKS(1);
        taskRuntimeMarkPeriodResult(TASK_RUNTIME_S3_CAN_RX, periodOverrun);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    vTaskDelete(NULL);
}
