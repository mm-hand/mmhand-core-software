/**
 * @file NvsPersistenceTask.cpp
 * @brief 按 20 ms 周期驱动非阻塞持久化服务。
 * @details 任务只拥有 FreeRTOS 调度；NVS 合并、静稳等待和提交规则位于 service。
 */
#include "NvsPersistenceTask.h"

#include <Arduino.h>

#include "NvsPersistenceService.h"
#include "../../platform/time/Clock.h"
#include "../../runtime/SystemRuntimeContext.h"
#include "../../runtime/adapters/ServiceContextAdapter.h"

void nvsNvsPersistenceTask(void* parameter)
{
    SystemRuntimeContext* sharedData = static_cast<SystemRuntimeContext*>(parameter);
    NvsPersistenceServiceContext context = {};
    if (!initNvsPersistenceService(&context, sharedData)) {
        vTaskDelete(NULL);
        return;
    }

    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t periodTicks = pdMS_TO_TICKS(20);
    for (;;) {
        stepNvsPersistenceService(&context,
                               sharedData,
                               p4::platform::time::monotonicMillis(),
                               makeSystemServiceOutputPort(sharedData));
        vTaskDelayUntil(&lastWakeTime, periodTicks);
    }
}
