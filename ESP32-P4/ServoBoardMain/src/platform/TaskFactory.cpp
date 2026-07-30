/**
 * @file TaskFactory.cpp
 * @brief 实现统一的 FreeRTOS 任务创建入口。
 */
#include "TaskFactory.h"

#include "../system/FirmwareLog.h"

namespace p4::platform {

bool createTask(const TaskSpec& spec, TaskHandle_t* outHandle)
{
    if (!spec.entry || !spec.name || spec.stackWords == 0) {
        FW_LOG_ERROR_PRINTLN("[TASK] invalid task specification");
        return false;
    }

    TaskHandle_t localHandle = nullptr;
    TaskHandle_t* handleSlot = outHandle ? outHandle : &localHandle;
    const BaseType_t result = xTaskCreate(spec.entry,
                                          spec.name,
                                          spec.stackWords,
                                          spec.taskContext,
                                          spec.priority,
                                          handleSlot);
    if (result != pdPASS) {
        FW_LOG_ERROR_PRINTF("[TASK] create failed: %s stack=%lu priority=%lu\r\n",
                            spec.name,
                            static_cast<unsigned long>(spec.stackWords),
                            static_cast<unsigned long>(spec.priority));
        if (outHandle) {
            *outHandle = nullptr;
        }
        return false;
    }
    return true;
}

} // namespace p4::platform
