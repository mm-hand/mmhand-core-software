/**
 * @file TaskFactory.cpp
 * @brief 实现统一的 FreeRTOS 任务创建入口。
 */
#include "TaskFactory.h"

namespace p4::platform::rtos {

bool createTask(const TaskSpec& spec, TaskHandle_t* outHandle)
{
    if (!spec.entry || !spec.name || spec.stackWords == 0) {
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
        if (outHandle) {
            *outHandle = nullptr;
        }
        return false;
    }
    return true;
}

} // namespace p4::platform::rtos
