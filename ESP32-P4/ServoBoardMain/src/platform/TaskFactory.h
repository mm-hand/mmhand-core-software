/**
 * @file TaskFactory.h
 * @brief 统一 FreeRTOS 任务创建参数、错误检查和任务句柄返回。
 */
#ifndef P4_PLATFORM_TASK_FACTORY_H
#define P4_PLATFORM_TASK_FACTORY_H

#include <Arduino.h>

namespace p4::platform {

/** 创建任务所需的静态描述；调用者仍拥有 taskContext 的生命周期。 */
struct TaskSpec {
    TaskFunction_t entry;
    const char* name;
    uint32_t stackWords;
    void* taskContext;
    UBaseType_t priority;
};

/** 创建普通 FreeRTOS 任务；参数或创建失败时返回 false 并输出固件日志。 */
bool createTask(const TaskSpec& spec, TaskHandle_t* outHandle);

} // namespace p4::platform

#endif // P4_PLATFORM_TASK_FACTORY_H
