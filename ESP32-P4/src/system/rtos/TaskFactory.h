/**
 * @file TaskFactory.h
 * @brief 定义统一的 FreeRTOS 任务创建参数和平台创建入口。
 * @details
 * - 线程：仅在系统装配或受控的任务生命周期管理代码中调用。
 * - 输入输出：输入静态 TaskSpec，成功时输出 TaskHandle_t。
 * - 所有权：调用方负责 taskContext 生命周期；TaskFactory 不保存业务状态。
 * - 禁止依赖：不得记录业务日志，也不得包含标定、控制、通信或安全策略。
 */
#ifndef P4_PLATFORM_RTOS_TASK_FACTORY_H
#define P4_PLATFORM_RTOS_TASK_FACTORY_H

#include <stdint.h>

#include "FreeRtosKernel.h"

namespace p4::platform::rtos {

/** 创建任务所需的静态描述；stackWords 使用 FreeRTOS 栈元素单位。 */
struct TaskSpec {
    TaskFunction_t entry;  ///< 任务入口函数，不得为空。
    const char* name;      ///< FreeRTOS 诊断名称，生命周期必须覆盖创建调用。
    uint32_t stackWords;   ///< 任务栈深度，单位不是字节。
    void* taskContext;     ///< 传给入口函数的上下文指针。
    UBaseType_t priority;  ///< FreeRTOS 任务优先级。
};

/**
 * 创建普通 FreeRTOS 任务。
 * 参数非法或创建失败时返回 false；失败日志由调用方按所属任务统一记录。
 */
bool createTask(const TaskSpec& spec, TaskHandle_t* outHandle);

} // namespace p4::platform::rtos

#endif // P4_PLATFORM_RTOS_TASK_FACTORY_H
