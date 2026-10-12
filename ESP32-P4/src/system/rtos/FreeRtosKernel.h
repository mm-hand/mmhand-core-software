/**
 * @file FreeRtosKernel.h
 * @brief 统一引入 FreeRTOS 内核类型与 API，保证头文件包含顺序稳定。
 * @details
 * - 线程：可被任意 P4 任务或平台适配代码包含，本文件自身不执行运行时逻辑。
 * - 输入输出：仅导出 FreeRTOS 的任务、队列和互斥量类型/API。
 * - 所有权：属于 platform/rtos；业务模块不得自行调整 FreeRTOS 头文件顺序。
 * - 禁止依赖：不得包含 system、modules、safety 或具体硬件驱动。
 */
#ifndef P4_PLATFORM_RTOS_FREERTOS_KERNEL_H
#define P4_PLATFORM_RTOS_FREERTOS_KERNEL_H

// FreeRTOS.h 必须最先包含，否则 Arduino-ESP32 3.3.x 可能在 task.h 中缺少基础类型。
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#endif // P4_PLATFORM_RTOS_FREERTOS_KERNEL_H
