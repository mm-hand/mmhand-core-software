/**
 * @file TaskSignal.h
 * @brief 封装任务通知位，用于“新数据立即唤醒 + 周期轮询兜底”的轻量调度。
 * @details
 * - 线程：通知可由任意任务调用；wait 只能由已 bind 的目标任务调用。
 * - 输入输出：notify 合并原因位；wait 取走本轮累计原因，不保存业务数据。
 * - 所有权：TaskHandle 只在任务启动时绑定一次，SystemRuntimeContext 负责保存。
 * - 禁止依赖：不包含业务模块、协议和硬件驱动，不能替代需要顺序语义的 Queue。
 */
#ifndef P4_PLATFORM_RTOS_TASK_SIGNAL_H
#define P4_PLATFORM_RTOS_TASK_SIGNAL_H

#include <stdint.h>

#include "FreeRtosKernel.h"

namespace p4::platform::rtos {

/** 单消费者任务通知；重复事件合并为 bit，不排队，也不分配内存。 */
class TaskSignal {
public:
    void bind(TaskHandle_t task) { task_ = task; }

    /** 通知目标任务；任务尚未启动时静默忽略，周期轮询仍会兜底。 */
    bool notify(uint32_t reasonBits) const
    {
        return task_ && reasonBits != 0 &&
               xTaskNotify(task_, reasonBits, eSetBits) == pdPASS;
    }

    /** 等待新原因或 timeoutTicks 超时；0 表示由周期兜底唤醒。 */
    uint32_t wait(TickType_t timeoutTicks) const
    {
        uint32_t reasons = 0;
        (void)xTaskNotifyWait(0, UINT32_MAX, &reasons, timeoutTicks);
        return reasons;
    }

private:
    TaskHandle_t task_ = nullptr;
};

} // namespace p4::platform::rtos

namespace p4::runtime::wake {

// 仅描述唤醒来源，不携带命令内容；真实目标仍由各自 mailbox 保存。
inline constexpr uint32_t kHostRx = 1UL << 0;
inline constexpr uint32_t kStateEvent = 1UL << 0;
inline constexpr uint32_t kStateEmergencyStop = 1UL << 1;
inline constexpr uint32_t kMotionTarget = 1UL << 0;
inline constexpr uint32_t kMotionStateChanged = 1UL << 1;
inline constexpr uint32_t kServoTarget = 1UL << 0;
inline constexpr uint32_t kServoEmergencyStop = 1UL << 1;

} // namespace p4::runtime::wake

#endif // P4_PLATFORM_RTOS_TASK_SIGNAL_H
