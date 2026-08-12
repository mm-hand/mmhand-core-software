/**
 * @file MessageChannel.h
 * @brief 为业务模块提供带类型的 FreeRTOS 消息通道，隐藏 QueueHandle_t 和 xQueue API。
 * @details
 * - 线程：允许多生产者/多消费者，遵循 FreeRTOS Queue 的并发语义。
 * - 输入输出：T 类型消息；不进行堆分配，队列存储由 FreeRTOS 静态/动态堆管理。
 * - 所有权：SystemAssembly 创建，业务模块只持有 SystemContext 中的通道引用。
 * - 最新快照使用 overwrite/peek；顺序事件使用 send/receive，二者语义不可混用。
 */
#ifndef P4_PLATFORM_RTOS_MESSAGE_CHANNEL_H
#define P4_PLATFORM_RTOS_MESSAGE_CHANNEL_H

#include <stddef.h>
#include <stdint.h>

#include "FreeRtosKernel.h"

template <typename T>
class MessageChannel {
public:
    MessageChannel() : handle_(nullptr), depth_(0) {}

    /** 创建固定深度队列；只能由系统装配阶段调用一次。 */
    bool create(uint16_t depth)
    {
        if (handle_ || depth == 0) return false;
        handle_ = xQueueCreate(depth, sizeof(T));
        depth_ = handle_ ? depth : 0;
        return handle_ != nullptr;
    }

    bool valid() const { return handle_ != nullptr; }

    /** 顺序发送；timeoutTicks 为 0 时不会阻塞实时任务。 */
    bool send(const T& value, TickType_t timeoutTicks = 0)
    {
        return handle_ && xQueueSend(handle_, &value, timeoutTicks) == pdTRUE;
    }

    /** 接收并移除最旧消息。 */
    bool receive(T* outValue, TickType_t timeoutTicks = 0)
    {
        return handle_ && outValue &&
               xQueueReceive(handle_, outValue, timeoutTicks) == pdTRUE;
    }

    /** 读取但不移除最旧消息，主要用于深度 1 的状态快照。 */
    bool peek(T* outValue, TickType_t timeoutTicks = 0) const
    {
        return handle_ && outValue &&
               xQueuePeek(handle_, outValue, timeoutTicks) == pdTRUE;
    }

    /** 覆盖深度 1 的最新值邮箱。 */
    bool overwrite(const T& value)
    {
        return handle_ && depth_ == 1 && xQueueOverwrite(handle_, &value) == pdTRUE;
    }

    void reset()
    {
        if (handle_) xQueueReset(handle_);
    }

    uint32_t pending() const
    {
        return handle_ ? (uint32_t)uxQueueMessagesWaiting(handle_) : 0;
    }

private:
    QueueHandle_t handle_; ///< 仅 platform 层保存原生 FreeRTOS 句柄。
    uint16_t depth_;       ///< 创建时固定深度，用于约束 overwrite 语义。
};

#endif // P4_PLATFORM_RTOS_MESSAGE_CHANNEL_H
