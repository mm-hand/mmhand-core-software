/**
 * @file PeriodicDeadline.h
 * @brief 提供可被任务通知提前唤醒、但不会被事件改变相位的绝对周期截止时间。
 * @details
 * - 线程：对象只由所属任务访问，不需要锁。
 * - 输入输出：输入当前 FreeRTOS tick，输出是否到期及剩余等待 tick。
 * - 状态所有权：每个周期任务各自持有一个实例。
 * - 禁止依赖：不包含业务状态；超期后只跳过旧截止点，不负责补发业务周期。
 */
#ifndef P4_PLATFORM_RTOS_PERIODIC_DEADLINE_H
#define P4_PLATFORM_RTOS_PERIODIC_DEADLINE_H

#include <stdint.h>

#include "FreeRtosKernel.h"

namespace p4::platform::rtos {

class PeriodicDeadline {
public:
    explicit PeriodicDeadline(TickType_t periodTicks,
                              TickType_t startTicks = xTaskGetTickCount())
        : periodTicks_(periodTicks == 0 ? 1 : periodTicks),
          nextTicks_(startTicks)
    {
    }

    /** 使用有符号差值处理 TickType_t 自然回绕。 */
    bool due(TickType_t nowTicks) const
    {
        return (int32_t)(nowTicks - nextTicks_) >= 0;
    }

    /** 消费当前到期点，并跳过所有已错过的旧周期，禁止形成补发突发。 */
    void advancePast(TickType_t nowTicks)
    {
        do {
            nextTicks_ += periodTicks_;
        } while ((int32_t)(nowTicks - nextTicks_) >= 0);
    }

    TickType_t remaining(TickType_t nowTicks) const
    {
        const int32_t delta = (int32_t)(nextTicks_ - nowTicks);
        return delta > 0 ? (TickType_t)delta : 0;
    }

private:
    TickType_t periodTicks_;
    TickType_t nextTicks_;
};

} // namespace p4::platform::rtos

#endif // P4_PLATFORM_RTOS_PERIODIC_DEADLINE_H
