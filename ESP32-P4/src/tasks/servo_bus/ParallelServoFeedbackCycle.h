/**
 * @file ParallelServoFeedbackCycle.h
 * @brief 协调四条独立 UART 的两阶段同步读事务。
 * @details 本类不维护路由和在线状态，只保证四条 BUS 同轮 Begin、统一 Collect 与元数据生成。
 */
#ifndef P4_TASKS_SERVO_BUS_PARALLEL_SERVO_FEEDBACK_CYCLE_H
#define P4_TASKS_SERVO_BUS_PARALLEL_SERVO_FEEDBACK_CYCLE_H

#include <stdint.h>

#include "../../config/DimensionsConfig.h"
#include "../../data/ServoTypes.h"

class ParallelServoFeedbackCycle {
public:
    /** 向四条有目标的 BUS 依次发送请求；UART 随后并行接收。 */
    bool begin(const uint8_t ids[NUM_BUSES][MAX_SERVOS_PER_BUS],
               const uint8_t counts[NUM_BUSES],
               uint64_t nowUs);

    /** 非阻塞收集四路回包；返回 true 表示本轮已统一终结。 */
    bool collect(uint64_t nowUs, ServoFeedbackCycleMeta_t* completedMeta);

    /** E-STOP、UART维护或周期异常时取消全部未完成事务。 */
    void abort();

    bool active() const { return active_; }
    bool collectDue(uint64_t nowUs) const;
    uint32_t waitMs(uint64_t nowUs) const;

private:
    bool active_ = false;
    uint32_t sequence_ = 0;
    uint64_t startedUs_ = 0;
    uint64_t nextCollectUs_ = 0;
    uint64_t deadlineUs_ = 0;
    uint8_t requestedBusMask_ = 0;
    uint8_t terminalBusMask_ = 0;
    uint8_t freshBusMask_ = 0;
    uint8_t timeoutBusMask_ = 0;
    uint8_t requestedCount_[NUM_BUSES] = {0};
    uint64_t busSampleTimestampUs_[NUM_BUSES] = {0};
};

#endif // P4_TASKS_SERVO_BUS_PARALLEL_SERVO_FEEDBACK_CYCLE_H
