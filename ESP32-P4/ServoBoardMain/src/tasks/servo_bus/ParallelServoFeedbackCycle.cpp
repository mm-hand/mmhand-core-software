/**
 * @file ParallelServoFeedbackCycle.cpp
 * @brief 实现四 BUS 并行同步读的 Begin/Collect 两阶段状态机。
 * @details 全程使用固定数组；collect 只读取 UART 已到达字节，不阻塞等待。
 */
#include "ParallelServoFeedbackCycle.h"

#include <string.h>

#include "../../config/ServoConfig.h"
#include "../../hal/servo/ServoBusRegistry.h"
#include "../../runtime/metrics/TaskRuntimeMetrics.h"

bool ParallelServoFeedbackCycle::begin(
    const uint8_t ids[NUM_BUSES][MAX_SERVOS_PER_BUS],
    const uint8_t counts[NUM_BUSES],
    uint64_t nowUs)
{
    if (active_ || !ids || !counts) {
        return false;
    }

    ++sequence_;
    if (sequence_ == 0) ++sequence_;
    startedUs_ = nowUs;
    nextCollectUs_ =
        nowUs + mmhand::config::servo::kServoFeedbackCollectDelayMs * 1000ULL;
    deadlineUs_ = nowUs + mmhand::config::servo::kServoFeedbackDeadlineUs;
    requestedBusMask_ = terminalBusMask_ = freshBusMask_ = timeoutBusMask_ = 0;
    memset(requestedCount_, 0, sizeof(requestedCount_));
    memset(busSampleTimestampUs_, 0, sizeof(busSampleTimestampUs_));

    for (uint8_t busIndex = 0; busIndex < NUM_BUSES; ++busIndex) {
        if (counts[busIndex] == 0) continue;
        const uint8_t bit = (uint8_t)(1U << busIndex);
        requestedBusMask_ |= bit;
        requestedCount_[busIndex] = counts[busIndex];
        FeetechServoDriver* driver = servoBusRegistry().driver(busIndex);
        if (!driver || !driver->beginSyncRead(
                           ids[busIndex], counts[busIndex], sequence_, nowUs)) {
            terminalBusMask_ |= bit;
            timeoutBusMask_ |= bit;
            taskRuntimeMarkIoSample(busIndex, 0, counts[busIndex], 0);
        }
    }

    // 即使当前没有可读路由，也生成一个可立即终结的空轮次，让扫描和
    // 在线状态维护继续按10 ms节奏推进，不因“零舵机”停住后台恢复。
    active_ = true;
    if (requestedBusMask_ == terminalBusMask_) nextCollectUs_ = nowUs;
    return true;
}

bool ParallelServoFeedbackCycle::collect(
    uint64_t nowUs,
    ServoFeedbackCycleMeta_t* completedMeta)
{
    if (!completedMeta || !active_) {
        return false;
    }

    for (uint8_t busIndex = 0; busIndex < NUM_BUSES; ++busIndex) {
        const uint8_t bit = (uint8_t)(1U << busIndex);
        if ((requestedBusMask_ & bit) == 0 || (terminalBusMask_ & bit) != 0) {
            continue;
        }
        FeetechServoDriver* driver = servoBusRegistry().driver(busIndex);
        const ServoSyncReadResult result = driver
            ? driver->collectSyncRead(nowUs)
            : ServoSyncReadResult{ServoSyncReadState::TimedOut,
                                  requestedCount_[busIndex], 0, 0, 0,
                                  sequence_, startedUs_, nowUs};
        if (result.state == ServoSyncReadState::Pending) continue;

        terminalBusMask_ |= bit;
        busSampleTimestampUs_[busIndex] = result.completedUs;
        if (result.state == ServoSyncReadState::Complete &&
            result.successCount == result.requestedCount) {
            freshBusMask_ |= bit;
        } else {
            timeoutBusMask_ |= bit;
        }
        const uint32_t elapsedUs = result.completedUs >= result.startedUs
            ? (uint32_t)(result.completedUs - result.startedUs)
            : 0;
        taskRuntimeMarkIoSample(busIndex,
                                elapsedUs,
                                result.requestedCount,
                                result.successCount);
    }

    if ((terminalBusMask_ & requestedBusMask_) != requestedBusMask_) {
        nextCollectUs_ = deadlineUs_;
        return false;
    }

    completedMeta->sequence = sequence_;
    completedMeta->cycleStartedUs = startedUs_;
    completedMeta->cycleCompletedUs = nowUs;
    memcpy(completedMeta->busSampleTimestampUs,
           busSampleTimestampUs_, sizeof(busSampleTimestampUs_));
    completedMeta->requestedBusMask = requestedBusMask_;
    completedMeta->freshBusMask = freshBusMask_;
    completedMeta->timeoutBusMask = timeoutBusMask_;
    active_ = false;
    requestedBusMask_ = 0;
    return true;
}

void ParallelServoFeedbackCycle::abort()
{
    for (uint8_t busIndex = 0; busIndex < NUM_BUSES; ++busIndex) {
        FeetechServoDriver* driver = servoBusRegistry().driver(busIndex);
        if (driver) driver->abortSyncRead(false);
    }
    active_ = false;
    requestedBusMask_ = terminalBusMask_ = freshBusMask_ = timeoutBusMask_ = 0;
}

bool ParallelServoFeedbackCycle::collectDue(uint64_t nowUs) const
{
    return active_ && nowUs >= nextCollectUs_;
}

uint32_t ParallelServoFeedbackCycle::waitMs(uint64_t nowUs) const
{
    if (!active_ || nowUs >= nextCollectUs_) return 0;
    const uint64_t remainingUs = nextCollectUs_ - nowUs;
    return (uint32_t)((remainingUs + 999ULL) / 1000ULL);
}
