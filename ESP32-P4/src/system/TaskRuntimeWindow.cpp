/**
 * @file TaskRuntimeWindow.cpp
 * @brief 实现任务执行时间的固定直方图统计。
 * @details 0~2 ms 使用 50 us 分桶，2~10 ms 使用 500 us 分桶，之后使用倍增桶。
 * 输入：由各实时任务上报的单轮执行微秒数。
 * 输出：近似但保守的 P95/P99，适合实时调度余量判断。
 * 状态所有权：不持有全局状态；全部状态由调用方传入。
 * 禁止依赖：禁止 FreeRTOS 锁、串口日志和动态内存。
 */
#include "TaskRuntimeWindow.h"

#include <string.h>

namespace {

uint8_t bucketForExecUs(uint32_t execUs)
{
    if (execUs < 2000U) return (uint8_t)(execUs / 50U);
    if (execUs < 10000U) return (uint8_t)(40U + ((execUs - 2000U) / 500U));
    if (execUs < 20000U) return 56U;
    if (execUs < 40000U) return 57U;
    if (execUs < 80000U) return 58U;
    if (execUs < 160000U) return 59U;
    if (execUs < 320000U) return 60U;
    return 61U;
}

uint32_t bucketUpperUs(uint8_t bucket)
{
    if (bucket < 40U) return ((uint32_t)bucket + 1U) * 50U;
    if (bucket < 56U) return 2000U + ((uint32_t)(bucket - 39U) * 500U);
    static const uint32_t kUpperBounds[] = {
        20000U, 40000U, 80000U, 160000U, 320000U, 0xFFFFFFFFUL,
    };
    return kUpperBounds[bucket - 56U];
}

} // namespace

void taskRuntimeWindowReset(TaskRuntimeWindowAccumulator* window)
{
    if (window) memset(window, 0, sizeof(*window));
}

void taskRuntimeWindowAddExec(TaskRuntimeWindowAccumulator* window, uint32_t execUs)
{
    if (!window) return;
    window->loopCount++;
    window->execTotalUs += execUs;
    if (execUs > window->maxExecUs) window->maxExecUs = execUs;
    window->execBuckets[bucketForExecUs(execUs)]++;
}

uint32_t taskRuntimeWindowQuantileUs(const TaskRuntimeWindowAccumulator& window,
                                     uint16_t permille)
{
    if (window.loopCount == 0U) return 0U;
    const uint64_t required = ((uint64_t)window.loopCount * permille + 999ULL) / 1000ULL;
    uint64_t observed = 0;
    for (uint8_t bucket = 0; bucket < kTaskRuntimeExecBucketCount; bucket++) {
        observed += window.execBuckets[bucket];
        if (observed >= required) return bucketUpperUs(bucket);
    }
    return window.maxExecUs;
}
