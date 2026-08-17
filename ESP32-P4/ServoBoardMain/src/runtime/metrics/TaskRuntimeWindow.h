/**
 * @file TaskRuntimeWindow.h
 * @brief 定义任务一秒执行时间窗口的无动态内存累计器。
 * @details 实时任务只写固定计数和直方图桶；P95/P99 在 RuntimeMonitor 中计算。
 * 输入：每轮任务的执行时间，单位微秒。
 * 输出：窗口次数、平均值、P95/P99 与窗口最大值。
 * 状态所有权：TaskRuntimeMetrics 的任务槽拥有本结构。
 * 禁止依赖：不依赖 FreeRTOS、串口、日志或堆分配。
 */
#ifndef P4_RUNTIME_METRICS_TASK_RUNTIME_WINDOW_H
#define P4_RUNTIME_METRICS_TASK_RUNTIME_WINDOW_H

#include <stdint.h>

#include "RuntimeMonitorConfig.h"

struct TaskRuntimeWindowAccumulator {
    uint32_t loopCount;
    uint64_t execTotalUs;
    uint32_t maxExecUs;
    uint32_t execBuckets[kTaskRuntimeExecBucketCount];
};

void taskRuntimeWindowReset(TaskRuntimeWindowAccumulator* window);
void taskRuntimeWindowAddExec(TaskRuntimeWindowAccumulator* window, uint32_t execUs);
uint32_t taskRuntimeWindowQuantileUs(const TaskRuntimeWindowAccumulator& window,
                                     uint16_t permille);

#endif // P4_RUNTIME_METRICS_TASK_RUNTIME_WINDOW_H
