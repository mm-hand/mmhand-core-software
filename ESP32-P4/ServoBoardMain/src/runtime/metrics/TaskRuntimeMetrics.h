/**
 * @file TaskRuntimeMetrics.h
 * @brief 定义跨任务可调用的运行时监控数据与打点接口。
 * @details 业务任务只调用本接口记录周期和执行耗时；低优先级 RuntimeMonitor 负责一秒汇总。
 * 输入：任务循环开始、结束和周期结果。
 * 输出：冻结后的 TaskRuntimeSnapshot，供上位机诊断读取。
 * 状态所有权：统计状态由 RuntimeMonitorService 独占维护。
 * 禁止依赖：禁止依赖 UI、串口协议和业务控制模块。
 */
#ifndef P4_RUNTIME_METRICS_TASK_RUNTIME_METRICS_H
#define P4_RUNTIME_METRICS_TASK_RUNTIME_METRICS_H

#include <Arduino.h>

#include "../../platform/rtos/FreeRtosKernel.h"

enum TaskRuntimeId : uint8_t {
    TASK_RUNTIME_HOST_COMM = 0,
    TASK_RUNTIME_SYSTEM_STATE = 1,
    TASK_RUNTIME_S3_CAN_RX = 2,
    TASK_RUNTIME_SERVO_BUS = 3,
    TASK_RUNTIME_MOTION_CONTROL = 4,
    TASK_RUNTIME_RESERVED_5 = 5, ///< 旧自动标定槽已退役，保留后续任务的线协议编号。
    TASK_RUNTIME_MONITOR = 6,
    TASK_RUNTIME_COUNT = 7,
};

inline constexpr uint8_t TASK_RUNTIME_IO_CHANNEL_COUNT = 4;

/**
 * @brief 单个任务的冻结诊断快照。
 * @details ``window*`` 字段描述最近一秒，``max*`` 字段保留固件启动后的历史最坏值。
 */
struct TaskRuntimeSnapshot {
    uint8_t taskId;
    uint8_t active;
    uint8_t priority;
    int8_t coreId;
    uint32_t stackAllocatedBytes;
    uint32_t stackCurrentFreeBytes;
    uint32_t stackMinFreeBytes;
    uint32_t loopCount;
    uint32_t maxGapUs;
    uint32_t maxExecUs;
    uint32_t windowSequence;
    uint32_t windowDurationUs;
    uint32_t windowLoopCount;
    uint32_t windowPeriodicCount;
    uint32_t windowEventCount;
    uint32_t avgExecUs;
    uint32_t p95ExecUs;
    uint32_t p99ExecUs;
    uint32_t windowMaxExecUs;
    uint16_t cpuPermille;
    uint32_t overrunCount;
    uint16_t consecutiveOverruns;
    uint16_t maxConsecutiveOverruns;
    uint16_t core0IdlePermille;
    uint16_t core1IdlePermille;
    uint32_t systemStatusMaxGapMs;
    uint32_t stopLastLatencyUs;
    uint32_t stopMaxLatencyUs;
    uint32_t timestampMs;
    uint32_t ioSampleCount[TASK_RUNTIME_IO_CHANNEL_COUNT];
    uint32_t ioRequestedCount[TASK_RUNTIME_IO_CHANNEL_COUNT];
    uint32_t ioSucceededCount[TASK_RUNTIME_IO_CHANNEL_COUNT];
    uint32_t ioAvgUs[TASK_RUNTIME_IO_CHANNEL_COUNT];
    uint32_t ioP95Us[TASK_RUNTIME_IO_CHANNEL_COUNT];
    uint32_t ioP99Us[TASK_RUNTIME_IO_CHANNEL_COUNT];
};

void taskRuntimeRegister(uint8_t taskId, TaskHandle_t handle, uint32_t stackAllocatedBytes);
void taskRuntimeUnregister(uint8_t taskId);
void taskRuntimeMarkLoopStart(uint8_t taskId);
void taskRuntimeMarkLoopEnd(uint8_t taskId);
/** 记录一次调度来源；同一轮可同时属于周期和事件，用于识别事件/周期合并。 */
void taskRuntimeMarkCycle(uint8_t taskId, bool periodicDue, bool eventWake);
/** 记录 ServoBus 单条 BUS 的同步读耗时与反馈数量；固定数组、无日志、无堆分配。 */
void taskRuntimeMarkIoSample(uint8_t channel,
                             uint32_t elapsedUs,
                             uint32_t requestedCount,
                             uint32_t succeededCount);
void taskRuntimeMarkPeriodResult(uint8_t taskId, bool overrun);
bool taskRuntimeGetSnapshot(uint8_t taskId, TaskRuntimeSnapshot* out);
uint32_t taskRuntimeGetWindowSequence();
void taskRuntimeMarkSystemStatusSent(void);
void taskRuntimeMarkStopReceived(void);
void taskRuntimeMarkStopRequested(void);
void taskRuntimeMarkStopTorqueOffComplete(void);

#endif // P4_RUNTIME_METRICS_TASK_RUNTIME_METRICS_H
