/**
 * @file TaskRuntimeMetrics.h
 * @brief 定义跨任务可调用的运行时监控数据与打点接口。
 * @details 业务服务依赖本端口而非 RuntimeMonitorTask，避免 core 反向依赖 tasks。
 */
#ifndef P4_RUNTIME_METRICS_TASK_RUNTIME_METRICS_H
#define P4_RUNTIME_METRICS_TASK_RUNTIME_METRICS_H

#include <Arduino.h>
#include "../../platform/rtos/FreeRtosKernel.h"

enum TaskRuntimeId : uint8_t {
    TASK_RUNTIME_HOST_COMM = 0,   ///< 上位机串口收发任务。
    TASK_RUNTIME_SYSTEM_STATE = 1,///< 系统状态机任务。
    TASK_RUNTIME_S3_CAN_RX = 2,     ///< S3 CAN 通信任务。
    TASK_RUNTIME_SERVO_BUS = 3,   ///< 舵机总线收发任务。
    TASK_RUNTIME_MOTION_CONTROL = 4,      ///< 负反馈控制任务。
    TASK_RUNTIME_AUTO_CALIB = 5,   ///< 动态创建的自动标定任务。
    TASK_RUNTIME_MONITOR = 6,      ///< 运行时监控任务。
    TASK_RUNTIME_COUNT = 7,        ///< 任务统计槽位总数。
};

/**
 * @brief 单个 FreeRTOS 任务的运行时快照。
 * @details 字段仅用于诊断上报，不参与任务调度，也不得由上位机回写。
 */
struct TaskRuntimeSnapshot {
    uint8_t taskId;                    ///< TaskRuntimeId，对应稳定任务编号。
    uint8_t active;                    ///< 任务已注册且仍存活时为 1。
    uint8_t priority;                  ///< FreeRTOS 当前任务优先级。
    int8_t coreId;                     ///< 固定运行核；未固定时为 -1。
    uint32_t stackAllocatedBytes;      ///< 创建任务时分配的栈空间。
    uint32_t stackCurrentFreeBytes;    ///< 当前剩余栈空间估计值。
    uint32_t stackMinFreeBytes;        ///< 历史最小剩余栈空间。
    uint32_t loopCount;                ///< 自注册以来完成的主循环次数。
    uint32_t maxGapUs;                 ///< 相邻两次循环开始之间的最大间隔。
    uint32_t maxExecUs;                ///< 单次循环的最大执行时间。
    uint32_t overrunCount;             ///< 超出目标周期的累计次数。
    uint16_t consecutiveOverruns;      ///< 当前连续超期次数。
    uint16_t maxConsecutiveOverruns;   ///< 历史最大连续超期次数。
    uint16_t core0IdlePermille;        ///< Core0 空闲率，千分比。
    uint16_t core1IdlePermille;        ///< Core1 空闲率，千分比。
    uint32_t systemStatusMaxGapMs;     ///< 系统状态包的历史最大发送间隔。
    uint32_t stopLastLatencyUs;        ///< 最近一次 STOP 到扭矩关闭的耗时。
    uint32_t stopMaxLatencyUs;         ///< STOP 到扭矩关闭的历史最大耗时。
    uint32_t timestampMs;              ///< 生成快照时的 P4 单调时钟。
};

// 以下接口由任务实现层提供，core 只依赖稳定端口，避免反向包含 tasks 头文件。
void taskRuntimeRegister(uint8_t taskId, TaskHandle_t handle, uint32_t stackAllocatedBytes);
void taskRuntimeUnregister(uint8_t taskId);
void taskRuntimeMarkLoopStart(uint8_t taskId);
void taskRuntimeMarkLoopEnd(uint8_t taskId);
void taskRuntimeMarkPeriodResult(uint8_t taskId, bool overrun);
bool taskRuntimeGetSnapshot(uint8_t taskId, TaskRuntimeSnapshot* out);
void taskRuntimeMarkSystemStatusSent(void);
void taskRuntimeMarkStopReceived(void);
void taskRuntimeMarkStopRequested(void);
void taskRuntimeMarkStopTorqueOffComplete(void);

#endif // P4_RUNTIME_METRICS_TASK_RUNTIME_METRICS_H
