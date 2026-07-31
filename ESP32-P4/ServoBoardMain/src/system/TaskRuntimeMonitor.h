/**
 * @file TaskRuntimeMonitor.h
 * @brief 定义任务运行时监测数据结构及记录接口。
 */
#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// 固定诊断槽编号同时也是上位机 TASK_RUNTIME 数据包中的 taskId。
enum TaskRuntimeId : uint8_t {
    TASK_RUNTIME_UPPER_COMM = 0,
    TASK_RUNTIME_STATE_MACHINE = 1,
    TASK_RUNTIME_CAN_COMM = 2,
    TASK_RUNTIME_SERVO_COMM = 3,
    TASK_RUNTIME_CONTROL = 4,
    TASK_RUNTIME_AUTO_CALIB = 5,
    TASK_RUNTIME_MONITOR = 6,
    TASK_RUNTIME_COUNT = 7,
};

// 单个任务的累计运行诊断。时间单位除明确标注 Ms 外均为微秒。
struct TaskRuntimeSnapshot {
    uint8_t taskId;                 ///< TASK_RUNTIME_* 固定槽号。
    uint8_t active;                 ///< 任务是否已注册且句柄有效。
    uint8_t priority;               ///< FreeRTOS 当前优先级。
    int8_t coreId;                  ///< -1 未绑定；0/1 为固定核心。
    uint32_t stackAllocatedBytes;   ///< 创建任务时分配的栈字节数。
    uint32_t stackCurrentFreeBytes; ///< 最近采样的剩余栈。
    uint32_t stackMinFreeBytes;     ///< 启动以来最小剩余栈。
    uint32_t loopCount;             ///< 完成的任务循环总数。
    uint32_t maxGapUs;              ///< 相邻循环开始的最大间隔。
    uint32_t maxExecUs;             ///< 单轮循环最大执行时间。
    uint32_t overrunCount;          ///< 周期超限累计次数。
    uint16_t consecutiveOverruns;   ///< 当前连续超限次数。
    uint16_t maxConsecutiveOverruns;///< 历史最大连续超限次数。
    uint16_t core0IdlePermille;     ///< Core0 idle 千分比。
    uint16_t core1IdlePermille;     ///< Core1 idle 千分比。
    uint32_t systemStatusMaxGapMs;  ///< SYSTEM_STATUS 最大发送间隔。
    uint32_t stopLastLatencyUs;     ///< 最近 STOP 到断扭矩延迟。
    uint32_t stopMaxLatencyUs;      ///< 历史最大 STOP 延迟。
    uint32_t timestampMs;           ///< 生成快照的 P4 millis()。
};

// 任务创建/销毁时登记句柄和分配栈，AutoCalibrationTask 会动态注册和注销。
void taskRuntimeRegister(uint8_t taskId, TaskHandle_t handle, uint32_t stackAllocatedBytes);
void taskRuntimeUnregister(uint8_t taskId);

// 在任务每轮循环首尾打点，并记录该轮是否错过预期周期。
void taskRuntimeMarkLoopStart(uint8_t taskId);
void taskRuntimeMarkLoopEnd(uint8_t taskId);
void taskRuntimeMarkPeriodResult(uint8_t taskId, bool overrun);
bool taskRuntimeGetSnapshot(uint8_t taskId, TaskRuntimeSnapshot* out);

// 记录 SYSTEM_STATUS 周期和 STOP 从接收、请求到舵机断扭矩完成的时延。
void taskRuntimeMarkSystemStatusSent(void);
void taskRuntimeMarkStopReceived(void);
void taskRuntimeMarkStopRequested(void);
void taskRuntimeMarkStopTorqueOffComplete(void);

// 1 Hz 后台采样任务：更新任务栈高水位和双核 idle 比例。
void taskRuntimeMonitorTask(void* parameter);
