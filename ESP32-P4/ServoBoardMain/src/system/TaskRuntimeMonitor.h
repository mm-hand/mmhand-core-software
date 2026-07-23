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
    uint32_t overrunCount;
    uint16_t consecutiveOverruns;
    uint16_t maxConsecutiveOverruns;
    uint16_t core0IdlePermille;
    uint16_t core1IdlePermille;
    uint32_t systemStatusMaxGapMs;
    uint32_t stopLastLatencyUs;
    uint32_t stopMaxLatencyUs;
    uint32_t timestampMs;
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
