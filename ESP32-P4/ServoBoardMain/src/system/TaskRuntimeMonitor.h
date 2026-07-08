#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

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

void taskRuntimeMonitorTask(void* parameter);
