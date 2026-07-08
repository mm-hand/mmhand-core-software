#include "TaskRuntimeMonitor.h"

#include <esp_timer.h>
#include <string.h>

namespace {

struct TaskRuntimeSlot {
    TaskHandle_t handle;
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
    int64_t lastLoopStartUs;
    int64_t currentLoopStartUs;
};

TaskRuntimeSlot g_slots[TASK_RUNTIME_COUNT] = {};
portMUX_TYPE g_runtimeLock = portMUX_INITIALIZER_UNLOCKED;
uint16_t g_coreIdlePermille[2] = {0xFFFF, 0xFFFF};
uint32_t g_lastSystemStatusMs = 0;
uint32_t g_systemStatusMaxGapMs = 0;
int64_t g_stopReceivedUs = 0;
int64_t g_stopRequestedUs = 0;
uint32_t g_stopLastLatencyUs = 0;
uint32_t g_stopMaxLatencyUs = 0;

#if defined(configGENERATE_RUN_TIME_STATS) && (configGENERATE_RUN_TIME_STATS == 1) && \
    defined(configUSE_TRACE_FACILITY) && (configUSE_TRACE_FACILITY == 1)
#if defined(configRUN_TIME_COUNTER_TYPE)
using TaskRuntimeCounterType = configRUN_TIME_COUNTER_TYPE;
#else
using TaskRuntimeCounterType = uint32_t;
#endif
TaskStatus_t g_taskStatusBuffer[32];
TaskRuntimeCounterType g_previousTotalRuntime = 0;
TaskRuntimeCounterType g_previousIdleRuntime[2] = {0, 0};
#endif

uint32_t clampUs(int64_t value)
{
    if (value <= 0) return 0;
    if (value > 0xFFFFFFFFLL) return 0xFFFFFFFFUL;
    return (uint32_t)value;
}

uint32_t sampleCurrentTaskStackFreeBytes()
{
#if defined(INCLUDE_pxTaskGetStackStart) && (INCLUDE_pxTaskGetStackStart == 1)
    volatile uint8_t marker = 0;
    uint8_t* stackStart = (uint8_t*)pxTaskGetStackStart(NULL);
    if (!stackStart) return 0;
    const intptr_t freeBytes = (intptr_t)&marker - (intptr_t)stackStart;
    return freeBytes > 0 ? (uint32_t)freeBytes : 0;
#else
    return 0;
#endif
}

void sampleTaskStacks()
{
    for (uint8_t id = 0; id < TASK_RUNTIME_COUNT; id++) {
        TaskHandle_t handle = NULL;
        portENTER_CRITICAL(&g_runtimeLock);
        handle = g_slots[id].handle;
        portEXIT_CRITICAL(&g_runtimeLock);
        if (!handle) continue;

        // ESP-IDF FreeRTOS reports the high-water mark in bytes.
        const uint32_t freeBytes = (uint32_t)uxTaskGetStackHighWaterMark(handle);
        const uint8_t priority = (uint8_t)uxTaskPriorityGet(handle);
        portENTER_CRITICAL(&g_runtimeLock);
        g_slots[id].priority = priority;
        if (g_slots[id].stackMinFreeBytes == 0 || freeBytes < g_slots[id].stackMinFreeBytes) {
            g_slots[id].stackMinFreeBytes = freeBytes;
        }
        portEXIT_CRITICAL(&g_runtimeLock);
    }
}

void sampleCoreIdle()
{
#if defined(configGENERATE_RUN_TIME_STATS) && (configGENERATE_RUN_TIME_STATS == 1) && \
    defined(configUSE_TRACE_FACILITY) && (configUSE_TRACE_FACILITY == 1)
    TaskRuntimeCounterType totalRuntime = 0;
    const UBaseType_t count = uxTaskGetSystemState(
        g_taskStatusBuffer,
        sizeof(g_taskStatusBuffer) / sizeof(g_taskStatusBuffer[0]),
        &totalRuntime
    );
    if (count == 0 || g_previousTotalRuntime == 0) {
        g_previousTotalRuntime = totalRuntime;
        for (UBaseType_t i = 0; i < count; i++) {
            if (strcmp(g_taskStatusBuffer[i].pcTaskName, "IDLE0") == 0) {
                g_previousIdleRuntime[0] = g_taskStatusBuffer[i].ulRunTimeCounter;
            } else if (strcmp(g_taskStatusBuffer[i].pcTaskName, "IDLE1") == 0) {
                g_previousIdleRuntime[1] = g_taskStatusBuffer[i].ulRunTimeCounter;
            }
        }
        return;
    }

    const TaskRuntimeCounterType totalDelta = totalRuntime - g_previousTotalRuntime;
#if defined(configNUMBER_OF_CORES)
    const uint32_t coreCount = configNUMBER_OF_CORES;
#elif defined(portNUM_PROCESSORS)
    const uint32_t coreCount = portNUM_PROCESSORS;
#else
    const uint32_t coreCount = 1;
#endif
    const TaskRuntimeCounterType perCoreBudget = coreCount > 0 ? totalDelta / coreCount : totalDelta;
    TaskRuntimeCounterType idleNow[2] = {g_previousIdleRuntime[0], g_previousIdleRuntime[1]};
    for (UBaseType_t i = 0; i < count; i++) {
        if (strcmp(g_taskStatusBuffer[i].pcTaskName, "IDLE0") == 0) {
            idleNow[0] = g_taskStatusBuffer[i].ulRunTimeCounter;
        } else if (strcmp(g_taskStatusBuffer[i].pcTaskName, "IDLE1") == 0) {
            idleNow[1] = g_taskStatusBuffer[i].ulRunTimeCounter;
        }
    }

    portENTER_CRITICAL(&g_runtimeLock);
    for (uint8_t core = 0; core < 2; core++) {
        if (perCoreBudget > 0 && idleNow[core] >= g_previousIdleRuntime[core]) {
            const TaskRuntimeCounterType idleDelta = idleNow[core] - g_previousIdleRuntime[core];
            uint32_t permille = (uint32_t)(((uint64_t)idleDelta * 1000ULL) / perCoreBudget);
            if (permille > 1000) permille = 1000;
            g_coreIdlePermille[core] = (uint16_t)permille;
        }
    }
    portEXIT_CRITICAL(&g_runtimeLock);

    g_previousTotalRuntime = totalRuntime;
    g_previousIdleRuntime[0] = idleNow[0];
    g_previousIdleRuntime[1] = idleNow[1];
#endif
}

} // namespace

void taskRuntimeRegister(uint8_t taskId, TaskHandle_t handle, uint32_t stackAllocatedBytes)
{
    if (taskId >= TASK_RUNTIME_COUNT || !handle) return;
    const uint8_t priority = (uint8_t)uxTaskPriorityGet(handle);
    portENTER_CRITICAL(&g_runtimeLock);
    TaskRuntimeSlot& slot = g_slots[taskId];
    memset(&slot, 0, sizeof(slot));
    slot.handle = handle;
    slot.active = 1;
    slot.priority = priority;
    slot.coreId = -1;
    slot.stackAllocatedBytes = stackAllocatedBytes;
    portEXIT_CRITICAL(&g_runtimeLock);
}

void taskRuntimeUnregister(uint8_t taskId)
{
    if (taskId >= TASK_RUNTIME_COUNT) return;
    const uint32_t freeBytes = (uint32_t)uxTaskGetStackHighWaterMark(NULL);
    portENTER_CRITICAL(&g_runtimeLock);
    TaskRuntimeSlot& slot = g_slots[taskId];
    if (slot.stackMinFreeBytes == 0 || freeBytes < slot.stackMinFreeBytes) {
        slot.stackMinFreeBytes = freeBytes;
    }
    slot.stackCurrentFreeBytes = freeBytes;
    slot.active = 0;
    slot.handle = NULL;
    portEXIT_CRITICAL(&g_runtimeLock);
}

void taskRuntimeMarkLoopStart(uint8_t taskId)
{
    if (taskId >= TASK_RUNTIME_COUNT) return;
    const int64_t nowUs = esp_timer_get_time();
    const int8_t coreId = (int8_t)xPortGetCoreID();
    const uint32_t currentStackFreeBytes = sampleCurrentTaskStackFreeBytes();
    portENTER_CRITICAL(&g_runtimeLock);
    TaskRuntimeSlot& slot = g_slots[taskId];
    if (slot.lastLoopStartUs > 0) {
        const uint32_t gapUs = clampUs(nowUs - slot.lastLoopStartUs);
        if (gapUs > slot.maxGapUs) slot.maxGapUs = gapUs;
    }
    slot.lastLoopStartUs = nowUs;
    slot.currentLoopStartUs = nowUs;
    slot.coreId = coreId;
    if (currentStackFreeBytes > 0) {
        slot.stackCurrentFreeBytes = currentStackFreeBytes;
    }
    slot.loopCount++;
    portEXIT_CRITICAL(&g_runtimeLock);
}

void taskRuntimeMarkLoopEnd(uint8_t taskId)
{
    if (taskId >= TASK_RUNTIME_COUNT) return;
    const int64_t nowUs = esp_timer_get_time();
    const uint32_t currentStackFreeBytes = sampleCurrentTaskStackFreeBytes();
    portENTER_CRITICAL(&g_runtimeLock);
    TaskRuntimeSlot& slot = g_slots[taskId];
    if (slot.currentLoopStartUs > 0) {
        const uint32_t execUs = clampUs(nowUs - slot.currentLoopStartUs);
        if (execUs > slot.maxExecUs) slot.maxExecUs = execUs;
    }
    if (currentStackFreeBytes > 0) {
        slot.stackCurrentFreeBytes = currentStackFreeBytes;
    }
    portEXIT_CRITICAL(&g_runtimeLock);
}

void taskRuntimeMarkPeriodResult(uint8_t taskId, bool overrun)
{
    if (taskId >= TASK_RUNTIME_COUNT) return;
    portENTER_CRITICAL(&g_runtimeLock);
    TaskRuntimeSlot& slot = g_slots[taskId];
    if (overrun) {
        slot.overrunCount++;
        if (slot.consecutiveOverruns < 0xFFFF) slot.consecutiveOverruns++;
        if (slot.consecutiveOverruns > slot.maxConsecutiveOverruns) {
            slot.maxConsecutiveOverruns = slot.consecutiveOverruns;
        }
    } else {
        slot.consecutiveOverruns = 0;
    }
    portEXIT_CRITICAL(&g_runtimeLock);
}

bool taskRuntimeGetSnapshot(uint8_t taskId, TaskRuntimeSnapshot* out)
{
    if (taskId >= TASK_RUNTIME_COUNT || !out) return false;
    portENTER_CRITICAL(&g_runtimeLock);
    const TaskRuntimeSlot& slot = g_slots[taskId];
    out->taskId = taskId;
    out->active = slot.active;
    out->priority = slot.priority;
    out->coreId = slot.coreId;
    out->stackAllocatedBytes = slot.stackAllocatedBytes;
    out->stackCurrentFreeBytes = slot.stackCurrentFreeBytes;
    if (out->stackAllocatedBytes > 0 && out->stackCurrentFreeBytes > out->stackAllocatedBytes) {
        out->stackCurrentFreeBytes = out->stackAllocatedBytes;
    }
    out->stackMinFreeBytes = slot.stackMinFreeBytes;
    out->loopCount = slot.loopCount;
    out->maxGapUs = slot.maxGapUs;
    out->maxExecUs = slot.maxExecUs;
    out->overrunCount = slot.overrunCount;
    out->consecutiveOverruns = slot.consecutiveOverruns;
    out->maxConsecutiveOverruns = slot.maxConsecutiveOverruns;
    out->core0IdlePermille = g_coreIdlePermille[0];
    out->core1IdlePermille = g_coreIdlePermille[1];
    out->systemStatusMaxGapMs = g_systemStatusMaxGapMs;
    out->stopLastLatencyUs = g_stopLastLatencyUs;
    out->stopMaxLatencyUs = g_stopMaxLatencyUs;
    out->timestampMs = millis();
    portEXIT_CRITICAL(&g_runtimeLock);
    return true;
}

void taskRuntimeMarkSystemStatusSent(void)
{
    const uint32_t nowMs = millis();
    portENTER_CRITICAL(&g_runtimeLock);
    if (g_lastSystemStatusMs != 0) {
        const uint32_t gapMs = nowMs - g_lastSystemStatusMs;
        if (gapMs > g_systemStatusMaxGapMs) g_systemStatusMaxGapMs = gapMs;
    }
    g_lastSystemStatusMs = nowMs;
    portEXIT_CRITICAL(&g_runtimeLock);
}

void taskRuntimeMarkStopReceived(void)
{
    const int64_t nowUs = esp_timer_get_time();
    portENTER_CRITICAL(&g_runtimeLock);
    g_stopReceivedUs = nowUs;
    g_stopRequestedUs = 0;
    portEXIT_CRITICAL(&g_runtimeLock);
}

void taskRuntimeMarkStopRequested(void)
{
    const int64_t nowUs = esp_timer_get_time();
    portENTER_CRITICAL(&g_runtimeLock);
    g_stopRequestedUs = nowUs;
    portEXIT_CRITICAL(&g_runtimeLock);
}

void taskRuntimeMarkStopTorqueOffComplete(void)
{
    const int64_t nowUs = esp_timer_get_time();
    portENTER_CRITICAL(&g_runtimeLock);
    if (g_stopReceivedUs > 0 && g_stopRequestedUs >= g_stopReceivedUs) {
        g_stopLastLatencyUs = clampUs(nowUs - g_stopReceivedUs);
        if (g_stopLastLatencyUs > g_stopMaxLatencyUs) {
            g_stopMaxLatencyUs = g_stopLastLatencyUs;
        }
        g_stopReceivedUs = 0;
        g_stopRequestedUs = 0;
    }
    portEXIT_CRITICAL(&g_runtimeLock);
}

void taskRuntimeMonitorTask(void* parameter)
{
    (void)parameter;
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t periodTicks = pdMS_TO_TICKS(1000);
    for (;;) {
        taskRuntimeMarkLoopStart(TASK_RUNTIME_MONITOR);
        sampleTaskStacks();
        sampleCoreIdle();
        taskRuntimeMarkLoopEnd(TASK_RUNTIME_MONITOR);
        const TickType_t nextWakeTime = lastWakeTime + periodTicks;
        const bool overrun = (int32_t)(xTaskGetTickCount() - nextWakeTime) >= 0;
        taskRuntimeMarkPeriodResult(TASK_RUNTIME_MONITOR, overrun);
        vTaskDelayUntil(&lastWakeTime, periodTicks);
        if (overrun) vTaskDelay(1);
    }
}
