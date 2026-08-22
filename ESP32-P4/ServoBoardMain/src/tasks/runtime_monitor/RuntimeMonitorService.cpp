/**
 * @file RuntimeMonitorService.cpp
 * @brief 汇总各实时任务的执行时间、CPU、栈和周期诊断。
 * @details 实时任务只调用轻量打点接口；本服务每秒读取 FreeRTOS 运行计数并冻结诊断快照。
 * 输入：TaskRuntimeMetrics 打点、FreeRTOS 任务运行计数和栈水位。
 * 输出：稳定的一秒 TaskRuntimeSnapshot，供 HostComm 按需上报。
 * 状态所有权：本文件独占任务统计槽、窗口快照和 CPU 基线。
 * 禁止依赖：禁止依赖上位机协议、UI、控制逻辑或舵机 SDK。
 */
#include "RuntimeMonitorService.h"

#include "../../platform/time/Clock.h"
#include "../../runtime/metrics/RuntimeMonitorConfig.h"
#include "../../runtime/metrics/TaskRuntimeMetrics.h"
#include "../../runtime/metrics/TaskRuntimeWindow.h"
#include "../../safety/SafetyEventReporter.h"
#include "../../safety/policies/RuntimeSafetyPolicy.h"

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
    uint16_t cpuPermille;
    uint32_t overrunCount;
    uint16_t consecutiveOverruns;
    uint16_t maxConsecutiveOverruns;
    int64_t lastLoopStartUs;
    int64_t currentLoopStartUs;
    TaskRuntimeWindowAccumulator window;
    bool overrunEventLatched;
    bool stackEventLatched;
};

TaskRuntimeSlot g_slots[TASK_RUNTIME_COUNT] = {};
TaskRuntimeSnapshot g_frozenSnapshots[TASK_RUNTIME_COUNT] = {};
// 仅 RuntimeMonitor 使用的静态暂存区，避免在 4 KiB 任务栈上创建大数组。
TaskRuntimeSlot g_windowSlots[TASK_RUNTIME_COUNT] = {};
TaskRuntimeSnapshot g_windowSnapshots[TASK_RUNTIME_COUNT] = {};
TaskRuntimeWindowAccumulator g_servoIoWindows[TASK_RUNTIME_IO_CHANNEL_COUNT] = {};
TaskRuntimeWindowAccumulator g_windowServoIo[TASK_RUNTIME_IO_CHANNEL_COUNT] = {};
uint32_t g_servoIoRequested[TASK_RUNTIME_IO_CHANNEL_COUNT] = {};
uint32_t g_servoIoSucceeded[TASK_RUNTIME_IO_CHANNEL_COUNT] = {};
uint32_t g_windowServoIoRequested[TASK_RUNTIME_IO_CHANNEL_COUNT] = {};
uint32_t g_windowServoIoSucceeded[TASK_RUNTIME_IO_CHANNEL_COUNT] = {};
portMUX_TYPE g_runtimeLock = portMUX_INITIALIZER_UNLOCKED;
uint16_t g_coreIdlePermille[2] = {
    kTaskRuntimeUnknownPermille, kTaskRuntimeUnknownPermille,
};
uint32_t g_windowSequence = 0;
int64_t g_windowStartUs = 0;
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
inline constexpr uint8_t kRuntimeStatusCapacity = 40U;
TaskStatus_t g_taskStatusBuffer[kRuntimeStatusCapacity];
TaskRuntimeCounterType g_previousTotalRuntime = 0;
TaskRuntimeCounterType g_previousIdleRuntime[2] = {0, 0};
TaskRuntimeCounterType g_previousTaskRuntime[TASK_RUNTIME_COUNT] = {};
TaskHandle_t g_previousTaskHandles[TASK_RUNTIME_COUNT] = {};
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
        uint32_t allocated = 0;
        bool stackLatched = false;
        portENTER_CRITICAL(&g_runtimeLock);
        handle = g_slots[id].handle;
        allocated = g_slots[id].stackAllocatedBytes;
        stackLatched = g_slots[id].stackEventLatched;
        portEXIT_CRITICAL(&g_runtimeLock);
        if (!handle) continue;

        const uint32_t freeBytes = (uint32_t)uxTaskGetStackHighWaterMark(handle);
        const uint8_t priority = (uint8_t)uxTaskPriorityGet(handle);
        const uint32_t threshold = p4::safety::runtimeStackWarningThreshold(allocated, 512U);
        const p4::safety::RuntimeLatchDecision decision =
            p4::safety::evaluateRuntimeStack(freeBytes, threshold, stackLatched);
        portENTER_CRITICAL(&g_runtimeLock);
        TaskRuntimeSlot& slot = g_slots[id];
        slot.priority = priority;
        if (slot.stackMinFreeBytes == 0 || freeBytes < slot.stackMinFreeBytes) {
            slot.stackMinFreeBytes = freeBytes;
        }
        slot.stackEventLatched = decision.active;
        portEXIT_CRITICAL(&g_runtimeLock);
        if (decision.entered) {
            p4::safety::reportSafetyRuleEvent(
                nullptr, SafetyRuleId::RuntimeTaskStackLow,
                (int32_t)threshold, (int32_t)freeBytes);
        }
    }
}

#if defined(configGENERATE_RUN_TIME_STATS) && (configGENERATE_RUN_TIME_STATS == 1) && \
    defined(configUSE_TRACE_FACILITY) && (configUSE_TRACE_FACILITY == 1)
const TaskStatus_t* findTaskStatus(TaskHandle_t handle, UBaseType_t count)
{
    if (!handle) return nullptr;
    for (UBaseType_t i = 0; i < count; i++) {
        if (g_taskStatusBuffer[i].xHandle == handle) return &g_taskStatusBuffer[i];
    }
    return nullptr;
}
#endif

/**
 * @brief 从 FreeRTOS 运行计数计算每核 Idle 与每任务 CPU 占比。
 * @details totalRuntime 是全局单调时钟而非双核累计预算；每个 IDLEn 必须直接除以该时钟增量。
 */
void sampleCpuUsage()
{
#if defined(configGENERATE_RUN_TIME_STATS) && (configGENERATE_RUN_TIME_STATS == 1) && \
    defined(configUSE_TRACE_FACILITY) && (configUSE_TRACE_FACILITY == 1)
    TaskHandle_t handles[TASK_RUNTIME_COUNT] = {};
    portENTER_CRITICAL(&g_runtimeLock);
    for (uint8_t id = 0; id < TASK_RUNTIME_COUNT; id++) handles[id] = g_slots[id].handle;
    portEXIT_CRITICAL(&g_runtimeLock);

    TaskRuntimeCounterType totalRuntime = 0;
    const UBaseType_t count = uxTaskGetSystemState(
        g_taskStatusBuffer, kRuntimeStatusCapacity, &totalRuntime);
    if (count == 0U) return;

    TaskRuntimeCounterType idleNow[2] = {0, 0};
    bool idleFound[2] = {false, false};
    for (UBaseType_t i = 0; i < count; i++) {
        const char* name = g_taskStatusBuffer[i].pcTaskName;
        if (name && strcmp(name, "IDLE0") == 0) {
            idleNow[0] = g_taskStatusBuffer[i].ulRunTimeCounter;
            idleFound[0] = true;
        } else if (name && strcmp(name, "IDLE1") == 0) {
            idleNow[1] = g_taskStatusBuffer[i].ulRunTimeCounter;
            idleFound[1] = true;
        }
    }

    if (g_previousTotalRuntime == 0U) {
        g_previousTotalRuntime = totalRuntime;
        for (uint8_t core = 0; core < 2; core++) {
            if (idleFound[core]) g_previousIdleRuntime[core] = idleNow[core];
        }
        for (uint8_t id = 0; id < TASK_RUNTIME_COUNT; id++) {
            const TaskStatus_t* status = findTaskStatus(handles[id], count);
            g_previousTaskHandles[id] = handles[id];
            g_previousTaskRuntime[id] = status ? status->ulRunTimeCounter : 0U;
        }
        return;
    }

    const TaskRuntimeCounterType totalDelta = totalRuntime - g_previousTotalRuntime;
    uint16_t coreIdle[2] = {
        kTaskRuntimeUnknownPermille, kTaskRuntimeUnknownPermille,
    };
    uint16_t cpuPermille[TASK_RUNTIME_COUNT] = {};
    for (uint8_t id = 0; id < TASK_RUNTIME_COUNT; id++) {
        cpuPermille[id] = kTaskRuntimeUnknownPermille;
    }

    if (totalDelta > 0U) {
        for (uint8_t core = 0; core < 2; core++) {
            if (idleFound[core]) {
                const TaskRuntimeCounterType idleDelta = idleNow[core] - g_previousIdleRuntime[core];
                if (idleDelta <= totalDelta) {
                    coreIdle[core] = (uint16_t)(((uint64_t)idleDelta * 1000ULL) / totalDelta);
                }
            }
        }
        for (uint8_t id = 0; id < TASK_RUNTIME_COUNT; id++) {
            const TaskStatus_t* status = findTaskStatus(handles[id], count);
            if (status && handles[id] == g_previousTaskHandles[id]) {
                const TaskRuntimeCounterType taskDelta =
                    status->ulRunTimeCounter - g_previousTaskRuntime[id];
                if (taskDelta <= totalDelta) {
                    cpuPermille[id] = (uint16_t)(((uint64_t)taskDelta * 1000ULL) / totalDelta);
                }
            }
        }
    }

    g_previousTotalRuntime = totalRuntime;
    for (uint8_t core = 0; core < 2; core++) {
        if (idleFound[core]) g_previousIdleRuntime[core] = idleNow[core];
    }
    for (uint8_t id = 0; id < TASK_RUNTIME_COUNT; id++) {
        const TaskStatus_t* status = findTaskStatus(handles[id], count);
        g_previousTaskHandles[id] = handles[id];
        g_previousTaskRuntime[id] = status ? status->ulRunTimeCounter : 0U;
    }

    portENTER_CRITICAL(&g_runtimeLock);
    g_coreIdlePermille[0] = coreIdle[0];
    g_coreIdlePermille[1] = coreIdle[1];
    for (uint8_t id = 0; id < TASK_RUNTIME_COUNT; id++) {
        g_slots[id].cpuPermille = cpuPermille[id];
    }
    portEXIT_CRITICAL(&g_runtimeLock);
#else
    portENTER_CRITICAL(&g_runtimeLock);
    g_coreIdlePermille[0] = kTaskRuntimeUnknownPermille;
    g_coreIdlePermille[1] = kTaskRuntimeUnknownPermille;
    for (uint8_t id = 0; id < TASK_RUNTIME_COUNT; id++) {
        g_slots[id].cpuPermille = kTaskRuntimeUnknownPermille;
    }
    portEXIT_CRITICAL(&g_runtimeLock);
#endif
}

/**
 * @brief 冻结最近一秒结果并立刻清空下一秒累计器。
 * @details 复制和重置只在低优先级监控任务中进行，实时任务最多等待一次很短的临界区。
 */
void freezeRuntimeWindow(int64_t nowUs)
{
    if (g_windowStartUs == 0) {
        g_windowStartUs = nowUs;
        return;
    }
    const uint32_t durationUs = clampUs(nowUs - g_windowStartUs);
    if (durationUs == 0U) return;
    g_windowStartUs = nowUs;

    uint32_t sequence = 0;
    uint16_t coreIdlePermille[2] = {};
    uint32_t systemStatusMaxGapMs = 0;
    uint32_t stopLastLatencyUs = 0;
    uint32_t stopMaxLatencyUs = 0;
    const uint32_t timestampMs = p4::platform::time::monotonicMillis();

    // 临界区只做“复制旧窗口 + 清空新窗口”，分位数统计一律在锁外完成。
    portENTER_CRITICAL(&g_runtimeLock);
    sequence = ++g_windowSequence;
    memcpy(g_windowSlots, g_slots, sizeof(g_windowSlots));
    coreIdlePermille[0] = g_coreIdlePermille[0];
    coreIdlePermille[1] = g_coreIdlePermille[1];
    systemStatusMaxGapMs = g_systemStatusMaxGapMs;
    stopLastLatencyUs = g_stopLastLatencyUs;
    stopMaxLatencyUs = g_stopMaxLatencyUs;
    for (uint8_t id = 0; id < TASK_RUNTIME_COUNT; id++) {
        taskRuntimeWindowReset(&g_slots[id].window);
    }
    memcpy(g_windowServoIo, g_servoIoWindows, sizeof(g_windowServoIo));
    memcpy(g_windowServoIoRequested,
           g_servoIoRequested,
           sizeof(g_windowServoIoRequested));
    memcpy(g_windowServoIoSucceeded,
           g_servoIoSucceeded,
           sizeof(g_windowServoIoSucceeded));
    for (uint8_t channel = 0; channel < TASK_RUNTIME_IO_CHANNEL_COUNT; ++channel) {
        taskRuntimeWindowReset(&g_servoIoWindows[channel]);
        g_servoIoRequested[channel] = 0;
        g_servoIoSucceeded[channel] = 0;
    }
    portEXIT_CRITICAL(&g_runtimeLock);

    for (uint8_t id = 0; id < TASK_RUNTIME_COUNT; id++) {
        const TaskRuntimeSlot& slot = g_windowSlots[id];
        TaskRuntimeSnapshot& snapshot = g_windowSnapshots[id];
        snapshot.taskId = id;
        snapshot.active = slot.active;
        snapshot.priority = slot.priority;
        snapshot.coreId = slot.coreId;
        snapshot.stackAllocatedBytes = slot.stackAllocatedBytes;
        snapshot.stackCurrentFreeBytes = slot.stackCurrentFreeBytes;
        snapshot.stackMinFreeBytes = slot.stackMinFreeBytes;
        snapshot.loopCount = slot.loopCount;
        snapshot.maxGapUs = slot.maxGapUs;
        snapshot.maxExecUs = slot.maxExecUs;
        snapshot.windowSequence = sequence;
        snapshot.windowDurationUs = durationUs;
        snapshot.windowLoopCount = slot.window.loopCount;
        snapshot.windowPeriodicCount = slot.window.periodicCount;
        snapshot.windowEventCount = slot.window.eventCount;
        snapshot.avgExecUs = slot.window.loopCount
            ? (uint32_t)(slot.window.execTotalUs / slot.window.loopCount) : 0U;
        snapshot.p95ExecUs = taskRuntimeWindowQuantileUs(slot.window, 950U);
        snapshot.p99ExecUs = taskRuntimeWindowQuantileUs(slot.window, 990U);
        snapshot.windowMaxExecUs = slot.window.maxExecUs;
        snapshot.cpuPermille = slot.cpuPermille;
        snapshot.overrunCount = slot.overrunCount;
        snapshot.consecutiveOverruns = slot.consecutiveOverruns;
        snapshot.maxConsecutiveOverruns = slot.maxConsecutiveOverruns;
        snapshot.core0IdlePermille = coreIdlePermille[0];
        snapshot.core1IdlePermille = coreIdlePermille[1];
        snapshot.systemStatusMaxGapMs = systemStatusMaxGapMs;
        snapshot.stopLastLatencyUs = stopLastLatencyUs;
        snapshot.stopMaxLatencyUs = stopMaxLatencyUs;
        snapshot.timestampMs = timestampMs;
        memset(snapshot.ioSampleCount, 0, sizeof(snapshot.ioSampleCount));
        memset(snapshot.ioRequestedCount, 0, sizeof(snapshot.ioRequestedCount));
        memset(snapshot.ioSucceededCount, 0, sizeof(snapshot.ioSucceededCount));
        memset(snapshot.ioAvgUs, 0, sizeof(snapshot.ioAvgUs));
        memset(snapshot.ioP95Us, 0, sizeof(snapshot.ioP95Us));
        memset(snapshot.ioP99Us, 0, sizeof(snapshot.ioP99Us));
        if (id == TASK_RUNTIME_SERVO_BUS) {
            for (uint8_t channel = 0;
                 channel < TASK_RUNTIME_IO_CHANNEL_COUNT;
                 ++channel) {
                const TaskRuntimeWindowAccumulator& io =
                    g_windowServoIo[channel];
                snapshot.ioSampleCount[channel] = io.loopCount;
                snapshot.ioRequestedCount[channel] =
                    g_windowServoIoRequested[channel];
                snapshot.ioSucceededCount[channel] =
                    g_windowServoIoSucceeded[channel];
                snapshot.ioAvgUs[channel] = io.loopCount
                    ? (uint32_t)(io.execTotalUs / io.loopCount) : 0U;
                snapshot.ioP95Us[channel] =
                    taskRuntimeWindowQuantileUs(io, 950U);
                snapshot.ioP99Us[channel] =
                    taskRuntimeWindowQuantileUs(io, 990U);
            }
        }
    }

    portENTER_CRITICAL(&g_runtimeLock);
    memcpy(g_frozenSnapshots, g_windowSnapshots, sizeof(g_frozenSnapshots));
    portEXIT_CRITICAL(&g_runtimeLock);
}

} // namespace

void taskRuntimeRegister(uint8_t taskId, TaskHandle_t handle, uint32_t stackAllocatedBytes)
{
    if (taskId >= TASK_RUNTIME_COUNT || !handle) return;
    portENTER_CRITICAL(&g_runtimeLock);
    TaskRuntimeSlot& slot = g_slots[taskId];
    memset(&slot, 0, sizeof(slot));
    slot.handle = handle;
    slot.active = 1;
    slot.priority = (uint8_t)uxTaskPriorityGet(handle);
    slot.coreId = -1;
    slot.stackAllocatedBytes = stackAllocatedBytes;
    slot.cpuPermille = kTaskRuntimeUnknownPermille;
    portEXIT_CRITICAL(&g_runtimeLock);
}

void taskRuntimeUnregister(uint8_t taskId)
{
    if (taskId >= TASK_RUNTIME_COUNT) return;
    portENTER_CRITICAL(&g_runtimeLock);
    g_slots[taskId].active = 0;
    g_slots[taskId].handle = NULL;
    g_slots[taskId].cpuPermille = kTaskRuntimeUnknownPermille;
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
    if (currentStackFreeBytes > 0U) slot.stackCurrentFreeBytes = currentStackFreeBytes;
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
        // 这里只进行固定桶累加；不排序、不打印，避免给控制与舵机任务引入可见阻塞。
        taskRuntimeWindowAddExec(&slot.window, execUs);
    }
    if (currentStackFreeBytes > 0U) slot.stackCurrentFreeBytes = currentStackFreeBytes;
    portEXIT_CRITICAL(&g_runtimeLock);
}

void taskRuntimeMarkCycle(uint8_t taskId, bool periodicDue, bool eventWake)
{
    if (taskId >= TASK_RUNTIME_COUNT) return;
    portENTER_CRITICAL(&g_runtimeLock);
    if (periodicDue) g_slots[taskId].window.periodicCount++;
    if (eventWake) g_slots[taskId].window.eventCount++;
    portEXIT_CRITICAL(&g_runtimeLock);
}

void taskRuntimeMarkIoSample(uint8_t channel,
                             uint32_t elapsedUs,
                             uint32_t requestedCount,
                             uint32_t succeededCount)
{
    if (channel >= TASK_RUNTIME_IO_CHANNEL_COUNT) return;
    portENTER_CRITICAL(&g_runtimeLock);
    taskRuntimeWindowAddExec(&g_servoIoWindows[channel], elapsedUs);
    g_servoIoRequested[channel] += requestedCount;
    g_servoIoSucceeded[channel] += succeededCount;
    portEXIT_CRITICAL(&g_runtimeLock);
}

void taskRuntimeMarkPeriodResult(uint8_t taskId, bool overrun)
{
    if (taskId >= TASK_RUNTIME_COUNT) return;
    bool reportOverrun = false;
    uint16_t observedConsecutive = 0;
    portENTER_CRITICAL(&g_runtimeLock);
    TaskRuntimeSlot& slot = g_slots[taskId];
    if (overrun) {
        slot.overrunCount++;
        if (slot.consecutiveOverruns < 0xFFFFU) slot.consecutiveOverruns++;
        if (slot.consecutiveOverruns > slot.maxConsecutiveOverruns) {
            slot.maxConsecutiveOverruns = slot.consecutiveOverruns;
        }
        const p4::safety::RuntimeLatchDecision decision =
            p4::safety::evaluateRuntimeOverrun(
                slot.consecutiveOverruns, 5U, slot.overrunEventLatched);
        slot.overrunEventLatched = decision.active;
        reportOverrun = decision.entered;
        observedConsecutive = slot.consecutiveOverruns;
    } else {
        slot.consecutiveOverruns = 0;
        slot.overrunEventLatched = p4::safety::evaluateRuntimeOverrun(
            0U, 5U, slot.overrunEventLatched).active;
    }
    portEXIT_CRITICAL(&g_runtimeLock);
    if (reportOverrun) {
        p4::safety::reportSafetyRuleEvent(
            nullptr, SafetyRuleId::RuntimeTaskPeriodOverrun,
            taskId, observedConsecutive);
    }
}

bool taskRuntimeGetSnapshot(uint8_t taskId, TaskRuntimeSnapshot* out)
{
    if (taskId >= TASK_RUNTIME_COUNT || !out) return false;
    portENTER_CRITICAL(&g_runtimeLock);
    const bool ready = g_windowSequence != 0U;
    if (ready) *out = g_frozenSnapshots[taskId];
    portEXIT_CRITICAL(&g_runtimeLock);
    return ready;
}

uint32_t taskRuntimeGetWindowSequence()
{
    portENTER_CRITICAL(&g_runtimeLock);
    const uint32_t sequence = g_windowSequence;
    portEXIT_CRITICAL(&g_runtimeLock);
    return sequence;
}

void taskRuntimeMarkSystemStatusSent(void)
{
    const uint32_t nowMs = p4::platform::time::monotonicMillis();
    portENTER_CRITICAL(&g_runtimeLock);
    if (g_lastSystemStatusMs != 0U) {
        const uint32_t gapMs = nowMs - g_lastSystemStatusMs;
        if (gapMs > g_systemStatusMaxGapMs) g_systemStatusMaxGapMs = gapMs;
    }
    g_lastSystemStatusMs = nowMs;
    portEXIT_CRITICAL(&g_runtimeLock);
}

void taskRuntimeMarkStopReceived(void)
{
    portENTER_CRITICAL(&g_runtimeLock);
    g_stopReceivedUs = esp_timer_get_time();
    g_stopRequestedUs = 0;
    portEXIT_CRITICAL(&g_runtimeLock);
}

void taskRuntimeMarkStopRequested(void)
{
    portENTER_CRITICAL(&g_runtimeLock);
    g_stopRequestedUs = esp_timer_get_time();
    portEXIT_CRITICAL(&g_runtimeLock);
}

void taskRuntimeMarkStopTorqueOffComplete(void)
{
    const int64_t nowUs = esp_timer_get_time();
    portENTER_CRITICAL(&g_runtimeLock);
    if (g_stopReceivedUs > 0 && g_stopRequestedUs >= g_stopReceivedUs) {
        g_stopLastLatencyUs = clampUs(nowUs - g_stopReceivedUs);
        if (g_stopLastLatencyUs > g_stopMaxLatencyUs) g_stopMaxLatencyUs = g_stopLastLatencyUs;
        g_stopReceivedUs = 0;
        g_stopRequestedUs = 0;
    }
    portEXIT_CRITICAL(&g_runtimeLock);
}

void stepRuntimeMonitorService(uint32_t nowMs)
{
    (void)nowMs;
    sampleTaskStacks();
    sampleCpuUsage();
    freezeRuntimeWindow(esp_timer_get_time());
}
