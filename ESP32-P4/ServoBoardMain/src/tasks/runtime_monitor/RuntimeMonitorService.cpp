/**
 * @file RuntimeMonitorService.cpp
 * @brief 采集各实时任务的周期、执行时间、堆栈和超限统计。
 */
#include "RuntimeMonitorService.h"
#include "../../platform/time/Clock.h"
#include "../../runtime/metrics/TaskRuntimeMetrics.h"
#include "../../safety/SafetyEventReporter.h"
#include "../../safety/policies/RuntimeSafetyPolicy.h"

#include <esp_timer.h>
#include <string.h>

namespace {

// 每个诊断槽只累计最坏值，不在实时任务中分配内存或打印日志。
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
    bool overrunEventLatched; ///< 连续周期超限事件已上报，恢复一帧后解除。
    bool stackEventLatched;   ///< 低栈事件已上报，栈余量恢复后解除。
};

TaskRuntimeSlot g_slots[TASK_RUNTIME_COUNT] = {};
// 所有任务都会更新诊断槽，使用自旋锁保护短小的字段复制和计数操作。
portMUX_TYPE g_runtimeLock = portMUX_INITIALIZER_UNLOCKED;
// 0xFFFF 表示当前 FreeRTOS 配置未提供可用的 runtime stats。
uint16_t g_coreIdlePermille[2] = {0xFFFF, 0xFFFF};
uint32_t g_lastSystemStatusMs = 0;
uint32_t g_systemStatusMaxGapMs = 0;
int64_t g_stopReceivedUs = 0;
int64_t g_stopRequestedUs = 0;
uint32_t g_stopLastLatencyUs = 0;
uint32_t g_stopMaxLatencyUs = 0;

// 只有同时开启 FreeRTOS runtime stats 与 trace facility 才能计算双核 idle。
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

/** 将有符号微秒差安全压入 uint32 诊断字段。 */
uint32_t clampUs(int64_t value)
{
    if (value <= 0) return 0;
    if (value > 0xFFFFFFFFLL) return 0xFFFFFFFFUL;
    return (uint32_t)value;
}

/** 读取当前任务剩余栈字节；FreeRTOS high-water mark 原始单位为 StackType_t。 */
uint32_t sampleCurrentTaskStackFreeBytes()
{
    // 当前栈指针减去栈起始地址，用于估算“此刻剩余栈”；
    // 不支持 pxTaskGetStackStart 的构建返回 0，由上位机显示为未知。
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

/** 采样全部已注册任务的当前/历史最小剩余栈。 */
void sampleTaskStacks()
{
    for (uint8_t id = 0; id < TASK_RUNTIME_COUNT; id++) {
        TaskHandle_t handle = NULL;
        portENTER_CRITICAL(&g_runtimeLock);
        handle = g_slots[id].handle;
        portEXIT_CRITICAL(&g_runtimeLock);
        if (!handle) continue;

        // ESP-IDF FreeRTOS 的高水位单位是字节，表示历史最小剩余栈。
        const uint32_t freeBytes = (uint32_t)uxTaskGetStackHighWaterMark(handle);
        const uint8_t priority = (uint8_t)uxTaskPriorityGet(handle);
        bool reportLowStack = false;
        uint32_t stackThreshold = 0;
        portENTER_CRITICAL(&g_runtimeLock);
        g_slots[id].priority = priority;
        if (g_slots[id].stackMinFreeBytes == 0 || freeBytes < g_slots[id].stackMinFreeBytes) {
            g_slots[id].stackMinFreeBytes = freeBytes;
        }
        stackThreshold = p4::safety::runtimeStackWarningThreshold(
            g_slots[id].stackAllocatedBytes, 512U);
        const p4::safety::RuntimeLatchDecision stackDecision =
            p4::safety::evaluateRuntimeStack(
                freeBytes,
                stackThreshold,
                g_slots[id].stackEventLatched);
        g_slots[id].stackEventLatched = stackDecision.active;
        reportLowStack = stackDecision.entered;
        portEXIT_CRITICAL(&g_runtimeLock);
        if (reportLowStack) {
            p4::safety::reportSafetyRuleEvent(
                nullptr,
                SafetyRuleId::RuntimeTaskStackLow,
                (int32_t)stackThreshold,
                (int32_t)freeBytes);
        }
    }
}

/** 用两个核心 idle hook 的增量估算最近窗口 idle 千分比。 */
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
    // 第一帧只建立基线，下一秒才有可比较的增量。
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
    // 动态任务退出前保留最后一次栈余量，但清空句柄，避免监控访问已删除任务。
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
    // gap 是相邻两轮开始时刻之差；exec 是本轮 Start 到 End 的执行时间。
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
    bool reportOverrun = false;
    uint16_t observedConsecutive = 0;
    portENTER_CRITICAL(&g_runtimeLock);
    TaskRuntimeSlot& slot = g_slots[taskId];
    if (overrun) {
        slot.overrunCount++;
        if (slot.consecutiveOverruns < 0xFFFF) slot.consecutiveOverruns++;
        if (slot.consecutiveOverruns > slot.maxConsecutiveOverruns) {
            slot.maxConsecutiveOverruns = slot.consecutiveOverruns;
        }
        // 单次偶发超限保留在统计表；连续 5 次才形成 RUN-8005 安全证据。
        const p4::safety::RuntimeLatchDecision overrunDecision =
            p4::safety::evaluateRuntimeOverrun(
                slot.consecutiveOverruns,
                5,
                slot.overrunEventLatched);
        slot.overrunEventLatched = overrunDecision.active;
        reportOverrun = overrunDecision.entered;
        if (reportOverrun) observedConsecutive = slot.consecutiveOverruns;
    } else {
        slot.consecutiveOverruns = 0;
        const p4::safety::RuntimeLatchDecision overrunDecision =
            p4::safety::evaluateRuntimeOverrun(
                0,
                5,
                slot.overrunEventLatched);
        slot.overrunEventLatched = overrunDecision.active;
    }
    portEXIT_CRITICAL(&g_runtimeLock);
    if (reportOverrun) {
        p4::safety::reportSafetyRuleEvent(
            nullptr,
            SafetyRuleId::RuntimeTaskPeriodOverrun,
            taskId,
            observedConsecutive);
    }
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
    // 某些平台无法直接采当前栈，异常值不得超过创建任务时声明的栈大小。
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
    out->timestampMs = p4::platform::time::monotonicMillis();
    portEXIT_CRITICAL(&g_runtimeLock);
    return true;
}

void taskRuntimeMarkSystemStatusSent(void)
{
    const uint32_t nowMs = p4::platform::time::monotonicMillis();
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
    // 只有完整经历“收到 STOP -> 发出急停请求 -> 舵机完成断扭矩”才记一次时延。
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

void stepRuntimeMonitorService(uint32_t nowMs)
{
    (void)nowMs;
    sampleTaskStacks();
    sampleCoreIdle();
}
