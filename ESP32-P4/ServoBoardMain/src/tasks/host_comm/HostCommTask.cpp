/**
 * @file HostCommTask.cpp
 * @brief 驱动 200 Hz 上位机通信服务，拥有周期等待与截止时间统计。
 * @details 本文件不解析协议；全部收发行为委托给 modules/communication 服务。
 */
#include "HostCommTask.h"
#include <Arduino.h>

#include "HostCommService.h"
#include "../../config/TaskConfig.h"
#include "../../hal/serial/HostSerialPort.h"
#include "../../platform/rtos/PeriodicDeadline.h"
#include "../../platform/rtos/TaskSignal.h"
#include "../../platform/time/Clock.h"
#include "../../runtime/metrics/TaskRuntimeMetrics.h"
#include "../../runtime/SystemRuntimeContext.h"

namespace {

/** USB/UART 数据就绪时只置位通知；协议读取仍由 HostCommTask 串行完成。 */
void wakeHostComm(void* context)
{
    SystemRuntimeContext* sharedData = static_cast<SystemRuntimeContext*>(context);
    if (sharedData) {
        sharedData->hostCommWake.notify(p4::runtime::wake::kHostRx);
    }
}

} // namespace

void hostCommTask(void* parameter)
{
    SystemRuntimeContext* sharedData = static_cast<SystemRuntimeContext*>(parameter);
    if (!initHostCommService(sharedData)) {
        vTaskDelete(NULL);
        return;
    }

    sharedData->hostCommWake.bind(xTaskGetCurrentTaskHandle());
    HostSerialPort::setRxWakeHandler(wakeHostComm, sharedData);
    const TickType_t periodTicks = pdMS_TO_TICKS(HOST_COMM_SERVICE_PERIOD_MS);
    p4::platform::rtos::PeriodicDeadline deadline(periodTicks);
    uint32_t wakeReasons = 0;
    for (;;) {
        const TickType_t dispatchTicks = xTaskGetTickCount();
        const bool periodicDue = deadline.due(dispatchTicks);
        if (!periodicDue && wakeReasons == 0U) {
            wakeReasons = sharedData->hostCommWake.wait(
                deadline.remaining(dispatchTicks));
            continue;
        }
        if (periodicDue) deadline.advancePast(dispatchTicks);

        const TickType_t loopBegin = xTaskGetTickCount();
        taskRuntimeMarkLoopStart(TASK_RUNTIME_HOST_COMM);
        taskRuntimeMarkCycle(TASK_RUNTIME_HOST_COMM,
                             periodicDue,
                             wakeReasons != 0U);
        const HostCommStepRequest request = {wakeReasons, periodicDue};
        wakeReasons = 0;
        stepHostCommService(sharedData,
                            p4::platform::time::monotonicMillis(),
                            request);
        taskRuntimeMarkLoopEnd(TASK_RUNTIME_HOST_COMM);
        if (periodicDue) {
            const TickType_t loopEnd = xTaskGetTickCount();
            const bool periodOverrun =
                (loopEnd - loopBegin) >= periodTicks || deadline.due(loopEnd);
            taskRuntimeMarkPeriodResult(TASK_RUNTIME_HOST_COMM, periodOverrun);
            if (periodOverrun) {
                if (deadline.due(loopEnd)) deadline.advancePast(loopEnd);
                vTaskDelay(1);
            }
        }
    }
    vTaskDelete(NULL);
}
