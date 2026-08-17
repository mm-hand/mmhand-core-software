/**
 * @file HostCommTask.cpp
 * @brief 驱动 200 Hz 上位机通信服务，拥有周期等待与截止时间统计。
 * @details 本文件不解析协议；全部收发行为委托给 modules/communication 服务。
 */
#include "HostCommTask.h"
#include <Arduino.h>

#include "HostCommService.h"
#include "../../hal/serial/HostSerialPort.h"
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
    const TickType_t periodTicks = pdMS_TO_TICKS(5);
    for (;;) {
        const TickType_t loopBegin = xTaskGetTickCount();
        taskRuntimeMarkLoopStart(TASK_RUNTIME_HOST_COMM);
        stepHostCommService(sharedData,
                             p4::platform::time::monotonicMillis());
        taskRuntimeMarkLoopEnd(TASK_RUNTIME_HOST_COMM);
        const bool periodOverrun =
            (xTaskGetTickCount() - loopBegin) >= periodTicks;
        taskRuntimeMarkPeriodResult(TASK_RUNTIME_HOST_COMM, periodOverrun);
        if (periodOverrun) {
            vTaskDelay(1);
        } else {
            (void)sharedData->hostCommWake.wait(periodTicks);
        }
    }
    vTaskDelete(NULL);
}
