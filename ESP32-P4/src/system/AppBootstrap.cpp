/**
 * @file AppBootstrap.cpp
 * @brief 提供 Arduino setup/loop 与 P4 系统装配之间的最薄入口。
 * @details 串口初始化留在平台入口；队列、状态和任务创建统一委托给 SystemAssembly。
 */
#include "AppBootstrap.h"

#include "../config/BoardConfig.h"
#include "log/FirmwareLog.h"
#include "SystemAssembly.h"
#include "../hal/serial/HostSerialPort.h"

#include <Arduino.h>

void appBootstrapSetup()
{
    // 二进制协议和固件日志共用该串口，波特率必须与 desktop 保持一致。
    HostSerialPort::begin(kHostUartBaud);
    while (!Serial) {
        delay(10);
    }
    delay(1000);

    if (!assembleAndStartSystem()) {
        FW_LOG_ERROR_PRINTLN("[SYSTEM] bootstrap failed; halt");
        while (true) {
            delay(1000);
        }
    }
}

void appBootstrapLoop()
{
    // 业务全部运行于 FreeRTOS 任务；Arduino loop 仅主动让出调度时间。
    vTaskDelay(pdMS_TO_TICKS(10));
}
