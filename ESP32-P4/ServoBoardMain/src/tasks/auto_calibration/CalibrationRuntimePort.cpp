/**
 * @file CalibrationRuntimePort.cpp
 * @brief 保存并调用自动标定运行端口，隔离业务状态机与 RTOS。
 */
#include "CalibrationRuntimePort.h"

static CalibrationRuntimePort g_calibrationRuntimePort = {};

bool configureCalibrationRuntimePort(const CalibrationRuntimePort& port)
{
    if (!port.nowMs || !port.waitMs) {
        return false;
    }
    g_calibrationRuntimePort = port;
    return true;
}

uint32_t calibrationNowMs()
{
    return g_calibrationRuntimePort.nowMs
        ? g_calibrationRuntimePort.nowMs(g_calibrationRuntimePort.context)
        : 0;
}

void calibrationWaitMs(uint32_t waitMs)
{
    if (g_calibrationRuntimePort.waitMs) {
        g_calibrationRuntimePort.waitMs(g_calibrationRuntimePort.context,
                                        waitMs);
    }
}
