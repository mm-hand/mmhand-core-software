/**
 * @file AutoCalibrationTask.cpp
 * @brief 提供自动标定 FreeRTOS 任务的薄入口。
 * @details 本文件不实现标定规则；所有业务推进委托给 modules/calibration 编排器。
 */
#include "AutoCalibrationTask.h"

#include <Arduino.h>

#include "AutoCalibrationTaskRuntime.h"
#include "CalibrationRuntimePort.h"
#include "CalibrationSessionController.h"
#include "../../platform/time/Clock.h"
#include "../../runtime/SystemRuntimeContext.h"
#include "../../runtime/adapters/ServiceContextAdapter.h"

static uint32_t calibrationTaskNowMs(void*)
{
    return p4::platform::time::monotonicMillis();
}

static void calibrationTaskWaitMs(void*, uint32_t waitMs)
{
    vTaskDelay(pdMS_TO_TICKS(waitMs));
}

void autoCalibrationTask(void* parameter)
{
    SystemRuntimeContext* sharedData = static_cast<SystemRuntimeContext*>(parameter);
    const CalibrationRuntimePort runtimePort = {
        NULL,
        calibrationTaskNowMs,
        calibrationTaskWaitMs,
    };
    configureCalibrationRuntimePort(runtimePort);
    const uint32_t nowMs = calibrationTaskNowMs(NULL);
    const CalibrationServiceInputSnapshot input =
        captureCalibrationServiceInput(sharedData, nowMs);
    const ServiceOutputPort output =
        makeSystemServiceOutputPort(sharedData);
    runAutoCalibrationSession(sharedData, input, output);
    finishAutoCalibrationTaskRuntime();
    vTaskDelete(NULL);
}
