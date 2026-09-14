/**
 * @file AutoCalibrationTaskRuntime.cpp
 * @brief 实现自动标定 FreeRTOS 任务句柄的系统级生命周期管理。
 */
#include "AutoCalibrationTaskRuntime.h"

#include "AutoCalibrationTask.h"
#include "CalibrationDiagnostics.h"
#include "../../config/TaskConfig.h"
#include "../../platform/log/FirmwareLog.h"
#include "../../platform/rtos/TaskFactory.h"
#include "../../runtime/SystemRuntimeContext.h"
#include "../../runtime/metrics/TaskRuntimeMetrics.h"

/** 非空表示标定 Task 已创建；该句柄只由系统装配层写入。 */
static TaskHandle_t s_autoCalibrationTaskHandle = NULL;

bool startAutoCalibrationTaskRuntime(SystemRuntimeContext* sharedData)
{
    if (!sharedData) {
        return false;
    }
    if (s_autoCalibrationTaskHandle != NULL) {
        FW_LOG_INFO_PRINTLN("[AUTO CALIB] task already running");
        return true;
    }
    clearCalibrationDiag(sharedData);

    const bool created = p4::platform::rtos::createTask(
        {autoCalibrationTask,
         "autoCalibrationTask",
         AUTO_CALIB_TASK_STACK_SIZE,
         sharedData,
         TASK_AUTO_CALIB_PRIORITY},
        &s_autoCalibrationTaskHandle);
    if (created) {
        taskRuntimeRegister(TASK_RUNTIME_AUTO_CALIB,
                            s_autoCalibrationTaskHandle,
                            AUTO_CALIB_TASK_STACK_SIZE);
    }
    return created;
}

void finishAutoCalibrationTaskRuntime()
{
    taskRuntimeUnregister(TASK_RUNTIME_AUTO_CALIB);
    s_autoCalibrationTaskHandle = NULL;
}
