/**
 * @file CalibrationSessionController.h
 * @brief 声明一次完整自动标定会话的业务入口。
 *
 * 会话控制器拥有 scope、预检、开始/结束标志和系统事件；不实现机械阈值，
 * 不直接访问舵机 SDK，也不创建或删除 FreeRTOS 任务。
 */
#ifndef P4_MODULES_CALIBRATION_CALIBRATION_SESSION_CONTROLLER_H
#define P4_MODULES_CALIBRATION_CALIBRATION_SESSION_CONTROLLER_H

#include "../../runtime/ports/ServiceOutputPort.h"
#include "../../data/ServiceSnapshots.h"

struct SystemRuntimeContext;

/** 在当前任务上下文内执行一次自动标定会话。 */
void runAutoCalibrationSession(SystemRuntimeContext* sharedData,
                               const CalibrationServiceInputSnapshot& input,
                               const ServiceOutputPort& output);

#endif // P4_MODULES_CALIBRATION_CALIBRATION_SESSION_CONTROLLER_H
