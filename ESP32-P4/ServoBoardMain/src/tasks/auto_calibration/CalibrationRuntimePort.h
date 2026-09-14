/**
 * @file CalibrationRuntimePort.h
 * @brief 定义自动标定对单调时钟和协作等待的最小运行端口。
 * @details 标定业务不得直接调用 Arduino/FreeRTOS；AutoCalibrationTask 注入平台实现。
 */
#ifndef P4_MODULES_CALIBRATION_CALIBRATION_RUNTIME_PORT_H
#define P4_MODULES_CALIBRATION_CALIBRATION_RUNTIME_PORT_H

#include <stdint.h>

struct CalibrationRuntimePort {
    void* context;                         ///< 平台适配层私有上下文。
    uint32_t (*nowMs)(void* context);       ///< 返回单调毫秒时间。
    void (*waitMs)(void* context, uint32_t);///< 协作等待，不允许忙等。
};

/** 安装 AutoCalibrationTask 提供的运行端口。 */
bool configureCalibrationRuntimePort(const CalibrationRuntimePort& port);

/** 标定状态机统一读取当前单调时间。 */
uint32_t calibrationNowMs();

/** 标定状态机主动让出 CPU，具体实现属于 task/platform。 */
void calibrationWaitMs(uint32_t waitMs);

#endif // P4_MODULES_CALIBRATION_CALIBRATION_RUNTIME_PORT_H
