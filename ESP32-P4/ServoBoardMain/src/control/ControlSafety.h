/**
 * @file ControlSafety.h
 * @brief 声明控制安全判定结果与边界检查接口。
 */
#ifndef CONTROL_SAFETY_H
#define CONTROL_SAFETY_H

#include <Arduino.h>

#include "../shared/TaskSharedData.h"

// 闭环输出前必须满足的反馈有效性检查。
// 返回 true 表示调用方应保持该关节当前电机位置，而不是发布新的 PID 目标。
bool shouldEmergencyStop(bool canBusOnline,
                         const RemoteSensorData_t& sensorData,
                         const MappedAngleData_t& mappedData,
                         uint8_t jointIndex,
                         bool encoderFeedbackFault);

#endif // CONTROL_SAFETY_H
