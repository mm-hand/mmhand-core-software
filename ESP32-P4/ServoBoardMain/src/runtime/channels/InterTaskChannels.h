/**
 * @file InterTaskChannels.h
 * @brief 保存任务之间共享的 FreeRTOS 队列与互斥锁句柄。
 * @details
 * 该结构只描述运行时资源，不保存业务状态；资源仅由 SystemAssembly 创建和销毁。
 */
#ifndef P4_RUNTIME_CHANNELS_INTER_TASK_CHANNELS_H
#define P4_RUNTIME_CHANNELS_INTER_TASK_CHANNELS_H

#include "../../platform/rtos/FreeRtosKernel.h"
#include "../../platform/rtos/MessageChannel.h"
#include "../../platform/rtos/TaskSignal.h"
#include "../../data/ActuatorTypes.h"
#include "../../data/CalibrationTypes.h"
#include "../../data/JointTypes.h"
#include "../../data/ServoTypes.h"
#include "../../data/SystemTypes.h"
#include "../../data/TelemetryTypes.h"

struct InterTaskChannels {
    // 任务通知不传输业务载荷，只用于缩短“数据已到达但任务仍在 delay”的调度等待。
    p4::platform::rtos::TaskSignal hostCommWake; ///< USB/UART 收到字节后唤醒 HostCommTask。
    p4::platform::rtos::TaskSignal systemStateWake; ///< 上位机事件和 E-STOP 唤醒状态机。
    p4::platform::rtos::TaskSignal motionControlWake; ///< 新关节目标或状态授权变化唤醒控制任务。
    p4::platform::rtos::TaskSignal servoBusWake; ///< 新舵机批次或 E-STOP 唤醒舵机总线任务。
    MessageChannel<SystemEvent_t> stateEventQueue; ///< 顺序系统事件；急停另走 token。
    MessageChannel<RemoteSensorData_t> canRxQueue; ///< 最新 CAN 传感器快照。
    MessageChannel<ServoTargetBatch_t> servoTargetQueue; ///< 唯一舵机执行批次邮箱。
    MessageChannel<ServoAngleData_t> servoFeedbackQueue; ///< 控制、标定和上行共读的 logical ABS 快照。
    MessageChannel<ServoAngleData_t> servoRawQueue; ///< 单圈 raw 快照。
    MessageChannel<ServoTelemetryData_t> servoTelemetryQueue; ///< 控制、标定和上行共读的最新遥测。
    MessageChannel<MappedAngleData_t> mappedAngleQueue; ///< 21 关节角快照。
    MessageChannel<RemoteTactileData_t> tactileQueue; ///< 顺序触觉数据。
    SemaphoreHandle_t commandStateMutex; ///< 保护目标角和直控命令的成组读写。
};

#endif // P4_RUNTIME_CHANNELS_INTER_TASK_CHANNELS_H
