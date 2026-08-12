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
#include "../../data/ActuatorTypes.h"
#include "../../data/CalibrationTypes.h"
#include "../../data/JointTypes.h"
#include "../../data/PersistenceTypes.h"
#include "../../data/RecoveryTypes.h"
#include "../../data/ServoTypes.h"
#include "../../data/SystemTypes.h"
#include "../../data/TelemetryTypes.h"

struct InterTaskChannels {
    MessageChannel<SystemEvent_t> stateEventQueue; ///< 顺序系统事件；急停另走 token。
    MessageChannel<RemoteSensorData_t> canRxQueue; ///< 最新 CAN 传感器快照。
    MessageChannel<ServoTargetBatch_t> servoTargetQueue; ///< 唯一舵机执行批次邮箱。
    MessageChannel<ServoAngleData_t> servoFeedbackQueue; ///< 控制、标定和上行共读的 logical ABS 快照。
    MessageChannel<ServoAngleData_t> servoRawQueue; ///< 单圈 raw 快照。
    MessageChannel<ServoTelemetryData_t> servoTelemetryQueue; ///< 控制、标定和上行共读的最新遥测。
    MessageChannel<MappedAngleData_t> mappedAngleQueue; ///< 21 关节角快照。
    MessageChannel<RemoteTactileData_t> tactileQueue; ///< 顺序触觉数据。
    MessageChannel<MultiTurnNvsSaveJob_t> multiturnNvsSaveQueue; ///< NVS 保存请求。
    MessageChannel<MultiTurnNvsSaveResult_t> multiturnNvsResultQueue; ///< NVS 保存结果。
    MessageChannel<ManualNvsSaveJob> manualNvsSaveQueue; ///< Manual 端点保存请求邮箱。
    MessageChannel<ManualNvsSaveResult> manualNvsResultQueue; ///< Manual 保存读回结果邮箱。
    MessageChannel<NvsControlJob> nvsControlQueue; ///< 多圈 NVS 开关/清空请求邮箱。
    MessageChannel<NvsControlResult> nvsControlResultQueue; ///< 多圈 NVS 控制结果邮箱。
    SemaphoreHandle_t commandStateMutex; ///< 保护目标角和直控命令的成组读写。
};

#endif // P4_RUNTIME_CHANNELS_INTER_TASK_CHANNELS_H
