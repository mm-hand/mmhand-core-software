/**
 * @file HostTelemetryPublisher.h
 * @brief 声明 P4 各类状态快照到上位机协议包的唯一编码与发送入口。
 * @details 本模块只读运行快照并发送串口帧，不解释下行命令、不迁移系统状态。
 */
#ifndef P4_TASKS_HOST_COMM_HOST_TELEMETRY_PUBLISHER_H
#define P4_TASKS_HOST_COMM_HOST_TELEMETRY_PUBLISHER_H

#include <stdint.h>

#include "../../system/calibration/CalibrationRuntimeStore.h"
#include "../../system/TaskRuntimeMetrics.h"
#include "../../system/SystemRuntimeContext.h"
#include "../../data/SafetyRuleTypes.h"
#include "../../data/ControlTraceTypes.h"

namespace p4::host {

bool calibrationTendonLutMatchesJoint(
    uint8_t jointIndex,
    const JointCalibrationResult& result);
bool activeManualReferencesReady(SystemRuntimeContext* sharedData);
bool sendSystemStatusPacket(SystemRuntimeContext* sharedData);
bool sendManualEncoderReferencePacket(SystemRuntimeContext* sharedData);
bool sendTaskRuntimePacket(const TaskRuntimeSnapshot& diag);
bool sendFaultStatusPacket(SystemRuntimeContext* sharedData,
                           uint32_t overloadFaultBitmap);
bool sendMotorAbsLimitPacket(SystemRuntimeContext* sharedData);
bool sendServoLogicalRangePacket(SystemRuntimeContext* sharedData);
bool sendJointCalibSnapshotPacket(uint8_t jointIndex);
bool sendEncoderRawPacket(const RemoteSensorData_t& sensorRaw);
bool sendMappedAnglePacket(const MappedAngleData_t& mappedAngle);
bool sendControlTracePacket(const ControlTraceData_t& trace);
bool sendServoPositionPackets(const ServoAngleData_t& position);
bool sendServoTelemetryPacket(const ServoTelemetryData_t& telemetry);
bool sendTactileSummaryPacket(const RemoteTactileData_t& tactile);
bool sendTactileFullPackets(const RemoteTactileData_t& tactile);
/** 上报异步多圈恢复最终结果；仅在结果 token 变化或同步请求时发送。 */
bool sendMultiTurnRestoreResultPacket(uint32_t resultToken,
                                      uint32_t restoredMask,
                                      uint32_t rejectedMask);
/** 实际 ABS 设零的最终结果；原帧长度保留。 */
bool sendServoRebaseZeroResultPacket(uint32_t resultToken,
                                     uint32_t successMask,
                                     uint32_t failedMask);
/** 普通直控总线写入结果；0 token 表示未请求回执关联。 */
bool sendDirectTargetResultPacket(uint32_t token,
                                           uint32_t requestedMask,
                                           uint32_t appliedMask,
                                           uint32_t rejectedMask,
                                           uint8_t status);
bool sendServoRouteStatusPacket(SystemRuntimeContext* sharedData);
/** 发送一条已去重的安全规则事件；时间戳使用事件实际触发时刻。 */
bool sendSafetyEventPacket(const SafetyEventRecord& event,
                           uint32_t overflowCount);
bool sendProductFingerprintPacket();

} // namespace p4::host

#endif // P4_TASKS_HOST_COMM_HOST_TELEMETRY_PUBLISHER_H
