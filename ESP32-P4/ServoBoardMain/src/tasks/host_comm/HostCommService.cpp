/**
 * @file HostCommService.cpp
 * @brief 实现上位机串口收包、命令分发和状态遥测发包服务。
 */
#include "HostCommService.h"
#include "HostProtocolCodec.h"
#include "HostCommandDispatcher.h"
#include "HostTelemetryPublisher.h"
#include "HostAckPublisher.h"
#include "HostMultiTurnPublisher.h"
#include "HostCalibrationCommandHandler.h"
#include "../../data/SafetyTypes.h"
#include "../../runtime/SystemRuntimeContext.h"
#include "../../platform/rtos/SharedSafetyState.h"
#include "../../modules/calibration/CalibrationMath.h"
#include "../../modules/calibration/CalibrationRuntimeStore.h"
#include "../../modules/calibration/ManualEncoderReferenceStore.h"
#include "../../modules/protocol/HostProtocol.h"
#include "../../generated/UrdfJointLimits.generated.h"
#include "../../generated/ProductFingerprint.generated.h"
#include "../../hal/serial/HostSerialPort.h"
#include "../../config/DiagnosticsConfig.h"
#include "../../config/topology/HardwareMap.h"
#include "../../platform/rtos/AtomicValue.h"
#include "../../platform/rtos/TaskSignal.h"
#include "../../platform/log/FirmwareLog.h"
#include "../../runtime/ports/SystemEventPort.h"
#include "../../app/AppBootstrap.h"
#include "../../runtime/metrics/TaskRuntimeMetrics.h"
#include "../../runtime/metrics/RuntimeMonitorConfig.h"
#include "HostCommandRouter.h"
#include "HostFrameReader.h"
#include "../../safety/SafetyEventReporter.h"
#include "../../safety/policies/CommunicationSafetyPolicy.h"
#include "../../safety/SafetyEventStore.h"
#include <math.h>
#include <string.h>
using namespace p4::host;
/** 上位机通信跨周期状态；仅由 HostCommTask 单线程访问。 */
struct HostCommRuntimeContext {
    SystemRuntimeContext* sharedData;
    bool initialized;
    RemoteSensorData_t sensorRawData;
    MappedAngleData_t mappedAngleData;
    ControlTraceData_t controlTraceData;
    p4::host::HostFrameReader frameReader;
    uint8_t decodedFrame[kSerialRxBufferSize];
    uint32_t lastSensorTimestampSent;
    uint32_t lastControlTraceSequenceSent;
    uint32_t lastControlTraceTimestampSent;
    bool controlTraceSentInitialized;
    uint32_t lastEncoderRawTimestampSent;
    uint32_t lastServoAngleTimestampSent;
    uint32_t lastServoRawTimestampSent;
    uint32_t lastServoTelemetryTimestampSent;
    uint32_t lastFaultStatusSent;
    uint32_t lastFaultStatusSentMs;
    bool faultStatusSentInitialized;
    uint32_t lastSystemStatusTransitionSent;
    uint32_t lastFeedbackCollisionTransitionSent;
    uint32_t lastSystemStatusSentMs;
    bool systemStatusSentInitialized;
    uint32_t lastServoRouteGenerationSent;
    uint32_t lastManualReferenceSequenceSent;
    uint32_t lastManualReferenceValidMaskSent;
    uint8_t lastManualReferenceCommittedSent;
    uint32_t lastMotorAbsLimitHashSent;
    bool motorAbsLimitSentInitialized;
    uint32_t lastServoLogicalRangeHashSent;
    bool servoLogicalRangeSentInitialized;
    uint32_t lastCalibrationCompletedMask;
    uint32_t lastCalibrationDiagSequenceSent;
    uint32_t lastTaskDiagWindowSequence;
    uint32_t sendingTaskDiagWindowSequence;
    uint8_t nextTaskDiagId;
    bool taskDiagSubscribed;
    bool controlTraceSubscribed;
    uint32_t lastTaskDiagRenewMs; ///< 最近一次独立诊断订阅/续订时间，不参与运动控制租约。
    uint32_t pendingStateSyncMask;
    uint32_t lastServoResetZeroResultToken;
    uint32_t lastSafetyEventSequenceSent;
    uint32_t lastUpperFrameRejectEventMs; ///< COM-6005 限频时间戳。
    uint32_t pendingUartRxErrorFlags; ///< 尚未成功上报的 UART overflow/frame/parity 错误位。
};
static HostCommRuntimeContext g_hostCommRuntime;
/** 计算静态 int32 数组的轻量摘要，用于发现“有效位不变但边界值改变”。 */
static uint32_t appendFnv1a(uint32_t hash, const void* data, size_t size)
{
    const uint8_t* bytes = static_cast<const uint8_t*>(data);
    for (size_t i = 0; i < size; ++i) {
        hash ^= bytes[i];
        hash *= 16777619UL;
    }
    return hash;
}
/** 根据已完成一致性拷贝的 ABS 限位快照计算摘要；本函数不得直接读取共享上下文。 */
static uint32_t motorAbsLimitStateHash(const int32_t* mins,
                                       const int32_t* maxs,
                                       uint32_t validMask)
{
    uint32_t hash = 2166136261UL;
    hash = appendFnv1a(hash, &validMask, sizeof(validMask));
    hash = appendFnv1a(hash, mins, sizeof(int32_t) * SERVO_TOTAL_NUM);
    return appendFnv1a(hash, maxs, sizeof(int32_t) * SERVO_TOTAL_NUM);
}
/** 根据已完成一致性拷贝的 logical range 快照计算摘要；用于变化触发上报。 */
static uint32_t servoLogicalRangeStateHash(const int32_t* mins,
                                           const int32_t* maxs,
                                           uint32_t validMask)
{
    uint32_t hash = 2166136261UL;
    hash = appendFnv1a(hash, &validMask, sizeof(validMask));
    hash = appendFnv1a(hash, mins, sizeof(int32_t) * SERVO_TOTAL_NUM);
    return appendFnv1a(hash, maxs, sizeof(int32_t) * SERVO_TOTAL_NUM);
}
void requestHostStateSync(uint32_t syncMask)
{
    g_hostCommRuntime.pendingStateSyncMask |=
        syncMask & (uint32_t)STATE_SYNC_ALL;
}
void setHostTaskDiagSubscribed(bool subscribed, uint32_t nowMs)
{
    if (subscribed) {
        g_hostCommRuntime.lastTaskDiagRenewMs = nowMs;
        if (g_hostCommRuntime.taskDiagSubscribed) {
            // 周期续订不得清空发送进度，否则可能永远收不齐同一秒的七个任务包。
            return;
        }
        g_hostCommRuntime.taskDiagSubscribed = true;
    } else {
        g_hostCommRuntime.taskDiagSubscribed = false;
        g_hostCommRuntime.lastTaskDiagRenewMs = 0;
    }
    g_hostCommRuntime.lastTaskDiagWindowSequence = 0;
    g_hostCommRuntime.sendingTaskDiagWindowSequence = 0;
    g_hostCommRuntime.nextTaskDiagId = 0;
}

void setHostControlTraceSubscribed(bool subscribed)
{
    auto* sharedData = g_hostCommRuntime.sharedData;
    if (sharedData) {
        sharedAtomicStore(&sharedData->controlTraceRecordingEnabled, (uint8_t)0);
        sharedData->controlTraceQueue.reset();
        sharedAtomicStore(&sharedData->controlTraceRecordingEnabled, (uint8_t)subscribed);
    }
    g_hostCommRuntime.controlTraceSubscribed = subscribed;
    if (subscribed) {
        // 强制发送订阅后的第一份最新快照。
        g_hostCommRuntime.lastControlTraceSequenceSent = 0;
        g_hostCommRuntime.lastControlTraceTimestampSent = 0;
        g_hostCommRuntime.controlTraceSentInitialized = false;
    }
}
bool initHostCommService(SystemRuntimeContext* sharedData)
{
    if (!sharedData) {
        return false;
    }
    memset(&g_hostCommRuntime, 0, sizeof(g_hostCommRuntime));
    g_hostCommRuntime.sharedData = sharedData;
    g_hostCommRuntime.lastServoRouteGenerationSent = 0xFFFFFFFFUL;
    g_hostCommRuntime.lastManualReferenceSequenceSent = 0xFFFFFFFFUL;
    g_hostCommRuntime.lastManualReferenceValidMaskSent = 0xFFFFFFFFUL;
    g_hostCommRuntime.lastManualReferenceCommittedSent = 0xFF;
    g_hostCommRuntime.lastCalibrationCompletedMask =
        sharedAtomicLoad(&sharedData->calibration_completed_joint_bitmap);
    g_hostCommRuntime.lastServoResetZeroResultToken =
        sharedAtomicLoad(&sharedData->servo_reset_zero_result_token);
    initHostMultiTurnPublisher();
    FW_LOG_INFO_PRINTLN("<<<SYS_READY>>>");
    g_hostCommRuntime.initialized = true;
    return true;
}
bool stepHostCommService(SystemRuntimeContext* sharedData,
                         uint32_t nowMs,
                         const HostCommStepRequest& request)
{
    if (!sharedData || !g_hostCommRuntime.initialized) {
        return false;
    }
    HostCommRuntimeContext& runtime = g_hostCommRuntime;
    auto& sensorRawData = runtime.sensorRawData;
    auto& mappedAngleData = runtime.mappedAngleData;
    auto& controlTraceData = runtime.controlTraceData;
    auto& frameReader = runtime.frameReader;
    auto& decodedFrame = runtime.decodedFrame;
    auto& lastSensorTimestampSent = runtime.lastSensorTimestampSent;
    auto& lastControlTraceSequenceSent = runtime.lastControlTraceSequenceSent;
    auto& lastEncoderRawTimestampSent = runtime.lastEncoderRawTimestampSent;
    auto& lastServoAngleTimestampSent = runtime.lastServoAngleTimestampSent;
    auto& lastServoRawTimestampSent = runtime.lastServoRawTimestampSent;
    auto& lastServoTelemetryTimestampSent = runtime.lastServoTelemetryTimestampSent;
    auto& lastFaultStatusSent = runtime.lastFaultStatusSent;
    auto& lastFaultStatusSentMs = runtime.lastFaultStatusSentMs;
    auto& faultStatusSentInitialized = runtime.faultStatusSentInitialized;
    auto& lastSystemStatusTransitionSent = runtime.lastSystemStatusTransitionSent;
    auto& lastFeedbackCollisionTransitionSent = runtime.lastFeedbackCollisionTransitionSent;
    auto& lastSystemStatusSentMs = runtime.lastSystemStatusSentMs;
    auto& systemStatusSentInitialized = runtime.systemStatusSentInitialized;
    auto& lastServoRouteGenerationSent = runtime.lastServoRouteGenerationSent;
    auto& lastManualReferenceSequenceSent = runtime.lastManualReferenceSequenceSent;
    auto& lastManualReferenceValidMaskSent = runtime.lastManualReferenceValidMaskSent;
    auto& lastManualReferenceCommittedSent = runtime.lastManualReferenceCommittedSent;
    auto& lastMotorAbsLimitHashSent = runtime.lastMotorAbsLimitHashSent;
    auto& motorAbsLimitSentInitialized = runtime.motorAbsLimitSentInitialized;
    auto& lastServoLogicalRangeHashSent = runtime.lastServoLogicalRangeHashSent;
    auto& servoLogicalRangeSentInitialized = runtime.servoLogicalRangeSentInitialized;
    auto& lastCalibrationCompletedMask = runtime.lastCalibrationCompletedMask;
    auto& lastCalibrationDiagSequenceSent = runtime.lastCalibrationDiagSequenceSent;
    auto& lastTaskDiagWindowSequence = runtime.lastTaskDiagWindowSequence;
    auto& sendingTaskDiagWindowSequence = runtime.sendingTaskDiagWindowSequence;
    auto& nextTaskDiagId = runtime.nextTaskDiagId;
    auto& taskDiagSubscribed = runtime.taskDiagSubscribed;
    auto& pendingStateSyncMask = runtime.pendingStateSyncMask;
    auto& lastServoResetZeroResultToken = runtime.lastServoResetZeroResultToken;
    auto& lastSafetyEventSequenceSent = runtime.lastSafetyEventSequenceSent;
    const bool processRx = request.periodicDue ||
        (request.wakeReasons & p4::runtime::wake::kHostRx) != 0U;
    if (processRx) {
        runtime.pendingUartRxErrorFlags |= HostSerialPort::takeRxErrorFlags();
        if (runtime.pendingUartRxErrorFlags != 0U &&
            p4::safety::communicationEventMayReport(
                nowMs,
                runtime.lastUpperFrameRejectEventMs,
                1000U)) {
            runtime.lastUpperFrameRejectEventMs = nowMs;
            p4::safety::reportSafetyRuleEvent(
                sharedData,
                SafetyRuleId::CommunicationUpperFrameRejected,
                0,
                (int32_t)(0x10000UL | runtime.pendingUartRxErrorFlags));
            runtime.pendingUartRxErrorFlags = 0U;
        }
        // 旧 ACK 未写出时不再消费下一条命令，保证命令执行与 ACK 顺序一致。
        if (!retryPendingProtoAckPacket()) {
            return true;
        }
        while (HostSerialPort::available())
        {
            const int byteVal = HostSerialPort::readByte();
            if (byteVal < 0) {
                break;
            }
            frameReader.push((uint8_t)byteVal);
        }
        size_t decodedLength = 0;
        while (frameReader.pop(decodedFrame,
                               sizeof(decodedFrame),
                               &decodedLength)) {
            dispatchHostCommand(sharedData, decodedFrame, decodedLength);
            if (!retryPendingProtoAckPacket()) {
                return true;
            }
        }
        ProtocolFrameRejectReason frameRejectReason =
            ProtocolFrameRejectReason::None;
        if (frameReader.takeRejectReason(&frameRejectReason) &&
            p4::safety::communicationEventMayReport(
                nowMs,
                runtime.lastUpperFrameRejectEventMs,
                1000U)) {
            runtime.lastUpperFrameRejectEventMs = nowMs;
            p4::safety::reportSafetyRuleEvent(
                sharedData,
                SafetyRuleId::CommunicationUpperFrameRejected,
                0,
                (int32_t)frameRejectReason);
        }
    }

    // RX 通知只负责尽快完成解帧、命令分发和 ACK。遥测与静态状态同步
    // 固定在 5 ms 周期，避免每个输入事件都触发一轮完整上行广播。
    if (!request.periodicDue) {
        return true;
    }
    processServoResetZeroResult(
        sharedData, &lastServoResetZeroResultToken);

        // 50 ms 批次最多发送 8 帧；失败保留队首重试，控制任务永不等待串口。
        if (runtime.controlTraceSubscribed &&
            (!runtime.controlTraceSentInitialized ||
             (uint32_t)(nowMs -
                        runtime.lastControlTraceTimestampSent) >=
                 mmhand::config::diagnostics::kControlTracePublishPeriodMs)) {
            runtime.lastControlTraceTimestampSent = nowMs;
            runtime.controlTraceSentInitialized = true;
            for (uint8_t count = 0; count < 8 &&
                 sharedData->controlTraceQueue.peek(&controlTraceData); ++count) {
                if (!sendControlTracePacket(controlTraceData)) break;
                lastControlTraceSequenceSent = controlTraceData.sequence;
                sharedData->controlTraceQueue.receive(&controlTraceData);
            }
        }

        if (sharedData->mappedAngleQueue.peek(&mappedAngleData) &&
            (!runtime.controlTraceSubscribed || (uint32_t)(mappedAngleData.timestamp - lastSensorTimestampSent) >= 50) &&
            mappedAngleData.timestamp != lastSensorTimestampSent)
        {
            if (sendMappedAnglePacket(mappedAngleData)) {
                lastSensorTimestampSent = mappedAngleData.timestamp;
            }
        }

        if (sharedData->canRxQueue.peek(&sensorRawData) &&
            sensorRawData.timestamp != lastEncoderRawTimestampSent)
        {
            if (sendEncoderRawPacket(sensorRawData)) {
                lastEncoderRawTimestampSent = sensorRawData.timestamp;
            }
        }

        RemoteTactileData_t tactileData;
        if (sharedData->tactileQueue.peek(&tactileData))
        {
            bool tactileSent = true;
            if (tactileData.isValid) {
                tactileSent = sendTactileSummaryPacket(tactileData);
            }
            if (tactileSent && tactileData.fullValid) {
                tactileSent = sendTactileFullPackets(tactileData);
            }
            if (tactileSent) {
                // 只有整个摘要/全量事件写完，才提交队首。
                sharedData->tactileQueue.receive(&tactileData);
            }
        }

        ServoAngleData_t servoRawData;
        if (sharedData->servoRawQueue.peek(&servoRawData) &&
            servoRawData.timestamp != lastServoRawTimestampSent)
        {
            if (sendServoRawPacket(servoRawData)) {
                lastServoRawTimestampSent = servoRawData.timestamp;
            }
        }

        ServoTelemetryData_t telemetryData;
        if (sharedData->servoTelemetryQueue.peek(&telemetryData) &&
            telemetryData.timestamp != lastServoTelemetryTimestampSent)
        {
            if (sendServoTelemetryPacket(telemetryData)) {
                lastServoTelemetryTimestampSent = telemetryData.timestamp;
            }
        }

        ServoAngleData_t servoAngleData;
        if (sharedData->servoFeedbackQueue.peek(&servoAngleData) &&
            (!runtime.controlTraceSubscribed || (uint32_t)(servoAngleData.timestamp - lastServoAngleTimestampSent) >= 50) &&
            servoAngleData.timestamp != lastServoAngleTimestampSent)
        {
            if (sendServoAnglePacket(servoAngleData)) {
                lastServoAngleTimestampSent = servoAngleData.timestamp;
            }
        }
        const uint32_t faultBitmap =
            sharedAtomicLoad(&sharedData->overload_fault_bitmap);
        const bool bitmapChanged =
            (!faultStatusSentInitialized) || (faultBitmap != lastFaultStatusSent);
        const bool heartbeatDue =
            (!faultStatusSentInitialized) || ((nowMs - lastFaultStatusSentMs) >= kFaultStatusHeartbeatMs);
        if (bitmapChanged || heartbeatDue) {
            if (sendFaultStatusPacket(sharedData, faultBitmap)) {
                lastFaultStatusSent = faultBitmap;
                lastFaultStatusSentMs = nowMs;
                faultStatusSentInitialized = true;
            }
        }


        const uint32_t stateTransitionCount = sharedData->state_transition_count;
        const uint32_t feedbackCollisionTransitionCount =
            sharedAtomicLoad(&sharedData->feedback_collision_transition_count);
        const bool systemStatusChanged =
            (!systemStatusSentInitialized) ||
            (stateTransitionCount != lastSystemStatusTransitionSent) ||
            (feedbackCollisionTransitionCount !=
             lastFeedbackCollisionTransitionSent);
        const bool systemStatusHeartbeatDue =
            (!systemStatusSentInitialized) ||
            ((nowMs - lastSystemStatusSentMs) >= kSystemStatusHeartbeatMs);
        if (systemStatusChanged || systemStatusHeartbeatDue) {
            if (sendSystemStatusPacket(sharedData)) {
                taskRuntimeMarkSystemStatusSent();
                lastSystemStatusTransitionSent = stateTransitionCount;
                lastFeedbackCollisionTransitionSent =
                    feedbackCollisionTransitionCount;
                lastSystemStatusSentMs = nowMs;
                systemStatusSentInitialized = true;
            }
        }

        const bool multiTurnSyncRequested =
            (pendingStateSyncMask & STATE_SYNC_MULTITURN_STATE) != 0;
        if (stepHostMultiTurnPublisher(
                sharedData, multiTurnSyncRequested)) {
            pendingStateSyncMask &= ~((uint32_t)STATE_SYNC_MULTITURN_STATE);
        }

        // 每周期最多发送四条新事件，既不丢失规则证据，也不让突发诊断占满串口。
        SafetyEventRecord safetyEvents[4] = {};
        const size_t safetyEventCount = p4::safety::copySafetyEventsAfter(
            lastSafetyEventSequenceSent,
            safetyEvents,
            sizeof(safetyEvents) / sizeof(safetyEvents[0]));
        for (size_t i = 0; i < safetyEventCount; ++i) {
            if (sendSafetyEventPacket(
                    safetyEvents[i], p4::safety::safetyEventOverflowCount())) {
                lastSafetyEventSequenceSent = safetyEvents[i].sequence;
            } else {
                // 保持 sequence，不得越过失败事件发送后续事件。
                break;
            }
        }

        const uint32_t manualReferenceSequence =
            sharedAtomicLoad(&sharedData->manual_reference_revision);
        const uint32_t manualReferenceValidMask =
            sharedAtomicLoad(&sharedData->manual_reference_valid_bitmap);
        const uint8_t manualReferenceCommitted =
            sharedData->manual_reference_committed ? 1 : 0;
        const bool manualReferenceChanged =
            manualReferenceSequence != lastManualReferenceSequenceSent ||
            manualReferenceValidMask != lastManualReferenceValidMaskSent ||
            manualReferenceCommitted != lastManualReferenceCommittedSent;
        const bool manualReferenceSyncRequested =
            (pendingStateSyncMask & STATE_SYNC_MANUAL_REFERENCE) != 0;
        if (manualReferenceChanged || manualReferenceSyncRequested) {
            if (sendManualEncoderReferencePacket(sharedData)) {
                lastManualReferenceSequenceSent = manualReferenceSequence;
                lastManualReferenceValidMaskSent = manualReferenceValidMask;
                lastManualReferenceCommittedSent = manualReferenceCommitted;
                pendingStateSyncMask &= ~((uint32_t)STATE_SYNC_MANUAL_REFERENCE);
            }
        }

        // 诊断窗口使用独立租约；控制心跳偶发抖动不能永久关闭 Task Runtime。
        if (taskDiagSubscribed &&
            (uint32_t)(nowMs - runtime.lastTaskDiagRenewMs) >
                kTaskRuntimeSubscriptionLeaseMs) {
            setHostTaskDiagSubscribed(false, nowMs);
        }
        // 仅在 RuntimeMonitor 冻结出新的一秒窗口后发送一组诊断；每次只发一个任务包，
        // 避免诊断窗口抢占正常状态、传感器和舵机协议带宽。
        const uint32_t latestTaskDiagWindow = taskRuntimeGetWindowSequence();
        if (taskDiagSubscribed && latestTaskDiagWindow != 0U &&
            latestTaskDiagWindow != lastTaskDiagWindowSequence &&
            latestTaskDiagWindow != sendingTaskDiagWindowSequence) {
            sendingTaskDiagWindowSequence = latestTaskDiagWindow;
            nextTaskDiagId = 0;
        }
        if (taskDiagSubscribed && sendingTaskDiagWindowSequence != 0U) {
            TaskRuntimeSnapshot runtimeDiag;
            if (taskRuntimeGetSnapshot(nextTaskDiagId, &runtimeDiag)) {
                if (runtimeDiag.windowSequence != sendingTaskDiagWindowSequence) {
                    // 发送过程中恰好切入下一秒窗口，丢弃半组旧包并从新窗口重新开始。
                    sendingTaskDiagWindowSequence = runtimeDiag.windowSequence;
                    nextTaskDiagId = 0;
                } else if (sendTaskRuntimePacket(runtimeDiag)) {
                    nextTaskDiagId =
                        (uint8_t)((nextTaskDiagId + 1) % TASK_RUNTIME_COUNT);
                    if (nextTaskDiagId == 0U) {
                        lastTaskDiagWindowSequence = sendingTaskDiagWindowSequence;
                        sendingTaskDiagWindowSequence = 0;
                    }
                }
            } else {
                sendingTaskDiagWindowSequence = 0;
            }
        }

        // 先在同一临界区内复制边界，再在临界区外计算摘要，避免长时间占用共享锁。
        int32_t rangeHashMins[SERVO_TOTAL_NUM];
        int32_t rangeHashMaxs[SERVO_TOTAL_NUM];
        uint32_t rangeHashValidMask = 0;
        {
            SharedSafetyGuard guard;
            memcpy(rangeHashMins,
                   sharedData->motor_abs_limit_min,
                   sizeof(rangeHashMins));
            memcpy(rangeHashMaxs,
                   sharedData->motor_abs_limit_max,
                   sizeof(rangeHashMaxs));
            rangeHashValidMask =
                sharedAtomicLoad(&sharedData->motor_abs_limit_valid_bitmap);
        }
        const uint32_t motorAbsLimitHash = motorAbsLimitStateHash(
            rangeHashMins,
            rangeHashMaxs,
            rangeHashValidMask);
        const bool motorAbsLimitSyncRequested =
            (pendingStateSyncMask & STATE_SYNC_MOTOR_ABS_LIMIT) != 0;
        if (!motorAbsLimitSentInitialized ||
            motorAbsLimitHash != lastMotorAbsLimitHashSent ||
            motorAbsLimitSyncRequested) {
            if (sendMotorAbsLimitPacket(sharedData)) {
                lastMotorAbsLimitHashSent = motorAbsLimitHash;
                motorAbsLimitSentInitialized = true;
                pendingStateSyncMask &= ~((uint32_t)STATE_SYNC_MOTOR_ABS_LIMIT);
            }
        }

        {
            SharedSafetyGuard guard;
            memcpy(rangeHashMins,
                   sharedData->servo_logical_range_min,
                   sizeof(rangeHashMins));
            memcpy(rangeHashMaxs,
                   sharedData->servo_logical_range_max,
                   sizeof(rangeHashMaxs));
            rangeHashValidMask =
                sharedAtomicLoad(&sharedData->servo_logical_range_valid_bitmap);
        }
        const uint32_t servoLogicalRangeHash = servoLogicalRangeStateHash(
            rangeHashMins,
            rangeHashMaxs,
            rangeHashValidMask);
        const bool logicalRangeSyncRequested =
            (pendingStateSyncMask & STATE_SYNC_SERVO_LOGICAL_RANGE) != 0;
        if (!servoLogicalRangeSentInitialized ||
            servoLogicalRangeHash != lastServoLogicalRangeHashSent ||
            logicalRangeSyncRequested) {
            if (sendServoLogicalRangePacket(sharedData)) {
                lastServoLogicalRangeHashSent = servoLogicalRangeHash;
                servoLogicalRangeSentInitialized = true;
                pendingStateSyncMask &= ~((uint32_t)STATE_SYNC_SERVO_LOGICAL_RANGE);
            }
        }

        // 标定快照不再每秒全量广播：新完成关节立即上报；重连或查询时全量同步。
        const uint32_t completedThisRun =
            sharedAtomicLoad(&sharedData->calibration_completed_joint_bitmap) &
            ((1UL << ENCODER_TOTAL_NUM) - 1UL);
        const uint32_t newlyCompleted =
            completedThisRun & ~lastCalibrationCompletedMask;
        // 已清除的关节必须从已发送位图中退出，后续再次完成时才能重新上报。
        lastCalibrationCompletedMask &= completedThisRun;
        const bool calibrationSyncRequested =
            (pendingStateSyncMask & STATE_SYNC_JOINT_CALIBRATION) != 0;
        const uint32_t snapshotMask = calibrationSyncRequested
            ? ((1UL << ENCODER_TOTAL_NUM) - 1UL)
            : newlyCompleted;
        bool calibrationSyncSucceeded = true;
        for (uint8_t jointIndex = 0; jointIndex < ENCODER_TOTAL_NUM; ++jointIndex) {
            JointCalibrationRuntimeSnapshot snapshot;
            const bool snapshotValid =
                readJointCalibrationRuntimeSnapshot(jointIndex, &snapshot);
            if ((snapshotMask & (1UL << jointIndex)) != 0 &&
                snapshotValid && snapshot.result.success) {
                if (sendJointCalibSnapshotPacket(jointIndex)) {
                    lastCalibrationCompletedMask |= 1UL << jointIndex;
                    // 本周期的映射角可能早于快照发送。强制下一周期再发一帧，
                    // 保证上位机在应用新快照后立即获得同一标定代次的 Current。
                    lastSensorTimestampSent = 0xFFFFFFFFUL;
                } else {
                    calibrationSyncSucceeded = false;
                }
            }
        }
        if (calibrationSyncRequested && calibrationSyncSucceeded) {
            pendingStateSyncMask &= ~((uint32_t)STATE_SYNC_JOINT_CALIBRATION);
        }

        CalibrationDiagData_t calibDiag;
        {
            SharedSafetyGuard guard;
            calibDiag = sharedData->calibration_diag;
        }
        const bool calibrationDiagChanged =
            calibDiag.sequence != lastCalibrationDiagSequenceSent;
        if (calibrationDiagChanged) {
            if (sendCalibrationDiagPacket(calibDiag)) {
                lastCalibrationDiagSequenceSent = calibDiag.sequence;
            }
        }

        const uint32_t servoRouteGeneration =
            sharedAtomicLoad(&sharedData->servo_route_generation);
        const bool servoRouteSyncRequested =
            (pendingStateSyncMask & STATE_SYNC_SERVO_ROUTE) != 0;
        if (servoRouteGeneration != lastServoRouteGenerationSent ||
            servoRouteSyncRequested) {
            if (sendServoRouteStatusPacket(sharedData)) {
                lastServoRouteGenerationSent = servoRouteGeneration;
                pendingStateSyncMask &= ~((uint32_t)STATE_SYNC_SERVO_ROUTE);
            }
        }

        if ((pendingStateSyncMask & STATE_SYNC_PRODUCT_FINGERPRINT) != 0) {
            if (sendProductFingerprintPacket()) {
                pendingStateSyncMask &= ~((uint32_t)STATE_SYNC_PRODUCT_FINGERPRINT);
            }
        }

    return true;
}
