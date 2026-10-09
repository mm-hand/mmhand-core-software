/**
 * @file HostCommandDispatcher.cpp
 * @brief 实现上位机命令的长度校验、状态门禁、执行分发与 ACK 返回。
 * @details 本文件只处理已经由 HostFrameReader 解码完成的 CMD+payload。
 */
#include "HostCommandDispatcher.h"
#include "HostCommService.h"
#include "../../platform/time/Clock.h"
#include "HostTelemetryPublisher.h"
#include "HostAckPublisher.h"
#include "HostProtocolCodec.h"
#include "HostCalibrationCommandHandler.h"
#include "../../data/RecoveryTypes.h"
#include "../../data/SafetyTypes.h"
#include "../../runtime/SystemRuntimeContext.h"
#include "../../platform/rtos/SharedSafetyState.h"
#include "../../modules/calibration/CalibrationMath.h"
#include "../../modules/calibration/CalibrationRuntimeStore.h"
#include "../../modules/calibration/ManualEncoderReferenceStore.h"
#include "../../modules/calibration/TendonCalibrationLut.h"
#include "../../modules/protocol/HostProtocol.h"
#include "../../generated/UrdfJointLimits.generated.h"
#include "../../generated/ProductFingerprint.generated.h"
#include "../../hal/serial/HostSerialPort.h"
#include "../../config/topology/HardwareMap.h"
#include "../../config/CalibrationConfig.h"
#include "../../config/ServoConfig.h"
#include "../../platform/rtos/AtomicValue.h"
#include "../../platform/log/FirmwareLog.h"
#include "../../runtime/ports/SystemEventPort.h"
#include "../../runtime/metrics/TaskRuntimeMetrics.h"
#include "../../safety/SafetyEventReporter.h"
#include "../../runtime/ports/DirectTargetResult.h"
#include "HostCommandRouter.h"

#include <math.h>
#include <string.h>

using namespace p4::host;

namespace p4::host {

namespace {

bool servoRebaseZeroPending(const SystemRuntimeContext* sharedData)
{
    return sharedAtomicLoad(&sharedData->servo_rebase_zero_request_token) !=
           sharedAtomicLoad(&sharedData->servo_rebase_zero_result_token);
}

bool commandConflictsWithServoRebaseZero(uint8_t command)
{
    switch (command) {
        case CMD_START:
        case CMD_RUN_FEEDBACK:
        case CMD_ROPE_INIT:
        case CMD_MOTOR_POS_ABS:
        case CMD_MOTOR_POS_ABS_ONE:
        case CMD_ANGLE_CTRL:
        case CMD_ANGLE_CTRL_MASKED:
        case CMD_CALIB_LOAD_JOINT:
        case CMD_CALIB_LOAD_LIMITS:
            return true;
        default:
            return false;
    }
}

/**
 * Calib Load 的 ABS Limit 包可能来自较早的控制策略。单腱关节加载完
 * 完整结果后，必须以本次结果记录的机械紧端重新生成范围，避免旧快照
 * 中的 LUT 末点提前钳住紧绳方向。双腱范围仍沿用快照中的两电机结果。
 */
bool refreshLoadedSingleTendonAbsLimit(SystemRuntimeContext* sharedData,
                                       uint8_t jointIndex,
                                       const JointCalibrationResult& result)
{
    JointActuatorGroup group = {};
    if (getJointActuatorGroup(jointIndex, &group) && group.count > 1) {
        return true;
    }

    const int primaryMotor = findPrimaryMotorChannelForJoint(jointIndex);
    if (!sharedData || primaryMotor < 0 || primaryMotor >= SERVO_TOTAL_NUM) {
        return false;
    }

    int32_t limitMin = 0;
    int32_t limitMax = 0;
    if (!singleTendonCalibrationMotorAbsRange(
            result,
            0,
            (uint8_t)primaryMotor,
            mmhand::config::calibration::kMotorAbsReleaseMarginCounts,
            mmhand::config::calibration::kMotorAbsTightMarginCounts,
            &limitMin,
            &limitMax)) {
        return false;
    }

    const uint32_t motorBit = 1UL << (uint8_t)primaryMotor;
    {
        SharedSafetyGuard guard;
        sharedData->motor_abs_limit_min[primaryMotor] = limitMin;
        sharedData->motor_abs_limit_max[primaryMotor] = limitMax;
        sharedAtomicFetchOr(&sharedData->motor_abs_limit_valid_bitmap, motorBit);
    }
    FW_LOG_INFO_PRINTF(
        "[UPPER] CALIB_LOAD_SINGLE_LIMIT J%02u M%02d limit=[%ld,%ld] "
        "measuredTight=%ld margins=[%ld,%ld]\r\n",
        (unsigned)jointIndex,
        primaryMotor,
        (long)limitMin,
        (long)limitMax,
        (long)result.tightServoAbs,
        (long)mmhand::config::calibration::kMotorAbsReleaseMarginCounts,
        (long)mmhand::config::calibration::kMotorAbsTightMarginCounts);
    return true;
}

} // namespace

/**
 * 上报 Mxx 到物理 BUS 的运行时路由快照。
 *
 * BUS 位置不再属于 HardwareMap，因此上位机必须依据本包区分扫描中、离线、
 * 重复 ID 和 BUS 超容量。数组顺序固定为 M00..M26。
 */
// 应用一帧已通过长度检查的下行命令，并返回对应协议 ACK。
void dispatchHostCommand(SystemRuntimeContext* sharedData, const uint8_t* frame, size_t frameLen)
{
    if (!sharedData || !frame || frameLen == 0) {
        return;
    }

    const uint8_t cmd = frame[0];
    const uint32_t nowMs = p4::platform::time::monotonicMillis();
    // 每一帧通过结构校验的上位机命令都刷新心跳，不要求它一定是运动命令。
    // SystemStateTask 在另一核心读取该字段，统一走原子接口避免租约边界读到撕裂值。
    sharedAtomicStore(&sharedData->host_rx_heartbeat_ms, nowMs);

    if (servoRebaseZeroPending(sharedData) &&
        commandConflictsWithServoRebaseZero(cmd)) {
        sendProtoAckPacket(cmd, sharedData->system_state,
                           PROTO_ACK_STATUS_UNSUPPORTED_MODE);
        return;
    }
    if (sharedAtomicLoad(&sharedData->direct_target_request_generation) !=
            sharedAtomicLoad(&sharedData->direct_target_result_generation) &&
        (cmd == CMD_SERVO_REBASE_ZERO || cmd == CMD_SERVO_RESET_ZERO ||
         cmd == CMD_RESTORE_MULTITURN || cmd == CMD_ACTIVE_SCOPE)) {
        sendProtoAckPacket(cmd, sharedData->system_state,
                           PROTO_ACK_STATUS_UNSUPPORTED_MODE);
        return;
    }

    if (cmd == CMD_HOST_HEARTBEAT)
    {
        return;
    }
    if (cmd == CMD_MANUAL_ENCODER_QUERY)
    {
        const bool sent = sendManualEncoderReferencePacket(sharedData);
        sendProtoAckPacket(CMD_MANUAL_ENCODER_QUERY,
                           sharedData->system_state,
                           sent ? PROTO_ACK_STATUS_OK
                                : PROTO_ACK_STATUS_UNSUPPORTED_MODE);
        return;
    }
    if (cmd == CMD_STATE_SYNC_REQUEST)
    {
        const uint32_t requestedMask = readU32BigEndian(frame + 1);
        if ((requestedMask & ~((uint32_t)STATE_SYNC_ALL)) != 0) {
            sendProtoAckPacket(CMD_STATE_SYNC_REQUEST,
                               sharedData->system_state,
                               PROTO_ACK_STATUS_INVALID_DATA);
        } else {
            requestHostStateSync(requestedMask);
            sendProtoAckPacket(CMD_STATE_SYNC_REQUEST,
                               sharedData->system_state,
                               PROTO_ACK_STATUS_OK);
        }
        return;
    }
    if (cmd == CMD_TASK_DIAG_SUBSCRIBE)
    {
        const uint8_t enabled = frame[1];
        if (enabled > 1) {
            sendProtoAckPacket(CMD_TASK_DIAG_SUBSCRIBE,
                               sharedData->system_state,
                               PROTO_ACK_STATUS_INVALID_DATA);
        } else {
            setHostTaskDiagSubscribed(enabled != 0, nowMs);
            sendProtoAckPacket(CMD_TASK_DIAG_SUBSCRIBE,
                               sharedData->system_state,
                               PROTO_ACK_STATUS_OK);
        }
        return;
    }
    if (cmd == CMD_CONTROL_TRACE_SUBSCRIBE)
    {
        const uint8_t enabled = frame[1];
        if (enabled > 1) {
            sendProtoAckPacket(CMD_CONTROL_TRACE_SUBSCRIBE,
                               sharedData->system_state,
                               PROTO_ACK_STATUS_INVALID_DATA);
        } else {
            setHostControlTraceSubscribed(enabled != 0);
            sendProtoAckPacket(CMD_CONTROL_TRACE_SUBSCRIBE,
                               sharedData->system_state,
                               PROTO_ACK_STATUS_OK);
        }
        return;
    }

    if (cmd == CMD_ROPE_INIT)
    {
        const uint32_t requestedMask =
            readU32BigEndian(frame + 1) & kManualEncoderReferenceFullMask;
        const uint32_t activeMask =
            sharedAtomicLoad(&sharedData->active_joint_mask) &
            kManualEncoderReferenceFullMask;
        uint8_t status = PROTO_ACK_STATUS_QUEUED;
        if (requestedMask == 0 || requestedMask != activeMask) {
            status = PROTO_ACK_STATUS_INVALID_DATA;
        } else if (!activeManualReferencesReady(sharedData)) {
            status = PROTO_ACK_STATUS_MANUAL_REFERENCE_REQUIRED;
        } else if ((autoCalibrationValidMask(sharedData) & requestedMask) !=
                   requestedMask) {
            status = PROTO_ACK_STATUS_CALIBRATION_REQUIRED;
        } else if (sharedData->system_state != SYSTEM_STATE_IDLE &&
                   sharedData->system_state != SYSTEM_STATE_READY &&
                   sharedData->system_state != SYSTEM_STATE_RECOVERY_REQUIRED) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else if (!postSystemEvent(sharedData, SYSTEM_EVENT_ROPE_INIT)) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        }
        sendProtoAckPacket(CMD_ROPE_INIT, sharedData->system_state, status);
        return;
    }

    if (commandRequiresManualReference(cmd) &&
        !activeManualReferencesReady(sharedData)) {
        sendProtoAckPacket(cmd,
                           sharedData->system_state,
                           PROTO_ACK_STATUS_MANUAL_REFERENCE_REQUIRED);
        return;
    }

    if (cmd == CMD_CALIBRATE)
    {
        // 保留旧命令编号和帧长度，明确拒绝已迁至上位机的固件标定入口。
        sendProtoAckPacket(CMD_CALIBRATE,
                           sharedData->system_state,
                           PROTO_ACK_STATUS_UNSUPPORTED_MODE);
        return;
    }

    if (cmd == CMD_START)
    {
        postSystemEvent(sharedData, SYSTEM_EVENT_START);
        sendProtoAckPacket(CMD_START, sharedData->system_state, PROTO_ACK_STATUS_QUEUED);
        return;
    }

    if (cmd == CMD_RUN_FEEDBACK)
    {
        const uint32_t requestSequence =
            sharedAtomicAddFetch(&sharedData->feedback_admission_sequence, (uint32_t)1);
        sharedAtomicStore(&sharedData->feedback_admission_status,
                          (uint8_t)FEEDBACK_ADMISSION_PENDING);
        sharedAtomicStore(&sharedData->feedback_reject_reason,
                          (uint8_t)FEEDBACK_REJECT_NONE);
        if (!activeManualReferencesReady(sharedData)) {
            sharedAtomicStore(&sharedData->feedback_admission_status,
                              (uint8_t)FEEDBACK_ADMISSION_REJECTED);
            sharedAtomicStore(&sharedData->feedback_reject_reason,
                              (uint8_t)FEEDBACK_REJECT_MANUAL_REFERENCE);
            sendProtoAckPacket(
                CMD_RUN_FEEDBACK,
                sharedData->system_state,
                PROTO_ACK_STATUS_MANUAL_REFERENCE_REQUIRED);
        } else {
            if (!postSystemEventForToken(sharedData,
                                         SYSTEM_EVENT_RUN_FEEDBACK,
                                         requestSequence)) {
                sharedAtomicStore(&sharedData->feedback_admission_status,
                                  (uint8_t)FEEDBACK_ADMISSION_REJECTED);
                sharedAtomicStore(&sharedData->feedback_reject_reason,
                                  (uint8_t)FEEDBACK_REJECT_EVENT_QUEUE);
            }
            sendProtoAckPacket(CMD_RUN_FEEDBACK,
                               sharedData->system_state,
                               PROTO_ACK_STATUS_QUEUED);
        }
        return;
    }

    if (cmd == CMD_PAUSE)
    {
        postSystemEvent(sharedData, SYSTEM_EVENT_PAUSE);
        sendProtoAckPacket(CMD_PAUSE, sharedData->system_state, PROTO_ACK_STATUS_QUEUED);
        return;
    }

    if (cmd == CMD_STOP)
    {
        taskRuntimeMarkStopReceived();
        const bool accepted = requestEmergencyStop(sharedData);
        sendProtoAckPacket(CMD_STOP,
                           sharedData->system_state,
                           accepted ? PROTO_ACK_STATUS_QUEUED
                                    : PROTO_ACK_STATUS_UNSUPPORTED_MODE);
        return;
    }

    if (cmd == CMD_RESET)
    {
        postSystemEvent(sharedData, SYSTEM_EVENT_RESET);
        sendProtoAckPacket(CMD_RESET, sharedData->system_state, PROTO_ACK_STATUS_QUEUED);
        return;
    }

    if (cmd == CMD_ANGLE_CTRL)
    {
        float parsedAngles[ENCODER_TOTAL_NUM] = {0.0f};
        if (parseFloatArrayLittleEndian(frame + 1, frameLen - 1, parsedAngles, ENCODER_TOTAL_NUM)) {
            upperApplyTargetAngles(sharedData, parsedAngles, ENCODER_TOTAL_NUM);
        }
        return;
    }

    if (cmd == CMD_ANGLE_CTRL_MASKED)
    {
        if (frameLen < 1 + kMaskedFloatPayloadBytes) {
            return;
        }
        const uint32_t jointMask =
            ((uint32_t)frame[1] << 24) |
            ((uint32_t)frame[2] << 16) |
            ((uint32_t)frame[3] << 8) |
            (uint32_t)frame[4];
        float parsedAngles[ENCODER_TOTAL_NUM] = {0.0f};
        if (parseFloatArrayLittleEndian(frame + 5,
                                        frameLen - 5,
                                        parsedAngles,
                                        ENCODER_TOTAL_NUM)) {
            upperApplyTargetAnglesMasked(sharedData,
                                         parsedAngles,
                                         ENCODER_TOTAL_NUM,
                                         jointMask);
        }
        return;
    }

    if (cmd == CMD_MOTOR_POS_ABS)
    {
        uint8_t status = PROTO_ACK_STATUS_QUEUED;
        int32_t targets[SERVO_TOTAL_NUM] = {};
        const uint32_t token = frameLen == 1 + kMotorPosAbsPayloadBytes
            ? readU32BigEndian(frame + 1) : 0;
        const uint32_t wireMask = frameLen == 1 + kMotorPosAbsPayloadBytes
            ? readU32BigEndian(frame + 5) : 0;
        const uint32_t activeMask =
            sharedAtomicLoad(&sharedData->active_motor_mask) & validMotorChannelMask();
        const uint32_t requestedMask = wireMask != 0 ? wireMask : activeMask;
        if (frameLen != 1 + kMotorPosAbsPayloadBytes ||
            requestedMask == 0 || (requestedMask & ~activeMask) != 0 ||
            !parseInt32ArrayBigEndian(frame + 9, SERVO_TOTAL_NUM * 4U,
                                       targets, SERVO_TOTAL_NUM)) {
            status = PROTO_ACK_STATUS_INVALID_DATA;
        } else if (!upperApplyMotorTargets(
            sharedData, targets, SERVO_TOTAL_NUM, token, requestedMask,
            readU16BigEndian(frame + 117), frame[119])) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        }
        sendProtoAckPacket(CMD_MOTOR_POS_ABS, sharedData->system_state, status);
        return;
    }

    if (cmd == CMD_MOTOR_DIRECT_RELEASE)
    {
        const uint8_t status =
            frameLen != 1 + kMotorDirectReleasePayloadBytes
                ? PROTO_ACK_STATUS_INVALID_DATA
                : upperReleaseMotorDirect(sharedData, readU32BigEndian(frame + 1))
                    ? PROTO_ACK_STATUS_QUEUED : PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        sendProtoAckPacket(CMD_MOTOR_DIRECT_RELEASE, sharedData->system_state, status);
        return;
    }

    if (cmd == CMD_MOTOR_POS_ABS_ONE)
    {
        if (frameLen >= 1 + kMotorPosAbsOnePayloadBytes) {
            const uint8_t motorIndex = frame[1];
            upperApplySingleMotorTarget(sharedData,
                                        motorIndex,
                                        readI32BigEndian(frame + 2));
        }
        return;
    }


    if (cmd == CMD_ACTIVE_SCOPE)
    {
        uint8_t status = PROTO_ACK_STATUS_OK;
        if (frameLen < 1 + kActiveScopePayloadBytes) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else {
            const uint32_t activeMotorMask = readU32BigEndian(frame + 1);
            const uint32_t activeJointMask = readU32BigEndian(frame + 5);
            const uint8_t requireCanFeedback = frame[9] ? 1 : 0;
            if (activeMotorMask == 0) {
                status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
            } else {
                if (!upperApplyActiveScope(sharedData,
                                           activeMotorMask,
                                           activeJointMask,
                                           requireCanFeedback)) {
                    status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
                }
            }
        }
        if (status == PROTO_ACK_STATUS_OK) {
            postSystemEvent(sharedData, SYSTEM_EVENT_REEVALUATE_READY);
        }
        sendProtoAckPacket(CMD_ACTIVE_SCOPE, sharedData->system_state, status);
        return;
    }

    if (cmd == CMD_RESTORE_MULTITURN)
    {
        uint8_t status = PROTO_ACK_STATUS_OK;
        if (frameLen < 1 + kRestoreMultiTurnPayloadBytes) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else if (servoRebaseZeroPending(sharedData)) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else if ((sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap) &
                    validMotorChannelMask()) == 0) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
            FW_LOG_INFO_PRINTLN(
                "[UPPER] RESTORE_MULTITURN rejected: all motor turns are already trusted");
        } else {
            const uint32_t motorMask = readU32BigEndian(frame + 1);
            const uint32_t unknownMask =
                sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap) &
                validMotorChannelMask();
            const uint32_t onlineMask =
                sharedAtomicLoad(&sharedData->servo_online_bitmap) &
                validMotorChannelMask();
            const uint32_t configuredMask =
                sharedAtomicLoad(&sharedData->servo_multiturn_configured_bitmap) &
                validMotorChannelMask();
            const uint32_t recoveryPendingMask =
                sharedAtomicLoad(&sharedData->servo_recovery_pending_bitmap) &
                validMotorChannelMask();
            const uint32_t recoveryFailedMask =
                sharedAtomicLoad(&sharedData->servo_recovery_failed_bitmap) &
                validMotorChannelMask();
            // 自动恢复仍在重试时不能插入人工 turn0，避免两条恢复路径竞争。
            // 已经明确失败的舵机必须允许上位机 JSON 恢复接管，否则
            // recoveryFailed 会把唯一的人工恢复入口永久锁死。
            const uint32_t recoveryBusyMask =
                recoveryPendingMask & ~recoveryFailedMask;
            int32_t motorAbs[SERVO_TOTAL_NUM] = {0};
            // JSON 局部恢复只处理“在线、多圈模式已配置、当前确实 Unknown”
            // 的舵机；自动恢复失败允许人工接管，但正常可信舵机仍不能重定义 turn0。
            if (motorMask == 0 ||
                (motorMask & ~unknownMask) != 0 ||
                (motorMask & ~onlineMask) != 0 ||
                (motorMask & ~configuredMask) != 0 ||
                (motorMask & recoveryBusyMask) != 0) {
                status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
                FW_LOG_WARN_PRINTF(
                    "[UPPER] RESTORE_MULTITURN scoped reject request=0x%08lX unknown=0x%08lX online=0x%08lX configured=0x%08lX recoveryBusy=0x%08lX recoveryFailed=0x%08lX\r\n",
                    (unsigned long)motorMask,
                    (unsigned long)unknownMask,
                    (unsigned long)onlineMask,
                    (unsigned long)configuredMask,
                    (unsigned long)recoveryBusyMask,
                    (unsigned long)recoveryFailedMask);
            } else if (!parseInt32ArrayBigEndian(frame + 5,
                                                 frameLen - 5,
                                                 motorAbs,
                                                 SERVO_TOTAL_NUM)) {
                status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
            } else {
                SemaphoreHandle_t lock = sharedData->commandStateMutex;
                if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
                    status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
                } else {
                    memcpy(sharedData->restore_multi_turn_abs,
                           motorAbs,
                           sizeof(sharedData->restore_multi_turn_abs));
                    sharedData->restore_multi_turn_mask =
                        motorMask & validMotorChannelMask();
                    sharedAtomicAddFetch(&sharedData->restore_multi_turn_request_token, (uint32_t)1);
                    if (lock) {
                        xSemaphoreGive(lock);
                    }
                }
            }
        }
        FW_LOG_INFO_PRINTF("[UPPER] RESTORE_MULTITURN request status=%u mask=0x%08lX token=%lu\r\n",
                      (unsigned)status,
                      (unsigned long)sharedData->restore_multi_turn_mask,
                      (unsigned long)sharedAtomicLoad(&sharedData->restore_multi_turn_request_token));
        sendProtoAckPacket(CMD_RESTORE_MULTITURN, sharedData->system_state, status);
        return;
    }

    if (cmd == CMD_CALIB_LOAD_LIMITS)
    {
        uint8_t status = PROTO_ACK_STATUS_OK;
        if (frameLen != 1 + kCalibLoadLimitsPayloadBytes) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else if (sharedData->system_state == SYSTEM_STATE_RUN_DIRECT ||
                   sharedData->system_state == SYSTEM_STATE_RUN_FEEDBACK ||
                   sharedData->system_state == SYSTEM_STATE_ROPE_INIT_RUNNING) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else {
            const uint32_t updateMask =
                readU32BigEndian(frame + 1) & validMotorChannelMask();
            const uint32_t validMask =
                readU32BigEndian(frame + 5) & validMotorChannelMask();
            int32_t loadedMin[SERVO_TOTAL_NUM] = {0};
            int32_t loadedMax[SERVO_TOTAL_NUM] = {0};
            const uint8_t* p = frame + 9;
            for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
                loadedMin[ch] = readI32BigEndian(p);
                p += 4;
                loadedMax[ch] = readI32BigEndian(p);
                p += 4;
            }
            if (updateMask == 0) {
                status = PROTO_ACK_STATUS_INVALID_DATA;
            } else {
                // 只覆盖快照声明的电机，保留其余关节已经加载或刚标定的 ABS 限位。
                SharedSafetyGuard guard;
                const uint32_t previousValid =
                    sharedAtomicLoad(&sharedData->motor_abs_limit_valid_bitmap);
                for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
                    if ((updateMask & (1UL << ch)) == 0) {
                        continue;
                    }
                    sharedData->motor_abs_limit_min[ch] = loadedMin[ch];
                    sharedData->motor_abs_limit_max[ch] = loadedMax[ch];
                }
                const uint32_t appliedValidMask =
                    (previousValid & ~updateMask) | (validMask & updateMask);
                sharedAtomicStore(&sharedData->motor_abs_limit_valid_bitmap,
                                  appliedValidMask);
                FW_LOG_INFO_PRINTF(
                    "[UPPER] CALIB_LOAD_LIMITS update=0x%08lX valid=0x%08lX\r\n",
                    (unsigned long)updateMask,
                    (unsigned long)appliedValidMask);
            }
        }
        sendProtoAckPacket(CMD_CALIB_LOAD_LIMITS, sharedData->system_state, status);
        return;
    }

    if (cmd == CMD_CALIB_LOAD_JOINT)
    {
        uint8_t status = PROTO_ACK_STATUS_OK;
        uint8_t loadedJointIndex = 0xFF;
        if (frameLen != 1 + kCalibLoadJointPayloadBytes) {
            status = PROTO_ACK_STATUS_INVALID_DATA;
        } else if (sharedData->system_state == SYSTEM_STATE_RUN_DIRECT ||
                   sharedData->system_state == SYSTEM_STATE_RUN_FEEDBACK ||
                   sharedData->system_state == SYSTEM_STATE_ROPE_INIT_RUNNING) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else {
            const uint8_t* p = frame + 1;
            const uint8_t jointIndex = *p++;
            loadedJointIndex = jointIndex;
            if (jointIndex >= ENCODER_TOTAL_NUM) {
                status = PROTO_ACK_STATUS_INVALID_DATA;
            } else {
                JointCalibrationResult result;
                memset(&result, 0, sizeof(result));
                result.success = (*p++ != 0);
                result.angleMax = readFloatBigEndian(p); p += 4;
                result.startEncoder = readI32BigEndian(p); p += 4;
                result.tightEncoder = readI32BigEndian(p); p += 4;
                result.zeroEncoder = readI32BigEndian(p); p += 4;
                result.encoderSpan = readI32BigEndian(p); p += 4;
                result.startServoAbs = readI32BigEndian(p); p += 4;
                result.tightServoAbs = readI32BigEndian(p); p += 4;
                result.zeroServoAbs = readI32BigEndian(p); p += 4;
                result.startDeg = readFloatBigEndian(p); p += 4;
                result.tightDeg = readFloatBigEndian(p); p += 4;
                result.totalDeg = readFloatBigEndian(p); p += 4;
                result.scaled = (*p++ != 0);
                const int8_t direction = ((int8_t)(*p++)) < 0 ? -1 : 1;
                result.tendonLutValid = (*p++ != 0) ? 1 : 0;
                result.tendonLutCount = *p++;
                result.tendonLutMotor[0] = *p++;
                result.tendonLutMotor[1] = *p++;
                result.tendonLutAngleStartDeg = readFloatBigEndian(p); p += 4;
                result.tendonLutAngleStepDeg = readFloatBigEndian(p); p += 4;
                for (uint8_t actuator = 0;
                     actuator < TENDON_LUT_ACTUATOR_COUNT; actuator++) {
                    const int32_t anchor = readI32BigEndian(p);
                    p += 4;
                    for (uint8_t sample = 0;
                         sample < TENDON_LUT_MAX_SAMPLES; sample++) {
                        const int16_t encoded = readI16BigEndian(p);
                        p += 2;
                        result.tendonLutMotorAbs[actuator][sample] =
                            anchor + (int32_t)encoded;
                    }
                }
                const uint8_t incomingMappingVersion = *p++;
                result.angleMin = readFloatBigEndian(p); p += 4;
                const uint32_t incomingUrdfCrc = readU32BigEndian(p); p += 4;
                const uint16_t autoMinRaw = readU16BigEndian(p); p += 2;
                const uint16_t autoMaxRaw = readU16BigEndian(p); p += 2;
                const uint32_t incomingManualCrc = readU32BigEndian(p); p += 4;
                const float incomingCurrentMin = readFloatBigEndian(p); p += 4;
                const float incomingCurrentMax = readFloatBigEndian(p); p += 4;
                (void)incomingCurrentMin;
                (void)incomingCurrentMax;

                const uint32_t localManualEntryCrc =
                    manualEncoderReferenceEntryCrc(
                        jointIndex, g_manualEncoderReference[jointIndex]);
                const uint32_t localValidMask =
                    sharedAtomicLoad(&sharedData->manual_reference_valid_bitmap);
                // V4 Auto 端点必须是物理 14-bit raw；方向化标定坐标不能写入这些字段，
                // 否则 direction=-1 关节会产生错误的补码跨度。
                const uint16_t expectedAutoMinRaw =
                    (uint16_t)calibrationOrientEncoderRaw(
                        (uint16_t)calibrationWrapEncoderCount(
                            result.startEncoder),
                        direction);
                const uint16_t expectedAutoMaxRaw =
                    (uint16_t)calibrationOrientEncoderRaw(
                        (uint16_t)calibrationWrapEncoderCount(
                            result.tightEncoder),
                        direction);
                if (incomingMappingVersion !=
                        kManualScaledUrdfFeedbackMappingVersion ||
                    incomingUrdfCrc != kManualScaledUrdfConfigCrc) {
                    status = PROTO_ACK_STATUS_CALIBRATION_REQUIRED;
                } else if (incomingManualCrc != localManualEntryCrc) {
                    status = PROTO_ACK_STATUS_MANUAL_CRC_MISMATCH;
                } else if ((localValidMask & (1UL << jointIndex)) == 0) {
                    status = PROTO_ACK_STATUS_MANUAL_REFERENCE_REQUIRED;
                } else if (autoMinRaw != expectedAutoMinRaw ||
                           autoMaxRaw != expectedAutoMaxRaw) {
                    FW_LOG_WARN_PRINTF(
                        "[UPPER] CALIB_LOAD reject J%02u raw endpoint "
                        "contract: incoming=%u->%u expected=%u->%u dir=%d; "
                        "recalibration required\r\n",
                        (unsigned)jointIndex,
                        (unsigned)autoMinRaw,
                        (unsigned)autoMaxRaw,
                        (unsigned)expectedAutoMinRaw,
                        (unsigned)expectedAutoMaxRaw,
                        (int)direction);
                    status = PROTO_ACK_STATUS_INVALID_DATA;
                } else if (!applyManualScaledUrdfMappingToCalibration(
                               jointIndex,
                               g_manualEncoderReference[jointIndex],
                               autoMinRaw,
                               autoMaxRaw,
                               direction,
                               localManualEntryCrc,
                               &result)) {
                    status = PROTO_ACK_STATUS_INVALID_DATA;
                } else if (result.success &&
                           (!isfinite(result.startDeg) ||
                            !isfinite(result.tightDeg) ||
                            !isfinite(result.totalDeg) ||
                            result.encoderSpan <= 0 || result.totalDeg <= 0.0f ||
                            !calibrationTendonLutMatchesJoint(jointIndex,
                                                             result))) {
                    status = PROTO_ACK_STATUS_INVALID_DATA;
                } else if (result.success &&
                           !refreshLoadedSingleTendonAbsLimit(
                               sharedData, jointIndex, result)) {
                    status = PROTO_ACK_STATUS_INVALID_DATA;
                    FW_LOG_WARN_PRINTF(
                        "[UPPER] CALIB_LOAD reject J%02u: single tendon "
                        "measured tight endpoint is invalid\r\n",
                        (unsigned)jointIndex);
                } else {
                    publishJointCalibrationResult(jointIndex, result);
                    setRuntimeEncoderDirection(jointIndex, direction);
                    if (result.success) {
                        updateCalibrationAssemblyWarning(
                            sharedData, jointIndex, result);
                    }
                    FW_LOG_INFO_PRINTF(
                        "[UPPER] CALIB_LOAD J%02u auto=%u->%u manualCrc=0x%08lX current=%.3f..%.3f\r\n",
                        (unsigned)jointIndex,
                        (unsigned)result.autoMinRaw,
                        (unsigned)result.autoMaxRaw,
                        (unsigned long)result.manualReferenceCrc,
                        result.currentMinDeg,
                        result.currentMaxDeg);
                }
            }
        }
        sendProtoAckPacket(CMD_CALIB_LOAD_JOINT,
                           sharedData->system_state,
                           status);
        if (status == PROTO_ACK_STATUS_OK) {
            // Calib Load 改写了 Auto/LUT 与边界：ACK 后立即回传该关节及关联范围，
            // 不再等待旧的一秒周期广播，上位机可据此完成逐关节闭环校验。
            if (loadedJointIndex < ENCODER_TOTAL_NUM) {
                if (!sendJointCalibSnapshotPacket(loadedJointIndex)) {
                    requestHostStateSync(STATE_SYNC_JOINT_CALIBRATION);
                }
            }
            requestHostStateSync(
                STATE_SYNC_MOTOR_ABS_LIMIT |
                STATE_SYNC_SERVO_LOGICAL_RANGE);
            postSystemEvent(sharedData, SYSTEM_EVENT_REEVALUATE_READY);
        }
        return;
    }
    if (cmd == CMD_MANUAL_ENCODER_SINGLE)
    {
        uint8_t status = PROTO_ACK_STATUS_OK;
        const uint8_t joint = frame[1];
        const uint16_t minRaw = readU16BigEndian(frame + 2);
        const uint16_t maxRaw = readU16BigEndian(frame + 4);
        if (manualUpdateMotionActive(sharedData)) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else {
            if (joint >= ENCODER_TOTAL_NUM ||
                minRaw >= kEncoderModulo || maxRaw >= kEncoderModulo) {
                status = PROTO_ACK_STATUS_INVALID_DATA;
            } else {
                ManualEncoderReference references[ENCODER_TOTAL_NUM];
                {
                    SharedSafetyGuard guard;
                    memcpy(references, g_manualEncoderReference,
                           sizeof(references));
                }
                references[joint].valid = 1;
                references[joint].minRaw = minRaw;
                references[joint].maxRaw = maxRaw;
                if (!manualEncoderReferencesCoverMask(
                        references, (1UL << joint), NULL)) {
                    status = PROTO_ACK_STATUS_INVALID_DATA;
                } else {
                    status = commitManualReferences(sharedData, references);
                }
            }
        }
        sendProtoAckPacket(CMD_MANUAL_ENCODER_SINGLE,
                           sharedData->system_state,
                           status);
        if (status == PROTO_ACK_STATUS_OK) {
            sendManualEncoderReferencePacket(sharedData);
        }
        FW_LOG_INFO_PRINTF(
            "[MANUAL RAM] single J%02u min=%u max=%u status=%u "
            "committed=%u result=%u valid=0x%08lX crc=0x%08lX revision=%lu\r\n",
            (unsigned)joint,
            (unsigned)minRaw,
            (unsigned)maxRaw,
            (unsigned)status,
            (unsigned)sharedData->manual_reference_committed,
            (unsigned)sharedData->manual_reference_commit_status,
            (unsigned long)sharedAtomicLoad(
                &sharedData->manual_reference_valid_bitmap),
            (unsigned long)sharedAtomicLoad(
                &sharedData->manual_reference_crc),
            (unsigned long)sharedAtomicLoad(
                &sharedData->manual_reference_revision));
        return;
    }
    if (cmd == CMD_MANUAL_ENCODER_REFERENCE)
    {
        uint8_t status = PROTO_ACK_STATUS_OK;
        const uint8_t action = frame[1];
        const bool motionActive =
            sharedData->control_enabled != 0 ||
            sharedData->servo_target_owner != SERVO_TARGET_OWNER_NONE ||
            sharedData->system_state == SYSTEM_STATE_RUN_DIRECT ||
            sharedData->system_state == SYSTEM_STATE_RUN_FEEDBACK ||
            sharedData->system_state == SYSTEM_STATE_ROPE_INIT_RUNNING ||
            sharedData->system_state == SYSTEM_STATE_PAUSED;

        if (motionActive) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else if (action == 0) {
            // P4 只清会话 RAM；桌面端收到 ACK 后同步删除自身 JSON 条目。
            sharedData->manual_reference_committed = 0;
            manualEncoderReferencesClearRam();
            sharedAtomicStore(&sharedData->manual_reference_valid_bitmap,
                              (uint32_t)0);
            sharedAtomicStore(&sharedData->manual_reference_crc,
                              (uint32_t)0);
            sharedAtomicStore(&sharedData->manual_reference_revision,
                              (uint32_t)0);
            sharedData->manual_reference_commit_status =
                MANUAL_SAVE_STATUS_NONE;
            invalidateAutomaticCalibration(
                sharedData, kManualEncoderReferenceFullMask);
            postSystemEvent(sharedData, SYSTEM_EVENT_REEVALUATE_READY);
        } else if (action == 1) {
            ManualEncoderReference references[ENCODER_TOTAL_NUM];
            const uint8_t* p = frame + 2;
            for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; joint++) {
                references[joint].valid = 1;
                references[joint].minRaw = readU16BigEndian(p);
                p += 2;
                references[joint].maxRaw = readU16BigEndian(p);
                p += 2;
            }

            uint32_t validMask = 0;
            if (!manualEncoderReferencesValidate(references, &validMask)) {
                status = PROTO_ACK_STATUS_INVALID_DATA;
            } else {
                status = commitManualReferences(sharedData, references);
            }
        } else {
            status = PROTO_ACK_STATUS_INVALID_DATA;
        }
        FW_LOG_INFO_PRINTF(
            "[MANUAL RAM] action=%u status=%u valid=0x%08lX "
            "crc=0x%08lX revision=%lu committed=%u result=%u\r\n",
            (unsigned)action,
            (unsigned)status,
            (unsigned long)sharedAtomicLoad(
                &sharedData->manual_reference_valid_bitmap),
            (unsigned long)sharedAtomicLoad(
                &sharedData->manual_reference_crc),
            (unsigned long)sharedAtomicLoad(
                &sharedData->manual_reference_revision),
            (unsigned)sharedData->manual_reference_committed,
            (unsigned)sharedData->manual_reference_commit_status);
        sendProtoAckPacket(CMD_MANUAL_ENCODER_REFERENCE,
                           sharedData->system_state,
                           status);
        return;
    }
    if (cmd == CMD_CLEAR_CALIBRATION)
    {
        uint8_t status = PROTO_ACK_STATUS_OK;
        if (sharedData->system_state == SYSTEM_STATE_RUN_DIRECT ||
            sharedData->system_state == SYSTEM_STATE_RUN_FEEDBACK ||
            sharedData->system_state == SYSTEM_STATE_ROPE_INIT_RUNNING) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else {
            {
                SharedSafetyGuard guard;
                sharedAtomicStore(&sharedData->motor_abs_limit_valid_bitmap, (uint32_t)0);
                memset(sharedData->motor_abs_limit_min, 0, sizeof(sharedData->motor_abs_limit_min));
                memset(sharedData->motor_abs_limit_max, 0, sizeof(sharedData->motor_abs_limit_max));
            }
            clearAllJointCalibrationResults();
            clearCalibrationAssemblyWarnings(
                sharedData, kManualEncoderReferenceFullMask);
            postSystemEvent(sharedData, SYSTEM_EVENT_REEVALUATE_READY);
            FW_LOG_INFO_PRINTLN("[UPPER] CLEAR_CALIBRATION: joint calibration and ABS limits cleared");
        }
        sendProtoAckPacket(CMD_CLEAR_CALIBRATION, sharedData->system_state, status);
        return;
    }

    if (cmd == CMD_CLEAR_CALIBRATION_SCOPE)
    {
        uint8_t status = PROTO_ACK_STATUS_OK;
        const uint32_t motorMask =
            frameLen >= 1 + kClearCalibrationScopePayloadBytes
                ? (readU32BigEndian(frame + 1) & validMotorChannelMask())
                : 0;
        const uint32_t restoredMask =
            sharedAtomicLoad(&sharedData->restore_multi_turn_restored_mask) &
            validMotorChannelMask();
        const uint32_t jointMask = jointMaskForMotorMask(motorMask);
        if (sharedData->system_state == SYSTEM_STATE_RUN_DIRECT ||
            sharedData->system_state == SYSTEM_STATE_RUN_FEEDBACK ||
            sharedData->system_state == SYSTEM_STATE_ROPE_INIT_RUNNING) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else if (motorMask == 0 || jointMask == 0 ||
                   (motorMask & ~restoredMask) != 0) {
            status = PROTO_ACK_STATUS_INVALID_DATA;
            FW_LOG_WARN_PRINTF(
                "[UPPER] CLEAR_CALIBRATION_SCOPE reject motors=0x%08lX restored=0x%08lX joints=0x%08lX\r\n",
                (unsigned long)motorMask,
                (unsigned long)restoredMask,
                (unsigned long)jointMask);
        } else {
            invalidateAutomaticCalibration(sharedData, jointMask);
            sharedAtomicFetchAnd(
                &sharedData->restore_multi_turn_restored_mask, ~motorMask);
            postSystemEvent(sharedData, SYSTEM_EVENT_REEVALUATE_READY);
            FW_LOG_INFO_PRINTF(
                "[UPPER] CLEAR_CALIBRATION_SCOPE motors=0x%08lX joints=0x%08lX remainingAuto=0x%08lX\r\n",
                (unsigned long)motorMask,
                (unsigned long)jointMask,
                (unsigned long)autoCalibrationValidMask(sharedData));
        }
        sendProtoAckPacket(
            CMD_CLEAR_CALIBRATION_SCOPE, sharedData->system_state, status);
        return;
    }

    if (cmd == CMD_SERVO_RESET_ZERO)
    {
        const uint32_t requestedMask =
            readU32BigEndian(frame + 1) & validMotorChannelMask();
        const uint32_t onlineMask =
            sharedAtomicLoad(&sharedData->servo_online_bitmap) &
            validMotorChannelMask();
        uint8_t status = PROTO_ACK_STATUS_QUEUED;
        const bool safeState =
            sharedData->system_state == SYSTEM_STATE_UNCONTROLLABLE ||
            sharedData->system_state == SYSTEM_STATE_IDLE ||
            sharedData->system_state == SYSTEM_STATE_READY ||
            sharedData->system_state == SYSTEM_STATE_RECOVERY_REQUIRED;
        if (requestedMask == 0 ||
            (requestedMask & ~onlineMask) != 0) {
            status = PROTO_ACK_STATUS_INVALID_DATA;
        } else if (!safeState ||
                   sharedData->control_enabled != 0 ||
                   sharedData->servo_target_owner !=
                       SERVO_TARGET_OWNER_NONE) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else if (sharedAtomicLoad(
                       &sharedData->servo_reset_zero_status) ==
                       SERVO_RESET_ZERO_STATUS_RUNNING ||
                   servoRebaseZeroPending(sharedData)) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else {
            sharedAtomicStore(
                &sharedData->servo_reset_zero_request_mask,
                requestedMask);
            sharedAtomicStore(
                &sharedData->servo_reset_zero_success_mask,
                (uint32_t)0);
            sharedAtomicStore(
                &sharedData->servo_reset_zero_failed_mask,
                (uint32_t)0);
            sharedAtomicStore(
                &sharedData->servo_reset_zero_invalidated_joint_mask,
                (uint32_t)0);
            sharedAtomicStore(
                &sharedData->servo_reset_zero_status,
                (uint8_t)SERVO_RESET_ZERO_STATUS_RUNNING);
            sharedAtomicAddFetch(
                &sharedData->servo_reset_zero_request_token,
                (uint32_t)1);
        }
        sendProtoAckPacket(
            CMD_SERVO_RESET_ZERO, sharedData->system_state, status);
        return;
    }

    if (cmd == CMD_SERVO_REBASE_ZERO)
    {
        uint8_t status = PROTO_ACK_STATUS_QUEUED;
        const uint32_t requestedMask =
            frameLen == 1 + kServoRebaseZeroPayloadBytes
                ? readU32BigEndian(frame + 1)
                : 0;
        // 必须先取命令 mutex，不能在短跨核临界区内等待 RTOS mutex。
        // 它与 pending→applied 提升串行，接受归零即撤销旧坐标待执行目标。
        SemaphoreHandle_t commandLock = sharedData->commandStateMutex;
        if (commandLock &&
            xSemaphoreTake(commandLock, pdMS_TO_TICKS(10)) != pdTRUE) {
            sendProtoAckPacket(CMD_SERVO_REBASE_ZERO,
                               sharedData->system_state,
                               PROTO_ACK_STATUS_UNSUPPORTED_MODE);
            return;
        }
        {
            // 与状态机授予运动 owner 的最后一步使用同一短临界区。
            // 不在此处访问 UART；已有结果和 offset 保留到新事务完整提交。
            SharedSafetyGuard guard;
            const uint8_t state = sharedData->system_state;
            const bool safeState =
                state == SYSTEM_STATE_UNCONTROLLABLE ||
                state == SYSTEM_STATE_IDLE ||
                state == SYSTEM_STATE_READY ||
                state == SYSTEM_STATE_RECOVERY_REQUIRED;
            const uint32_t onlineMask =
                sharedAtomicLoad(&sharedData->servo_online_bitmap);
            const uint32_t controllableMask =
                sharedAtomicLoad(&sharedData->servo_route_controllable_bitmap);
            const uint32_t recoveryBusyMask =
                sharedAtomicLoad(&sharedData->servo_recovery_pending_bitmap) &
                ~sharedAtomicLoad(&sharedData->servo_recovery_failed_bitmap);
            if (requestedMask == 0 ||
                (requestedMask & ~validMotorChannelMask()) != 0 ||
                (requestedMask & ~onlineMask) != 0 ||
                (requestedMask & ~controllableMask) != 0) {
                status = PROTO_ACK_STATUS_INVALID_DATA;
            } else if (!safeState || sharedData->control_enabled != 0 ||
                       sharedData->servo_target_owner != SERVO_TARGET_OWNER_NONE ||
                       servoRebaseZeroPending(sharedData) ||
                       sharedAtomicLoad(&sharedData->restore_multi_turn_request_token) !=
                           sharedAtomicLoad(&sharedData->restore_multi_turn_result_token) ||
                       sharedAtomicLoad(&sharedData->servo_reset_zero_status) ==
                           SERVO_RESET_ZERO_STATUS_RUNNING ||
                       (requestedMask & recoveryBusyMask) != 0) {
                status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
            } else {
                sharedData->pending_direct_valid = 0;
                sharedData->pending_direct_source = MOTOR_DIRECT_SOURCE_NONE;
                sharedData->pending_direct_motor_mask = 0;
                sharedData->pending_direct_joint_mask = 0;
                sharedData->pending_direct_require_can = 0;
                sharedData->direct_reject_reason = DIRECT_REJECT_STALE;
                sharedData->motor_direct_command_source = MOTOR_DIRECT_SOURCE_NONE;
                sharedData->pending_direct_token = 0;
                sharedData->direct_release_generation = 0;
                sharedData->applied_direct_token = 0;
                sharedData->applied_direct_motor_mask = 0;
                sharedAtomicStore(&sharedData->servo_rebase_zero_request_mask,
                                  requestedMask);
                uint32_t token = sharedAtomicLoad(
                    &sharedData->servo_rebase_zero_request_token) + 1U;
                if (token == 0) token = 1;
                sharedAtomicStore(&sharedData->servo_rebase_zero_request_token,
                                  token);
            }
        }
        if (commandLock) xSemaphoreGive(commandLock);
        sendProtoAckPacket(CMD_SERVO_REBASE_ZERO,
                           sharedData->system_state, status);
        return;
    }

}

} // namespace p4::host
