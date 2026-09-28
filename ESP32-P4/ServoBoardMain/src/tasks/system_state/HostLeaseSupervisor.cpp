/**
 * @file HostLeaseSupervisor.cpp
 * @brief 实现上位机控制租约超时后的当前 ABS Hold 和安全收尾。
 * @details 租约失效后先停控制生产者，再发布一次 Hold；只有执行层确认 Hold 后才释放 owner。
 */
#include "HostLeaseSupervisor.h"

#include <string.h>

#include "../../safety/policies/SystemReadinessPolicy.h"
#include "../../modules/actuator/ActuatorCommandGateway.h"
#include "../../config/SafetyConfig.h"
#include "../../data/CalibrationTypes.h"
#include "../../data/SystemTypes.h"
#include "../../platform/rtos/AtomicValue.h"
#include "../../platform/rtos/SharedSafetyState.h"
#include "../../platform/time/Clock.h"
#include "../../safety/policies/HostLeaseSafetyPolicy.h"
#include "../../safety/SafetyEventReporter.h"
#include "../../safety/SafetyRuleRegistry.h"
#include "../../platform/log/FirmwareLog.h"
#include "../../runtime/SystemRuntimeContext.h"
#include "../../runtime/ports/SystemEventPort.h"

using namespace mmhand::config::safety;
using p4::platform::time::monotonicMillis;

namespace p4::state {

void resetHostControlLeaseRuntime(SystemRuntimeContext* sharedData)
{
    if (!sharedData) {
        return;
    }
    sharedAtomicStore(&sharedData->host_control_hold_status,
                      (uint8_t)HOST_CONTROL_HOLD_IDLE);
    sharedAtomicStore(&sharedData->host_control_hold_result_token,
                      (uint32_t)0);
    sharedAtomicStore(&sharedData->host_control_hold_started_ms,
                      (uint32_t)0);
}

/**
 * 新的 UI/ROS2/Replay 命令必须来自一条仍然新鲜的上位机链路。
 * 失联 Hold 一旦开始，不允许心跳恢复后自动续跑旧目标；必须等 Hold 完成并重新发命令。
 */
bool prepareHostControlCommandAdmission(SystemRuntimeContext* sharedData)
{
    if (!sharedData) {
        return false;
    }
    const uint32_t nowMs = monotonicMillis();
    const uint32_t heartbeatMs =
        sharedAtomicLoad(&sharedData->host_rx_heartbeat_ms);
    if (!p4::system::isHostControlLeaseFresh(nowMs, heartbeatMs) ||
        sharedAtomicLoad(&sharedData->host_control_hold_status) ==
            HOST_CONTROL_HOLD_PENDING) {
        p4::safety::reportSafetyRuleEvent(
            sharedData,
            SafetyRuleId::SystemHostLeaseExpired,
            0,
            (int32_t)(nowMs - heartbeatMs));
        return false;
    }
    resetHostControlLeaseRuntime(sharedData);
    return true;
}

/**
 * 使用最新舵机反馈构造 active scope 的当前 ABS Hold 批次。
 * 本函数先停 MotionControlTask 再清邮箱，保证 Hold 不会被普通控制批次覆盖。
 */
static bool beginHostControlHold(SystemRuntimeContext* sharedData,
                                 uint32_t nowMs)
{
    if (!sharedData) {
        return false;
    }

    const uint8_t interruptedState =
        sharedAtomicLoad(&sharedData->system_state);
    const uint8_t activeOwner =
        sharedAtomicLoad(&sharedData->servo_target_owner);
    if (!p4::system::requiresHostControlLease(interruptedState,
                                               activeOwner)) {
        return false;
    }

    sharedAtomicStore(&sharedData->control_enabled, (uint8_t)0);
    if (interruptedState == SYSTEM_STATE_CALIBRATION_RUNNING) {
        // 先关闭标定运动标志，使舵机通信任务停止标定专用轮询；真正的 owner/state
        // 保留到 Hold 被执行层确认，避免 Hold 在模式切换窗口中被拒绝。
        sharedAtomicStore(&sharedData->calibration_motion_started,
                          (uint8_t)0);
    }
    p4::actuator::clearPending(sharedData);

    const uint32_t token =
        sharedAtomicAddFetch(&sharedData->host_control_hold_request_token,
                             (uint32_t)1);
    sharedAtomicStore(&sharedData->host_control_hold_result_token,
                      (uint32_t)0);
    sharedAtomicStore(&sharedData->host_control_hold_started_ms, nowMs);
    sharedAtomicStore(&sharedData->host_control_hold_status,
                      (uint8_t)HOST_CONTROL_HOLD_PENDING);

    ServoAngleData_t servoData;
    memset(&servoData, 0, sizeof(servoData));
    if (!sharedData->servoFeedbackQueue.peek(&servoData) ||
        servoData.timestamp == 0 ||
        (uint32_t)(nowMs - servoData.timestamp) >
            kStateServoFeedbackTimeoutMs) {
        sharedAtomicStore(&sharedData->host_control_hold_status,
                          (uint8_t)HOST_CONTROL_HOLD_FAILED);
        sharedAtomicStore(&sharedData->host_control_hold_result_token, token);
        FW_LOG_WARN_PRINTF(
            "[HOST LEASE HOLD] feedback stale token=%lu ageMs=%lu\r\n",
            (unsigned long)token,
            (unsigned long)(servoData.timestamp == 0
                ? UINT32_MAX
                : (uint32_t)(nowMs - servoData.timestamp)));
        return false;
    }

    // 自动标定可能临时使用 active scope 之外的双腱辅助电机，Hold 必须覆盖
    // 当前运动态实际可能驱动的完整集合，而不能只看 active_motor_mask。
    const uint32_t activeMotorMask =
        p4::state::requiredMotorMask(sharedData);
    const uint32_t onlineMask =
        sharedAtomicLoad(&sharedData->servo_online_bitmap);
    const uint32_t controllableRouteMask =
        sharedAtomicLoad(&sharedData->servo_route_controllable_bitmap);
    if (activeMotorMask == 0 ||
        (onlineMask & activeMotorMask) != activeMotorMask ||
        (controllableRouteMask & activeMotorMask) != activeMotorMask) {
        sharedAtomicStore(&sharedData->host_control_hold_status,
                          (uint8_t)HOST_CONTROL_HOLD_FAILED);
        sharedAtomicStore(&sharedData->host_control_hold_result_token, token);
        FW_LOG_WARN_PRINTF(
            "[HOST LEASE HOLD] hardware unavailable token=%lu active=0x%08lX online=0x%08lX route=0x%08lX\r\n",
            (unsigned long)token,
            (unsigned long)activeMotorMask,
            (unsigned long)onlineMask,
            (unsigned long)controllableRouteMask);
        return false;
    }

    ServoTargetBatch_t batch;
    memset(&batch, 0, sizeof(batch));
    batch.source = activeOwner;
    batch.purpose = SERVO_TARGET_PURPOSE_HOST_LEASE_HOLD;
    batch.timestamp = nowMs;
    batch.transactionToken = token;
    for (uint8_t motor = 0; motor < SERVO_TOTAL_NUM; ++motor) {
        const uint32_t bit = 1UL << motor;
        if ((activeMotorMask & bit) == 0) {
            continue;
        }
        ServoTargetCommand_t& command = batch.commands[batch.count++];
        command.motorChannel = motor;
        command.position = servoData.servoAngles[motor];
        // 复用当前项目的全局默认速度/加速度；Hold 目标等于当前 ABS，参数只作为驱动帧必填项。
        command.speed = SERVO_TARGET_SPEED_DEFAULT;
        command.acc = SERVO_TARGET_ACC_DEFAULT;
    }

    if (!p4::actuator::publishLatest(sharedData, batch)) {
        sharedAtomicStore(&sharedData->host_control_hold_status,
                          (uint8_t)HOST_CONTROL_HOLD_FAILED);
        sharedAtomicStore(&sharedData->host_control_hold_result_token, token);
        return false;
    }
    FW_LOG_WARN_PRINTF(
        "[HOST LEASE HOLD] started token=%lu state=%u owner=%u motors=0x%08lX heartbeatAgeMs=%lu\r\n",
        (unsigned long)token,
        (unsigned)interruptedState,
        (unsigned)activeOwner,
        (unsigned long)activeMotorMask,
        (unsigned long)(nowMs -
            sharedAtomicLoad(&sharedData->host_rx_heartbeat_ms)));
    return true;
}

/** 标记内部运动任务因上位机失联而取消；任务会在 owner/state 被撤销后自行退出。 */
static void cancelInternalMotionForHostLeaseLoss(
    SystemRuntimeContext* sharedData,
    uint8_t interruptedState,
    const HostLeaseSupervisorPort& port)
{
    if (!sharedData) {
        return;
    }
    if (interruptedState == SYSTEM_STATE_CALIBRATION_RUNNING) {
        sharedAtomicStore(&sharedData->calibration_motion_started,
                          (uint8_t)0);
        CalibrationDiagData_t diag;
        memset(&diag, 0, sizeof(diag));
        diag.valid = 1;
        diag.reason = CALIB_DIAG_REASON_CONTROL_LOST;
        diag.phase = CALIB_DIAG_PHASE_SEQUENCE;
        diag.joint = 0xFF;
        diag.motor = 0xFF;
        diag.valueA = (int32_t)(monotonicMillis() -
            sharedAtomicLoad(&sharedData->host_rx_heartbeat_ms));
        diag.valueB = (int32_t)kHostControlLeaseTimeoutMs;
        diag.bitmap = p4::state::requiredMotorMask(sharedData);
        diag.timestamp = monotonicMillis();
        {
            SharedSafetyGuard guard;
            diag.sequence = sharedData->calibration_diag.sequence + 1;
            sharedData->calibration_diag = diag;
        }
        port.setCalibrationStatus(CALIB_STATUS_FAILED);
    }
    if (interruptedState == SYSTEM_STATE_ROPE_INIT_RUNNING) {
        sharedData->rope_init_status = ROPE_INIT_STATUS_FAILED;
        sharedData->rope_init_reason = ROPE_INIT_REASON_CANCELLED;
        sharedData->rope_init_timestamp = monotonicMillis();
        if (sharedAtomicLoad(
                &sharedData->calibration_rope_init_pending) != 0) {
            // 标定结果已经提交，仅 READY 收尾被租约中断；保留结果并报告 WARN。
            port.setCalibrationStatus(CALIB_STATUS_WARN);
            sharedAtomicStore(&sharedData->calibration_rope_init_pending,
                              (uint8_t)0);
        }
        sharedAtomicStore(&sharedData->feedback_recovery_pending,
                          (uint8_t)0);
        sharedAtomicStore(&sharedData->feedback_recovery_request_sequence,
                          (uint32_t)0);
    }
}

/**
 * 监视所有运动状态的上位机租约并完成 Hold 确认闭环。
 * CONTROL 与 CALIBRATION owner 均使用当前 ABS Hold；ROS2 不设置独立消息超时，
 * 仍由桌面端公共串口心跳续租。
 */
void updateHostControlLease(SystemRuntimeContext* sharedData,
                            const HostLeaseSupervisorPort& port)
{
    if (!sharedData || !port.resolveIdleOrReady || !port.transitionTo ||
        !port.setCalibrationStatus ||
        sharedAtomicLoad(&sharedData->emergency_stop_latched) != 0) {
        return;
    }
    const uint8_t state = sharedAtomicLoad(&sharedData->system_state);
    const uint8_t owner =
        sharedAtomicLoad(&sharedData->servo_target_owner);
    if (!p4::system::requiresHostControlLease(state, owner)) {
        return;
    }

    const uint32_t nowMs = monotonicMillis();
    const uint8_t holdStatus =
        sharedAtomicLoad(&sharedData->host_control_hold_status);
    const uint32_t requestToken =
        sharedAtomicLoad(&sharedData->host_control_hold_request_token);
    const uint32_t resultToken =
        sharedAtomicLoad(&sharedData->host_control_hold_result_token);

    if (holdStatus == HOST_CONTROL_HOLD_APPLIED &&
        resultToken == requestToken) {
        const uint8_t interruptedState =
            sharedAtomicLoad(&sharedData->system_state);
        // Hold 已经落入总线，先使内部任务进入取消收尾，再释放 owner。
        cancelInternalMotionForHostLeaseLoss(sharedData,
                                             interruptedState,
                                             port);
        p4::actuator::clearPending(sharedData);
        sharedAtomicStore(&sharedData->servo_target_owner,
                          (uint8_t)SERVO_TARGET_OWNER_NONE);
        sharedAtomicStore(&sharedData->control_enabled, (uint8_t)0);
        port.transitionTo(sharedData, port.resolveIdleOrReady(sharedData));
        FW_LOG_WARN_PRINTF(
            "[HOST LEASE HOLD] completed token=%lu interruptedState=%u nextState=%u\r\n",
            (unsigned long)requestToken,
            (unsigned)interruptedState,
            (unsigned)sharedData->system_state);
        return;
    }
    if (holdStatus == HOST_CONTROL_HOLD_PENDING) {
        const uint32_t startedMs =
            sharedAtomicLoad(&sharedData->host_control_hold_started_ms);
        const uint8_t latestStatus =
            sharedAtomicLoad(&sharedData->host_control_hold_status);
        if (latestStatus == HOST_CONTROL_HOLD_FAILED ||
            (uint32_t)(nowMs - startedMs) >
                kHostControlHoldApplyTimeoutMs) {
            sharedAtomicStore(&sharedData->host_control_hold_status,
                              (uint8_t)HOST_CONTROL_HOLD_FAILED);
            FW_LOG_WARN_PRINTF(
                "[HOST LEASE HOLD] failed token=%lu status=%u elapsedMs=%lu -> E-STOP\r\n",
                (unsigned long)requestToken,
                (unsigned)latestStatus,
                (unsigned long)(nowMs - startedMs));
            requestEmergencyStop(sharedData);
        }
        return;
    }
    if (holdStatus == HOST_CONTROL_HOLD_FAILED) {
        requestEmergencyStop(sharedData);
        return;
    }

    const uint32_t heartbeatMs =
        sharedAtomicLoad(&sharedData->host_rx_heartbeat_ms);
    if (!p4::system::isHostControlLeaseFresh(nowMs, heartbeatMs)) {
        p4::safety::reportSafetyRuleEvent(
            sharedData,
            SafetyRuleId::SystemHostLeaseExpired,
            0,
            (int32_t)(nowMs - heartbeatMs));
        if (!beginHostControlHold(sharedData, nowMs)) {
            requestEmergencyStop(sharedData);
        }
    }
}


} // namespace p4::state
