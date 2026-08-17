/**
 * @file SystemEventDispatcher.cpp
 * @brief 实现 P4 系统事件迁移、READY 判定、故障保持和运动 owner 切换。
 * @details 本文件不创建 RTOS 任务；所有状态修改只能由协调循环串行调用。
 */
#include "SystemEventDispatcher.h"
#include "SystemStateCoordinator.h"
#include "../../runtime/ports/SystemEventPort.h"
#include "../../platform/time/Clock.h"
#include "DirectCommandCoordinator.h"
#include "FeedbackReadyDiagnostics.h"
#include "HostLeaseSupervisor.h"
#include "../../safety/policies/SystemReadinessPolicy.h"
#include "../../safety/policies/SystemFaultPolicy.h"
#include "RopeInitController.h"
#include "../auto_calibration/AutoCalibrationTaskRuntime.h"
#include "../../modules/actuator/ActuatorCommandGateway.h"
#include "../../modules/calibration/CalibrationMath.h"
#include "../../modules/calibration/CalibrationRuntimeStore.h"
#include "../../modules/persistence/ManualCalibrationNvsStore.h"
#include "../../safety/policies/HostLeaseSafetyPolicy.h"
#include "../../safety/policies/StateAdmissionPolicy.h"
#include "../../safety/SafetyEventReporter.h"
#include "../../safety/SafetyRuleRegistry.h"

#include <math.h>
#include <string.h>

#include "../../config/SafetyConfig.h"
#include "../../generated/UrdfJointLimits.generated.h"
#include "../../config/topology/HardwareMap.h"
#include "../../platform/rtos/AtomicValue.h"
#include "../../platform/rtos/SharedSafetyState.h"
#include "../../platform/log/FirmwareLog.h"
#include "../../app/AppBootstrap.h"
#include "../../runtime/metrics/TaskRuntimeMetrics.h"

using namespace mmhand::system;

using namespace mmhand::config::safety;
using namespace p4::state;
static bool activeJointsInControlWorkspace(SystemRuntimeContext* sharedData);
static bool activeJointsInControlWorkspace(SystemRuntimeContext* sharedData,
                                           uint32_t* recoverableBelowMask);
uint8_t idleOrReadyState(SystemRuntimeContext* sharedData);
static void logManualReferenceReject(SystemRuntimeContext* sharedData,
                                     const char* eventName)
{
    // 同一有效位图 1 秒内只打印一次，避免状态机 100 Hz 刷屏。
    static uint32_t s_lastLogMs = 0;
    static uint32_t s_lastMask = 0;
    const uint32_t nowMs = p4::platform::time::monotonicMillis();
    const uint32_t validMask = sharedData
        ? sharedAtomicLoad(&sharedData->manual_reference_valid_bitmap) : 0;
    if (validMask == s_lastMask &&
        (uint32_t)(nowMs - s_lastLogMs) <= 1000) {
        return;
    }
    s_lastMask = validMask;
    s_lastLogMs = nowMs;
    const uint32_t requiredMask = sharedData
        ? (sharedAtomicLoad(&sharedData->active_joint_mask) &
           kManualEncoderReferenceFullMask)
        : 0;
    FW_LOG_INFO_PRINTF(
        "[STATE] reject %s: Manual encoder reference incomplete "
        "required=0x%08lX valid=0x%08lX missing=0x%08lX nvs=%u\r\n",
        eventName ? eventName : "MOTION",
        (unsigned long)requiredMask,
        (unsigned long)validMask,
        (unsigned long)(requiredMask & ~validMask),
        sharedData && sharedData->manual_reference_nvs_valid ? 1U : 0U);
}

void setCalibrationStatus(uint8_t status)
{
    g_calibrationStatus = status;
}

bool postSystemEvent(SystemRuntimeContext* sharedData, uint8_t event)
{
    return postSystemEventForToken(sharedData, event, 0);
}

bool requestEmergencyStop(SystemRuntimeContext* sharedData)
{
    if (!sharedData) {
        return false;
    }

    // 先锁存最终发送门禁，再递增 token。即使状态机和舵机任务尚未处理 STOP，
    // ActuatorSafetyPolicy 也会立即拒绝竞态中的旧位置批次。
    sharedAtomicStore(&sharedData->emergency_stop_latched, (uint8_t)1);
    // STOP 独立于普通事件队列锁存，同时立即通知舵机任务。
    // 两个 token 单调递增，即使普通事件队列已满也不会丢失急停。
    sharedAtomicAddFetch(&sharedData->emergency_stop_request_token, (uint32_t)1);
    sharedAtomicAddFetch(&sharedData->servo_emergency_stop_token, (uint32_t)1);
    // STOP 不能等待状态机的下一个固定周期；状态机负责状态收敛，ServoBus 负责立即断力矩。
    sharedData->systemStateWake.notify(p4::runtime::wake::kStateEmergencyStop);
    sharedData->servoBusWake.notify(p4::runtime::wake::kServoEmergencyStop);
    taskRuntimeMarkStopRequested();
    return true;
}

bool postSystemEventForToken(SystemRuntimeContext* sharedData, uint8_t event, uint32_t token)
{
    if (event == SYSTEM_EVENT_STOP) {
        return requestEmergencyStop(sharedData);
    }
    if (!sharedData) {
        return false;
    }

    SystemEvent_t msg;
    msg.event = event;
    msg.timestamp = p4::platform::time::monotonicMillis();
    msg.token = token;
    const bool queued = sharedData->stateEventQueue.send(msg);
    if (queued) {
        sharedData->systemStateWake.notify(p4::runtime::wake::kStateEvent);
    }
    return queued;
}

void requestMultiTurnNvsSave(SystemRuntimeContext* sharedData, uint8_t reason)
{
    if (!sharedData) {
        return;
    }
#if !MULTITURN_NVS_PERSISTENCE_ENABLE
    (void)reason;
    return;
#else
    // 状态机只发布保存请求，真正的 Flash 写入由低优先级持久化任务完成。
    sharedData->multiturn_nvs_save_reason = reason;
    sharedAtomicAddFetch(&sharedData->multiturn_nvs_save_request_token, (uint32_t)1);
#endif
}

static bool shouldSaveMultiTurnOnInterrupt(uint8_t state)
{
    // 只有可能改变舵机多圈位置的运行态在 STOP/RESET 前需要保存。
    return state == SYSTEM_STATE_RUN_DIRECT ||
           state == SYSTEM_STATE_RUN_FEEDBACK ||
           state == SYSTEM_STATE_CALIBRATION_RUNNING ||
           state == SYSTEM_STATE_ROPE_INIT_RUNNING ||
           state == SYSTEM_STATE_PAUSED;
}

void setSystemState(SystemRuntimeContext* sharedData, uint8_t state)
{
    if (!sharedData) {
        return;
    }
    // 统一维护 transition_count，禁止其他模块直接“顺手”改 system_state。
    if (sharedData->system_state != state) {
        sharedData->system_state = state;
        p4::platform::rtos::atomicAddFetch(&sharedData->state_transition_count,
                                     (uint32_t)1);
    }
}

static void resetServoTargetQueue(SystemRuntimeContext* sharedData)
{
    p4::actuator::clearPending(sharedData);
}

static void setFeedbackAdmissionResult(SystemRuntimeContext* sharedData,
                                       uint32_t requestSequence,
                                       uint8_t status,
                                       uint8_t rejectReason)
{
    if (!sharedData || requestSequence == 0) {
        return;
    }
    // 上位机可能连续发目标；只允许最新请求写回准入结果。
    if (sharedAtomicLoad(&sharedData->feedback_admission_sequence) != requestSequence) {
        return;
    }
    sharedAtomicStore(&sharedData->feedback_admission_status, status);
    sharedAtomicStore(&sharedData->feedback_reject_reason, rejectReason);
    if (status == FEEDBACK_ADMISSION_REJECTED) {
        // 所有负反馈拒绝都经本函数回写，因此在这里统一登记可避免分支漏报。
        const SafetyRuleId ruleId =
            p4::safety::ruleForFeedbackRejectReason(rejectReason);
        p4::safety::reportSafetyRuleEvent(sharedData,
                                          ruleId,
                                          (int32_t)requestSequence,
                                          (int32_t)rejectReason);
    }
}

static void requestServoEmergencyTorqueOff(SystemRuntimeContext* sharedData)
{
    if (sharedData) {
        sharedAtomicAddFetch(&sharedData->servo_emergency_stop_token, (uint32_t)1);
    }
}

static void logMultiTurnUnknownReject(SystemRuntimeContext* sharedData, const char* eventName)
{
    if (!sharedData) {
        return;
    }
    FW_LOG_INFO_PRINTF("[STATE] reject %s: multi-turn unknown active=0x%08lX unknown=0x%08lX\r\n",
                  eventName ? eventName : "RUN",
                  (unsigned long)p4::state::requiredMotorMask(sharedData),
                  (unsigned long)sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap));
}

void enterFaultHold(SystemRuntimeContext* sharedData, uint32_t faultBits)
{
    if (!sharedData) {
        return;
    }
    // 运行态故障统一撤销输出所有权并锁存，必须由 RESET/Clear Fault 显式恢复。
    const uint8_t interruptedState = sharedData->system_state;
    clearPendingDirectCommand(sharedData, DIRECT_REJECT_FAULT);
    sharedAtomicStore(&sharedData->feedback_recovery_pending, (uint8_t)0);
    sharedAtomicStore(&sharedData->feedback_recovery_request_sequence,
                      (uint32_t)0);
    resetServoTargetQueue(sharedData);
    const uint32_t previousLatchedFaults =
        sharedAtomicLoad(&sharedData->system_fault_latched_bitmap);
    const uint32_t newlyLatchedFaults = faultBits & ~previousLatchedFaults;
    sharedAtomicFetchOr(&sharedData->system_fault_latched_bitmap, faultBits);
    const uint32_t latchedFaults =
        sharedAtomicLoad(&sharedData->system_fault_latched_bitmap);
    sharedAtomicFetchOr(&sharedData->system_fault_bitmap, latchedFaults);
    // 每个新锁存故障位只登记一次稳定规则 ID；重复轮询同一故障不会刷爆事件缓存。
    for (uint8_t bitIndex = 0; bitIndex < 32; ++bitIndex) {
        const uint32_t bit = 1UL << bitIndex;
        if ((newlyLatchedFaults & bit) == 0) {
            continue;
        }
        p4::safety::reportSafetyRuleEvent(
            sharedData,
            p4::safety::ruleForSystemFaultBit(bit),
            (int32_t)faultBits,
            (int32_t)interruptedState);
    }
    sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
    sharedData->control_enabled = 0;
    sharedAtomicStore(&sharedData->calibration_motion_started, (uint8_t)0);
    if (interruptedState == SYSTEM_STATE_CALIBRATION_RUNNING) {
        setCalibrationStatus(CALIB_STATUS_FAILED);
        sharedAtomicStore(&sharedData->calibration_fault_latched, (uint8_t)1);
    }
    if (interruptedState == SYSTEM_STATE_ROPE_INIT_RUNNING) {
        sharedData->rope_init_status = ROPE_INIT_STATUS_FAILED;
        sharedData->rope_init_reason = ROPE_INIT_REASON_SYSTEM_FAULT;
        sharedData->rope_init_timestamp = p4::platform::time::monotonicMillis();
    }
    requestServoEmergencyTorqueOff(sharedData);
    setSystemState(sharedData, SYSTEM_STATE_UNCONTROLLABLE);
}

static void resetControlProtectionFaults(SystemRuntimeContext* sharedData)
{
    if (!sharedData) {
        return;
    }
    {
        SharedSafetyGuard guard;
        // bitmap 和对应负载数组属于同一份故障快照，必须成组清零。
        sharedAtomicStore(&sharedData->overload_fault_bitmap, (uint32_t)0);
        for (uint8_t i = 0; i < SERVO_TOTAL_NUM; i++) {
            sharedData->overload_fault_load[i] = 0;
        }
    }
    sharedAtomicAddFetch(&sharedData->overload_fault_reset_token, (uint32_t)1);

}

// 反馈 READY 的总门禁：
// 1) 当前 active joints 必须有 Manual、Auto v3、有效磁编映射和舵机 ABS 限位；
// 2) 磁编/CAN/舵机反馈必须在线且新鲜，多圈状态必须可信；
// 3) READY 只由工作空间下边界决定：低于 Auto Min 时进入 RECOVERY_REQUIRED，
//    上边界越界只由控制层限制目标并记录警告，不在这里强制退出 READY；
// 4) 24/48 counts 迟滞和连续三帧确认用于消除边界抖动。
static bool activeJointsInControlWorkspace(SystemRuntimeContext* sharedData,
                                           uint32_t* recoverableBelowMask)
{
    const p4::state::FeedbackReadinessDecision decision =
        p4::state::evaluateFeedbackReadiness(
            sharedData, p4::platform::time::monotonicMillis());
    if (recoverableBelowMask) {
        *recoverableBelowMask = decision.recoverableBelowMask;
    }
    if (!decision.ready &&
        decision.reason != p4::state::FEEDBACK_READY_REJECT_NONE) {
        logFeedbackReadyReject(sharedData,
                               decision.reason,
                               decision.a,
                               decision.b,
                               decision.c,
                               decision.d);
    }
    return decision.ready;
}

static bool activeJointsInControlWorkspace(SystemRuntimeContext* sharedData)
{
    return activeJointsInControlWorkspace(sharedData, NULL);
}

static bool feedbackTargetsPointIntoWorkspace(SystemRuntimeContext* sharedData)
{
    if (!sharedData) {
        return false;
    }
    const uint32_t activeMask =
        sharedAtomicLoad(&sharedData->active_joint_mask) &
        kManualEncoderReferenceFullMask;
    if (activeMask == 0) {
        return false;
    }

    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        return false;
    }

    // 自动恢复只允许“向工作空间内部”运动；V4 中 currentMin 固定等于 URDF Min。
    bool safe = true;
    for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; joint++) {
        if ((activeMask & (1UL << joint)) == 0) {
            continue;
        }
        const JointCalibrationResult& calib = g_jointCalibResult[joint];
        const float targetDeg = sharedData->targetAngles[joint];
        if (!calib.success || !isfinite(targetDeg) ||
            !isfinite(calib.currentMinDeg) ||
            targetDeg < calib.currentMinDeg) {
            safe = false;
            break;
        }
    }

    if (lock) {
        xSemaphoreGive(lock);
    }
    return safe;
}

static bool beginRopeInitMotion(SystemRuntimeContext* sharedData,
                                bool feedbackRecovery,
                                uint32_t feedbackRequestSequence)
{
    if (!sharedData) {
        return false;
    }
    // Rope Init 临时借用 CALIBRATION 输出所有权，但 system_state 使用独立状态，
    // 便于 UI、故障处理和完成事件区分自动标定。
    p4::platform::rtos::atomicAddFetch(&sharedData->rope_init_sequence,
                                 (uint32_t)1);
    sharedData->rope_init_joint_mask =
        sharedAtomicLoad(&sharedData->active_joint_mask) &
        kManualEncoderReferenceFullMask;
    sharedData->rope_init_timestamp = p4::platform::time::monotonicMillis();
    resetServoTargetQueue(sharedData);
    sharedData->servo_target_owner = SERVO_TARGET_OWNER_CALIBRATION;
    sharedData->control_enabled = 0;
    sharedData->rope_init_status = ROPE_INIT_STATUS_RUNNING;
    sharedData->rope_init_reason = ROPE_INIT_REASON_NONE;
    sharedAtomicStore(&sharedData->feedback_recovery_pending,
                      (uint8_t)(feedbackRecovery ? 1U : 0U));
    sharedAtomicStore(&sharedData->feedback_recovery_request_sequence,
                      feedbackRecovery ? feedbackRequestSequence : 0U);
    setCommandModeDirect(sharedData);
    setSystemState(sharedData, SYSTEM_STATE_ROPE_INIT_RUNNING);
    // Rope Init 与状态机同线程推进，不再创建临时 FreeRTOS Task。
    // 每次启动先清理上一会话残留，随后建立新的非阻塞运行上下文。
    p4::recovery::cancelRopeInitController(sharedData);
    return p4::recovery::startRopeInitController(
        sharedData, p4::platform::time::monotonicMillis());
}

uint8_t idleOrReadyState(SystemRuntimeContext* sharedData)
{
    // E-STOP 是最高优先级门禁。即使重连后硬件反馈恢复、active scope 触发
    // REEVALUATE_READY，也不得在 RESET 前把对外状态伪装成 IDLE/READY。
    if (!sharedData ||
        sharedAtomicLoad(&sharedData->emergency_stop_latched) != 0 ||
        !p4::state::controllableEntryReady(sharedData)) {
        return SYSTEM_STATE_UNCONTROLLABLE;
    }
    uint32_t recoverableBelowMask = 0;
    if (activeJointsInControlWorkspace(sharedData, &recoverableBelowMask)) {
        return SYSTEM_STATE_READY;
    }
    return recoverableBelowMask != 0
        ? SYSTEM_STATE_RECOVERY_REQUIRED
        : SYSTEM_STATE_IDLE;
}

/** 清理 Host Hold 运行时；请求 token 保持单调递增，用于隔离迟到确认。 */
void updateReadyStateIfIdle(SystemRuntimeContext* sharedData)
{
    if (!sharedData) {
        return;
    }
    if (sharedData->control_enabled != 0 || sharedData->servo_target_owner != SERVO_TARGET_OWNER_NONE) {
        return;
    }
    const uint8_t state = sharedData->system_state;
    if (state == SYSTEM_STATE_IDLE ||
        state == SYSTEM_STATE_READY ||
        state == SYSTEM_STATE_RECOVERY_REQUIRED) {
        setSystemState(sharedData, idleOrReadyState(sharedData));
    }
}

static bool feedbackCompletionMatchesCurrentCommand(SystemRuntimeContext* sharedData,
                                                    const SystemEvent_t& event)
{
    // token=0 保留给不绑定代次的内部完成事件；正常控制使用 joint_command_token。
    return sharedData &&
           (event.token == 0 || event.token == sharedData->joint_command_token);
}

static void releaseFeedbackRun(SystemRuntimeContext* sharedData)
{
    if (!sharedData) {
        return;
    }
    // 保留舵机最后一次接受的位置作为稳定保持，仅释放反馈任务所有权，
    // 这样上位机可以立即切换 active scope 并发送下一条关节命令。
    sharedData->control_enabled = 0;
    sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
    setSystemState(sharedData, idleOrReadyState(sharedData));
    requestMultiTurnNvsSave(sharedData, MULTITURN_NVS_SAVE_REASON_RUN_DONE);
}

/**
 * E-STOP锁存是所有运动状态的共同硬门禁，只有RESET可以解除。
 * 把检查放在状态所有权授予之前，避免UI显示RUN_DIRECT但驱动层持续拒绝目标。
 */
static bool rejectMotionWhenEmergencyStopLatched(SystemRuntimeContext* sharedData,
                                                 const char* eventName)
{
    if (!sharedData ||
        sharedAtomicLoad(&sharedData->emergency_stop_latched) == 0) {
        return false;
    }
    FW_LOG_WARN_PRINTF(
        "[STATE] reject %s: E-STOP latched; RESET required\r\n",
        eventName ? eventName : "MOTION");
    return true;
}

void handleSystemEvent(SystemRuntimeContext* sharedData, const SystemEvent_t& event)
{
    if (!sharedData) {
        return;
    }

    // 所有状态迁移集中在此处；事件处理必须先做门禁，再授予输出所有权。
    switch (event.event)
    {
        case SYSTEM_EVENT_START:
            if (!p4::system::canAcceptStart(sharedData->system_state)) {
                p4::safety::reportSafetyRuleEvent(
                    sharedData,
                    SafetyRuleId::SystemStartStateRejected,
                    (int32_t)event.token,
                    sharedData->system_state);
                FW_LOG_INFO_PRINTF("[STATE] reject START in state=%u\r\n",
                              (unsigned)sharedData->system_state);
                break;
            }

            if (sharedData->system_state == SYSTEM_STATE_PAUSED) {
                const uint8_t resumeState = sharedData->paused_resume_state;
                const bool resumeReady =
                    (resumeState == SYSTEM_STATE_RUN_DIRECT)
                        ? p4::state::controllableEntryReady(sharedData)
                        : ((resumeState == SYSTEM_STATE_RUN_FEEDBACK) &&
                           activeJointsInControlWorkspace(sharedData));
                if (!resumeReady) {
                    sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
                    sharedData->control_enabled = 0;
                    setSystemState(sharedData, idleOrReadyState(sharedData));
                    break;
                }
                resetServoTargetQueue(sharedData);
                sharedData->servo_target_owner = SERVO_TARGET_OWNER_CONTROL;
                sharedData->control_enabled = 1;
                if (resumeState == SYSTEM_STATE_RUN_DIRECT) {
                    setCommandModeDirect(sharedData);
                } else if (resumeState == SYSTEM_STATE_RUN_FEEDBACK) {
                    setCommandModeJoint(sharedData);
                }
                setSystemState(sharedData,
                               (resumeState == SYSTEM_STATE_RUN_DIRECT ||
                                resumeState == SYSTEM_STATE_RUN_FEEDBACK)
                                   ? resumeState
                                   : idleOrReadyState(sharedData));
                break;
            }

            resetServoTargetQueue(sharedData);
            sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
            sharedData->control_enabled = 0;
            setSystemState(sharedData, idleOrReadyState(sharedData));
            break;

        case SYSTEM_EVENT_STOP:
            // 丢弃 STOP 之前已排队的运动事件。上位机必须在看到
            // UNCONTROLLABLE 后重新发送命令，旧事件不能让设备自行复动。
            if (shouldSaveMultiTurnOnInterrupt(sharedData->system_state)) {
                requestMultiTurnNvsSave(sharedData, MULTITURN_NVS_SAVE_REASON_RUN_DONE);
            }
            sharedData->stateEventQueue.reset();
            clearPendingDirectCommand(sharedData, DIRECT_REJECT_STATE);
            sharedAtomicStore(&sharedData->feedback_recovery_pending,
                              (uint8_t)0);
            sharedAtomicStore(&sharedData->feedback_recovery_request_sequence,
                              (uint32_t)0);
            resetServoTargetQueue(sharedData);
            sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
            sharedData->control_enabled = 0;
            p4::state::resetHostControlLeaseRuntime(sharedData);
            if (sharedData->system_state == SYSTEM_STATE_ROPE_INIT_RUNNING) {
                sharedData->rope_init_status = ROPE_INIT_STATUS_FAILED;
                sharedData->rope_init_reason = ROPE_INIT_REASON_CANCELLED;
                sharedData->rope_init_timestamp = p4::platform::time::monotonicMillis();
            }
            if (sharedData->system_state == SYSTEM_STATE_CALIBRATION_RUNNING) {
                setCalibrationStatus(CALIB_STATUS_IDLE);
            }
            setSystemState(sharedData, SYSTEM_STATE_UNCONTROLLABLE);
            break;

        case SYSTEM_EVENT_RESET:
            // RESET 清命令、清可复位保护并断扭矩，但不伪造多圈或标定数据。
            if (shouldSaveMultiTurnOnInterrupt(sharedData->system_state)) {
                requestMultiTurnNvsSave(sharedData, MULTITURN_NVS_SAVE_REASON_RUN_DONE);
            }
            resetCommandState(sharedData);
            sharedData->control_enabled = 0;
            sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
            resetServoTargetQueue(sharedData);
            requestServoEmergencyTorqueOff(sharedData);
            // RESET 是唯一解除 E-STOP 最终门禁的用户动作；先撤销 owner/旧队列再清锁存。
            sharedAtomicStore(&sharedData->emergency_stop_latched, (uint8_t)0);
            p4::state::resetHostControlLeaseRuntime(sharedData);
            resetControlProtectionFaults(sharedData);
            sharedAtomicStore(&sharedData->calibration_fault_latched, (uint8_t)0);
            sharedAtomicStore(&sharedData->system_fault_latched_bitmap, (uint32_t)0);
            sharedAtomicStore(&sharedData->system_fault_bitmap, (uint32_t)0);
            setCalibrationStatus(CALIB_STATUS_IDLE);
            setSystemState(sharedData, SYSTEM_STATE_UNCONTROLLABLE);
            break;

        case SYSTEM_EVENT_RUN_DIRECT:
            // staged 直控必须同时满足代次、状态、在线和多圈可信四项门禁。
            if (!pendingDirectCommandMatches(sharedData, event.token)) {
                FW_LOG_INFO_PRINTF("[STATE] ignore stale RUN_DIRECT generation=%lu pending=%lu valid=%u\r\n",
                                   (unsigned long)event.token,
                                   (unsigned long)sharedData->pending_direct_generation,
                                   (unsigned)sharedData->pending_direct_valid);
                break;
            }
            if (rejectMotionWhenEmergencyStopLatched(sharedData,
                                                     "RUN_DIRECT")) {
                rejectPendingDirectCommand(sharedData,
                                           event.token,
                                           DIRECT_REJECT_STATE);
                break;
            }
            if (!p4::state::prepareHostControlCommandAdmission(sharedData)) {
                FW_LOG_INFO_PRINTLN(
                    "[STATE] reject RUN_DIRECT: host lease stale or Hold pending");
                rejectPendingDirectCommand(sharedData,
                                           event.token,
                                           DIRECT_REJECT_STALE);
                break;
            }
            if (!p4::system::canAcceptRunDirect(sharedData->system_state)) {
                p4::safety::reportSafetyRuleEvent(
                    sharedData,
                    SafetyRuleId::SystemDirectStateRejected,
                    (int32_t)event.token,
                    sharedData->system_state);
                FW_LOG_INFO_PRINTF("[STATE] reject RUN_DIRECT in state=%u\r\n",
                              (unsigned)sharedData->system_state);
                rejectPendingDirectCommand(sharedData,
                                           event.token,
                                           DIRECT_REJECT_STATE);
                break;
            }
            {
                const uint32_t pendingMotorMask =
                    pendingDirectMotorMask(sharedData, event.token);
                if ((sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap) &
                     pendingMotorMask) != 0) {
                    FW_LOG_INFO_PRINTF("[STATE] reject RUN_DIRECT generation=%lu: multi-turn unknown motor=0x%08lX unknown=0x%08lX\r\n",
                                       (unsigned long)event.token,
                                       (unsigned long)pendingMotorMask,
                                       (unsigned long)sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap));
                    rejectPendingDirectCommand(sharedData,
                                               event.token,
                                               DIRECT_REJECT_MULTITURN);
                    break;
                }
                if (!p4::state::directMotorMaskReady(sharedData,
                                                     pendingMotorMask)) {
                    FW_LOG_INFO_PRINTF("[STATE] reject RUN_DIRECT generation=%lu: pending hardware not ready motor=0x%08lX\r\n",
                                       (unsigned long)event.token,
                                       (unsigned long)pendingMotorMask);
                    rejectPendingDirectCommand(sharedData,
                                               event.token,
                                               DIRECT_REJECT_HARDWARE);
                    break;
                }
            }
            if (!promotePendingDirectCommand(sharedData, event.token)) {
                rejectPendingDirectCommand(sharedData,
                                           event.token,
                                           DIRECT_REJECT_STALE);
                break;
            }
            sharedAtomicFetchAnd(&sharedData->system_fault_bitmap,
                                 (uint32_t)~(SYSTEM_FAULT_CAN_OFFLINE |
                                             SYSTEM_FAULT_SERVO_OFFLINE |
                                             SYSTEM_FAULT_MULTITURN_UNKNOWN));
            sharedAtomicFetchAnd(&sharedData->system_fault_latched_bitmap,
                                 (uint32_t)~(SYSTEM_FAULT_CAN_OFFLINE |
                                             SYSTEM_FAULT_SERVO_OFFLINE |
                                             SYSTEM_FAULT_MULTITURN_UNKNOWN));
            if (sharedData->system_state != SYSTEM_STATE_RUN_DIRECT) {
                resetServoTargetQueue(sharedData);
            }
            sharedAtomicStore(&sharedData->feedback_recovery_pending,
                              (uint8_t)0);
            sharedAtomicStore(&sharedData->feedback_recovery_request_sequence,
                              (uint32_t)0);
            sharedData->servo_target_owner = SERVO_TARGET_OWNER_CONTROL;
            sharedData->control_enabled = 1;
            setSystemState(sharedData, SYSTEM_STATE_RUN_DIRECT);
            break;

        case SYSTEM_EVENT_RUN_FEEDBACK:
            // 反馈准入结果通过 sequence 回写给上位机；RECOVERY_REQUIRED
            // 会自动启动 Rope Init，完成后再恢复本次反馈请求。
            if (rejectMotionWhenEmergencyStopLatched(sharedData,
                                                     "RUN_FEEDBACK")) {
                setFeedbackAdmissionResult(sharedData, event.token,
                                           FEEDBACK_ADMISSION_REJECTED,
                                           FEEDBACK_REJECT_STATE);
                break;
            }
            if (!p4::state::prepareHostControlCommandAdmission(sharedData)) {
                FW_LOG_INFO_PRINTLN(
                    "[STATE] reject RUN_FEEDBACK: host lease stale or Hold pending");
                setFeedbackAdmissionResult(sharedData, event.token,
                                           FEEDBACK_ADMISSION_REJECTED,
                                           FEEDBACK_REJECT_STATE);
                break;
            }
            if (!manualReferenceGateReady(sharedData)) {
                logManualReferenceReject(sharedData, "RUN_FEEDBACK");
                setFeedbackAdmissionResult(sharedData, event.token,
                                           FEEDBACK_ADMISSION_REJECTED,
                                           FEEDBACK_REJECT_MANUAL_REFERENCE);
                break;
            }
            if (sharedData->system_state == SYSTEM_STATE_ROPE_INIT_RUNNING &&
                sharedAtomicLoad(&sharedData->feedback_recovery_pending) != 0) {
                if (!feedbackTargetsPointIntoWorkspace(sharedData)) {
                    sharedAtomicStore(&sharedData->feedback_recovery_pending,
                                      (uint8_t)0);
                    sharedAtomicStore(
                        &sharedData->feedback_recovery_request_sequence,
                        (uint32_t)0);
                    setFeedbackAdmissionResult(sharedData, event.token,
                                               FEEDBACK_ADMISSION_REJECTED,
                                               FEEDBACK_REJECT_WORKSPACE);
                    FW_LOG_INFO_PRINTLN(
                        "[STATE] recovery resume cancelled: latest target is below currentMin");
                } else {
                    sharedAtomicStore(
                        &sharedData->feedback_recovery_request_sequence,
                        event.token);
                    setFeedbackAdmissionResult(sharedData, event.token,
                                               FEEDBACK_ADMISSION_ACCEPTED,
                                               FEEDBACK_REJECT_NONE);
                }
                break;
            }
            // 每个上位机关节目标都会附带 RUN_FEEDBACK。状态包可能落后一帧，
            // 因此已经在反馈运行时收到重复请求必须幂等；不能重置输出队列，
            // MotionControlTask 会在下一周期消费更新后的 joint_command_token。
            if (sharedData->system_state == SYSTEM_STATE_RUN_FEEDBACK) {
                setFeedbackAdmissionResult(sharedData, event.token,
                                           FEEDBACK_ADMISSION_ACCEPTED,
                                           FEEDBACK_REJECT_NONE);
                break;
            }
            if (sharedData->system_state == SYSTEM_STATE_RECOVERY_REQUIRED) {
                uint32_t recoverableBelowMask = 0;
                const bool workspaceReady = activeJointsInControlWorkspace(
                    sharedData, &recoverableBelowMask);
                if (workspaceReady) {
                    setSystemState(sharedData, SYSTEM_STATE_READY);
                } else if (recoverableBelowMask == 0 ||
                           !feedbackTargetsPointIntoWorkspace(sharedData)) {
                    FW_LOG_INFO_PRINTF(
                        "[STATE] reject recovery target below=0x%08lX\r\n",
                        (unsigned long)recoverableBelowMask);
                    setFeedbackAdmissionResult(sharedData, event.token,
                                               FEEDBACK_ADMISSION_REJECTED,
                                               FEEDBACK_REJECT_WORKSPACE);
                    break;
                } else {
                    const uint8_t preflightReason =
                        p4::recovery::ropeInitActiveScopeRejectReason(sharedData);
                    const bool multiTurnKnown =
                        activeMotorsMultiTurnKnown(sharedData);
                    if (preflightReason != ROPE_INIT_REASON_NONE ||
                        !multiTurnKnown) {
                        FW_LOG_INFO_PRINTF(
                            "[STATE] reject automatic recovery preflight=%u multiturn=%u\r\n",
                            (unsigned)preflightReason,
                            multiTurnKnown ? 1U : 0U);
                        setFeedbackAdmissionResult(
                            sharedData, event.token,
                            FEEDBACK_ADMISSION_REJECTED,
                            !multiTurnKnown
                                ? FEEDBACK_REJECT_MULTITURN
                                : FEEDBACK_REJECT_WORKSPACE);
                        break;
                    }
                    if (!beginRopeInitMotion(sharedData, true, event.token)) {
                        setFeedbackAdmissionResult(sharedData, event.token,
                                                   FEEDBACK_ADMISSION_REJECTED,
                                                   FEEDBACK_REJECT_STATE);
                        break;
                    }
                    FW_LOG_INFO_PRINTF(
                        "[STATE] automatic feedback recovery started joints=0x%08lX request=%lu\r\n",
                        (unsigned long)recoverableBelowMask,
                        (unsigned long)event.token);
                    setFeedbackAdmissionResult(sharedData, event.token,
                                               FEEDBACK_ADMISSION_ACCEPTED,
                                               FEEDBACK_REJECT_NONE);
                    break;
                }
            }
            if (!p4::system::canAcceptRunFeedback(sharedData->system_state)) {
                p4::safety::reportSafetyRuleEvent(
                    sharedData,
                    SafetyRuleId::SystemFeedbackStateRejected,
                    (int32_t)event.token,
                    sharedData->system_state);
                FW_LOG_INFO_PRINTF("[STATE] reject RUN_FEEDBACK in state=%u\r\n",
                              (unsigned)sharedData->system_state);
                setFeedbackAdmissionResult(sharedData, event.token,
                                           FEEDBACK_ADMISSION_REJECTED,
                                           FEEDBACK_REJECT_STATE);
                break;
            }
            if (!activeJointsInControlWorkspace(sharedData)) {
                FW_LOG_INFO_PRINTLN("[STATE] reject RUN_FEEDBACK: feedback gate changed");
                setFeedbackAdmissionResult(sharedData, event.token,
                                           FEEDBACK_ADMISSION_REJECTED,
                                           FEEDBACK_REJECT_WORKSPACE);
                break;
            }
            if (!activeMotorsMultiTurnKnown(sharedData)) {
                logMultiTurnUnknownReject(sharedData, "RUN_FEEDBACK");
                setFeedbackAdmissionResult(sharedData, event.token,
                                           FEEDBACK_ADMISSION_REJECTED,
                                           FEEDBACK_REJECT_MULTITURN);
                break;
            }
            if (!activeMotorsAbsLimitsCalibrated(sharedData)) {
                FW_LOG_INFO_PRINTLN("[STATE] reject RUN_FEEDBACK: active joints are not calibrated");
                setFeedbackAdmissionResult(sharedData, event.token,
                                           FEEDBACK_ADMISSION_REJECTED,
                                           FEEDBACK_REJECT_ABS_LIMIT);
                break;
            }
            if (!setCommandModeJoint(sharedData)) {
                FW_LOG_INFO_PRINTLN("[STATE] reject RUN_FEEDBACK: command mode mutex busy");
                setFeedbackAdmissionResult(sharedData, event.token,
                                           FEEDBACK_ADMISSION_REJECTED,
                                           FEEDBACK_REJECT_MODE_SWITCH);
                break;
            }
            resetServoTargetQueue(sharedData);
            sharedData->servo_target_owner = SERVO_TARGET_OWNER_CONTROL;
            sharedData->control_enabled = 1;
            setSystemState(sharedData, SYSTEM_STATE_RUN_FEEDBACK);
            setFeedbackAdmissionResult(sharedData, event.token,
                                       FEEDBACK_ADMISSION_ACCEPTED,
                                       FEEDBACK_REJECT_NONE);
            break;

        case SYSTEM_EVENT_RUN_DIRECT_DONE:
            if (sharedData->system_state != SYSTEM_STATE_RUN_DIRECT) {
                FW_LOG_INFO_PRINTF("[STATE] ignore RUN_DIRECT_DONE in state=%u\r\n",
                              (unsigned)sharedData->system_state);
                break;
            }
            if (event.token != sharedData->applied_direct_generation) {
                FW_LOG_INFO_PRINTF("[STATE] ignore stale RUN_DIRECT_DONE generation=%lu applied=%lu\r\n",
                                   (unsigned long)event.token,
                                   (unsigned long)sharedData->applied_direct_generation);
                break;
            }
            sharedData->control_enabled = 0;
            sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
            setSystemState(sharedData, idleOrReadyState(sharedData));
            requestMultiTurnNvsSave(sharedData, MULTITURN_NVS_SAVE_REASON_RUN_DONE);
            break;

        case SYSTEM_EVENT_RUN_FEEDBACK_DONE:
            if (sharedData->system_state != SYSTEM_STATE_RUN_FEEDBACK) {
                FW_LOG_INFO_PRINTF("[STATE] ignore RUN_FEEDBACK_DONE in state=%u\r\n",
                              (unsigned)sharedData->system_state);
                break;
            }
            if (!feedbackCompletionMatchesCurrentCommand(sharedData, event)) {
                FW_LOG_INFO_PRINTF("[STATE] ignore stale RUN_FEEDBACK_DONE token=%lu current=%lu\r\n",
                              (unsigned long)event.token,
                              (unsigned long)sharedData->joint_command_token);
                break;
            }
            releaseFeedbackRun(sharedData);
            break;

        case SYSTEM_EVENT_RUN_FEEDBACK_TIMEOUT:
            if (sharedData->system_state != SYSTEM_STATE_RUN_FEEDBACK) {
                FW_LOG_INFO_PRINTF("[STATE] ignore RUN_FEEDBACK_TIMEOUT in state=%u\r\n",
                              (unsigned)sharedData->system_state);
                break;
            }
            if (!feedbackCompletionMatchesCurrentCommand(sharedData, event)) {
                FW_LOG_INFO_PRINTF("[STATE] ignore stale RUN_FEEDBACK_TIMEOUT token=%lu current=%lu\r\n",
                              (unsigned long)event.token,
                              (unsigned long)sharedData->joint_command_token);
                break;
            }
            FW_LOG_INFO_PRINTF("[STATE] RUN_FEEDBACK timeout token=%lu; releasing control ownership\r\n",
                          (unsigned long)event.token);
            releaseFeedbackRun(sharedData);
            break;

        case SYSTEM_EVENT_PAUSE:
            // PAUSE 记录原运行态并断扭矩；START 仅在门禁仍满足时恢复。
            if (sharedData->system_state != SYSTEM_STATE_RUN_DIRECT &&
                sharedData->system_state != SYSTEM_STATE_RUN_FEEDBACK) {
                FW_LOG_INFO_PRINTF("[STATE] reject PAUSE in state=%u\r\n",
                              (unsigned)sharedData->system_state);
                break;
            }
            sharedData->paused_resume_state = sharedData->system_state;
            sharedData->control_enabled = 0;
            sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
            requestMultiTurnNvsSave(sharedData, MULTITURN_NVS_SAVE_REASON_RUN_DONE);
            requestServoEmergencyTorqueOff(sharedData);
            setSystemState(sharedData, SYSTEM_STATE_PAUSED);
            break;

        case SYSTEM_EVENT_CALIBRATE:
        {
            // 标定允许从 UNCONTROLLABLE 启动，但 Manual、任务所有权和 active scope
            // 仍必须在启动 AutoCalibrationTask 前通过检查。
            if (rejectMotionWhenEmergencyStopLatched(sharedData,
                                                     "CALIBRATE")) {
                setCalibrationStatus(CALIB_STATUS_FAILED);
                break;
            }
            if (!p4::state::prepareHostControlCommandAdmission(sharedData)) {
                FW_LOG_INFO_PRINTLN(
                    "[STATE] reject CALIBRATE: host lease stale or Hold pending");
                setCalibrationStatus(CALIB_STATUS_FAILED);
                break;
            }
            if (!manualReferenceGateReady(sharedData)) {
                logManualReferenceReject(sharedData, "CALIBRATE");
                setCalibrationStatus(CALIB_STATUS_FAILED);
                break;
            }
            if (!p4::system::canAcceptCalibration(sharedData->system_state)) {
                p4::safety::reportSafetyRuleEvent(
                    sharedData,
                    SafetyRuleId::SystemCalibrationStateRejected,
                    (int32_t)event.token,
                    sharedData->system_state);
                FW_LOG_INFO_PRINTF("[STATE] reject CALIBRATE in state=%u\r\n",
                              (unsigned)sharedData->system_state);
                break;
            }
            resetServoTargetQueue(sharedData);
            sharedAtomicStore(&sharedData->feedback_recovery_pending,
                              (uint8_t)0);
            sharedAtomicStore(&sharedData->feedback_recovery_request_sequence,
                              (uint32_t)0);
            // 刚进入 CALIBRATION_RUNNING 时仍处于无运动预检，允许舵机通信任务完成热插拔恢复。
            sharedAtomicStore(&sharedData->calibration_motion_started,
                              (uint8_t)0);
            sharedAtomicStore(&sharedData->calibration_trust_request_mask,
                              (uint32_t)0);
            sharedAtomicStore(&sharedData->calibration_trust_accepted_mask,
                              (uint32_t)0);
            sharedData->servo_target_owner = SERVO_TARGET_OWNER_CALIBRATION;
            sharedData->control_enabled = 0;
            sharedAtomicStore(&sharedData->calibration_fault_latched, (uint8_t)0);
            setCommandModeJoint(sharedData);
            setCalibrationStatus(CALIB_STATUS_RUNNING);
            setSystemState(sharedData, SYSTEM_STATE_CALIBRATION_RUNNING);
            if (!startAutoCalibrationTaskRuntime(sharedData)) {
                CalibrationDiagData_t diag;
                memset(&diag, 0, sizeof(diag));
                diag.valid = 1;
                diag.reason = CALIB_DIAG_REASON_TASK_START_FAILED;
                diag.phase = CALIB_DIAG_PHASE_SEQUENCE;
                diag.joint = 0xFF;
                diag.motor = 0xFF;
                diag.timestamp = p4::platform::time::monotonicMillis();
                {
                    SharedSafetyGuard guard;
                    diag.sequence = sharedData->calibration_diag.sequence + 1;
                    sharedData->calibration_diag = diag;
                }
                setCalibrationStatus(CALIB_STATUS_FAILED);
                sharedAtomicStore(&sharedData->calibration_fault_latched, (uint8_t)1);
                sharedAtomicStore(&sharedData->calibration_motion_started,
                                  (uint8_t)0);
                requestServoEmergencyTorqueOff(sharedData);
                setSystemState(sharedData, SYSTEM_STATE_UNCONTROLLABLE);
            }
            break;
        }

        case SYSTEM_EVENT_ROPE_INIT:
        {
            // 手动 Rope Init 只处理 active scope，并且要求其 Manual/Auto/多圈已可信。
            if (rejectMotionWhenEmergencyStopLatched(sharedData,
                                                     "ROPE_INIT")) {
                sharedData->rope_init_status = ROPE_INIT_STATUS_FAILED;
                sharedData->rope_init_reason = ROPE_INIT_REASON_CANCELLED;
                sharedData->rope_init_timestamp = p4::platform::time::monotonicMillis();
                break;
            }
            if (!p4::state::prepareHostControlCommandAdmission(sharedData)) {
                FW_LOG_INFO_PRINTLN(
                    "[STATE] reject ROPE_INIT: host lease stale or Hold pending");
                sharedData->rope_init_status = ROPE_INIT_STATUS_FAILED;
                sharedData->rope_init_reason = ROPE_INIT_REASON_CANCELLED;
                sharedData->rope_init_timestamp = p4::platform::time::monotonicMillis();
                break;
            }
            const uint8_t preflightReason =
                p4::recovery::ropeInitActiveScopeRejectReason(sharedData);
            if (preflightReason != ROPE_INIT_REASON_NONE) {
                sharedData->rope_init_status = ROPE_INIT_STATUS_FAILED;
                sharedData->rope_init_reason = preflightReason;
                sharedData->rope_init_timestamp = p4::platform::time::monotonicMillis();
                if (preflightReason == ROPE_INIT_REASON_MANUAL_REQUIRED) {
                    logManualReferenceReject(sharedData, "ROPE_INIT");
                } else {
                    FW_LOG_INFO_PRINTF("[STATE] reject ROPE_INIT preflight reason=%u\r\n",
                                       (unsigned)preflightReason);
                }
                break;
            }
            if (sharedData->system_state != SYSTEM_STATE_IDLE &&
                sharedData->system_state != SYSTEM_STATE_READY &&
                sharedData->system_state != SYSTEM_STATE_RECOVERY_REQUIRED) {
                FW_LOG_INFO_PRINTF("[STATE] reject ROPE_INIT in state=%u\r\n",
                                   (unsigned)sharedData->system_state);
                sharedData->rope_init_status = ROPE_INIT_STATUS_FAILED;
                sharedData->rope_init_reason = ROPE_INIT_REASON_CANCELLED;
                sharedData->rope_init_timestamp = p4::platform::time::monotonicMillis();
                break;
            }
            if (!activeMotorsMultiTurnKnown(sharedData)) {
                logMultiTurnUnknownReject(sharedData, "ROPE_INIT");
                sharedData->rope_init_status = ROPE_INIT_STATUS_FAILED;
                sharedData->rope_init_reason = ROPE_INIT_REASON_MULTITURN_UNKNOWN;
                sharedData->rope_init_timestamp = p4::platform::time::monotonicMillis();
                break;
            }
            beginRopeInitMotion(sharedData, false, 0);
            break;
        }

        case SYSTEM_EVENT_ROPE_INIT_DONE:
            // 租约 Hold 已取得唯一收口权时，Rope Init 控制器的迟到结果不能提前
            // 撤销 CALIBRATION owner，否则执行层会因 owner/state 不匹配拒绝 Hold。
            if (sharedAtomicLoad(&sharedData->host_control_hold_status) ==
                HOST_CONTROL_HOLD_PENDING) {
                FW_LOG_INFO_PRINTLN(
                    "[STATE] ignore ROPE_INIT_DONE while host Hold pending");
                break;
            }
            if (sharedData->system_state != SYSTEM_STATE_ROPE_INIT_RUNNING) {
                break;
            }
            {
                const bool resumeFeedback =
                    sharedAtomicLoad(&sharedData->feedback_recovery_pending) != 0;
                const uint32_t recoveryRequestSequence =
                    sharedAtomicLoad(
                        &sharedData->feedback_recovery_request_sequence);
                resetServoTargetQueue(sharedData);
                sharedData->control_enabled = 0;
                sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
                const bool commandModeJoint = setCommandModeJoint(sharedData);
                sharedData->rope_init_status = ROPE_INIT_STATUS_SUCCESS;
                sharedData->rope_init_reason = ROPE_INIT_REASON_NONE;
                sharedData->rope_init_timestamp = p4::platform::time::monotonicMillis();
                markActiveJointsWorkspaceReady(sharedData);
                sharedAtomicStore(&sharedData->feedback_recovery_pending,
                                  (uint8_t)0);
                sharedAtomicStore(
                    &sharedData->feedback_recovery_request_sequence,
                    (uint32_t)0);
                // 自动恢复场景下，Rope Init 完成后再次核验最新目标，再交还反馈所有权。
                if (resumeFeedback && commandModeJoint &&
                    feedbackTargetsPointIntoWorkspace(sharedData) &&
                    activeJointsInControlWorkspace(sharedData)) {
                    sharedData->servo_target_owner = SERVO_TARGET_OWNER_CONTROL;
                    sharedData->control_enabled = 1;
                    setSystemState(sharedData, SYSTEM_STATE_RUN_FEEDBACK);
                    setFeedbackAdmissionResult(sharedData,
                                               recoveryRequestSequence,
                                               FEEDBACK_ADMISSION_ACCEPTED,
                                               FEEDBACK_REJECT_NONE);
                    FW_LOG_INFO_PRINTF(
                        "[STATE] automatic feedback recovery complete request=%lu\r\n",
                        (unsigned long)recoveryRequestSequence);
                } else {
                    setSystemState(sharedData, idleOrReadyState(sharedData));
                    if (resumeFeedback) {
                        setFeedbackAdmissionResult(sharedData,
                                                   recoveryRequestSequence,
                                                   FEEDBACK_ADMISSION_REJECTED,
                                                   FEEDBACK_REJECT_WORKSPACE);
                        FW_LOG_INFO_PRINTLN(
                            "[STATE] automatic feedback recovery completed without resume");
                    }
                }
                requestMultiTurnNvsSave(sharedData,
                                        MULTITURN_NVS_SAVE_REASON_RUN_DONE);
                break;
            }

        case SYSTEM_EVENT_ROPE_INIT_FAILED:
            if (sharedAtomicLoad(&sharedData->host_control_hold_status) ==
                HOST_CONTROL_HOLD_PENDING) {
                FW_LOG_INFO_PRINTLN(
                    "[STATE] ignore ROPE_INIT_FAILED while host Hold pending");
                break;
            }
            if (sharedData->system_state != SYSTEM_STATE_ROPE_INIT_RUNNING) {
                break;
            }
            resetServoTargetQueue(sharedData);
            sharedData->control_enabled = 0;
            sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
            setCommandModeJoint(sharedData);
            if (sharedData->rope_init_status != ROPE_INIT_STATUS_FAILED) {
                sharedData->rope_init_status = ROPE_INIT_STATUS_FAILED;
                sharedData->rope_init_reason = ROPE_INIT_REASON_CANCELLED;
                sharedData->rope_init_timestamp = p4::platform::time::monotonicMillis();
            }
            {
                const uint32_t recoveryRequestSequence =
                    sharedAtomicLoad(
                        &sharedData->feedback_recovery_request_sequence);
                if (sharedAtomicLoad(&sharedData->feedback_recovery_pending) != 0) {
                    setFeedbackAdmissionResult(sharedData,
                                               recoveryRequestSequence,
                                               FEEDBACK_ADMISSION_REJECTED,
                                               FEEDBACK_REJECT_WORKSPACE);
                }
            }
            sharedAtomicStore(&sharedData->feedback_recovery_pending,
                              (uint8_t)0);
            sharedAtomicStore(&sharedData->feedback_recovery_request_sequence,
                              (uint32_t)0);
            requestServoEmergencyTorqueOff(sharedData);
            setSystemState(sharedData, idleOrReadyState(sharedData));
            requestMultiTurnNvsSave(sharedData,
                                    MULTITURN_NVS_SAVE_REASON_RUN_DONE);
            break;

        case SYSTEM_EVENT_CALIBRATION_DONE:
            // 标定任务已将结果写入 RAM；此处只更新状态、清对应多圈未知并请求 NVS。
            if (sharedAtomicLoad(&sharedData->host_control_hold_status) ==
                HOST_CONTROL_HOLD_PENDING) {
                FW_LOG_INFO_PRINTLN(
                    "[STATE] ignore CALIB_DONE while host Hold pending");
                break;
            }
            if (sharedData->system_state != SYSTEM_STATE_CALIBRATION_RUNNING) {
                FW_LOG_INFO_PRINTF("[STATE] ignore CALIB_DONE in state=%u\r\n",
                              (unsigned)sharedData->system_state);
                break;
            }
            sharedData->control_enabled = 0;
            sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
            sharedAtomicStore(&sharedData->calibration_motion_started,
                              (uint8_t)0);
            sharedAtomicStore(&sharedData->calibration_fault_latched, (uint8_t)0);
            setCalibrationStatus(
                sharedAtomicLoad(&sharedData->calibration_warning_bitmap) != 0
                    ? CALIB_STATUS_WARN
                    : CALIB_STATUS_SUCCESS);
            setSystemState(sharedData, idleOrReadyState(sharedData));
            requestMultiTurnNvsSave(sharedData, MULTITURN_NVS_SAVE_REASON_CALIB_DONE);
            break;

        case SYSTEM_EVENT_REEVALUATE_READY:
            // active scope、Manual 或 Calib Load 改变后触发一次显式重评估。
            if (sharedData->control_enabled == 0 &&
                sharedData->servo_target_owner == SERVO_TARGET_OWNER_NONE &&
                (sharedData->system_state == SYSTEM_STATE_UNCONTROLLABLE ||
                 sharedData->system_state == SYSTEM_STATE_IDLE ||
                 sharedData->system_state == SYSTEM_STATE_READY ||
                 sharedData->system_state == SYSTEM_STATE_RECOVERY_REQUIRED)) {
                const uint8_t previousState = sharedData->system_state;
                const uint8_t nextState = idleOrReadyState(sharedData);
                setSystemState(sharedData, nextState);
                FW_LOG_INFO_PRINTF("[STATE] READY reevaluate previous=%u next=%u motor=0x%08lX joint=0x%08lX\r\n",
                              (unsigned)previousState,
                              (unsigned)nextState,
                              (unsigned long)sharedAtomicLoad(&sharedData->active_motor_mask),
                              (unsigned long)sharedAtomicLoad(&sharedData->active_joint_mask));
            }
            break;

        case SYSTEM_EVENT_CALIBRATION_FAILED:
            // 标定失败属于锁存故障：撤销所有权、断扭矩并等待用户 Clear Fault。
            // 但若失败是租约 Hold 拒绝普通批次引起，则由 Hold 确认路径取消标定，
            // 不能把“上位机失联取消”误升级成“标定算法/硬件故障”。
            if (sharedAtomicLoad(&sharedData->host_control_hold_status) ==
                HOST_CONTROL_HOLD_PENDING) {
                FW_LOG_INFO_PRINTLN(
                    "[STATE] ignore CALIB_FAILED while host Hold pending");
                break;
            }
            if (sharedData->system_state != SYSTEM_STATE_CALIBRATION_RUNNING) {
                FW_LOG_INFO_PRINTF("[STATE] ignore CALIB_FAILED in state=%u\r\n",
                              (unsigned)sharedData->system_state);
                break;
            }
            sharedData->control_enabled = 0;
            sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
            sharedAtomicStore(&sharedData->calibration_motion_started,
                              (uint8_t)0);
            sharedAtomicStore(&sharedData->calibration_fault_latched, (uint8_t)1);
            setCalibrationStatus(CALIB_STATUS_FAILED);
            requestServoEmergencyTorqueOff(sharedData);
            setSystemState(sharedData, SYSTEM_STATE_UNCONTROLLABLE);
            break;

        default:
            break;
    }
}

bool stateOwnsMotionResources(SystemRuntimeContext* sharedData)
{
    if (!sharedData) {
        return false;
    }
    return sharedData->control_enabled != 0 ||
           sharedData->servo_target_owner != SERVO_TARGET_OWNER_NONE ||
           sharedData->system_state == SYSTEM_STATE_RUN_DIRECT ||
           sharedData->system_state == SYSTEM_STATE_RUN_FEEDBACK ||
           sharedData->system_state == SYSTEM_STATE_CALIBRATION_RUNNING ||
           sharedData->system_state == SYSTEM_STATE_ROPE_INIT_RUNNING;
}
