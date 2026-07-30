/**
 * @file StateMachineTask.cpp
 * @brief 运行 P4 总状态机，处理事件、READY 门禁、故障和运动 owner 切换。
 */
#include "StateMachineTask.h"
#include "DirectCommandCoordinator.h"
#include "SystemStatePolicy.h"
#include "../actuator/ActuatorCommandGateway.h"

#include <math.h>
#include <string.h>

#include "../calibration/AutoCalibrationTask.h"
#include "../calibration/CalibrationMath.h"
#include "../calibration/CalibrationTask.h"
#include "../calibration/RopeInitTask.h"
#include "../generated/UrdfJointLimits.generated.h"
#include "../hardware/HardwareMap.h"
#include "../platform/AtomicValue.h"
#include "../persistence/ManualCalibrationNvsStore.h"
#include "../shared/SharedSafetyState.h"
#include "FirmwareLog.h"
#include "SystemTask.h"
#include "TaskRuntimeMonitor.h"

using namespace mmhand::system;

// 通信/反馈新鲜度门限：超过该时间即视为链路离线。
static const uint32_t kStateCanOfflineTimeoutMs = 1000;
static const uint32_t kStateServoFeedbackTimeoutMs = 500;
// READY 下边界迟滞：进入允许低 24 counts，退出必须连续低于 48 counts。
static const int32_t kReadyEnterOutsideToleranceCounts = 24;
static const int32_t kReadyExitOutsideToleranceCounts = 48;
// 连续 3 个新磁编样本满足条件才改变 READY 锁存，避免边界抖动。
static const uint8_t kWorkspaceStableSampleCount = 3;
// 标定结果 ACK 重发次数，降低上位机漏掉瞬时结果的概率。
static const uint8_t kCalibrationAckRepeatCount = 20;

// READY 拒绝日志原因只用于诊断，不进入上下位机协议。
enum FeedbackReadyRejectLogReason : uint8_t {
    FEEDBACK_READY_REJECT_CAN_GATE = 1,
    FEEDBACK_READY_REJECT_MULTITURN = 2,
    FEEDBACK_READY_REJECT_EMPTY_SCOPE = 3,
    FEEDBACK_READY_REJECT_MAPPED_UNAVAILABLE = 4,
    FEEDBACK_READY_REJECT_CALIBRATION_MISSING = 5,
    FEEDBACK_READY_REJECT_MAPPED_INVALID = 6,
    FEEDBACK_READY_REJECT_BELOW_WORKSPACE = 7,
    FEEDBACK_READY_REJECT_CALIBRATION_MISMATCH = 8,
    FEEDBACK_READY_REJECT_ENCODER_CAN = 9,
    FEEDBACK_READY_REJECT_SYSTEM_FAULT = 10,
    FEEDBACK_READY_REJECT_SERVO_TURN_OVERFLOW = 11,
};
static uint32_t collectSystemFaults(TaskSharedData_t* sharedData);
static bool activeJointsInControlWorkspace(TaskSharedData_t* sharedData);
static bool activeJointsInControlWorkspace(TaskSharedData_t* sharedData,
                                           uint32_t* recoverableBelowMask);
static uint32_t readyServoRequiredMask(TaskSharedData_t* sharedData);
static uint8_t idleOrReadyState(TaskSharedData_t* sharedData);

// 每位对应一个关节的 READY 迟滞锁存状态。
static uint32_t s_workspaceReadyLatchMask = 0;
// 只对新 mapped sample 更新连续计数，状态机重复轮询同一帧不累计。
static uint32_t s_workspaceLastMappedTimestamp = 0;
static uint8_t s_workspaceEnterStable[ENCODER_TOTAL_NUM] = {0};
static uint8_t s_workspaceExitStable[ENCODER_TOTAL_NUM] = {0};
// 锁存绑定到当前 Auto/Manual 标定版本，标定变化时立即清除旧 READY。
static uint16_t s_workspaceAutoMinRaw[ENCODER_TOTAL_NUM] = {0};
static uint16_t s_workspaceAutoMaxRaw[ENCODER_TOTAL_NUM] = {0};
static uint32_t s_workspaceManualCrc[ENCODER_TOTAL_NUM] = {0};
static uint8_t s_workspaceMappingVersion[ENCODER_TOTAL_NUM] = {0};

// 反馈和自动标定只要求当前 active scope 的 Manual 数据完整；
// 未选择关节缺失 Manual 不会阻止单指调试或直控。
static bool manualReferenceGateReady(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return false;
    }
    const uint32_t activeMask =
        sharedAtomicLoad(&sharedData->active_joint_mask) &
        kManualEncoderReferenceFullMask;
    const uint32_t validMask =
        sharedAtomicLoad(&sharedData->manual_reference_valid_bitmap) &
        kManualEncoderReferenceFullMask;
    return activeMask != 0 &&
           sharedData->manual_reference_nvs_valid != 0 &&
           (validMask & activeMask) == activeMask &&
           manualEncoderReferencesCoverActiveMask(activeMask);
}

static void logManualReferenceReject(TaskSharedData_t* sharedData,
                                     const char* eventName)
{
    // 同一有效位图 1 秒内只打印一次，避免状态机 100 Hz 刷屏。
    static uint32_t s_lastLogMs = 0;
    static uint32_t s_lastMask = 0;
    const uint32_t nowMs = millis();
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

static void setCalibrationStatus(uint8_t status, bool emitAck)
{
    // 持续状态与瞬时 ACK 分开维护：清到 IDLE 时必须同时停止旧 ACK 重发。
    g_calibrationStatus = status;
    if (status == CALIB_STATUS_IDLE) {
        g_calibrationAckStatus = CALIB_STATUS_IDLE;
        g_calibrationAckRepeatRemaining = 0;
        return;
    }
    if (emitAck) {
        g_calibrationAckStatus = status;
        g_calibrationAckRepeatRemaining = kCalibrationAckRepeatCount;
    }
}

bool postSystemEvent(TaskSharedData_t* sharedData, uint8_t event)
{
    return postSystemEventForToken(sharedData, event, 0);
}

bool requestEmergencyStop(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return false;
    }

    // STOP 独立于普通事件队列锁存，同时立即通知舵机任务。
    // 两个 token 单调递增，即使普通事件队列已满也不会丢失急停。
    sharedAtomicAddFetch(&sharedData->emergency_stop_request_token, (uint32_t)1);
    sharedAtomicAddFetch(&sharedData->servo_emergency_stop_token, (uint32_t)1);
    taskRuntimeMarkStopRequested();
    return true;
}

bool postSystemEventForToken(TaskSharedData_t* sharedData, uint8_t event, uint32_t token)
{
    if (event == SYSTEM_EVENT_STOP) {
        return requestEmergencyStop(sharedData);
    }
    if (!sharedData || !sharedData->stateEventQueue) {
        return false;
    }

    SystemEvent_t msg;
    msg.event = event;
    msg.timestamp = millis();
    msg.token = token;
    return xQueueSend(sharedData->stateEventQueue, &msg, 0) == pdTRUE;
}

static void requestMultiTurnNvsSave(TaskSharedData_t* sharedData, uint8_t reason)
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

static void setSystemState(TaskSharedData_t* sharedData, uint8_t state)
{
    if (!sharedData) {
        return;
    }
    // 统一维护 transition_count，禁止其他模块直接“顺手”改 system_state。
    if (sharedData->system_state != state) {
        sharedData->system_state = state;
        p4::platform::atomicAddFetch(&sharedData->state_transition_count,
                                     (uint32_t)1);
    }
}

static void resetServoTargetQueue(TaskSharedData_t* sharedData)
{
    p4::actuator::clearPending(sharedData);
}

static void setFeedbackAdmissionResult(TaskSharedData_t* sharedData,
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
}

static void requestServoEmergencyTorqueOff(TaskSharedData_t* sharedData)
{
    if (sharedData) {
        sharedAtomicAddFetch(&sharedData->servo_emergency_stop_token, (uint32_t)1);
    }
}

static bool activeMotorsMultiTurnKnown(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return false;
    }
    // 只检查 active scope；未参与本次动作的离线舵机不阻塞单指测试。
    const uint32_t activeMask = readyServoRequiredMask(sharedData);
    return activeMask != 0 &&
           ((sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap) &
             activeMask) == 0) &&
           ((sharedAtomicLoad(&sharedData->servo_turn_overflow_bitmap) &
             activeMask) == 0);
}

static bool activeMotorsAbsLimitsCalibrated(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return false;
    }
    const uint32_t activeMask = readyServoRequiredMask(sharedData);
    if (activeMask == 0) {
        return false;
    }
    const uint32_t validLimitMask =
        sharedAtomicLoad(&sharedData->motor_abs_limit_valid_bitmap);
    const uint32_t missingMask = activeMask & ~validLimitMask;
    if (missingMask == 0) {
        return true;
    }

    static uint32_t s_lastMissingLimitLogMs = 0;
    static uint32_t s_lastMissingLimitMask = 0;
    const uint32_t nowMs = millis();
    if (missingMask != s_lastMissingLimitMask ||
        (uint32_t)(nowMs - s_lastMissingLimitLogMs) > 1000) {
        FW_LOG_INFO_PRINTF("[STATE] feedback ready rejected: uncalibrated ABS limit active=0x%08lX missing=0x%08lX valid=0x%08lX\r\n",
                      (unsigned long)activeMask,
                      (unsigned long)missingMask,
                      (unsigned long)validLimitMask);
        s_lastMissingLimitMask = missingMask;
        s_lastMissingLimitLogMs = nowMs;
    }
    return false;
}

static void logFeedbackReadyReject(TaskSharedData_t* sharedData,
                                   uint8_t reason,
                                   uint32_t a,
                                   uint32_t b,
                                   int32_t c,
                                   int32_t d)
{
    // READY 门禁在 100 Hz 被反复检查，按原因和上下文做 1 秒去重。
    static uint32_t s_lastLogMs = 0;
    static uint8_t s_lastReason = 0;
    static uint32_t s_lastA = 0;
    static uint32_t s_lastB = 0;
    static int32_t s_lastC = 0;
    static int32_t s_lastD = 0;
    const uint32_t nowMs = millis();
    const bool sameReject =
        reason == s_lastReason &&
        (reason == FEEDBACK_READY_REJECT_BELOW_WORKSPACE ||
         (a == s_lastA && b == s_lastB && c == s_lastC && d == s_lastD));
    if (sameReject &&
        (uint32_t)(nowMs - s_lastLogMs) <= 1000) {
        return;
    }
    s_lastReason = reason;
    s_lastA = a;
    s_lastB = b;
    s_lastC = c;
    s_lastD = d;
    s_lastLogMs = nowMs;

    switch (reason) {
        case FEEDBACK_READY_REJECT_CAN_GATE:
            FW_LOG_INFO_PRINTF("[STATE] feedback ready rejected: joint mode/CAN gate inactive motor=0x%08lX joint=0x%08lX mode=%ld require_can=%ld\r\n",
                          (unsigned long)a,
                          (unsigned long)b,
                          (long)c,
                          (long)d);
            break;
        case FEEDBACK_READY_REJECT_MULTITURN:
            FW_LOG_INFO_PRINTF("[STATE] feedback ready rejected: multi-turn unknown active=0x%08lX unknown=0x%08lX\r\n",
                          (unsigned long)a,
                          (unsigned long)b);
            break;
        case FEEDBACK_READY_REJECT_EMPTY_SCOPE:
            FW_LOG_INFO_PRINTF("[STATE] feedback ready rejected: active joint mask is zero motor=0x%08lX\r\n",
                          (unsigned long)a);
            break;
        case FEEDBACK_READY_REJECT_MAPPED_UNAVAILABLE:
            FW_LOG_INFO_PRINTF("[STATE] feedback ready rejected: mapped angle unavailable code=%lu age_ms=%ld\r\n",
                          (unsigned long)a,
                          (long)c);
            break;
        case FEEDBACK_READY_REJECT_CALIBRATION_MISSING:
            FW_LOG_INFO_PRINTF("[STATE] feedback ready rejected: joint calibration missing J%02lu active=0x%08lX\r\n",
                          (unsigned long)a,
                          (unsigned long)b);
            break;
        case FEEDBACK_READY_REJECT_MAPPED_INVALID:
            FW_LOG_INFO_PRINTF("[STATE] feedback ready rejected: mapped angle invalid J%02lu valid_flags=0x%08lX\r\n",
                          (unsigned long)a,
                          (unsigned long)b);
            break;
        case FEEDBACK_READY_REJECT_BELOW_WORKSPACE:
            FW_LOG_INFO_PRINTF("[STATE] feedback ready rejected: joint below workspace J%02lu preserve=%ld outside=%ld tolerance=%ld\r\n",
                          (unsigned long)a,
                          (long)((int32_t)b),
                          (long)c,
                          (long)d);
            break;
        case FEEDBACK_READY_REJECT_CALIBRATION_MISMATCH:
            FW_LOG_INFO_PRINTF("[STATE] feedback ready rejected: calibration/manual mismatch J%02lu mapping=%lu urdf_crc=0x%08lX manual_entry_crc=0x%08lX\r\n",
                          (unsigned long)a,
                          (unsigned long)b,
                          (unsigned long)(uint32_t)c,
                          (unsigned long)(uint32_t)d);
            break;
        case FEEDBACK_READY_REJECT_ENCODER_CAN:
            FW_LOG_INFO_PRINTF("[STATE] feedback ready rejected: encoder/CAN unavailable offline=0x%08lX fault=0x%08lX sensor_fresh=%ld can_link=%ld\r\n",
                          (unsigned long)a,
                          (unsigned long)b,
                          (long)c,
                          (long)d);
            break;
        case FEEDBACK_READY_REJECT_SYSTEM_FAULT:
            FW_LOG_INFO_PRINTF("[STATE] feedback ready rejected: system fault=0x%08lX\r\n",
                          (unsigned long)a);
            break;
        case FEEDBACK_READY_REJECT_SERVO_TURN_OVERFLOW:
            FW_LOG_WARN_PRINTF(
                "[STATE] feedback ready rejected: servo hardware ABS overflow "
                "active=0x%08lX overflow=0x%08lX\r\n",
                (unsigned long)a,
                (unsigned long)b);
            break;
        default:
            FW_LOG_INFO_PRINTLN("[STATE] feedback ready rejected: unknown reason");
            break;
    }
}

static void logMultiTurnUnknownReject(TaskSharedData_t* sharedData, const char* eventName)
{
    if (!sharedData) {
        return;
    }
    FW_LOG_INFO_PRINTF("[STATE] reject %s: multi-turn unknown active=0x%08lX unknown=0x%08lX\r\n",
                  eventName ? eventName : "RUN",
                  (unsigned long)readyServoRequiredMask(sharedData),
                  (unsigned long)sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap));
}

static void enterFaultHold(TaskSharedData_t* sharedData, uint32_t faultBits)
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
    sharedAtomicFetchOr(&sharedData->system_fault_latched_bitmap, faultBits);
    const uint32_t latchedFaults =
        sharedAtomicLoad(&sharedData->system_fault_latched_bitmap);
    sharedAtomicFetchOr(&sharedData->system_fault_bitmap, latchedFaults);
    sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
    sharedData->control_enabled = 0;
    sharedAtomicStore(&sharedData->calibration_motion_started, (uint8_t)0);
    if (interruptedState == SYSTEM_STATE_CALIBRATION_RUNNING) {
        setCalibrationStatus(CALIB_STATUS_FAILED, true);
        sharedAtomicStore(&sharedData->calibration_fault_latched, (uint8_t)1);
    }
    if (interruptedState == SYSTEM_STATE_ROPE_INIT_RUNNING) {
        sharedData->rope_init_status = ROPE_INIT_STATUS_FAILED;
        sharedData->rope_init_reason = ROPE_INIT_REASON_SYSTEM_FAULT;
        sharedData->rope_init_timestamp = millis();
    }
    requestServoEmergencyTorqueOff(sharedData);
    setSystemState(sharedData, SYSTEM_STATE_UNCONTROLLABLE);
}

static void resetControlProtectionFaults(TaskSharedData_t* sharedData)
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

static uint32_t readyServoRequiredMask(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return 0;
    }
    // 标定期间辅助双腱电机也会运动，故必须加入在线/多圈检查。
    uint32_t requiredMask = sharedAtomicLoad(&sharedData->active_motor_mask);
    if (sharedData->system_state == SYSTEM_STATE_CALIBRATION_RUNNING) {
        requiredMask |= sharedAtomicLoad(&sharedData->calibration_aux_motor_mask);
    }
    return requiredMask & validMotorChannelMask();
}

static bool readyRequiresCan(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return false;
    }
    // 纯舵机直控不依赖磁编/CAN；关节反馈、标定和 Rope Init 依赖。
    if (sharedData->control_mode == CONTROL_MODE_DIRECT_MOTOR) {
        return false;
    }
    return sharedAtomicLoad(&sharedData->require_can_feedback) != 0;
}

static bool requiredServosOnline(TaskSharedData_t* sharedData, const ServoAngleData_t& servoSnapshot)
{
    const uint32_t requiredMask = readyServoRequiredMask(sharedData);
    if (requiredMask == 0) {
        static uint32_t s_lastZeroMaskLogMs = 0;
        const uint32_t nowMs = millis();
        if ((uint32_t)(nowMs - s_lastZeroMaskLogMs) > 1000) {
            FW_LOG_INFO_PRINTLN("[STATE] servo ready rejected: active motor mask is zero");
            s_lastZeroMaskLogMs = nowMs;
        }
        return false;
    }

    uint32_t missingMask = 0;
    for (uint8_t i = 0; i < SERVO_TOTAL_NUM; i++) {
        if (((requiredMask >> i) & 0x1UL) != 0 && servoSnapshot.onlineStatus[i] == 0) {
            missingMask |= (1UL << i);
        }
    }
    if (missingMask == 0) {
        return true;
    }

    static uint32_t s_lastMissingServoLogMs = 0;
    static uint32_t s_lastMissingServoMask = 0;
    const uint32_t nowMs = millis();
    if (missingMask != s_lastMissingServoMask ||
        (uint32_t)(nowMs - s_lastMissingServoLogMs) > 1000) {
        FW_LOG_INFO_PRINTF("[STATE] servo offline activeMask=0x%08lX missing=0x%08lX\r\n",
                      (unsigned long)requiredMask,
                      (unsigned long)missingMask);
        s_lastMissingServoMask = missingMask;
        s_lastMissingServoLogMs = nowMs;
    }
    return false;
}

static bool directMotorMaskReady(TaskSharedData_t* sharedData,
                                 uint32_t requiredMask)
{
    if (!sharedData) {
        return false;
    }
    requiredMask &= validMotorChannelMask();
    if (requiredMask == 0) {
        return false;
    }

    ServoAngleData_t servoSnapshot;
    const uint32_t nowMs = millis();
    if (!sharedData->servoFeedbackQueue ||
        xQueuePeek(sharedData->servoFeedbackQueue, &servoSnapshot, 0) != pdTRUE ||
        (uint32_t)(nowMs - servoSnapshot.timestamp) > kStateServoFeedbackTimeoutMs) {
        return false;
    }
    for (uint8_t motor = 0; motor < SERVO_TOTAL_NUM; motor++) {
        const uint32_t bit = (1UL << motor);
        if ((requiredMask & bit) != 0 && servoSnapshot.onlineStatus[motor] == 0) {
            return false;
        }
    }
    if ((sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap) & requiredMask) != 0) {
        return false;
    }
    if ((sharedAtomicLoad(&sharedData->servo_turn_overflow_bitmap) &
         requiredMask) != 0) {
        return false;
    }

    // 新直控命令已经按 pending scope 独立检查舵机在线、CAN 依赖和多圈可信；
    // 此处忽略旧 scope 遗留的 CAN/SERVO/MULTITURN 位，只保留与 scope
    // 无关的过载、标定等全局故障。
    const uint32_t scopeDependentFaults =
        SYSTEM_FAULT_CAN_OFFLINE |
        SYSTEM_FAULT_SERVO_OFFLINE |
        SYSTEM_FAULT_MULTITURN_UNKNOWN;
    const uint32_t blockingFaults =
        (sharedAtomicLoad(&sharedData->system_fault_bitmap) |
         sharedAtomicLoad(&sharedData->system_fault_latched_bitmap)) &
        ~scopeDependentFaults;
    return blockingFaults == 0;
}

static bool isControllableEntryReady(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return false;
    }

    // IDLE 只表示直控硬件可用。Manual/Auto/CAN 属于反馈 READY 门禁，
    // 不应阻止设备恢复或纯舵机直控。
    return directMotorMaskReady(sharedData, readyServoRequiredMask(sharedData));
}

// 反馈 READY 的总门禁：
// 1) 当前 active joints 必须有 Manual、Auto v3、有效磁编映射和舵机 ABS 限位；
// 2) 磁编/CAN/舵机反馈必须在线且新鲜，多圈状态必须可信；
// 3) READY 只由工作空间下边界决定：低于 Auto Min 时进入 RECOVERY_REQUIRED，
//    上边界越界只由控制层限制目标并记录警告，不在这里强制退出 READY；
// 4) 24/48 counts 迟滞和连续三帧确认用于消除边界抖动。
static bool activeJointsInControlWorkspace(TaskSharedData_t* sharedData,
                                           uint32_t* recoverableBelowMask)
{
    if (recoverableBelowMask) {
        *recoverableBelowMask = 0;
    }
    if (!sharedData) {
        return false;
    }
    if (!manualReferenceGateReady(sharedData)) {
        logManualReferenceReject(sharedData, "READY");
        return false;
    }
    if (!readyRequiresCan(sharedData)) {
        logFeedbackReadyReject(sharedData,
                               FEEDBACK_READY_REJECT_CAN_GATE,
                               sharedAtomicLoad(&sharedData->active_motor_mask),
                               sharedAtomicLoad(&sharedData->active_joint_mask),
                               sharedData->control_mode,
                               sharedAtomicLoad(&sharedData->require_can_feedback));
        return false;
    }
    if (!activeMotorsMultiTurnKnown(sharedData)) {
        const uint32_t activeMotorMask =
            readyServoRequiredMask(sharedData);
        const uint32_t overflowMask =
            sharedAtomicLoad(&sharedData->servo_turn_overflow_bitmap) &
            activeMotorMask;
        if (overflowMask != 0) {
            logFeedbackReadyReject(
                sharedData,
                FEEDBACK_READY_REJECT_SERVO_TURN_OVERFLOW,
                activeMotorMask,
                overflowMask,
                0,
                0);
            return false;
        }
        logFeedbackReadyReject(sharedData,
                               FEEDBACK_READY_REJECT_MULTITURN,
                               activeMotorMask,
                               sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap),
                               0,
                               0);
        return false;
    }
    if (!activeMotorsAbsLimitsCalibrated(sharedData)) {
        return false;
    }

    const uint32_t activeMask =
        sharedAtomicLoad(&sharedData->active_joint_mask) &
        ((1UL << ENCODER_TOTAL_NUM) - 1UL);
    if (activeMask == 0) {
        logFeedbackReadyReject(sharedData,
                               FEEDBACK_READY_REJECT_EMPTY_SCOPE,
                               sharedAtomicLoad(&sharedData->active_motor_mask),
                               0,
                               0,
                               0);
        return false;
    }

    RemoteSensorData_t sensorData;
    const uint32_t encoderOnlineMask =
        sharedAtomicLoad(&sharedData->encoder_online_bitmap);
    const uint32_t encoderFaultMask =
        sharedAtomicLoad(&sharedData->encoder_fault_bitmap);
    const uint8_t canLinkOnline =
        sharedAtomicLoad(&sharedData->can_link_online);
    const uint32_t nowMs = millis();
    const bool sensorFresh =
        sharedData->canRxQueue &&
        xQueuePeek(sharedData->canRxQueue, &sensorData, 0) == pdTRUE &&
        sensorData.isValid &&
        (uint32_t)(nowMs - sensorData.timestamp) <= kStateCanOfflineTimeoutMs;
    if (!sensorFresh || canLinkOnline == 0 ||
        (activeMask & ~encoderOnlineMask) != 0 ||
        (activeMask & encoderFaultMask) != 0) {
        logFeedbackReadyReject(sharedData,
                               FEEDBACK_READY_REJECT_ENCODER_CAN,
                               activeMask & ~encoderOnlineMask,
                               activeMask & encoderFaultMask,
                               sensorFresh ? 1 : 0,
                               canLinkOnline);
        return false;
    }
    const uint32_t feedbackFaults = collectSystemFaults(sharedData);
    if (feedbackFaults != 0) {
        logFeedbackReadyReject(sharedData, FEEDBACK_READY_REJECT_SYSTEM_FAULT,
                               feedbackFaults, 0, 0, 0);
        return false;
    }

    MappedAngleData_t mappedData;
    if (!sharedData->mappedAngleQueue) {
        logFeedbackReadyReject(sharedData, FEEDBACK_READY_REJECT_MAPPED_UNAVAILABLE,
                               0, 0, 0, 0);
        return false;
    }
    if (xQueuePeek(sharedData->mappedAngleQueue, &mappedData, 0) != pdTRUE) {
        logFeedbackReadyReject(sharedData, FEEDBACK_READY_REJECT_MAPPED_UNAVAILABLE,
                               1, 0, 0, 0);
        return false;
    }
    const uint32_t mappedAgeMs = nowMs - mappedData.timestamp;
    if (!mappedData.isValid) {
        logFeedbackReadyReject(sharedData, FEEDBACK_READY_REJECT_MAPPED_UNAVAILABLE,
                               2, 0, (int32_t)mappedAgeMs, 0);
        return false;
    }
    if (mappedAgeMs > kStateCanOfflineTimeoutMs) {
        logFeedbackReadyReject(sharedData, FEEDBACK_READY_REJECT_MAPPED_UNAVAILABLE,
                               3, 0, (int32_t)mappedAgeMs, 0);
        return false;
    }

    const bool isNewMappedSample =
        mappedData.timestamp != s_workspaceLastMappedTimestamp;
    if (isNewMappedSample) {
        s_workspaceLastMappedTimestamp = mappedData.timestamp;
    }
    bool allWorkspaceReady = true;

    for (uint8_t calibJoint = 0; calibJoint < ENCODER_TOTAL_NUM; calibJoint++) {
        if ((activeMask & (1UL << calibJoint)) == 0) {
            continue;
        }
        const JointCalibrationResult& calib = g_jointCalibResult[calibJoint];
        if (!calib.success) {
            logFeedbackReadyReject(sharedData, FEEDBACK_READY_REJECT_CALIBRATION_MISSING,
                                   calibJoint, activeMask, 0, 0);
            return false;
        }
        const uint32_t manualEntryCrc =
            manualEncoderReferenceEntryCrc(
                calibJoint, g_manualEncoderReference[calibJoint]);
        if (calib.mappingVersion != kManualScaledUrdfFeedbackMappingVersion ||
            calib.urdfConfigCrc != kManualScaledUrdfConfigCrc ||
            calib.manualReferenceCrc != manualEntryCrc) {
            logFeedbackReadyReject(sharedData,
                                   FEEDBACK_READY_REJECT_CALIBRATION_MISMATCH,
                                   calibJoint,
                                   calib.mappingVersion,
                                   (int32_t)calib.urdfConfigCrc,
                                   (int32_t)manualEntryCrc);
            return false;
        }
        if (mappedData.validFlags[calibJoint] == 0) {
            uint32_t validMask = 0;
            for (uint8_t i = 0; i < ENCODER_TOTAL_NUM; i++) {
                if (mappedData.validFlags[i] != 0) {
                    validMask |= (1UL << i);
                }
            }
            logFeedbackReadyReject(sharedData, FEEDBACK_READY_REJECT_MAPPED_INVALID,
                                   calibJoint, validMask, 0, 0);
            return false;
        }

        const uint32_t jointBit = 1UL << calibJoint;
        const bool calibrationChanged =
            s_workspaceAutoMinRaw[calibJoint] != calib.autoMinRaw ||
            s_workspaceAutoMaxRaw[calibJoint] != calib.autoMaxRaw ||
            s_workspaceManualCrc[calibJoint] != calib.manualReferenceCrc ||
            s_workspaceMappingVersion[calibJoint] != calib.mappingVersion;
        if (calibrationChanged) {
            // Auto 端点、Manual CRC 或映射版本任一变化，旧 READY 锁存立即失效。
            s_workspaceReadyLatchMask &= ~jointBit;
            s_workspaceEnterStable[calibJoint] = 0;
            s_workspaceExitStable[calibJoint] = 0;
            s_workspaceAutoMinRaw[calibJoint] = calib.autoMinRaw;
            s_workspaceAutoMaxRaw[calibJoint] = calib.autoMaxRaw;
            s_workspaceManualCrc[calibJoint] = calib.manualReferenceCrc;
            s_workspaceMappingVersion[calibJoint] = calib.mappingVersion;
        }

        const int32_t outsideCounts = mappedData.outsideCounts[calibJoint];
        if (isNewMappedSample) {
            if ((s_workspaceReadyLatchMask & jointBit) != 0) {
                // 已 READY：只有连续三帧低于 -48 counts 才退出。
                s_workspaceEnterStable[calibJoint] = 0;
                if (outsideCounts < -kReadyExitOutsideToleranceCounts) {
                    if (s_workspaceExitStable[calibJoint] < 255) {
                        s_workspaceExitStable[calibJoint]++;
                    }
                    if (s_workspaceExitStable[calibJoint] >=
                        kWorkspaceStableSampleCount) {
                        s_workspaceReadyLatchMask &= ~jointBit;
                    }
                } else {
                    s_workspaceExitStable[calibJoint] = 0;
                }
            } else {
                // 未 READY：连续三帧回到 -24 counts 以内才进入。
                s_workspaceExitStable[calibJoint] = 0;
                if (outsideCounts >= -kReadyEnterOutsideToleranceCounts) {
                    if (s_workspaceEnterStable[calibJoint] < 255) {
                        s_workspaceEnterStable[calibJoint]++;
                    }
                    if (s_workspaceEnterStable[calibJoint] >=
                        kWorkspaceStableSampleCount) {
                        s_workspaceReadyLatchMask |= jointBit;
                    }
                } else {
                    s_workspaceEnterStable[calibJoint] = 0;
                }
            }
        }

        if ((s_workspaceReadyLatchMask & jointBit) == 0) {
            allWorkspaceReady = false;
            if (outsideCounts < -kReadyEnterOutsideToleranceCounts &&
                recoverableBelowMask) {
                *recoverableBelowMask |= jointBit;
            }
            if (outsideCounts < -kReadyEnterOutsideToleranceCounts) {
                logFeedbackReadyReject(sharedData,
                                       FEEDBACK_READY_REJECT_BELOW_WORKSPACE,
                                       calibJoint,
                                       0,
                                       outsideCounts,
                                       kReadyEnterOutsideToleranceCounts);
            }
        }
    }

    return allWorkspaceReady;
}

static bool activeJointsInControlWorkspace(TaskSharedData_t* sharedData)
{
    return activeJointsInControlWorkspace(sharedData, NULL);
}

static void markActiveJointsWorkspaceReady(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return;
    }
    // Rope Init 已实际把关节送回工作空间，直接建立与当前标定绑定的 READY 锁存，
    // 避免等待后续三帧时 UI 短暂回到 IDLE。
    const uint32_t activeMask =
        sharedAtomicLoad(&sharedData->active_joint_mask) &
        kManualEncoderReferenceFullMask;
    for (uint8_t joint = 0; joint < ENCODER_TOTAL_NUM; joint++) {
        const uint32_t jointBit = 1UL << joint;
        if ((activeMask & jointBit) == 0) {
            continue;
        }
        const JointCalibrationResult& calib = g_jointCalibResult[joint];
        s_workspaceAutoMinRaw[joint] = calib.autoMinRaw;
        s_workspaceAutoMaxRaw[joint] = calib.autoMaxRaw;
        s_workspaceManualCrc[joint] = calib.manualReferenceCrc;
        s_workspaceMappingVersion[joint] = calib.mappingVersion;
        s_workspaceEnterStable[joint] = kWorkspaceStableSampleCount;
        s_workspaceExitStable[joint] = 0;
        s_workspaceReadyLatchMask |= jointBit;
    }
}

static bool feedbackTargetsPointIntoWorkspace(TaskSharedData_t* sharedData)
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

    // 自动恢复只允许“向工作空间内部”运动，禁止目标仍低于动态 currentMin。
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

static bool beginRopeInitMotion(TaskSharedData_t* sharedData,
                                bool feedbackRecovery,
                                uint32_t feedbackRequestSequence)
{
    if (!sharedData) {
        return false;
    }
    // Rope Init 临时借用 CALIBRATION 输出所有权，但 system_state 使用独立状态，
    // 便于 UI、故障处理和完成事件区分自动标定。
    p4::platform::atomicAddFetch(&sharedData->rope_init_sequence,
                                 (uint32_t)1);
    sharedData->rope_init_joint_mask =
        sharedAtomicLoad(&sharedData->active_joint_mask) &
        kManualEncoderReferenceFullMask;
    sharedData->rope_init_timestamp = millis();
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
    if (startRopeInitTask(sharedData)) {
        return true;
    }

    sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
    sharedData->rope_init_status = ROPE_INIT_STATUS_FAILED;
    sharedData->rope_init_reason = ROPE_INIT_REASON_TASK_START_FAILED;
    sharedData->rope_init_timestamp = millis();
    sharedAtomicStore(&sharedData->feedback_recovery_pending, (uint8_t)0);
    sharedAtomicStore(&sharedData->feedback_recovery_request_sequence,
                      (uint32_t)0);
    setSystemState(sharedData, idleOrReadyState(sharedData));
    return false;
}

static uint8_t idleOrReadyState(TaskSharedData_t* sharedData)
{
    // 状态优先级：硬件不可控 > 工作空间已就绪 > 可通过 Rope Init 恢复 > 普通 IDLE。
    if (!isControllableEntryReady(sharedData)) {
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

static void updateReadyStateIfIdle(TaskSharedData_t* sharedData)
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

static bool feedbackCompletionMatchesCurrentCommand(TaskSharedData_t* sharedData,
                                                    const SystemEvent_t& event)
{
    // token=0 保留给不绑定代次的内部完成事件；正常控制使用 joint_command_token。
    return sharedData &&
           (event.token == 0 || event.token == sharedData->joint_command_token);
}

static void releaseFeedbackRun(TaskSharedData_t* sharedData)
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

static void handleSystemEvent(TaskSharedData_t* sharedData, const SystemEvent_t& event)
{
    if (!sharedData) {
        return;
    }

    // 所有状态迁移集中在此处；事件处理必须先做门禁，再授予输出所有权。
    switch (event.event)
    {
        case SYSTEM_EVENT_START:
            if (!p4::system::canAcceptStart(sharedData->system_state)) {
                FW_LOG_INFO_PRINTF("[STATE] reject START in state=%u\r\n",
                              (unsigned)sharedData->system_state);
                break;
            }

            if (sharedData->system_state == SYSTEM_STATE_PAUSED) {
                const uint8_t resumeState = sharedData->paused_resume_state;
                const bool resumeReady =
                    (resumeState == SYSTEM_STATE_RUN_DIRECT)
                        ? isControllableEntryReady(sharedData)
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
            if (sharedData->stateEventQueue) {
                xQueueReset(sharedData->stateEventQueue);
            }
            clearPendingDirectCommand(sharedData, DIRECT_REJECT_STATE);
            sharedAtomicStore(&sharedData->feedback_recovery_pending,
                              (uint8_t)0);
            sharedAtomicStore(&sharedData->feedback_recovery_request_sequence,
                              (uint32_t)0);
            resetServoTargetQueue(sharedData);
            sharedData->servo_target_owner = SERVO_TARGET_OWNER_NONE;
            sharedData->control_enabled = 0;
            if (sharedData->system_state == SYSTEM_STATE_ROPE_INIT_RUNNING) {
                sharedData->rope_init_status = ROPE_INIT_STATUS_FAILED;
                sharedData->rope_init_reason = ROPE_INIT_REASON_CANCELLED;
                sharedData->rope_init_timestamp = millis();
            }
            if (sharedData->system_state == SYSTEM_STATE_CALIBRATION_RUNNING) {
                setCalibrationStatus(CALIB_STATUS_IDLE, false);
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
            resetControlProtectionFaults(sharedData);
            sharedAtomicStore(&sharedData->calibration_fault_latched, (uint8_t)0);
            sharedAtomicStore(&sharedData->system_fault_latched_bitmap, (uint32_t)0);
            sharedAtomicStore(&sharedData->system_fault_bitmap, (uint32_t)0);
            setCalibrationStatus(CALIB_STATUS_IDLE, false);
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
            if (!p4::system::canAcceptRunDirect(sharedData->system_state)) {
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
                if (!directMotorMaskReady(sharedData, pendingMotorMask)) {
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
            // ControlTask 会在下一周期消费更新后的 joint_command_token。
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
                        ropeInitActiveScopeRejectReason(sharedData);
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
            if (!manualReferenceGateReady(sharedData)) {
                logManualReferenceReject(sharedData, "CALIBRATE");
                setCalibrationStatus(CALIB_STATUS_FAILED, true);
                break;
            }
            if (!p4::system::canAcceptCalibration(sharedData->system_state)) {
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
            setCalibrationStatus(CALIB_STATUS_RUNNING, true);
            setSystemState(sharedData, SYSTEM_STATE_CALIBRATION_RUNNING);
            if (!startAutoCalibrationTask(sharedData)) {
                CalibrationDiagData_t diag;
                memset(&diag, 0, sizeof(diag));
                diag.valid = 1;
                diag.reason = CALIB_DIAG_REASON_TASK_START_FAILED;
                diag.phase = CALIB_DIAG_PHASE_SEQUENCE;
                diag.joint = 0xFF;
                diag.motor = 0xFF;
                diag.timestamp = millis();
                {
                    SharedSafetyGuard guard;
                    diag.sequence = sharedData->calibration_diag.sequence + 1;
                    sharedData->calibration_diag = diag;
                }
                setCalibrationStatus(CALIB_STATUS_FAILED, true);
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
            const uint8_t preflightReason =
                ropeInitActiveScopeRejectReason(sharedData);
            if (preflightReason != ROPE_INIT_REASON_NONE) {
                sharedData->rope_init_status = ROPE_INIT_STATUS_FAILED;
                sharedData->rope_init_reason = preflightReason;
                sharedData->rope_init_timestamp = millis();
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
                sharedData->rope_init_timestamp = millis();
                break;
            }
            if (!activeMotorsMultiTurnKnown(sharedData)) {
                logMultiTurnUnknownReject(sharedData, "ROPE_INIT");
                sharedData->rope_init_status = ROPE_INIT_STATUS_FAILED;
                sharedData->rope_init_reason = ROPE_INIT_REASON_MULTITURN_UNKNOWN;
                sharedData->rope_init_timestamp = millis();
                break;
            }
            beginRopeInitMotion(sharedData, false, 0);
            break;
        }

        case SYSTEM_EVENT_ROPE_INIT_DONE:
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
                sharedData->rope_init_timestamp = millis();
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
                sharedData->rope_init_timestamp = millis();
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
                    : CALIB_STATUS_SUCCESS,
                true);
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
            setCalibrationStatus(CALIB_STATUS_FAILED, true);
            requestServoEmergencyTorqueOff(sharedData);
            setSystemState(sharedData, SYSTEM_STATE_UNCONTROLLABLE);
            break;

        default:
            break;
    }
}

static uint32_t collectSystemFaults(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return 0;
    }

    // 只汇总“当前观测”和已经锁存的故障，不在本函数执行状态迁移。
    const uint32_t nowMs = millis();
    uint32_t faults = 0;

    uint32_t activeJointMask = 0;
    uint32_t encoderOnlineMask = 0;
    uint32_t encoderFaultMask = 0;
    uint8_t canLinkOnline = 0;
    {
        SharedSafetyGuard guard;
        activeJointMask = sharedAtomicLoad(&sharedData->active_joint_mask) &
                          ((1UL << ENCODER_TOTAL_NUM) - 1UL);
        encoderOnlineMask = sharedAtomicLoad(&sharedData->encoder_online_bitmap);
        encoderFaultMask = sharedAtomicLoad(&sharedData->encoder_fault_bitmap);
        canLinkOnline = sharedAtomicLoad(&sharedData->can_link_online);
    }

    RemoteSensorData_t sensorData;
    const bool motionRequiresCan =
        readyRequiresCan(sharedData) ||
        sharedData->system_state == SYSTEM_STATE_ROPE_INIT_RUNNING;
    if (motionRequiresCan &&
        (xQueuePeek(sharedData->canRxQueue, &sensorData, 0) != pdTRUE ||
        !sensorData.isValid ||
        (uint32_t)(nowMs - sensorData.timestamp) > kStateCanOfflineTimeoutMs ||
        canLinkOnline == 0 ||
        ((activeJointMask & ~encoderOnlineMask) != 0) ||
        ((activeJointMask & encoderFaultMask) != 0))) {
        faults |= SYSTEM_FAULT_CAN_OFFLINE;
    }

    ServoAngleData_t servoSnapshot;
    const bool servoFeedbackFresh =
        sharedData->servoFeedbackQueue &&
        xQueuePeek(sharedData->servoFeedbackQueue, &servoSnapshot, 0) == pdTRUE &&
        (uint32_t)(nowMs - servoSnapshot.timestamp) <= kStateServoFeedbackTimeoutMs;
    if (servoFeedbackFresh) {
        if (!requiredServosOnline(sharedData, servoSnapshot)) {
            faults |= SYSTEM_FAULT_SERVO_OFFLINE;
        }
    } else {
        faults |= SYSTEM_FAULT_SERVO_OFFLINE;
    }

    // 过载和标定失败由产生故障的任务锁存，状态机只负责提升为系统故障。
    if (sharedAtomicLoad(&sharedData->overload_fault_bitmap) != 0) {
        faults |= SYSTEM_FAULT_OVERLOAD;
    }

    if (sharedAtomicLoad(&sharedData->calibration_fault_latched) != 0) {
        faults |= SYSTEM_FAULT_CALIBRATION;
    }
    if ((sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap) &
         readyServoRequiredMask(sharedData)) != 0) {
        faults |= SYSTEM_FAULT_MULTITURN_UNKNOWN;
    }
    faults |= sharedAtomicLoad(&sharedData->system_fault_latched_bitmap);

    return faults;
}

static uint32_t blockingFaultsForState(TaskSharedData_t* sharedData,
                                       uint8_t state,
                                       uint32_t faults)
{
    // 不同运动态依赖不同传感器：直控忽略 CAN；自动标定允许重建多圈参考。
    switch (state) {
        case SYSTEM_STATE_RUN_DIRECT:
            return faults & ~SYSTEM_FAULT_CAN_OFFLINE;
        case SYSTEM_STATE_RUN_FEEDBACK:
        case SYSTEM_STATE_ROPE_INIT_RUNNING:
            return faults;
        case SYSTEM_STATE_CALIBRATION_RUNNING:
            // 自动标定会主动重建多圈参考，因此标定期间出现 Turn Unknown
            // 不能作为终止本次标定的条件。
            // 预检尚未完成时也暂时忽略舵机离线，让通信任务有时间完成重连和多圈模式配置；
            // 一旦真正开始运动，舵机离线立即恢复为阻断故障。
            if (sharedAtomicLoad(
                    &sharedData->calibration_motion_started) == 0) {
                return faults &
                       ~(SYSTEM_FAULT_MULTITURN_UNKNOWN |
                         SYSTEM_FAULT_SERVO_OFFLINE);
            }
            return faults & ~SYSTEM_FAULT_MULTITURN_UNKNOWN;
        default:
            return 0;
    }
}

static bool stateOwnsMotionResources(TaskSharedData_t* sharedData)
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

// 系统顶层状态机，周期 10 ms（100 Hz）：
// 1) 优先消费不可丢失的 E-STOP token；
// 2) 顺序处理普通状态事件；
// 3) 汇总 CAN、舵机、多圈、过载和标定故障；
// 4) 运行态遇到阻塞故障立即撤销所有权并进入 UNCONTROLLABLE；
// 5) 无任务占用输出时持续重评估 UNCONTROLLABLE/IDLE/RECOVERY_REQUIRED/READY。
void stateMachineTask(void* parameter)
{
    TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;
    if (!sharedData) {
        vTaskDelete(NULL);
        return;
    }

    setSystemState(sharedData, SYSTEM_STATE_UNCONTROLLABLE);

    uint32_t handledEmergencyStopToken = 0;

    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t periodTicks = pdMS_TO_TICKS(10);

    for (;;)
    {
        taskRuntimeMarkLoopStart(TASK_RUNTIME_STATE_MACHINE);
        // E-STOP token 比普通队列优先，且只处理每个新 token 一次。
        const uint32_t emergencyStopToken =
            sharedAtomicLoad(&sharedData->emergency_stop_request_token);
        if (emergencyStopToken != handledEmergencyStopToken) {
            handledEmergencyStopToken = emergencyStopToken;
            SystemEvent_t stopEvent;
            stopEvent.event = SYSTEM_EVENT_STOP;
            stopEvent.timestamp = millis();
            stopEvent.token = emergencyStopToken;
            handleSystemEvent(sharedData, stopEvent);
        }

        SystemEvent_t event;
        // 单周期清空当前事件队列，保持状态切换延迟低于一个 10 ms 周期。
        while (xQueueReceive(sharedData->stateEventQueue, &event, 0) == pdTRUE) {
            handleSystemEvent(sharedData, event);
        }

        const uint32_t faults = collectSystemFaults(sharedData);
        sharedAtomicStore(&sharedData->system_fault_bitmap, faults);

        const uint32_t blockingFaults =
            blockingFaultsForState(sharedData,
                                   sharedData->system_state,
                                   faults);
        if (stateOwnsMotionResources(sharedData) && blockingFaults != 0) {
            enterFaultHold(sharedData, blockingFaults);
        } else if (!stateOwnsMotionResources(sharedData)) {
            // 空闲态不锁死 READY；新反馈、active scope 或标定变化会自动更新状态。
            updateReadyStateIfIdle(sharedData);
        }

        taskRuntimeMarkLoopEnd(TASK_RUNTIME_STATE_MACHINE);
        const TickType_t nextWakeTime = lastWakeTime + periodTicks;
        const bool periodOverrun = (int32_t)(xTaskGetTickCount() - nextWakeTime) >= 0;
        taskRuntimeMarkPeriodResult(TASK_RUNTIME_STATE_MACHINE, periodOverrun);
        vTaskDelayUntil(&lastWakeTime, periodTicks);
        if (periodOverrun) vTaskDelay(1);
    }
}
