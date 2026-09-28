/**
 * @file SystemReadinessPolicy.h
 * @brief 集中定义系统准入、反馈 READY 和工作空间迟滞规则。
 *
 * Policy 只读取快照、维护 READY 迟滞内部状态并返回结构化判定；不得迁移
 * system_state、切换 owner、发布舵机目标或打印日志。
 */
#ifndef P4_SAFETY_POLICIES_SYSTEM_READINESS_POLICY_H
#define P4_SAFETY_POLICIES_SYSTEM_READINESS_POLICY_H

#include <stdint.h>

#include "../../runtime/SystemRuntimeContext.h"

namespace p4::state {

enum FeedbackReadyRejectReason : uint8_t {
    FEEDBACK_READY_REJECT_NONE = 0,
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

/** READY 判定及诊断证据；a/b/c/d 的含义由 reason 决定。 */
struct FeedbackReadinessDecision {
    bool ready;
    uint32_t recoverableBelowMask;
    uint8_t reason;
    uint32_t a;
    uint32_t b;
    int32_t c;
    int32_t d;
};

uint32_t requiredMotorMask(SystemRuntimeContext* runtime);
bool requiresCan(SystemRuntimeContext* runtime);
bool requiredServosOnline(SystemRuntimeContext* runtime,
                          const ServoAngleData_t& servoSnapshot);
bool directMotorMaskReady(SystemRuntimeContext* runtime,
                          uint32_t requiredMask);
bool controllableEntryReady(SystemRuntimeContext* runtime);

/** 当前 active joints 是否完整具备 Manual 参考。 */
bool manualReferenceGateReady(SystemRuntimeContext* runtime);

/** 当前 active motors 是否无 Turn Unknown 和硬件圈数溢出。 */
bool activeMotorsMultiTurnKnown(SystemRuntimeContext* runtime);

/** 当前 active motors 是否全部具备有效 ABS/LUT 限位。 */
bool activeMotorsAbsLimitsCalibrated(SystemRuntimeContext* runtime);

/** 执行完整反馈 READY 判定，并更新 24/48 counts、连续三帧迟滞状态。 */
FeedbackReadinessDecision evaluateFeedbackReadiness(
    SystemRuntimeContext* runtime,
    uint32_t nowMs);

/** Rope Init 成功后，将 active joints 登记为已进入当前标定工作空间。 */
void markActiveJointsWorkspaceReady(SystemRuntimeContext* runtime);

} // namespace p4::state

#endif // P4_SAFETY_POLICIES_SYSTEM_READINESS_POLICY_H
