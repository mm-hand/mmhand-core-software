/**
 * @file SafetyRuleRegistry.cpp
 * @brief 将业务枚举、拒绝原因和故障位映射为稳定的安全规则 ID。
 * @details 当前仅承载确实需要枚举转换的映射；完整规则覆盖由生成索引和架构门禁验证。
 */
#include "SafetyRuleRegistry.h"
#include "policies/SystemReadinessPolicy.h"

namespace p4::safety {

SafetyRuleId ruleForServoRejectReason(ServoRejectReason reason)
{
    switch (reason) {
        case ServoRejectReason::None: return SafetyRuleId::None;
        case ServoRejectReason::InvalidCommand: return SafetyRuleId::ActuatorInvalidCommand;
        case ServoRejectReason::RouteUnknown: return SafetyRuleId::ActuatorRouteUnknown;
        case ServoRejectReason::RouteConflict: return SafetyRuleId::ActuatorRouteConflict;
        case ServoRejectReason::BusOverCapacity: return SafetyRuleId::ActuatorBusOverCapacity;
        case ServoRejectReason::EmergencyStop: return SafetyRuleId::ActuatorEmergencyStop;
        case ServoRejectReason::InvalidOwner: return SafetyRuleId::ActuatorInvalidOwner;
        case ServoRejectReason::InvalidState: return SafetyRuleId::ActuatorInvalidState;
        case ServoRejectReason::Offline: return SafetyRuleId::ActuatorOffline;
        case ServoRejectReason::MultiTurnUnknown: return SafetyRuleId::ActuatorMultiTurnUnknown;
        case ServoRejectReason::StaleTarget: return SafetyRuleId::ActuatorStaleTarget;
        case ServoRejectReason::LogicalRange: return SafetyRuleId::ActuatorLogicalRange;
        case ServoRejectReason::HardwareRange: return SafetyRuleId::ActuatorHardwareRange;
        case ServoRejectReason::TurnOverflow: return SafetyRuleId::ActuatorTurnOverflow;
        default: return SafetyRuleId::None;
    }
}

SafetyRuleId ruleForDirectRejectReason(uint8_t reason)
{
    switch (reason) {
        case DIRECT_REJECT_NONE: return SafetyRuleId::None;
        case DIRECT_REJECT_STATE: return SafetyRuleId::DirectStateRejected;
        case DIRECT_REJECT_HARDWARE: return SafetyRuleId::DirectHardwareRejected;
        case DIRECT_REJECT_MULTITURN: return SafetyRuleId::DirectMultiTurnRejected;
        case DIRECT_REJECT_EVENT_QUEUE: return SafetyRuleId::DirectEventQueueRejected;
        case DIRECT_REJECT_STALE: return SafetyRuleId::DirectStaleRejected;
        case DIRECT_REJECT_FAULT: return SafetyRuleId::DirectFaultRejected;
        default: return SafetyRuleId::None;
    }
}

SafetyRuleId ruleForFeedbackRejectReason(uint8_t reason)
{
    switch (reason) {
        case FEEDBACK_REJECT_NONE: return SafetyRuleId::None;
        case FEEDBACK_REJECT_MANUAL_REFERENCE: return SafetyRuleId::FeedbackManualMissing;
        case FEEDBACK_REJECT_STATE: return SafetyRuleId::FeedbackStateRejected;
        case FEEDBACK_REJECT_WORKSPACE: return SafetyRuleId::FeedbackWorkspaceRejected;
        case FEEDBACK_REJECT_MULTITURN: return SafetyRuleId::FeedbackMultiTurnRejected;
        case FEEDBACK_REJECT_ABS_LIMIT: return SafetyRuleId::FeedbackAbsLimitRejected;
        case FEEDBACK_REJECT_MODE_SWITCH: return SafetyRuleId::FeedbackModeSwitchRejected;
        case FEEDBACK_REJECT_EVENT_QUEUE: return SafetyRuleId::FeedbackEventQueueRejected;
        default: return SafetyRuleId::None;
    }
}

SafetyRuleId ruleForFeedbackReadyRejectReason(uint8_t reason)
{
    switch (reason) {
        case p4::state::FEEDBACK_READY_REJECT_CAN_GATE: return SafetyRuleId::ReadyCanGate;
        case p4::state::FEEDBACK_READY_REJECT_MULTITURN: return SafetyRuleId::ReadyMultiTurnGate;
        case p4::state::FEEDBACK_READY_REJECT_EMPTY_SCOPE: return SafetyRuleId::ReadyEmptyScope;
        case p4::state::FEEDBACK_READY_REJECT_MAPPED_UNAVAILABLE: return SafetyRuleId::ReadyMappedUnavailable;
        case p4::state::FEEDBACK_READY_REJECT_CALIBRATION_MISSING: return SafetyRuleId::ReadyCalibrationMissing;
        case p4::state::FEEDBACK_READY_REJECT_MAPPED_INVALID: return SafetyRuleId::ReadyMappedInvalid;
        case p4::state::FEEDBACK_READY_REJECT_BELOW_WORKSPACE: return SafetyRuleId::ReadyBelowWorkspace;
        case p4::state::FEEDBACK_READY_REJECT_CALIBRATION_MISMATCH: return SafetyRuleId::ReadyCalibrationMismatch;
        case p4::state::FEEDBACK_READY_REJECT_ENCODER_CAN: return SafetyRuleId::ReadyEncoderCanFault;
        case p4::state::FEEDBACK_READY_REJECT_SYSTEM_FAULT: return SafetyRuleId::ReadySystemFault;
        case p4::state::FEEDBACK_READY_REJECT_SERVO_TURN_OVERFLOW: return SafetyRuleId::ReadyTurnOverflow;
        default: return SafetyRuleId::None;
    }
}

SafetyRuleId ruleForRopeInitReason(uint8_t reason)
{
    switch (reason) {
        case ROPE_INIT_REASON_NONE: return SafetyRuleId::None;
        case ROPE_INIT_REASON_TIMEOUT: return SafetyRuleId::RopeInitTimeout;
        case ROPE_INIT_REASON_FEEDBACK_STALE: return SafetyRuleId::RopeInitFeedbackStale;
        case ROPE_INIT_REASON_SERVO_OFFLINE: return SafetyRuleId::RopeInitServoOffline;
        case ROPE_INIT_REASON_MULTITURN_UNKNOWN: return SafetyRuleId::RopeInitMultiTurnUnknown;
        case ROPE_INIT_REASON_MANUAL_REQUIRED: return SafetyRuleId::RopeInitManualRequired;
        case ROPE_INIT_REASON_AUTO_REQUIRED: return SafetyRuleId::RopeInitAutoRequired;
        case ROPE_INIT_REASON_TARGET_PUBLISH_FAILED: return SafetyRuleId::RopeInitTargetPublishFailed;
        case ROPE_INIT_REASON_SYSTEM_FAULT: return SafetyRuleId::RopeInitSystemFault;
        case ROPE_INIT_REASON_CANCELLED: return SafetyRuleId::RopeInitCancelled;
        default: return SafetyRuleId::None;
    }
}

SafetyRuleId ruleForCalibrationDiagReason(uint8_t reason)
{
    switch (reason) {
        case CALIB_DIAG_REASON_NONE: return SafetyRuleId::None;
        case CALIB_DIAG_REASON_ACTIVE_SCOPE_EMPTY: return SafetyRuleId::CalibrationActiveScopeEmpty;
        case CALIB_DIAG_REASON_ENCODER_ERROR: return SafetyRuleId::CalibrationEncoderError;
        case CALIB_DIAG_REASON_SERVO_OFFLINE: return SafetyRuleId::CalibrationServoOffline;
        case CALIB_DIAG_REASON_OVERLOAD: return SafetyRuleId::CalibrationOverload;
        case CALIB_DIAG_REASON_ZERO_CROSS_INVALID: return SafetyRuleId::CalibrationZeroCrossInvalid;
        case CALIB_DIAG_REASON_WRAP_TOO_LARGE: return SafetyRuleId::CalibrationWrapTooLarge;
        case CALIB_DIAG_REASON_ABS_LIMIT_INVALID: return SafetyRuleId::CalibrationAbsLimitInvalid;
        case CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED: return SafetyRuleId::CalibrationTargetPublishFailed;
        case CALIB_DIAG_REASON_FEEDBACK_LOST: return SafetyRuleId::CalibrationFeedbackLost;
        case CALIB_DIAG_REASON_TASK_START_FAILED: return SafetyRuleId::CalibrationTaskStartFailed;
        case CALIB_DIAG_REASON_CONTROL_LOST: return SafetyRuleId::CalibrationControlLost;
        case CALIB_DIAG_REASON_HOLD_TARGET_FAILED: return SafetyRuleId::CalibrationHoldTargetFailed;
        case CALIB_DIAG_REASON_JOINT_LIST_INVALID: return SafetyRuleId::CalibrationJointListInvalid;
        case CALIB_DIAG_REASON_DUPLICATE_JOINT: return SafetyRuleId::CalibrationDuplicateJoint;
        case CALIB_DIAG_REASON_WORKSPACE_PREP_FAILED: return SafetyRuleId::CalibrationWorkspacePrepFailed;
        case CALIB_DIAG_REASON_JOINT_STEP_FAILED: return SafetyRuleId::CalibrationJointStepFailed;
        case CALIB_DIAG_REASON_SERVO_RECOVERY_TIMEOUT: return SafetyRuleId::CalibrationServoRecoveryTimeout;
        case CALIB_DIAG_REASON_RELEASE_OVER_TWO_TURNS: return SafetyRuleId::CalibrationReleaseOverTwoTurns;
        case CALIB_DIAG_REASON_MOTOR_MAPPING_INVALID: return SafetyRuleId::CalibrationMotorMappingInvalid;
        case CALIB_DIAG_REASON_RESULT_SPAN_INVALID: return SafetyRuleId::CalibrationResultSpanInvalid;
        case CALIB_DIAG_REASON_RESULT_BUILD_FAILED: return SafetyRuleId::CalibrationResultBuildFailed;
        case CALIB_DIAG_REASON_RESULT_COMMIT_FAILED: return SafetyRuleId::CalibrationResultCommitFailed;
        case CALIB_DIAG_REASON_MULTITURN_TRUST_FAILED: return SafetyRuleId::CalibrationMultiTurnTrustFailed;
        case CALIB_DIAG_REASON_DUAL_EXCHANGE_STALLED: return SafetyRuleId::CalibrationDualExchangeStalled;
        default: return SafetyRuleId::None;
    }
}

SafetyRuleId ruleForSystemFaultBit(uint32_t faultBit)
{
    switch (faultBit) {
        case SYSTEM_FAULT_CAN_OFFLINE: return SafetyRuleId::FaultCanOffline;
        case SYSTEM_FAULT_OVERLOAD: return SafetyRuleId::FaultOverload;
        case SYSTEM_FAULT_SERVO_OFFLINE: return SafetyRuleId::FaultServoOffline;
        case SYSTEM_FAULT_CALIBRATION: return SafetyRuleId::FaultCalibration;
        case SYSTEM_FAULT_MULTITURN_UNKNOWN: return SafetyRuleId::FaultMultiTurnUnknown;
        default: return SafetyRuleId::None;
    }
}

SafetyRuleId ruleForCalibrationWarningBit(uint8_t warningBit)
{
    switch (warningBit) {
        case CALIB_WARN_AUTO_MIN_OFFSET:
            return SafetyRuleId::CalibrationWarnAutoMinOffset;
        case CALIB_WARN_AUTO_MAX_SHORTFALL:
            return SafetyRuleId::CalibrationWarnAutoMaxShortfall;
        case CALIB_WARN_URDF_MANUAL_SPAN_MISMATCH:
            return SafetyRuleId::CalibrationWarnSpanMismatch;
        default:
            return SafetyRuleId::None;
    }
}

} // namespace p4::safety
