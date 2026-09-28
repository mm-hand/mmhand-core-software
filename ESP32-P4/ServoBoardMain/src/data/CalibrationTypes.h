/**
 * @file CalibrationTypes.h
 * @brief 定义自动标定诊断原因、阶段和上报数据。
 * @details 诊断编号属于上下位机协议契约，现有数值不得重新编号。
 */
#ifndef MMHAND_CALIBRATION_TYPES_H
#define MMHAND_CALIBRATION_TYPES_H

#include <stdint.h>

/** 自动标定持续状态；数值进入上位机遥测协议，发布后禁止重新编号。 */
inline constexpr uint8_t CALIB_STATUS_IDLE = 0;    ///< 没有标定任务或可报告结果。
inline constexpr uint8_t CALIB_STATUS_RUNNING = 1; ///< 自动标定正在执行。
inline constexpr uint8_t CALIB_STATUS_SUCCESS = 2; ///< 本轮目标关节全部成功。
inline constexpr uint8_t CALIB_STATUS_FAILED = 3;  ///< 本轮至少一个目标关节失败。
inline constexpr uint8_t CALIB_STATUS_WARN = 4;    ///< 结果可控，但存在非阻断装配告警。

/** 标定完成后的非阻断装配告警位；可组合上报，但不会把系统切入 FAULT。 */
inline constexpr uint8_t CALIB_WARN_NONE = 0x00;
inline constexpr uint8_t CALIB_WARN_AUTO_MIN_OFFSET = 0x01;
inline constexpr uint8_t CALIB_WARN_AUTO_MAX_SHORTFALL = 0x02;
inline constexpr uint8_t CALIB_WARN_URDF_MANUAL_SPAN_MISMATCH = 0x04;

/**
 * 标定失败原因编号直接进入上位机协议；空缺 2/3/4/7/9/10/11/12/19
 * 属于旧固件已发布原因，禁止复用。每项注释固定 valueA/valueB/valueC/bitmap 契约。
 */
inline constexpr uint8_t CALIB_DIAG_REASON_NONE = 0; ///< A/B/C/bitmap=0。
inline constexpr uint8_t CALIB_DIAG_REASON_ACTIVE_SCOPE_EMPTY = 1; ///< A=activeJointMask, B=activeMotorMask, C=0, bitmap=0。
inline constexpr uint8_t CALIB_DIAG_REASON_ENCODER_ERROR = 5; ///< A=encoderValidMask, B=onlineJointMask, C=faultJointMask, bitmap=affectedJointMask。
inline constexpr uint8_t CALIB_DIAG_REASON_SERVO_OFFLINE = 6; ///< A=onlineMotorMask, B=configuredMotorMask, C=0, bitmap=affectedMotorMask。
inline constexpr uint8_t CALIB_DIAG_REASON_OVERLOAD = 8; ///< A=loadMagnitude, B=loadLimit, C=rawLoad, bitmap=affectedMotorMask。
inline constexpr uint8_t CALIB_DIAG_REASON_ZERO_CROSS_INVALID = 13; ///< A=tautEncoder, B=zeroEncoder, C=tightEncoder, bitmap=affectedMotorMask。
inline constexpr uint8_t CALIB_DIAG_REASON_WRAP_TOO_LARGE = 14; ///< A=encoderSpan, B=tightEncoder, C=maxEncoder, bitmap=affectedMotorMask。
inline constexpr uint8_t CALIB_DIAG_REASON_ABS_LIMIT_INVALID = 15; ///< A=commandAbs, B=limitMin, C=limitMax, bitmap=affectedMotorMask。
inline constexpr uint8_t CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED = 16; ///< A=targetA, B=targetB(单路为0), C=CALIBRATION_TARGET_APPLY_*, bitmap=targetMotorMask。
inline constexpr uint8_t CALIB_DIAG_REASON_FEEDBACK_LOST = 17; ///< A=encoderValidMask, B=servoAbsValidMask, C=servoRawValidMask, bitmap=affectedMotorMask。
inline constexpr uint8_t CALIB_DIAG_REASON_TASK_START_FAILED = 18; ///< A=taskCreateResult, B/C=0, bitmap=selectedJointMask。
inline constexpr uint8_t CALIB_DIAG_REASON_CONTROL_LOST = 20; ///< A=systemState, B=servoTargetOwner, C=controlEnabled, bitmap=activeJointMask。
inline constexpr uint8_t CALIB_DIAG_REASON_HOLD_TARGET_FAILED = 21; ///< A=currentIndex, B=jointCount, C=batchCount, bitmap=selectedJointMask。
inline constexpr uint8_t CALIB_DIAG_REASON_JOINT_LIST_INVALID = 22; ///< A=jointOrCount, B=maxJointCount, C=indexOrHasList, bitmap=seenJointMask。
inline constexpr uint8_t CALIB_DIAG_REASON_DUPLICATE_JOINT = 23; ///< A=duplicateIndex, B=jointCount, C=0, bitmap=seenJointMask。
inline constexpr uint8_t CALIB_DIAG_REASON_WORKSPACE_PREP_FAILED = 24; ///< A=helperJoint, B=DualWorkspacePose, C=DualWorkspacePlanStage, bitmap=helperMotorMask。
inline constexpr uint8_t CALIB_DIAG_REASON_JOINT_STEP_FAILED = 25; ///< 调度层兜底；A/B/C/bitmap=0，仅使用 completedJointMask/failedJointMask。
inline constexpr uint8_t CALIB_DIAG_REASON_SERVO_RECOVERY_TIMEOUT = 26; ///< A=onlineMotorMask, B=configuredMotorMask, C=pendingMotorMask, bitmap=unresolvedMotorMask。
inline constexpr uint8_t CALIB_DIAG_REASON_RELEASE_OVER_TWO_TURNS = 27; ///< A=releasedCounts, B=limitCounts, C=nextReleasedCounts, bitmap=affectedMotorMask。
inline constexpr uint8_t CALIB_DIAG_REASON_MOTOR_MAPPING_INVALID = 28; ///< A=joint, B=primaryMotor, C=secondaryMotor, bitmap=mappedMotorMask。
inline constexpr uint8_t CALIB_DIAG_REASON_RESULT_SPAN_INVALID = 29; ///< A=encoderSpan, B=endpointA, C=endpointB, bitmap=affectedMotorMask。
inline constexpr uint8_t CALIB_DIAG_REASON_RESULT_BUILD_FAILED = 30; ///< A=sampleCount, B=encoderSpan, C=lutCount, bitmap=affectedMotorMask。
inline constexpr uint8_t CALIB_DIAG_REASON_RESULT_COMMIT_FAILED = 31; ///< A=joint, B=encoderSpan, C=lutCount, bitmap=affectedMotorMask。
inline constexpr uint8_t CALIB_DIAG_REASON_MULTITURN_TRUST_FAILED = 32; ///< A=joint, B=primaryMotor, C=secondaryMotor, bitmap=affectedMotorMask。
inline constexpr uint8_t CALIB_DIAG_REASON_TARGET_APPLY_TIMEOUT = 33; ///< A=targetA, B=targetB(单路为0), C=CALIBRATION_TARGET_APPLY_*, bitmap=targetMotorMask。
inline constexpr uint8_t CALIB_DIAG_REASON_DUAL_EXCHANGE_STALLED = 34; ///< A=loadMagnitude, B=loadThreshold, C=encoderDelta, bitmap=dualMotorMask。

/** 标定阶段用于把同一原因定位到具体动作区间。 */
inline constexpr uint8_t CALIB_DIAG_PHASE_INIT = 1;
inline constexpr uint8_t CALIB_DIAG_PHASE_FIND_MAX = 2;
inline constexpr uint8_t CALIB_DIAG_PHASE_RELEASE = 3;
inline constexpr uint8_t CALIB_DIAG_PHASE_DUAL_PRELOAD = 5;
inline constexpr uint8_t CALIB_DIAG_PHASE_DUAL_EXCHANGE = 6;
inline constexpr uint8_t CALIB_DIAG_PHASE_SEQUENCE = 7;

struct CalibrationDiagData_t {
    uint8_t valid;     ///< 1 表示其余字段构成有效诊断。
    uint8_t reason;    ///< CALIB_DIAG_REASON_*。
    uint8_t phase;     ///< CALIB_DIAG_PHASE_*。
    uint8_t joint;     ///< 失败关节 Jxx，0xFF 表示不适用。
    uint8_t motor;     ///< 失败舵机 Mxx，0xFF 表示不适用。
    int32_t valueA;    ///< 原因相关诊断值，含义只由 reason 决定。
    int32_t valueB;    ///< 原因相关诊断值，含义只由 reason 决定。
    int32_t valueC;    ///< 原因相关诊断值，含义只由 reason 决定。
    uint32_t bitmap;   ///< 原因相关位图，含义只由 reason 决定。
    uint32_t timestamp; ///< 故障产生时间。
    uint32_t sequence;  ///< 单调诊断序号，供 UI 去重弹窗。
    uint32_t completedJointMask; ///< 本轮已成功提交标定结果的 Jxx 位图。
    uint32_t failedJointMask;    ///< 本轮失败、下次需要重标的 Jxx 位图。
};

#endif // MMHAND_CALIBRATION_TYPES_H
