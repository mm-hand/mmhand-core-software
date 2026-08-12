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

/** 标定失败原因编号直接进入上位机协议；空缺值属于已删除旧原因，禁止复用。 */
inline constexpr uint8_t CALIB_DIAG_REASON_NONE = 0;
inline constexpr uint8_t CALIB_DIAG_REASON_ACTIVE_SCOPE_EMPTY = 1;
inline constexpr uint8_t CALIB_DIAG_REASON_ENCODER_ERROR = 5;
inline constexpr uint8_t CALIB_DIAG_REASON_SERVO_OFFLINE = 6;
inline constexpr uint8_t CALIB_DIAG_REASON_OVERLOAD = 8;
inline constexpr uint8_t CALIB_DIAG_REASON_ZERO_CROSS_INVALID = 13;
inline constexpr uint8_t CALIB_DIAG_REASON_WRAP_TOO_LARGE = 14;
inline constexpr uint8_t CALIB_DIAG_REASON_ABS_LIMIT_INVALID = 15;
inline constexpr uint8_t CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED = 16;
inline constexpr uint8_t CALIB_DIAG_REASON_FEEDBACK_LOST = 17;
inline constexpr uint8_t CALIB_DIAG_REASON_TASK_START_FAILED = 18;
inline constexpr uint8_t CALIB_DIAG_REASON_CONTROL_LOST = 20;
inline constexpr uint8_t CALIB_DIAG_REASON_HOLD_TARGET_FAILED = 21;
inline constexpr uint8_t CALIB_DIAG_REASON_JOINT_LIST_INVALID = 22;
inline constexpr uint8_t CALIB_DIAG_REASON_DUPLICATE_JOINT = 23;
inline constexpr uint8_t CALIB_DIAG_REASON_WORKSPACE_PREP_FAILED = 24;
inline constexpr uint8_t CALIB_DIAG_REASON_JOINT_STEP_FAILED = 25;
inline constexpr uint8_t CALIB_DIAG_REASON_SERVO_RECOVERY_TIMEOUT = 26;
inline constexpr uint8_t CALIB_DIAG_REASON_RELEASE_OVER_TWO_TURNS = 27;
inline constexpr uint8_t CALIB_DIAG_REASON_MOTOR_MAPPING_INVALID = 28;
inline constexpr uint8_t CALIB_DIAG_REASON_RESULT_SPAN_INVALID = 29;
inline constexpr uint8_t CALIB_DIAG_REASON_RESULT_BUILD_FAILED = 30;
inline constexpr uint8_t CALIB_DIAG_REASON_RESULT_COMMIT_FAILED = 31;
inline constexpr uint8_t CALIB_DIAG_REASON_MULTITURN_TRUST_FAILED = 32;

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
    int32_t valueA;    ///< 原因相关诊断值，含义由 reason/phase 决定。
    int32_t valueB;    ///< 原因相关诊断值，含义由 reason/phase 决定。
    int32_t valueC;    ///< 原因相关诊断值，含义由 reason/phase 决定。
    uint32_t bitmap;   ///< 失败/依赖关节或舵机位图。
    uint32_t timestamp; ///< 故障产生时间。
    uint32_t sequence;  ///< 单调诊断序号，供 UI 去重弹窗。
    uint32_t completedJointMask; ///< 本轮已成功提交标定结果的 Jxx 位图。
    uint32_t failedJointMask;    ///< 本轮失败、下次需要重标的 Jxx 位图。
};

#endif // MMHAND_CALIBRATION_TYPES_H
