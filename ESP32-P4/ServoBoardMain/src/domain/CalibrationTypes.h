/**
 * @file CalibrationTypes.h
 * @brief 定义自动标定诊断原因、阶段和上报数据。
 * @details 诊断编号属于上下位机协议契约，现有数值不得重新编号。
 */
#ifndef MMHAND_CALIBRATION_TYPES_H
#define MMHAND_CALIBRATION_TYPES_H

#include <stdint.h>

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

inline constexpr uint8_t CALIB_DIAG_PHASE_NONE = 0;
inline constexpr uint8_t CALIB_DIAG_PHASE_INIT = 1;
inline constexpr uint8_t CALIB_DIAG_PHASE_FIND_MAX = 2;
inline constexpr uint8_t CALIB_DIAG_PHASE_RELEASE = 3;
inline constexpr uint8_t CALIB_DIAG_PHASE_RETURN_ZERO = 4;
inline constexpr uint8_t CALIB_DIAG_PHASE_DUAL_PRELOAD = 5;
inline constexpr uint8_t CALIB_DIAG_PHASE_DUAL_EXCHANGE = 6;
inline constexpr uint8_t CALIB_DIAG_PHASE_SEQUENCE = 7;

struct CalibrationDiagData_t {
    uint8_t valid;
    uint8_t reason;
    uint8_t phase;
    uint8_t joint;
    uint8_t motor;
    int32_t valueA;
    int32_t valueB;
    int32_t valueC;
    uint32_t bitmap;
    uint32_t timestamp;
    uint32_t sequence;
};

#endif // MMHAND_CALIBRATION_TYPES_H
