/**
 * @file CalibrationRuntimeState.h
 * @brief 保存自动标定运行、结果提交、ABS/LUT 边界和诊断状态。
 * @details AutoCalibrationTask 写运行事务；ServoBusTask 确认可相信 ABS；HostCommTask 只上报。
 */
#ifndef P4_SYSTEM_CONTEXT_CALIBRATION_RUNTIME_STATE_H
#define P4_SYSTEM_CONTEXT_CALIBRATION_RUNTIME_STATE_H

#include "../../config/Config.h"
#include "../../data/CalibrationTypes.h"
#include <stdint.h>

struct CalibrationRuntimeState {
    volatile uint32_t calibration_aux_motor_mask; ///< 工作空间准备额外轮询 Mxx。
    volatile uint8_t calibration_motion_started; ///< 已进入真实运动阶段。
    volatile uint32_t calibration_completed_joint_bitmap; ///< 已完整提交成功 Jxx。
    volatile uint32_t calibration_trust_request_mask; ///< 请求登记可信 Mxx。
    volatile uint32_t calibration_trust_request_token; ///< 登记请求代际。
    volatile uint32_t calibration_trust_result_token; ///< 登记完成代际。
    volatile uint32_t calibration_trust_accepted_mask; ///< 成功登记 Mxx。
    volatile uint32_t motor_abs_limit_valid_bitmap; ///< 已标定 ABS/LUT 边界 Mxx。
    volatile uint32_t motor_abs_limit_locked_bitmap; ///< UI LOCK 启用边界 Mxx。
    int32_t motor_abs_limit_min[SERVO_TOTAL_NUM]; ///< logical ABS 下界。
    int32_t motor_abs_limit_max[SERVO_TOTAL_NUM]; ///< logical ABS 上界。
    volatile uint8_t calibration_fault_latched; ///< 自动标定失败锁存。
    volatile uint32_t calibration_warning_bitmap; ///< 非阻断 CALIB WARN Jxx。
    int16_t calibration_auto_min_offset_counts[ENCODER_TOTAL_NUM]; ///< Auto/Manual Min 偏差。
    CalibrationDiagData_t calibration_diag; ///< 最近结构化标定诊断。
};

#endif // P4_SYSTEM_CONTEXT_CALIBRATION_RUNTIME_STATE_H
