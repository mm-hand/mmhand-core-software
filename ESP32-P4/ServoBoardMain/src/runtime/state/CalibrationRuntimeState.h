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
    // Manual 端点只存在本次 P4 RAM 会话；桌面 JSON 是唯一持久化来源。
    volatile uint32_t manual_reference_valid_bitmap; ///< 已写入 P4 RAM 的 Manual Jxx。
    volatile uint32_t manual_reference_crc; ///< 当前 RAM Manual 表 CRC。
    volatile uint32_t manual_reference_revision; ///< 每次完整提交后递增，供上位机读回确认。
    volatile uint8_t manual_reference_committed; ///< Manual RAM 表已完整提交，可用于 READY/标定。
    volatile uint8_t manual_reference_commit_status; ///< MANUAL_COMMIT_STATUS_*。
    volatile uint32_t calibration_aux_motor_mask; ///< 工作空间准备额外轮询 Mxx。
    volatile uint8_t calibration_motion_started; ///< 已进入真实运动阶段。
    volatile uint32_t calibration_target_request_token; ///< 标定目标发布代次。
    volatile uint32_t calibration_target_result_token; ///< ServoBus 已处理的目标代次。
    volatile uint32_t calibration_target_requested_mask; ///< 本次批次请求的全部 Mxx。
    volatile uint32_t calibration_target_applied_mask; ///< 已完整进入同步写缓存的 Mxx。
    volatile uint32_t calibration_target_rejected_mask; ///< 原子拒绝批次涉及的 Mxx。
    volatile uint8_t calibration_target_apply_status; ///< CALIBRATION_TARGET_APPLY_*。
    volatile uint32_t calibration_completed_joint_bitmap; ///< 已完整提交成功 Jxx。
    volatile uint32_t calibration_trust_request_mask; ///< 请求登记可信 Mxx。
    volatile uint32_t calibration_trust_request_token; ///< 登记请求代际。
    volatile uint32_t calibration_trust_result_token; ///< 登记完成代际。
    volatile uint32_t calibration_trust_accepted_mask; ///< 成功登记 Mxx。
    volatile uint32_t motor_abs_limit_valid_bitmap; ///< 已标定 ABS/LUT 边界 Mxx。
    int32_t motor_abs_limit_min[SERVO_TOTAL_NUM]; ///< logical ABS 下界。
    int32_t motor_abs_limit_max[SERVO_TOTAL_NUM]; ///< logical ABS 上界。
    volatile uint8_t calibration_fault_latched; ///< 自动标定失败锁存。
    volatile uint32_t calibration_warning_bitmap; ///< 非阻断 CALIB WARN Jxx。
    int16_t calibration_auto_min_offset_counts[ENCODER_TOTAL_NUM]; ///< Auto/Manual Min 偏差。
    CalibrationDiagData_t calibration_diag; ///< 最近结构化标定诊断。
};

#endif // P4_SYSTEM_CONTEXT_CALIBRATION_RUNTIME_STATE_H
