/**
 * @file CalibrationRuntimeState.h
 * @brief 保存上位机写入的 Manual、ABS/LUT 边界和装配告警状态。
 * @details P4 只消费标定结果；自动标定过程和诊断由上位机维护。
 */
#ifndef P4_SYSTEM_CONTEXT_CALIBRATION_RUNTIME_STATE_H
#define P4_SYSTEM_CONTEXT_CALIBRATION_RUNTIME_STATE_H

#include "../config/Config.h"
#include "CalibrationTypes.h"
#include <stdint.h>

struct CalibrationRuntimeState {
    // Manual 端点只存在本次 P4 RAM 会话；桌面 JSON 是唯一持久化来源。
    volatile uint32_t manual_reference_valid_bitmap; ///< 已写入 P4 RAM 的 Manual Jxx。
    volatile uint32_t manual_reference_crc; ///< 当前 RAM Manual 表 CRC。
    volatile uint32_t manual_reference_revision; ///< 每次完整提交后递增，供上位机读回确认。
    volatile uint8_t manual_reference_committed; ///< Manual RAM 表已完整提交，可用于 READY。
    volatile uint8_t manual_reference_commit_status; ///< MANUAL_COMMIT_STATUS_*。
    volatile uint32_t motor_abs_limit_valid_bitmap; ///< 已标定 ABS/LUT 边界 Mxx。
    int32_t motor_abs_limit_min[SERVO_TOTAL_NUM]; ///< logical ABS 下界。
    int32_t motor_abs_limit_max[SERVO_TOTAL_NUM]; ///< logical ABS 上界。
    volatile uint32_t calibration_warning_bitmap; ///< 非阻断 CALIB WARN Jxx。
    int16_t calibration_auto_min_offset_counts[ENCODER_TOTAL_NUM]; ///< Auto/Manual Min 偏差。
};

#endif // P4_SYSTEM_CONTEXT_CALIBRATION_RUNTIME_STATE_H
