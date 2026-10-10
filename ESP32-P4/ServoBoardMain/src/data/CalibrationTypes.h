/**
 * @file CalibrationTypes.h
 * @brief 定义标定结果的装配告警和旧遥测字段兼容值。
 * @details 自动标定状态机与诊断已由上位机维护；P4 只保存上传结果。
 */
#ifndef MMHAND_CALIBRATION_TYPES_H
#define MMHAND_CALIBRATION_TYPES_H

#include <stdint.h>

/** SYSTEM_STATUS 的旧标定状态槽固定发送 IDLE，保持既有帧布局。 */
inline constexpr uint8_t CALIB_STATUS_IDLE = 0;

/** 标定完成后的非阻断装配告警位；可组合上报，但不会把系统切入 FAULT。 */
inline constexpr uint8_t CALIB_WARN_NONE = 0x00;
inline constexpr uint8_t CALIB_WARN_AUTO_MIN_OFFSET = 0x01;
inline constexpr uint8_t CALIB_WARN_AUTO_MAX_SHORTFALL = 0x02;
inline constexpr uint8_t CALIB_WARN_URDF_MANUAL_SPAN_MISMATCH = 0x04;

#endif // MMHAND_CALIBRATION_TYPES_H
