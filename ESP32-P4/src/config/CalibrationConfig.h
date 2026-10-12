/**
 * @file CalibrationConfig.h
 * @brief 定义上位机 LUT 导入后生成电机 ABS 限位的余量。
 */
#ifndef MMHAND_CALIBRATION_CONFIG_H
#define MMHAND_CALIBRATION_CONFIG_H

#include <stdint.h>

namespace mmhand::config::calibration {

inline constexpr int32_t kMotorAbsReleaseMarginCounts = 500; ///< LUT 松端之外的 ABS Limit 余量。
inline constexpr int32_t kMotorAbsTightMarginCounts = 900; ///< LUT 紧端之外的 ABS Limit 余量。

} // namespace mmhand::config::calibration

#endif // MMHAND_CALIBRATION_CONFIG_H
