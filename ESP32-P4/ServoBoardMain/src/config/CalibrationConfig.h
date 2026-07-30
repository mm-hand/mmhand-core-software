/**
 * @file CalibrationConfig.h
 * @brief 汇总自动标定、工作空间规划和 Rope Init 的固定参数。
 * @details Manual/Auto/LUT 属于运行时标定数据，不得放入本配置文件。
 */
#ifndef MMHAND_CALIBRATION_CONFIG_H
#define MMHAND_CALIBRATION_CONFIG_H

namespace mmhand::config::calibration {
// 标定阈值会随标定模块拆分逐项迁入，并由编译期断言保护单位和范围。
}

#endif // MMHAND_CALIBRATION_CONFIG_H
