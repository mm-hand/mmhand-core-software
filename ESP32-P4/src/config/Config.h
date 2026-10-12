/**
 * @file Config.h
 * @brief ESP32-P4 固件编译期配置的唯一聚合入口。
 * @details 业务模块可按需包含子配置；本文件方便系统启动代码一次性加载。
 */
#ifndef MMHAND_CONFIG_H
#define MMHAND_CONFIG_H

#include "BoardConfig.h"
#include "CalibrationConfig.h"
#include "ControlConfig.h"
#include "DiagnosticsConfig.h"
#include "DimensionsConfig.h"
#include "SafetyConfig.h"
#include "ServoConfig.h"
#include "TaskConfig.h"

#endif // MMHAND_CONFIG_H
