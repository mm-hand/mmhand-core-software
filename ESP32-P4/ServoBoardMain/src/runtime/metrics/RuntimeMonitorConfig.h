/**
 * @file RuntimeMonitorConfig.h
 * @brief 集中定义运行时统计窗口和执行时间直方图参数。
 * @details 参数只影响诊断精度，不参与控制、标定或舵机命令决策。
 * 状态所有权：纯编译期常量，无运行时状态。
 * 禁止依赖：禁止依赖 FreeRTOS、硬件驱动和业务模块。
 */
#ifndef P4_RUNTIME_METRICS_RUNTIME_MONITOR_CONFIG_H
#define P4_RUNTIME_METRICS_RUNTIME_MONITOR_CONFIG_H

#include <stdint.h>

inline constexpr uint32_t kTaskRuntimeWindowMs = 1000U;
inline constexpr uint8_t kTaskRuntimeExecBucketCount = 62U;
inline constexpr uint16_t kTaskRuntimeUnknownPermille = 0xFFFFU;

#endif // P4_RUNTIME_METRICS_RUNTIME_MONITOR_CONFIG_H
