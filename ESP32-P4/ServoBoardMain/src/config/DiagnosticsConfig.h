/**
 * @file DiagnosticsConfig.h
 * @brief 定义日志节流、任务监控和诊断快照的固定参数入口。
 */
#ifndef MMHAND_DIAGNOSTICS_CONFIG_H
#define MMHAND_DIAGNOSTICS_CONFIG_H

#include <stdint.h>

namespace mmhand::config::diagnostics {

/** 舵机通信循环相邻两次调度超过该值时记录卡顿日志。 */
inline constexpr uint32_t kServoLoopGapLogThresholdMs = 50;
/** 应用一批舵机目标耗时超过该值时记录发送卡顿日志。 */
inline constexpr uint32_t kServoApplyStallLogThresholdMs = 50;
/** owner 不匹配导致命令丢弃时的日志节流间隔。 */
inline constexpr uint32_t kServoQueueDropLogIntervalMs = 200;
/** 控制跟踪流只发送最新同周期快照；20 Hz 足以分析低频闭环且不挤占常规遥测。 */
inline constexpr uint32_t kControlTracePublishPeriodMs = 50;
}

#endif // MMHAND_DIAGNOSTICS_CONFIG_H
