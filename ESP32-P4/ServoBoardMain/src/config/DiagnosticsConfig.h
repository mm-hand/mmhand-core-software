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
/** 临时启用 M19 目标寄存器回读，用于确认负反馈小修正是否真正写入舵机。 */
inline constexpr bool kM19GoalReadbackEnabled = true;
/** M19 目标寄存器回读周期；只在 RUN_FEEDBACK 且 M19 被选中时执行。 */
inline constexpr uint32_t kM19GoalReadbackPeriodMs = 50;
/** 诊断回读失败日志使用更低频率，避免断线时刷屏。 */
inline constexpr uint32_t kM19GoalReadbackFailureLogPeriodMs = 500;

}

#endif // MMHAND_DIAGNOSTICS_CONFIG_H
