/**
 * @file FeedbackReadyDiagnostics.h
 * @brief 声明负反馈 READY 门禁的结构化拒绝原因和去重诊断接口。
 * @details 原因枚举只在 P4 内部使用，稳定对外含义由 SafetyRuleId 提供。
 */
#ifndef P4_MODULES_STATE_FEEDBACK_READY_DIAGNOSTICS_H
#define P4_MODULES_STATE_FEEDBACK_READY_DIAGNOSTICS_H

#include <stdint.h>

struct SystemRuntimeContext;


/**
 * 记录一条 READY 拒绝；相同上下文 1 秒内只输出一次日志和 Safety 事件。
 * a/b/c/d 的解释由 reason 决定，仅用于保留现场，不参与控制裁决。
 */
void logFeedbackReadyReject(SystemRuntimeContext* sharedData,
                            uint8_t reason,
                            uint32_t a,
                            uint32_t b,
                            int32_t c,
                            int32_t d);

#endif // P4_MODULES_STATE_FEEDBACK_READY_DIAGNOSTICS_H
