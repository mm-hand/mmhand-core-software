/**
 * @file FeedbackReadyDiagnostics.cpp
 * @brief 实现 READY 拒绝日志去重、规则映射和最近安全事件登记。
 */
#include "FeedbackReadyDiagnostics.h"

#include "../../data/SafetyRuleTypes.h"
#include "../../platform/rtos/SharedSafetyState.h"
#include "../../platform/time/Clock.h"
#include "../../safety/SafetyEventReporter.h"
#include "../../safety/SafetyRuleRegistry.h"
#include "../../platform/log/FirmwareLog.h"
#include "../../runtime/SystemRuntimeContext.h"
#include "../../safety/policies/SystemReadinessPolicy.h"

using namespace p4::state;

void logFeedbackReadyReject(SystemRuntimeContext* sharedData,
                            uint8_t reason,
                            uint32_t a,
                            uint32_t b,
                            int32_t c,
                            int32_t d)
{
    // READY 门禁按 100 Hz 检查，必须同时对日志和事件缓存做去重。
    static uint32_t lastLogMs = 0;
    static uint8_t lastReason = 0;
    static uint32_t lastA = 0;
    static uint32_t lastB = 0;
    static int32_t lastC = 0;
    static int32_t lastD = 0;
    const uint32_t nowMs = p4::platform::time::monotonicMillis();
    const bool sameReject =
        reason == lastReason &&
        (reason == FEEDBACK_READY_REJECT_BELOW_WORKSPACE ||
         (a == lastA && b == lastB && c == lastC && d == lastD));
    if (sameReject && (uint32_t)(nowMs - lastLogMs) <= 1000U) {
        return;
    }
    lastReason = reason;
    lastA = a;
    lastB = b;
    lastC = c;
    lastD = d;
    lastLogMs = nowMs;
    p4::safety::reportSafetyRuleEvent(
        sharedData,
        p4::safety::ruleForFeedbackReadyRejectReason(reason),
        c,
        d);

    switch (reason) {
        case FEEDBACK_READY_REJECT_CAN_GATE:
            FW_LOG_INFO_PRINTF("[STATE] feedback ready rejected: joint mode/CAN gate inactive motor=0x%08lX joint=0x%08lX mode=%ld require_can=%ld\r\n",
                               (unsigned long)a, (unsigned long)b,
                               (long)c, (long)d);
            break;
        case FEEDBACK_READY_REJECT_MULTITURN:
            FW_LOG_INFO_PRINTF("[STATE] feedback ready rejected: multi-turn unknown active=0x%08lX unknown=0x%08lX\r\n",
                               (unsigned long)a, (unsigned long)b);
            break;
        case FEEDBACK_READY_REJECT_EMPTY_SCOPE:
            FW_LOG_INFO_PRINTF("[STATE] feedback ready rejected: active joint mask is zero motor=0x%08lX\r\n",
                               (unsigned long)a);
            break;
        case FEEDBACK_READY_REJECT_MAPPED_UNAVAILABLE:
            FW_LOG_INFO_PRINTF("[STATE] feedback ready rejected: mapped angle unavailable code=%lu age_ms=%ld\r\n",
                               (unsigned long)a, (long)c);
            break;
        case FEEDBACK_READY_REJECT_CALIBRATION_MISSING:
            FW_LOG_INFO_PRINTF("[STATE] feedback ready rejected: joint calibration missing J%02lu active=0x%08lX\r\n",
                               (unsigned long)a, (unsigned long)b);
            break;
        case FEEDBACK_READY_REJECT_MAPPED_INVALID:
            FW_LOG_INFO_PRINTF("[STATE] feedback ready rejected: mapped angle invalid J%02lu valid_flags=0x%08lX\r\n",
                               (unsigned long)a, (unsigned long)b);
            break;
        case FEEDBACK_READY_REJECT_BELOW_WORKSPACE:
            FW_LOG_INFO_PRINTF("[STATE] feedback ready rejected: joint below workspace J%02lu preserve=%ld outside=%ld tolerance=%ld\r\n",
                               (unsigned long)a, (long)((int32_t)b),
                               (long)c, (long)d);
            break;
        case FEEDBACK_READY_REJECT_CALIBRATION_MISMATCH:
            FW_LOG_INFO_PRINTF("[STATE] feedback ready rejected: calibration/manual mismatch J%02lu mapping=%lu urdf_crc=0x%08lX manual_entry_crc=0x%08lX\r\n",
                               (unsigned long)a, (unsigned long)b,
                               (unsigned long)(uint32_t)c,
                               (unsigned long)(uint32_t)d);
            break;
        case FEEDBACK_READY_REJECT_ENCODER_CAN:
            FW_LOG_INFO_PRINTF("[STATE] feedback ready rejected: encoder/CAN unavailable offline=0x%08lX fault=0x%08lX sensor_fresh=%ld can_link=%ld\r\n",
                               (unsigned long)a, (unsigned long)b,
                               (long)c, (long)d);
            break;
        case FEEDBACK_READY_REJECT_SYSTEM_FAULT:
            FW_LOG_INFO_PRINTF("[STATE] feedback ready rejected: system fault=0x%08lX\r\n",
                               (unsigned long)a);
            break;
        case FEEDBACK_READY_REJECT_SERVO_TURN_OVERFLOW:
            FW_LOG_WARN_PRINTF("[STATE] feedback ready rejected: servo hardware ABS overflow active=0x%08lX overflow=0x%08lX\r\n",
                               (unsigned long)a, (unsigned long)b);
            break;
        default:
            FW_LOG_INFO_PRINTLN("[STATE] feedback ready rejected: unknown reason");
            break;
    }
}
