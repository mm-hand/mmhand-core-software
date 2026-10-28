/**
 * @file ServoReadDiagnostics.cpp
 * @brief 将驱动保留的原始反馈失败证据转换为按 BUS/ID 限频的文本日志。
 */
#include "ServoReadDiagnostics.h"

#include "../../hal/servo/FeetechServoDriver.h"
#include "../../system/log/FirmwareLog.h"
#include "../../system/time/Clock.h"

namespace {

struct FailureLogGate {
    bool initialized = false;
    uint32_t lastMs = 0;
    uint32_t suppressed = 0;
};

// ServoBusTask is the only writer; no heap allocation or cross-task lock.
FailureLogGate gates[NUM_BUSES][MAX_SERVO_ID + 1];

const char* readFailureReason(const ServoReadDiagnostic& diagnostic)
{
    switch (diagnostic.communicationError) {
    case ERR_NO_REPLY: return "timeout";
    case ERR_CRC_CMP: return "checksum";
    case ERR_SLAVE_ID: return "wrong_id";
    case ERR_BUFF_LEN: return "length";
    default:
        return diagnostic.reply.statusValid && diagnostic.motorStatus != 0
            ? "motor_status" : "incomplete";
    }
}

} // namespace

void logServoReadFailure(const FeetechServoDriver* driver,
                         uint8_t busIndex, uint8_t servoId)
{
    if (!driver || busIndex >= NUM_BUSES || servoId > MAX_SERVO_ID) return;
    const ServoReadDiagnostic& diagnostic = driver->getReadDiagnostic(servoId);
    if (!diagnostic.failed) return;

    FailureLogGate& gate = gates[busIndex][servoId];
    const uint32_t nowMs = p4::platform::time::monotonicMillis();
    if (gate.initialized && (uint32_t)(nowMs - gate.lastMs) <
            mmhand::config::servo::kFeedbackFailureLogIntervalMs) {
        if (gate.suppressed != UINT32_MAX) ++gate.suppressed;
        return;
    }
    const uint32_t suppressed = gate.suppressed;
    gate.initialized = true;
    gate.lastMs = nowMs;
    gate.suppressed = 0;
    FW_LOG_WARN_PRINTF(
        "[SERVO READ FAIL] bus=%u id=%u mode=%s reason=%s err=%u "
        "status=0x%02X status_valid=%u data=%u/%u reply_len=%u "
        "bus_rx=%u/%u elapsed_ms=%lu seq=%lu suppressed=%lu\r\n",
        (unsigned)busIndex, (unsigned)servoId,
        diagnostic.sync ? "sync" : "probe", readFailureReason(diagnostic),
        (unsigned)diagnostic.communicationError, (unsigned)diagnostic.motorStatus,
        (unsigned)diagnostic.reply.statusValid,
        (unsigned)diagnostic.reply.payloadBytes,
        (unsigned)diagnostic.expectedPayloadBytes,
        (unsigned)diagnostic.reply.replyLength,
        (unsigned)diagnostic.busReceivedBytes,
        (unsigned)diagnostic.busExpectedBytes,
        (unsigned long)diagnostic.elapsedMs,
        (unsigned long)diagnostic.sequence, (unsigned long)suppressed);
}
