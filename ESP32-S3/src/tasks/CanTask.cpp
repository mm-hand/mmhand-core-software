#include "CanTask.h"
#include "../config/Config.h"
#include "../hal/HalTWAI.h"
#include "../system/FirmwareLog.h"

TaskHandle_t xCanTask = NULL;

namespace {
// CAN任务每4ms尝试发送一次最新磁编快照。
constexpr uint32_t kCanTaskPeriodMs = 4;
// 一批磁编数据包含若干数据帧和1帧CRC/元数据。
constexpr uint8_t kEncoderCanBatchFrameCount = CAN_ENC_FRAME_COUNT + 1;
constexpr uint32_t kCanHealthPeriodMs = 100;
constexpr uint32_t kCanSaturationTimeoutMs = 500;
constexpr uint32_t kCanReportPeriodMs = 10000;
// ESP-IDF counts the in-flight frame in addition to the software queue.
constexpr uint32_t kCanTxCapacity = HalTWAI::kTxQueueLength + 1;

struct CanSendCounts {
    uint32_t batches;
    uint32_t accepted;
    uint32_t failed;
};

void logCanFault(const char* reason, uint32_t nowMs, uint32_t durationMs,
                 const twai_status_info_t& status, uint32_t busErrorDelta,
                 const CanSendCounts& counts, const CanSendCounts& baseline) {
    FW_LOG_PRINTF(FW_LOG_LEVEL_WARN,
        "\n[S3 CAN FAULT] reason=%s up=%lu outageMs=%lu state=%d encQueue=%lu txOk=%lu txFail=%lu pending=%lu tec=%lu rec=%lu busErrDelta=%lu\r\n",
        reason, (unsigned long)nowMs, (unsigned long)durationMs, int(status.state),
        (unsigned long)(counts.batches - baseline.batches),
        (unsigned long)(counts.accepted - baseline.accepted),
        (unsigned long)(counts.failed - baseline.failed),
        (unsigned long)status.msgs_to_tx, (unsigned long)status.tx_error_counter,
        (unsigned long)status.rx_error_counter, (unsigned long)busErrorDelta);
}
}

extern QueueHandle_t xQueueEncoderData;

/**
 * @brief 维护TWAI状态并发送最新磁编快照。
 * @param pvParameters FreeRTOS预留任务参数，当前未使用。
 */
void Task_CanBus(void *pvParameters) {
    (void)pvParameters;

    EncoderData txData{};                       // 最近一次从覆盖队列取出的完整磁编快照。
    static uint8_t errorSendCounter = 0;        // 错误详情降频计数，5批检查一次。
    uint32_t lastCanDiagPrintMs = millis();
    uint32_t lastHealthMs = lastCanDiagPrintMs;
    uint32_t lastFaultReportMs = 0;
    uint32_t saturationSinceMs = 0;
    uint32_t faultSinceMs = 0;
    uint32_t previousBusErrors = 0;
    uint32_t faultBusErrors = 0;
    CanSendCounts counts{};
    CanSendCounts periodicBaseline{};
    CanSendCounts previousHealthCounts{};
    CanSendCounts faultBaseline{};
    bool saturated = false;
    bool faultActive = false;
    bool wasBusOff = false;
    bool reportedBusOff = false;
    uint32_t busRecoverCount = 0;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(kCanTaskPeriodMs);

    Serial.println("[Task_CanBus] Started!");

    for (;;) {
        // bus-off时发起恢复，STOPPED时尝试重新启动。
        twai_status_info_t beforeMaintenance{};
        bool haveStatus = false;
        const bool isBusOk = twaiBus.maintain(beforeMaintenance, haveStatus);
        const bool busOff = haveStatus && beforeMaintenance.state == TWAI_STATE_BUS_OFF;
        const bool enteredBusOff = busOff && !wasBusOff;
        wasBusOff = busOff;

        if (isBusOk) {
            if (xQueueEncoderData && xQueueReceive(xQueueEncoderData, &txData, 0) == pdTRUE) {
                counts.batches++;
                const uint8_t sentFrames = twaiBus.sendEncoderData(txData);
                counts.accepted += sentFrames;
                counts.failed += (sentFrames < kEncoderCanBatchFrameCount)
                                       ? (uint32_t)(kEncoderCanBatchFrameCount - sentFrames)
                                       : 0UL;

                // 详细错误码不必每4ms发送，约每20ms发送一次即可。
                errorSendCounter++;
                if (errorSendCounter >= 5) {
                    errorSendCounter = 0;

                    bool hasError = false;
                    for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
                        if (txData.errorFlags[i]) {
                            hasError = true;
                            break;
                        }
                    }

                    if (hasError) {
                        twaiBus.sendErrorStatus(txData);
                    }
                }
            }

        } else {
            busRecoverCount++;
            // 恢复期间降低轮询频率，避免持续占用CPU和TWAI驱动锁。
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        const uint32_t nowMs = millis();
        if (enteredBusOff || nowMs - lastHealthMs >= kCanHealthPeriodMs) {
            twai_status_info_t health = beforeMaintenance;
            // Preserve BUS_OFF before maintain() changes the driver state.
            const bool healthValid = busOff || twai_get_status_info(&health) == ESP_OK;
            if (healthValid) {
                const bool queueFull = health.state == TWAI_STATE_RUNNING &&
                    health.msgs_to_tx >= kCanTxCapacity;
                if (!queueFull) {
                    saturated = false;
                } else if (!saturated && counts.failed != previousHealthCounts.failed) {
                    saturated = true;
                    saturationSinceMs = nowMs;
                    if (!faultActive) {
                        faultBaseline = previousHealthCounts;
                        faultBusErrors = previousBusErrors;
                    }
                }
                const bool saturationTimedOut = saturated &&
                    nowMs - saturationSinceMs >= kCanSaturationTimeoutMs;
                const bool starting = !faultActive && (enteredBusOff || saturationTimedOut);
                if (starting) {
                    faultActive = true;
                    faultSinceMs = saturated ? saturationSinceMs : nowMs;
                    if (!saturated) {
                        faultBaseline = previousHealthCounts;
                        faultBusErrors = previousBusErrors;
                    }
                }
                if (faultActive && health.state == TWAI_STATE_RUNNING && !queueFull &&
                    counts.accepted != previousHealthCounts.accepted &&
                    counts.failed == previousHealthCounts.failed) {
                    FW_LOG_PRINTF(FW_LOG_LEVEL_INFO,
                        "\n[S3 CAN RECOVERED] up=%lu outageMs=%lu pending=%lu\r\n",
                        (unsigned long)nowMs, (unsigned long)(nowMs - faultSinceMs),
                        (unsigned long)health.msgs_to_tx);
                    faultActive = false;
                    reportedBusOff = false;
                } else if (faultActive && (starting || (enteredBusOff && !reportedBusOff) ||
                           nowMs - lastFaultReportMs >= kCanReportPeriodMs)) {
                    lastFaultReportMs = nowMs;
                    reportedBusOff = reportedBusOff || busOff;
                    logCanFault(busOff ? "bus_off" : queueFull ? "tx_saturated" : "waiting_tx",
                                nowMs, nowMs - faultSinceMs, health,
                                health.bus_error_count - faultBusErrors, counts, faultBaseline);
                }
                previousBusErrors = health.bus_error_count;
                previousHealthCounts = counts;
            } else {
                saturated = false;
            }
            lastHealthMs = nowMs;
        }
        // 低频链路诊断：USB可正常采集时也能区分CAN未安装、bus-off和发送失败。
        if (nowMs - lastCanDiagPrintMs >= kCanReportPeriodMs) {
            twai_status_info_t status = {};
            const bool statusValid = twai_get_status_info(&status) == ESP_OK;
            FW_LOG_PRINTF(FW_LOG_LEVEL_INFO,
                "[S3 CAN] state=%d encQueue=%lu txOk=%lu txFail=%lu recover=%lu busErr=%lu txErr=%lu rxErr=%lu pending=%lu\r\n",
                statusValid ? int(status.state) : -1,
                (unsigned long)(counts.batches - periodicBaseline.batches),
                (unsigned long)(counts.accepted - periodicBaseline.accepted),
                (unsigned long)(counts.failed - periodicBaseline.failed),
                (unsigned long)busRecoverCount,
                statusValid ? (unsigned long)status.bus_error_count : 0UL,
                statusValid ? (unsigned long)status.tx_error_counter : 0UL,
                statusValid ? (unsigned long)status.rx_error_counter : 0UL,
                statusValid ? (unsigned long)status.msgs_to_tx : 0UL);
            periodicBaseline = counts;
            busRecoverCount = 0;
            lastCanDiagPrintMs = nowMs;
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
