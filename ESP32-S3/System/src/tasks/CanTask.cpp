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
    uint32_t encQueueRxCount = 0;
    uint32_t encTxOkFrames = 0;
    uint32_t encTxFailFrames = 0;
    uint32_t busRecoverCount = 0;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(kCanTaskPeriodMs);

    Serial.println("[Task_CanBus] Started!");

    for (;;) {
        // bus-off时发起恢复，STOPPED时尝试重新启动。
        const bool isBusOk = twaiBus.maintain();

        if (isBusOk) {
            if (xQueueEncoderData && xQueueReceive(xQueueEncoderData, &txData, 0) == pdTRUE) {
                encQueueRxCount++;
                const uint8_t sentFrames = twaiBus.sendEncoderData(txData);
                encTxOkFrames += sentFrames;
                encTxFailFrames += (sentFrames < kEncoderCanBatchFrameCount)
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
        // DEBUG级统计每10秒归零；默认INFO等级下不会实际输出。
        if ((uint32_t)(nowMs - lastCanDiagPrintMs) >= 10000) {
            FW_LOG_DEBUG_PRINTF("[S3 CAN DBG] encQueue=%lu encTxOk=%lu encTxFail=%lu busRecover=%lu firstRaw=%u err0=%u\r\n",
                                (unsigned long)encQueueRxCount,
                                (unsigned long)encTxOkFrames,
                                (unsigned long)encTxFailFrames,
                                (unsigned long)busRecoverCount,
                                (unsigned)txData.rawAngles[0],
                                (unsigned)txData.errorFlags[0]);
            encQueueRxCount = 0;
            encTxOkFrames = 0;
            encTxFailFrames = 0;
            busRecoverCount = 0;
            lastCanDiagPrintMs = nowMs;
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
