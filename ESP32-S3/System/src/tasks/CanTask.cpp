#include "CanTask.h"
#include "../config/Config.h"
#include "../hal/HalTWAI.h"

TaskHandle_t xCanTask = NULL;

namespace {
constexpr uint32_t kCanTaskPeriodMs = 4;
constexpr uint8_t kEncoderCanBatchFrameCount = CAN_ENC_FRAME_COUNT + 1;
}

extern QueueHandle_t xQueueEncoderData;
extern TaskHandle_t xSysMgrTask;

void Task_CanBus(void *pvParameters) {
    (void)pvParameters;

    EncoderData txData{};
    static uint8_t errorSendCounter = 0;
    static bool calibrationCommandWarned = false;
    uint32_t lastCanDiagPrintMs = millis();
    uint32_t encQueueRxCount = 0;
    uint32_t encTxOkFrames = 0;
    uint32_t encTxFailFrames = 0;
    uint32_t busRecoverCount = 0;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(kCanTaskPeriodMs);

    Serial.println("[Task_CanBus] Started!");

    for (;;) {
        const bool isBusOk = twaiBus.maintain();

        if (isBusOk) {
            if (xQueueEncoderData && xQueueReceive(xQueueEncoderData, &txData, 0) == pdTRUE) {
                encQueueRxCount++;
                const uint8_t sentFrames = twaiBus.sendEncoderData(txData);
                encTxOkFrames += sentFrames;
                encTxFailFrames += (sentFrames < kEncoderCanBatchFrameCount)
                                       ? (uint32_t)(kEncoderCanBatchFrameCount - sentFrames)
                                       : 0UL;

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

            twai_message_t rxMsg;
            if (twai_receive(&rxMsg, 0) == ESP_OK) {
                if (rxMsg.identifier == 0x200 &&
                    rxMsg.data_length_code > 0 &&
                    rxMsg.data[0] == 0xCA) {
                    if (!calibrationCommandWarned) {
                        Serial.println("[Task_CanBus] Ignored legacy CAN calibration command 0x200/0xCA; P4 owns encoder calibration.");
                        calibrationCommandWarned = true;
                    }
                }
            }
        } else {
            busRecoverCount++;
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        const uint32_t nowMs = millis();
        if ((uint32_t)(nowMs - lastCanDiagPrintMs) >= 10000) {
            Serial.printf("[S3 CAN DBG] encQueue=%lu encTxOk=%lu encTxFail=%lu busRecover=%lu firstRaw=%u err0=%u\r\n",
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
