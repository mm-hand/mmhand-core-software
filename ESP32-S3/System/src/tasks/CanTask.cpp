#include "CanTask.h"
#include "../config/Config.h"
#include "../hal/HalTWAI.h"

TaskHandle_t xCanTask = NULL;

extern QueueHandle_t xQueueEncoderData;
extern QueueHandle_t xQueueTactileData;
extern TaskHandle_t xSysMgrTask;

void Task_CanBus(void* pvParameters) {
    (void)pvParameters;

    EncoderData encTx{};
    TactileData tactileTx{};
    uint8_t errorSendCounter = 0;
    uint8_t tactileSendCounter = 0;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100Hz

    Serial.println("[Task_CanBus] Started!");

    for (;;) {
        const bool isBusOk = twaiBus.maintain();
        if (isBusOk) {
            // Keep encoder reporting as the highest-priority payload.
            if (xQueueEncoderData && xQueueReceive(xQueueEncoderData, &encTx, 0) == pdTRUE) {
                twaiBus.sendEncoderData(encTx);

                errorSendCounter++;
                if (errorSendCounter >= 5) {
                    errorSendCounter = 0;
                    bool hasError = false;
                    for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
                        if (encTx.errorFlags[i]) {
                            hasError = true;
                            break;
                        }
                    }
                    if (hasError) {
                        twaiBus.sendErrorStatus(encTx);
                    }
                }
            }

            // Throttle tactile summary to lower CAN bandwidth pressure.
            if (xQueueTactileData && xQueuePeek(xQueueTactileData, &tactileTx, 0) == pdTRUE) {
                tactileSendCounter++;
                if (tactileSendCounter >= 5) { // 20Hz
                    tactileSendCounter = 0;
                    twaiBus.sendTactileSummary(tactileTx);
                }
            }

            twai_message_t rxMsg;
            if (twai_receive(&rxMsg, 0) == ESP_OK) {
                if (rxMsg.identifier == 0x200 &&
                    rxMsg.data_length_code > 0 &&
                    rxMsg.data[0] == 0xCA) {
                    if (xSysMgrTask != NULL) {
                        xTaskNotifyGive(xSysMgrTask);
                    }
                }
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

