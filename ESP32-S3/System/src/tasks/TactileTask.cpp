#include "TactileTask.h"
#include "../config/Config.h"
#include "../hal/HalTactile.h"

TaskHandle_t xTacTask = NULL;

extern QueueHandle_t xQueueTactileData;

void Task_Tactile(void* pvParameters) {
    (void)pvParameters;

    const TickType_t xFrequency = pdMS_TO_TICKS(5); // 200Hz task tick
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint8_t publishDivider = 0;
    TactileData snapshot{};

    Serial.println("[Task_Tactile] Started!");

    for (;;) {
        tactile.update();

        // Publish at 50Hz to keep queue copy cost bounded.
        publishDivider++;
        if (publishDivider >= 4) {
            publishDivider = 0;
            snapshot = tactile.getData();
            if (xQueueTactileData != NULL) {
                xQueueOverwrite(xQueueTactileData, &snapshot);
            }
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

