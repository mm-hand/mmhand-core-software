#include "SystemTasks.h"
#include "../config/Config.h"
#include "../tasks/EncodersTask.h"
#include "../tasks/TactileTask.h"
#include "../tasks/CanTask.h"
#include "../tasks/SysMgrTask.h"

QueueHandle_t xQueueEncoderData = NULL;
QueueHandle_t xQueueTactileData = NULL;

void startSystemTasks() {
    xQueueEncoderData = xQueueCreate(1, sizeof(EncoderData));
    xQueueTactileData = xQueueCreate(1, sizeof(TactileData));
    if (xQueueEncoderData == NULL || xQueueTactileData == NULL) {
        Serial.println("[FATAL] Queue creation failed!");
        while (1) {
            vTaskDelay(1000);
        }
    }

    Serial.println("[SystemTasks] Queues created.");

    xTaskCreatePinnedToCore(
        Task_Encoders, "EncTask", 4096, NULL, 10, &xEncTask, 1
    );

    xTaskCreatePinnedToCore(
        Task_Tactile, "TacTask", 8192, NULL, 5, &xTacTask, 0
    );

    xTaskCreatePinnedToCore(
        Task_CanBus, "CanTask", 4096, NULL, 5, &xCanTask, 0
    );

    xTaskCreatePinnedToCore(
        Task_SysMgr, "SysMgr", 6144, NULL, 1, &xSysMgrTask, 1
    );

    Serial.println("[SystemTasks] All tasks created.");
}

