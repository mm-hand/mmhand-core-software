#include "TactileTask.h"
#include "../config/Config.h"
#include "../hal/HalTactile.h"

// 任务句柄
TaskHandle_t xTacTask = NULL;

/**
 * 触觉任务 (10Hz)
 * 负责处理触觉传感器数据
 */
void Task_Tactile(void* pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(100);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    Serial.println("[Task_Tactile] Started!");

    for (;;) {
        // tactile.update();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}