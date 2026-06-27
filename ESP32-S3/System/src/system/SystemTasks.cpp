#include "SystemTasks.h"
#include "../config/Config.h"
// 包含所有任务的头文件
#include "../tasks/EncodersTask.h"
#include "../tasks/TactileTask.h"
#include "../tasks/CanTask.h"
#include "../tasks/SysMgrTask.h"

// 队列定义
QueueHandle_t xQueueEncoderData = NULL;
SemaphoreHandle_t g_sensorSelectMutex = NULL;

/**
 * 启动系统任务
 * 负责创建队列和所有任务
 */
void startSystemTasks() {
    // 1. 创建队列
    xQueueEncoderData = xQueueCreate(1, sizeof(EncoderData));
    if (xQueueEncoderData == NULL) {
        Serial.println("[FATAL] Queue creation failed!");
        while(1) {
            vTaskDelay(1000);
        }
    }

    Serial.println("[SystemTasks] Queue created successfully!");

    if (g_sensorSelectMutex == NULL) {
        g_sensorSelectMutex = xSemaphoreCreateMutex();
        if (g_sensorSelectMutex == NULL) {
            Serial.println("[FATAL] Sensor selector mutex creation failed!");
            while(1) {
                vTaskDelay(1000);
            }
        }
    }
    
    // 2. 创建编码器任务 - Core 1, 高优先级
    xTaskCreatePinnedToCore(
        Task_Encoders, "EncTask", 4096, NULL, 10, &xEncTask, 1
    );

    // 3. 创建触觉任务 - Core 0, 中优先级
#if ENABLE_TACTILE_TASK
    xTaskCreatePinnedToCore(
        Task_Tactile, "TacTask", 8192, NULL, 5, &xTacTask, 0
    );
#else
    xTacTask = NULL;
    Serial.println("[SystemTasks] Tactile task disabled (encoder-only test).");
#endif

    // 4. 创建 CAN 任务 - Core 0, 中优先级
    xTaskCreatePinnedToCore(
        Task_CanBus, "CanTask", 4096, NULL, 5, &xCanTask, 0
    );

    // 5. 创建系统管理任务 - Core 1, 低优先级
    xTaskCreatePinnedToCore(
        Task_SysMgr, "SysMgr", 6144, NULL, 1, &xSysMgrTask, 1 
    );
    
    Serial.println("[SystemTasks] All tasks created successfully!");
}
