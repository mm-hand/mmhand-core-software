#include "SysMgrTask.h"
#include "../config/Config.h"
#include "../hal/HalEncoders.h"
#include "../hal/HalTWAI.h"
#include "esp_task_wdt.h"

// 任务句柄
TaskHandle_t xSysMgrTask = NULL;

// 其他任务句柄（外部声明，用于挂起和恢复）
extern TaskHandle_t xEncTask;
extern TaskHandle_t xTacTask;
extern TaskHandle_t xCanTask;

/**
 * 系统管理任务
 * 负责处理低频、耗时、全局性的操作（如校准、保存配置等）
 */
void Task_SysMgr(void *pvParameters) {
    Serial.println("[SysMgr] Manager Task Started (Waiting for CMD)...");

    for (;;) {
        // 等待信号 (无限期阻塞，此时不消耗 CPU)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        Serial.println("[SysMgr] Calibration Sequence Started...");

        // 1. 先获取数据 (此时 xEncTask 还在运行，确保 SPI 锁能正常获取和释放)
        EncoderData currentData = encoders.getData(); 
        
        // 2. 数据拿到后，再挂起任务 (为了安全的操作)
        if (xEncTask) {
            vTaskSuspend(xEncTask);
        }
        if (xTacTask) {
            vTaskSuspend(xTacTask);
        }
        if (xCanTask) {
            vTaskSuspend(xCanTask);
        }
        
        // 3. 喂系统级看门狗
        esp_task_wdt_reset();

        // 4. 再次喂狗
        esp_task_wdt_reset();

        // 5. 恢复任务
        if (xCanTask) {
            vTaskResume(xCanTask);
        }
        if (xTacTask) {
            vTaskResume(xTacTask);
        }
        if (xEncTask) {
            vTaskResume(xEncTask);
        }
        
        Serial.println("[SysMgr] Calibration Done & Saved.");

        // 6. 发送 CAN 反馈
        vTaskDelay(pdMS_TO_TICKS(50)); // 给 CAN 任务一点恢复时间
        twaiBus.sendCalibrationAck(true);
    }
}