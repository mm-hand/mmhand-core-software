#include "SystemTasks.h"
#include "../config/Config.h"
// 各任务头文件提供任务入口和全局任务句柄声明。
#include "../tasks/EncodersTask.h"
#include "../tasks/TactileTask.h"
#include "../tasks/CanTask.h"

// EncTask生产、CanTask消费；长度1配合xQueueOverwrite避免历史数据积压。
QueueHandle_t xQueueEncoderData = NULL;

/**
 * @brief 创建磁编覆盖队列，并按统一配置启动采集与通信任务。
 * @note 队列创建失败时停在低频循环，防止任务带病运行。
 */
void startSystemTasks() {
    // 只传最新磁编快照，不需要排队保存旧采样。
    xQueueEncoderData = xQueueCreate(1, sizeof(EncoderData));
    if (xQueueEncoderData == NULL) {
        Serial.println("[FATAL] Queue creation failed!");
        while (1) {
            vTaskDelay(1000);
        }
    }

    Serial.println("[SystemTasks] Queue created successfully!");

    // 磁编2ms采样，优先级最高，固定在Core1。
    xTaskCreatePinnedToCore(
        Task_Encoders, "EncTask", 4096, NULL, TASK_ENC_PRIO, &xEncTask, TASK_ENC_CORE
    );

    // 触觉约33ms采样，固定在Core0；可通过编译开关关闭。
#if ENABLE_TACTILE_TASK
    xTaskCreatePinnedToCore(
        Task_Tactile, "TacTask", 8192, NULL, TASK_TAC_PRIO, &xTacTask, TASK_TAC_CORE
    );
#else
    xTacTask = NULL;
    Serial.println("[SystemTasks] Tactile task disabled (encoder-only test).");
#endif

    // CAN每4ms发送最新磁编快照，与触觉任务同在Core0。
    xTaskCreatePinnedToCore(
        Task_CanBus, "CanTask", 4096, NULL, TASK_CAN_PRIO, &xCanTask, TASK_CAN_CORE
    );

    Serial.println("[SystemTasks] All tasks created successfully!");
}
