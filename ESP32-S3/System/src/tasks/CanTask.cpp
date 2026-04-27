#include "CanTask.h"
#include "../config/Config.h"
#include "../hal/HalTWAI.h"

// 任务句柄
TaskHandle_t xCanTask = NULL;

// 队列（外部声明，在SystemTasks中定义）
extern QueueHandle_t xQueueEncoderData;

// 系统管理任务句柄（外部声明，用于发送通知）
extern TaskHandle_t xSysMgrTask;

/**
 * CAN 通信任务 (100Hz)
 * 负责发送编码器数据和接收远程命令
 */
void Task_CanBus(void *pvParameters) {
    (void)pvParameters;
    
    EncoderData txData{};
    static uint8_t errorSendCounter = 0;
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);

    Serial.println("[Task_CanBus] Started!");

    for (;;) {
        // 1. 维护总线状态 (自恢复、报警)
        bool isBusOk = twaiBus.maintain();

        if (isBusOk) {
            // 2. 从队列获取最新数据
            if (xQueueEncoderData && xQueueReceive(xQueueEncoderData, &txData, 0) == pdTRUE) {
                
                // 3. 发送角度数据
                twaiBus.sendEncoderData(txData);

                // 4. 每隔5帧检查并发送错误状态
                errorSendCounter++;
                if (errorSendCounter >= 5) {
                    errorSendCounter = 0;
                    
                    // 检查是否有错误
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

            // 5. 接收处理
            twai_message_t rxMsg;
            if (twai_receive(&rxMsg, 0) == ESP_OK) {
                
                // 收到校准指令 (ID:0x200, Data:0xCA)
                if (rxMsg.identifier == 0x200 && rxMsg.data_length_code > 0 && rxMsg.data[0] == 0xCA) {
                    
                    // 发送通知给 SysMgr 任务
                    if (xSysMgrTask != NULL) {
                        xTaskNotifyGive(xSysMgrTask); 
                    }
                }
            }
        } else {
            // 总线故障中 - 降低频率，给驱动恢复时间
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
