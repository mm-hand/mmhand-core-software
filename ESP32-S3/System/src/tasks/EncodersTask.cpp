#include "EncodersTask.h"
#include "../config/Config.h"
#include "../hal/HalEncoders.h"

// 任务句柄
TaskHandle_t xEncTask = NULL;

// 队列（外部声明，在SystemTasks中定义）
extern QueueHandle_t xQueueEncoderData;

// 测试计数器
static int g_testCounter = 0;

/**
 * 编码器任务 (200Hz)
 * 负责读取编码器原始数据并发送到队列
 */
void Task_Encoders(void* pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(5); // 200Hz
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    // 本地数据缓冲
    EncoderData localData{};

    Serial.println("[Task_Encoders] Started!");

    for (;;) {
        // 1. 读取编码器原始数据
        EncoderData rawData = encoders.getData();
        
        // 复制原始数据到本地结构
        localData = rawData;

        // 打印测试结果 (每100次循环打印一次)
        if ((g_testCounter % 100) == 1) {
            Serial.println(">>> Encoders (raw Angle: 0~16383)");
            for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
                // [ID:数据] 格式优化
                Serial.printf("[%02d:%05d] ", i, localData.rawAngles[i]);
                // 每 5 个换行
                if ((i + 1) % 5 == 0) {
                    Serial.println();
                }
            }
            if (ENCODER_TOTAL_NUM % 5 != 0) {
                Serial.println();
            }
        }

        // 2. 处理错误标记
        for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
            if (localData.errorFlags[i]) {
                localData.rawAngles[i] = 0xFFFF;  // 错误标记
            }
        }

        // 3. 发送到队列
        if (xQueueEncoderData != NULL) {
            xQueueOverwrite(xQueueEncoderData, &localData);
            
            // 调试打印 (每秒一次，避免刷屏)
            static uint32_t lastPrint = 0;
            if (millis() - lastPrint > 1000) {
                // Serial.println("[Task_Encoders] Queue filled OK.");
                lastPrint = millis();
            }
        } else {
            // 错误：队列未创建
            static uint32_t lastErr = 0;
            if (millis() - lastErr > 2000) {
                Serial.println("[FATAL] xQueueEncoderData is NULL!");
                lastErr = millis();
            }
        }
        
        g_testCounter++;
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
