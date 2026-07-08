#include "EncodersTask.h"
#include "../config/Config.h"
#include "../hal/HalEncoders.h"

namespace {
constexpr bool kEnableEncoderDebugDump = true;
constexpr int kEncoderDebugDumpDecimation = 100;
constexpr uint32_t kEncoderTaskPeriodMs = 2;
}

// 浠诲姟鍙ユ焺
TaskHandle_t xEncTask = NULL;

// 闃熷垪锛堝閮ㄥ０鏄庯紝鍦⊿ystemTasks涓畾涔夛級
extern QueueHandle_t xQueueEncoderData;

static int g_testCounter = 0;

/**
 * 缂栫爜鍣ㄤ换鍔?(200Hz)
 * 璐熻矗璇诲彇缂栫爜鍣ㄥ師濮嬫暟鎹苟鍙戦€佸埌闃熷垪
 */
void Task_Encoders(void* pvParameters) {
    const TickType_t xFrequency = pdMS_TO_TICKS(kEncoderTaskPeriodMs);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    // 鏈湴鏁版嵁缂撳啿
    EncoderData localData{};

    Serial.println("[Task_Encoders] Started!");

    for (;;) {
        EncoderData rawData = encoders.getData();
        
        localData = rawData;

        // 鎵撳嵃娴嬭瘯缁撴灉 (姣?00娆″惊鐜墦鍗颁竴娆?
        if (kEnableEncoderDebugDump && (g_testCounter % kEncoderDebugDumpDecimation) == 1) {
            Serial.println(">>> Encoders (raw Angle: 0~16383)");
            for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
                // [ID:鏁版嵁] 鏍煎紡浼樺寲
                Serial.printf("[%02d:%05d] ", i, localData.rawAngles[i]);
                if ((i + 1) % 5 == 0) {
                    Serial.println();
                }
            }
            if (ENCODER_TOTAL_NUM % 5 != 0) {
                Serial.println();
            }
        }

        // 2. 澶勭悊閿欒鏍囪
        for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
            if (localData.errorFlags[i]) {
                localData.rawAngles[i] = 0xFFFF;  // 閿欒鏍囪
            }
        }

        // 3. 鍙戦€佸埌闃熷垪
        if (xQueueEncoderData != NULL) {
            xQueueOverwrite(xQueueEncoderData, &localData);
            
            // 璋冭瘯鎵撳嵃 (姣忕涓€娆★紝閬垮厤鍒峰睆)
            static uint32_t lastPrint = 0;
            if (millis() - lastPrint > 1000) {
                // Serial.println("[Task_Encoders] Queue filled OK.");
                lastPrint = millis();
            }
        } else {
            // 閿欒锛氶槦鍒楁湭鍒涘缓
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
