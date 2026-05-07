#include <Arduino.h>
#include "src/config/Config.h"
#include "src/hal/HalEncoders.h"
#include "src/hal/HalTactile.h"
#include "src/hal/HalTWAI.h"
#include "src/system/SystemTasks.h"
#include "esp_task_wdt.h"
#include "esp_idf_version.h"
#include "esp_log.h"

extern TaskHandle_t xEncTask;
extern TaskHandle_t xTacTask;
extern TaskHandle_t xCanTask;

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n\n--- System Boot (XSimple AI) ---");

#if ESP_IDF_VERSION_MAJOR >= 5
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 1000,
        .idle_core_mask = (1 << 0) | (1 << 1),
        .trigger_panic = true,
    };
    esp_task_wdt_init(&wdt_config);
#else
    esp_task_wdt_init(1, true);
#endif

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("TWDT", ESP_LOG_ERROR);

    encoders.begin();
    Serial.println("[Init] Encoders... OK");

    tactile.begin();
    Serial.println("[Init] Tactile... OK");

    if (twaiBus.begin()) {
        Serial.println("[Init] TWAI CAN... OK");
    } else {
        Serial.println("[Init] TWAI CAN... FAILED");
    }

    startSystemTasks();
    Serial.println("[System] Tasks Started. Main Loop Running.");
}

void printSystemMonitor() {
    EncoderData enc = encoders.getData();

    Serial.println("\n======= [ XSimple Monitor ] =======");
    Serial.println(">>> Encoders (Final Angle: 0~16383)");
    for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
        Serial.printf("[%02d:%05u] ", i, enc.rawAngles[i]);
        if ((i + 1) % 5 == 0) {
            Serial.println();
        }
    }
    if (ENCODER_TOTAL_NUM % 5 != 0) {
        Serial.println();
    }

    Serial.println("===================================");
}

void loop() {
    static uint32_t lastPrintTime = 0;
    if (millis() - lastPrintTime > 500) {
        lastPrintTime = millis();
        printSystemMonitor();
    }

    vTaskDelay(pdMS_TO_TICKS(100));
}
