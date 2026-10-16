#include <Arduino.h>
#if !ARDUINO_USB_MODE || !ARDUINO_USB_CDC_ON_BOOT
#error "S3 requires Hardware CDC/JTAG and USB CDC On Boot Enabled"
#endif
#include "src/config/Config.h"
#include "src/hal/HalEncoders.h"
#include "src/hal/HalTactile.h"
#include "src/hal/HalTWAI.h"
#include "src/system/SystemTasks.h"
#include "esp_task_wdt.h"
#include "esp_idf_version.h"
#include "esp_log.h"

namespace {
// 上电后等待外围电源、下载串口和传感器选择线稳定；该延时会直接增加10秒启动时间。这里计划修改只等待5S即可
constexpr uint32_t kBootSettleDelayMs = 5000;
}

/**
 * @brief Arduino启动入口：初始化日志、看门狗、磁编、触觉、CAN和FreeRTOS任务。
 * @note 外设初始化完成后，实际周期工作均由FreeRTOS任务承担。
 */
void setup() {
    // USB is an optional observer. A closed/unread CDC port must never stall
    // sensor acquisition or CAN, including the low-rate diagnostic writers.
    Serial.setTxBufferSize(4096);
    Serial.setTxTimeoutMs(0);
    Serial.begin(921600);
#if ENABLE_TACTILE_TASK
    // 延时前先把触觉片选线置为非选中，防止上电阶段误选多个传感器。
    tactile.prepareBootSelectPins();
#endif
    delay(kBootSettleDelayMs);
    Serial.println("\n\n--- System Boot (XSimple AI) ---");

#if ESP_IDF_VERSION_MAJOR >= 5
    // 监控两个CPU核的Idle任务；任务长期占满CPU时触发panic，便于定位死循环。
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

#if ENABLE_TACTILE_TASK
    tactile.begin();
#endif
    Serial.println("[Init] Tactile... OK");

    if (twaiBus.begin()) {
        Serial.println("[Init] TWAI CAN... OK");
    } else {
        Serial.println("[Init] TWAI CAN... FAILED");
    }

    startSystemTasks();
    Serial.println("[System] Tasks Started. Main Loop Running.");
}

void loop() {
    // Arduino主循环不承载业务，主动让出CPU给FreeRTOS任务。
    vTaskDelay(pdMS_TO_TICKS(100));
}
