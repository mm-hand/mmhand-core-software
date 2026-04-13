#include <Arduino.h>
#include "src/config/Config.h"
#include "src/hal/HalEncoders.h"
#include "src/hal/HalTactile.h"
#include "src/hal/HalTWAI.h"
#include "src/system/SystemTasks.h"
#include "esp_task_wdt.h"
#include "esp_log.h" // 引入日志头文件

// 声明外部可能用到的句柄，防止编译错误（如果在 .h 中未申明 extern）
extern TaskHandle_t xEncTask;
extern TaskHandle_t xTacTask;
extern TaskHandle_t xCanTask;

void setup() {
    Serial.begin(115200);
    // 等待串口稳定，防止启动日志丢失
    delay(2000); 
    Serial.println("\n\n--- System Boot (XSimple AI) ---");

    // --------------------------------------------------------
    // [看门狗配置] ESP-IDF v5.x
    // --------------------------------------------------------
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 1000,                    // 10秒超时
        .idle_core_mask = (1 << 0) | (1 << 1), // 监控双核空闲任务
        .trigger_panic = true                  // 超时触发 Panic 重启
    };
    esp_task_wdt_init(&wdt_config);

    // [日志配置]
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("TWDT", ESP_LOG_ERROR);

    // --------------------------------------------------------
    // [硬件初始化]
    // --------------------------------------------------------
    
    // 1. 编码器
    encoders.begin();
    Serial.println("[Init] Encoders... OK");

    // 2. 触觉传感器
    tactile.begin();
    Serial.println("[Init] Tactile... OK");

    // 3. CAN 总线
    if (twaiBus.begin()) {
        Serial.println("[Init] TWAI CAN... OK");
    } else {
        Serial.println("[Init] TWAI CAN... FAILED");
    }


    // --------------------------------------------------------
    // [RTOS 任务启动]
    // --------------------------------------------------------
    startSystemTasks();
    Serial.println("[System] Tasks Started. Main Loop Running.");
}


// --------------------------------------------------------
// [系统监控打印]
// --------------------------------------------------------
void printSystemMonitor() {
    // 获取最新数据快照
    // 注意：getData() 可能会根据 HalEncoders 的实现触发 SPI，
    // 如果想要纯被动读取，建议 HalEncoders 提供一个 getCachedData() 接口
    EncoderData enc = encoders.getData(); 

    // 清屏 (可选)
    // Serial.print("\033[2J\033[H"); 

    Serial.println("\n======= [ XSimple Monitor ] =======");

    // --- 编码器数据 ---
    Serial.println(">>> Encoders (Final Angle: 0~16383)");
    for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
        // [ID:数据] 格式优化
        Serial.printf("[%02d:%05h] ", i, enc.rawAngles[i]);
        // 每 5 个换行
        if ((i + 1) % 5 == 0) Serial.println();
    }
    if (ENCODER_TOTAL_NUM % 5 != 0) Serial.println();

    // --- 触觉数据 (如需开启请取消注释) ---
    /*
    TactileData tac = tactile.getData();
    Serial.println(">>> Tactile Sensors");
    // ... 打印逻辑 ...
    */

    Serial.println("===================================");
}

void loop() {
    // 1. 处理串口指令
    // handleSerialCommands();

    // 2. 定时监控打印 (非阻塞)
    static uint32_t lastPrintTime = 0;
    if (millis() - lastPrintTime > 500) {
        lastPrintTime = millis();
        // printSystemMonitor();
    }

    // 3. 让出 CPU 给低优先级任务 (Idle Task 需要运行以喂看门狗)
    vTaskDelay(pdMS_TO_TICKS(100));
}