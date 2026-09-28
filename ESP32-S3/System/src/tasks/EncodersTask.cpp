#include "EncodersTask.h"
#include "../config/Config.h"
#include "../hal/HalEncoders.h"

namespace {
// [待确认是否关闭] 当前每约200ms打印一次21路角度，会占用921600波特率串口和CPU。
constexpr bool kEnableEncoderDebugDump = true;
constexpr int kEncoderDebugDumpDecimation = 100;
// 2ms周期对应理论500Hz；实际周期还受5组SPI传输和互斥等待影响。
constexpr uint32_t kEncoderTaskPeriodMs = 2;
}

// 任务句柄由SystemTasks::startSystemTasks写入。
TaskHandle_t xEncTask = NULL;

// 长度为1的覆盖队列，由SystemTasks创建，CanTask读取。
extern QueueHandle_t xQueueEncoderData;

// [待确认是否简化] 该计数器仅服务串口抽样打印，可改为函数内static或毫秒节拍。
static int g_testCounter = 0;

/**
 * @brief 周期采集21路AS5047P，并把最新有效性快照交给CAN任务。
 * @param pvParameters FreeRTOS预留任务参数，当前未使用。
 * @note 错误通道统一写成0xFFFF，使P4不会把旧值或0误认为有效角度。
 */
void Task_Encoders(void* pvParameters) {
    (void)pvParameters;
    const TickType_t xFrequency = pdMS_TO_TICKS(kEncoderTaskPeriodMs);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    // localData是提交给队列的一致快照，避免采集对象在发送过程中变化。
    EncoderData localData{};

    Serial.println("[Task_Encoders] Started!");

    for (;;) {
        // [待确认是否简化] rawData到localData发生一次完整结构体复制，可直接调用getData(localData)。
        EncoderData rawData = encoders.getData();
        localData = rawData;

        // 抽样输出Raw值；用于排查链序和FPC，不参与控制。
        if (kEnableEncoderDebugDump && (g_testCounter % kEncoderDebugDumpDecimation) == 1) {
            Serial.println(">>> Encoders (raw Angle: 0~16383)");
            for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
                Serial.printf("[%02d:%05d] ", i, localData.rawAngles[i]);
                if ((i + 1) % 5 == 0) {
                    Serial.println();
                }
            }
            if (ENCODER_TOTAL_NUM % 5 != 0) {
                Serial.println();
            }
        }

        // CAN原始角度协议用0xFFFF表示该通道不可用。
        for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
            if (localData.errorFlags[i]) {
                localData.rawAngles[i] = 0xFFFF;
            }
        }

        if (xQueueEncoderData != NULL) {
            // 队列长度为1，覆盖写保证CanTask永远拿到最新数据，不积压旧采样。
            xQueueOverwrite(xQueueEncoderData, &localData);
        } else {
            // 队列异常时限频打印，避免故障日志淹没串口。
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
