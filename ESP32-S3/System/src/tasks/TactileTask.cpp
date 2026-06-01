#include "TactileTask.h"
#include "../config/Config.h"
#include "../hal/HalTactile.h"
#include "../hal/HalTWAI.h"

// 任务句柄
TaskHandle_t xTacTask = NULL;

namespace {
uint8_t axisMagnitude(const uint8_t* axes) {
    const int x = abs((int8_t)axes[0]);
    const int y = abs((int8_t)axes[1]);
    const int z = axes[2];
    int value = x;
    if (y > value) {
        value = y;
    }
    if (z > value) {
        value = z;
    }
    return (uint8_t)value;
}

void printVec(const uint8_t* axes) {
    Serial.printf("%+04d,%+04d,%03u", (int8_t)axes[0], (int8_t)axes[1], axes[2]);
}
} // namespace

/**
 * 触觉任务 (10Hz)
 * 负责处理触觉传感器数据
 */
void Task_Tactile(void* pvParameters) {
    (void)pvParameters;
    const TickType_t xFrequency = pdMS_TO_TICKS(50);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    Serial.println("[Task_Tactile] Started!");

    for (;;) {
        tactile.update();
        twaiBus.sendTactileSummary(tactile.getData());

        static uint32_t lastPrint = 0;
        if (millis() - lastPrint > 1000) {
            lastPrint = millis();
            const TactileReadDiag diag = tactile.getDiagnostics();
            const TactileData data = tactile.getData();
            Serial.printf(
                "[Tactile] frame=%lu enabled=%04X ok=%u fail=%u mask=%04X miss=%04X st=%02X crc=%02X/%02X\n",
                (unsigned long)diag.frameCount,
                diag.enabledMask,
                diag.okReads,
                diag.failedReads,
                diag.okMask,
                (uint16_t)(diag.enabledMask & ~diag.okMask),
                diag.lastStatus,
                diag.lastRecvCrc,
                diag.lastCalcCrc
            );
            Serial.print("[TactileFz]");
            for (uint8_t g = 0; g < TACTILE_GROUP_NUM; g++) {
                const TacGroup& grp = data.groups[g];
                Serial.printf(
                    " G%u:%03u,%03u,%03u",
                    g,
                    grp.sensor_A.global[2],
                    grp.sensor_B.global[2],
                    grp.sensor_C.global[2]
                );
            }
            Serial.println();

            Serial.print("[TactileMag]");
            for (uint8_t g = 0; g < TACTILE_GROUP_NUM; g++) {
                const TacGroup& grp = data.groups[g];
                Serial.printf(
                    " G%u:%03u,%03u,%03u",
                    g,
                    axisMagnitude(grp.sensor_A.global),
                    axisMagnitude(grp.sensor_B.global),
                    axisMagnitude(grp.sensor_C.global)
                );
            }
            Serial.println();

        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
