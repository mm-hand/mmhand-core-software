#include "HalTactile.h"
#include <cstring>

// 全局实例
HalTactile tactile;

HalTactile::HalTactile() {
    _mutex = xSemaphoreCreateMutex();
    // 在堆上分配大结构体，避免栈溢出
    _data = new TactileData();
    memset(_data, 0, sizeof(TactileData));
}

void HalTactile::begin() {
    pinMode(PIN_TAC_CS_A, OUTPUT);
    digitalWrite(PIN_TAC_CS_A, HIGH);
    pinMode(PIN_TAC_CS_B, OUTPUT);
    digitalWrite(PIN_TAC_CS_B, HIGH);

    // SPI 初始化代码放这里...
}

void HalTactile::update() {
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(10))) {
        // 模拟数据更新 (实际替换为 SPI 读取)
        _data->groups[0].sensor_A.global[0]++;

        xSemaphoreGive(_mutex);
    }
}

TactileData HalTactile::getData() {
    // 使用 static 避免在栈上分配大结构体
    static TactileData ret;
    
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(10))) {
        ret = *_data;
        xSemaphoreGive(_mutex);
    }
    return ret;
}