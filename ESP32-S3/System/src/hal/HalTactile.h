#pragma once
#include "../config/Config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

class HalTactile {
public:
    HalTactile();
    void begin();
    void update();
    TactileData getData();

private:
    SemaphoreHandle_t _mutex;
    TactileData* _data;  // 指针，堆分配
};

extern HalTactile tactile;