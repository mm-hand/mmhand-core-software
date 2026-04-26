#pragma once

#include "../config/Config.h"
#include <SPI.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

class HalTactile {
public:
    HalTactile();
    void begin();
    void update();
    TactileData getData();

private:
    static constexpr uint8_t kGroupCount = TACTILE_GROUP_NUM;
    static constexpr uint8_t kSensorsPerGroup = 3;
    static constexpr uint8_t kTotalSensors = kGroupCount * kSensorsPerGroup;
    static constexpr uint16_t kReadAddr = 1008;
    static constexpr uint16_t kReadLen = 15;

    struct SensorRoute {
        uint8_t muxValue;
        uint8_t sensorIdx;
        bool useCsA;
        bool useCsB;
    };

    bool readSensorGlobal(const SensorRoute& route, uint8_t outGlobal[3]);
    void setMux(uint8_t muxValue);
    uint8_t calculateCRC8(const uint8_t* data, uint16_t len) const;

    SemaphoreHandle_t _mutex;
    SPIClass* _spi;
    TactileData* _data;
    uint8_t _scanIndex;
};

extern HalTactile tactile;

