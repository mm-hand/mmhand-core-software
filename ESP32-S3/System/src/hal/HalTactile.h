#pragma once

#include "../config/Config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

struct TactileReadDiag {
    uint32_t frameCount;
    uint16_t enabledMask;
    uint16_t okReads;
    uint16_t failedReads;
    uint16_t okMask;
    uint8_t lastStatus;
    uint8_t lastRecvCrc;
    uint8_t lastCalcCrc;
    uint8_t lastFailedPattern;
    uint8_t lastFailStatus;
    uint8_t lastFailRecvCrc;
    uint8_t lastFailCalcCrc;
};

class HalTactile {
public:
    HalTactile();
    void prepareBootSelectPins();
    void begin();
    void update();
    TactileData getData();
    TactileReadDiag getDiagnostics();

private:
    bool setupSPI();
    void setAllSelectPinsHigh();
    void setSelectorPattern(uint8_t pattern);
    bool readRegisters(uint8_t selectorPattern, uint8_t* payload, uint16_t payloadLen);
    bool readSensorSummary(uint8_t groupIndex, uint8_t sensorIndex, uint8_t* globalOut, uint8_t* rawOut);
    static uint8_t crc8(const uint8_t* data, uint32_t len);

    SemaphoreHandle_t _mutex;
    TactileData* _data;
    spi_device_handle_t _spi;
    bool _spiReady;
    TactileReadDiag _diag;
    uint8_t _zeroHoldCounts[TACTILE_GROUP_NUM][TACTILE_SENSOR_PER_GROUP];
};

extern HalTactile tactile;
