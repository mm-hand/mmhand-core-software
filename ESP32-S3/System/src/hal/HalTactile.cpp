#include "HalTactile.h"

#include <SPI.h>
#include <cstring>

namespace {
constexpr uint8_t kCmdRead = 0xFB;
constexpr uint16_t kLenCmdHead = 5;
constexpr uint16_t kLenStatus = 1;
constexpr uint16_t kLenCrc = 1;
constexpr uint16_t kForceSummaryAddr = 1008;
constexpr uint16_t kForceSummaryLen = 3;
constexpr uint16_t kDelayCsHoldUs = 10;
constexpr uint16_t kDelayResponseUs = 100;
constexpr uint16_t kDelaySelectSettleUs = 80;
constexpr uint8_t kReadRetryCount = 3;
constexpr uint8_t kRetryDelayMs = 8;
constexpr uint8_t kZeroNoiseFloor = 3;
constexpr uint8_t kZeroHoldFrames = 6;
constexpr uint8_t kHoldMinMagnitude = 12;

struct SensorPattern {
    bool enabled;
    uint8_t pattern;
};

// Logical tactile order follows the assembled finger order:
// G0=pinky(SPI5), G1=ring(SPI4), G2=middle(SPI3), G3=index(SPI2), G4=thumb(SPI1).
// Pattern bits are GPIO17 GPIO14 GPIO13 GPIO6 GPIO7 as printed by the scanner.
// Disabled slots are left zero so the 5x3 upload format stays compatible while
// Each finger uses tip/middle/base sensors in desktop order.
constexpr SensorPattern kSensorPatterns[TACTILE_GROUP_NUM][TACTILE_SENSOR_PER_GROUP] = {
    {{true, 0x13}, {true, 0x12}, {true, 0x11}}, // pinky / SPI5: 10011, 10010, 10001
    {{true, 0x0F}, {true, 0x0E}, {true, 0x0D}}, // ring / SPI4: 01111, 01110, 01101
    {{true, 0x0B}, {true, 0x0A}, {true, 0x09}}, // middle: 01011, 01010, 01001
    {{true, 0x07}, {true, 0x06}, {true, 0x05}}, // index / SPI2: 00111, 00110, 00101
    {{true, 0x03}, {true, 0x02}, {true, 0x01}}, // thumb / SPI1: 00011, 00010, 00001
};

SPIClass tactileSpi(HSPI);

void configureSelectPinHighWithPullup(int pin) {
    if (pin < 0) {
        return;
    }

    pinMode(pin, INPUT_PULLUP);
    digitalWrite(pin, HIGH);
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
    gpio_pulldown_dis((gpio_num_t)pin);
    gpio_pullup_en((gpio_num_t)pin);
}

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

void sensorPointers(TactileData& data, uint8_t group, uint8_t sensor, uint8_t** global, uint8_t** rawSummary) {
    TacGroup& grp = data.groups[group];
    uint8_t* globals[TACTILE_SENSOR_PER_GROUP] = {
        grp.sensor_A.global,
        grp.sensor_B.global,
        grp.sensor_C.global,
    };
    uint8_t* raws[TACTILE_SENSOR_PER_GROUP] = {
        grp.sensor_A.rawSummary,
        grp.sensor_B.rawSummary,
        grp.sensor_C.rawSummary,
    };
    if (global) {
        *global = globals[sensor];
    }
    if (rawSummary) {
        *rawSummary = raws[sensor];
    }
}

void copySensor(TactileData& dst, const TactileData& src, uint8_t group, uint8_t sensor) {
    TactileData& mutableSrc = const_cast<TactileData&>(src);
    uint8_t* dstGlobal = nullptr;
    uint8_t* dstRaw = nullptr;
    uint8_t* srcGlobal = nullptr;
    uint8_t* srcRaw = nullptr;
    sensorPointers(dst, group, sensor, &dstGlobal, &dstRaw);
    sensorPointers(mutableSrc, group, sensor, &srcGlobal, &srcRaw);
    memcpy(dstGlobal, srcGlobal, TACTILE_AXIS_COUNT);
    memcpy(dstRaw, srcRaw, kForceSummaryLen);
}

constexpr uint8_t kCrc8Table[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04,
    0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8,
    0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10,
    0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C,
    0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0,
    0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C,
    0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54,
    0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
    0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40,
};
} // namespace

HalTactile tactile;

HalTactile::HalTactile() : _mutex(xSemaphoreCreateMutex()), _data(new TactileData()), _spi(nullptr), _spiReady(false), _diag{}, _zeroHoldCounts{} {
    memset(_data, 0, sizeof(TactileData));
}

void HalTactile::prepareBootSelectPins() {
    configureSelectPinHighWithPullup(PIN_MUX_A);
    configureSelectPinHighWithPullup(PIN_MUX_B);
    configureSelectPinHighWithPullup(PIN_MUX_C);
    configureSelectPinHighWithPullup(PIN_MUX_G);

    configureSelectPinHighWithPullup(PIN_TAC_CS_A);
    configureSelectPinHighWithPullup(PIN_TAC_CS_B);

    configureSelectPinHighWithPullup(PIN_TAC_GRP_A);
    configureSelectPinHighWithPullup(PIN_TAC_GRP_B);
    configureSelectPinHighWithPullup(PIN_TAC_GRP_C);

#if PIN_TAC_GRP_EN >= 0
    configureSelectPinHighWithPullup(PIN_TAC_GRP_EN);
#endif
    setAllSelectPinsHigh();
}

void HalTactile::begin() {
    prepareBootSelectPins();
    _spiReady = setupSPI();
    if (!_spiReady) {
        Serial.println("[Tactile] SPI init failed");
    }
}

bool HalTactile::setupSPI() {
    tactileSpi.begin(PIN_TAC_SCLK, PIN_TAC_MISO, PIN_TAC_MOSI, -1);
    setAllSelectPinsHigh();
    return true;
}

uint8_t HalTactile::crc8(const uint8_t* data, uint32_t len) {
    uint8_t crc = 0xFF;
    while (len--) {
        crc = kCrc8Table[crc ^ *data++];
    }
    return crc;
}

void HalTactile::setAllSelectPinsHigh() {
    digitalWrite(PIN_MUX_A, HIGH);
    digitalWrite(PIN_MUX_B, HIGH);
    digitalWrite(PIN_MUX_C, HIGH);
    digitalWrite(PIN_MUX_G, HIGH);
    digitalWrite(PIN_TAC_CS_A, HIGH);
    digitalWrite(PIN_TAC_CS_B, HIGH);
    digitalWrite(PIN_TAC_GRP_A, HIGH);
    digitalWrite(PIN_TAC_GRP_B, HIGH);
    digitalWrite(PIN_TAC_GRP_C, HIGH);
#if PIN_TAC_GRP_EN >= 0
    digitalWrite(PIN_TAC_GRP_EN, PIN_TAC_GRP_EN_ACTIVE_LEVEL == HIGH ? LOW : HIGH);
#endif
    delayMicroseconds(kDelaySelectSettleUs);
}

void HalTactile::setSelectorPattern(uint8_t pattern) {
    digitalWrite(PIN_TAC_CS_A, (pattern & 0x01) ? HIGH : LOW);
    digitalWrite(PIN_TAC_CS_B, (pattern & 0x02) ? HIGH : LOW);
    digitalWrite(PIN_TAC_GRP_A, (pattern & 0x04) ? HIGH : LOW);
    digitalWrite(PIN_TAC_GRP_B, (pattern & 0x08) ? HIGH : LOW);
    digitalWrite(PIN_TAC_GRP_C, (pattern & 0x10) ? HIGH : LOW);
#if PIN_TAC_GRP_EN >= 0
    digitalWrite(PIN_TAC_GRP_EN, PIN_TAC_GRP_EN_ACTIVE_LEVEL);
#endif
    delayMicroseconds(kDelaySelectSettleUs);
}

bool HalTactile::readRegisters(uint8_t selectorPattern, uint8_t* payload, uint16_t payloadLen) {
    if (!_spiReady || !payload || payloadLen == 0 || payloadLen > kForceSummaryLen) {
        return false;
    }

    uint8_t raw[kLenCmdHead + kLenStatus + kForceSummaryLen + kLenCrc] = {0};
    const uint16_t totalLen = kLenCmdHead + kLenStatus + payloadLen + kLenCrc;

    raw[0] = kCmdRead;
    raw[1] = (uint8_t)(kForceSummaryAddr & 0xFF);
    raw[2] = (uint8_t)(kForceSummaryAddr >> 8);
    raw[3] = (uint8_t)(payloadLen & 0xFF);
    raw[4] = (uint8_t)(payloadLen >> 8);

    tactileSpi.beginTransaction(SPISettings(SPI_FREQ_TAC, MSBFIRST, SPI_MODE3));
    setAllSelectPinsHigh();
    setSelectorPattern(selectorPattern);
    delayMicroseconds(kDelayCsHoldUs);

    for (uint16_t i = 0; i < kLenCmdHead; i++) {
        tactileSpi.transfer(raw[i]);
    }
    delayMicroseconds(kDelayResponseUs);
    for (uint16_t i = kLenCmdHead; i < totalLen; i++) {
        raw[i] = tactileSpi.transfer(0x00);
    }

    delayMicroseconds(kDelayCsHoldUs);
    setAllSelectPinsHigh();
    tactileSpi.endTransaction();

    const uint8_t recvCrc = raw[totalLen - 1];
    const uint8_t calcCrc = crc8(raw, totalLen - 1);
    _diag.lastStatus = raw[kLenCmdHead];
    _diag.lastRecvCrc = recvCrc;
    _diag.lastCalcCrc = calcCrc;

    if (recvCrc != calcCrc) {
        _diag.lastFailedPattern = selectorPattern;
        _diag.lastFailStatus = raw[kLenCmdHead];
        _diag.lastFailRecvCrc = recvCrc;
        _diag.lastFailCalcCrc = calcCrc;
        memset(payload, 0, payloadLen);
        return false;
    }

    memcpy(payload, raw + kLenCmdHead + kLenStatus, payloadLen);
    return true;
}

bool HalTactile::readSensorSummary(uint8_t groupIndex, uint8_t sensorIndex, uint8_t* globalOut, uint8_t* rawOut) {
    if (!globalOut || groupIndex >= TACTILE_GROUP_NUM || sensorIndex >= TACTILE_SENSOR_PER_GROUP) {
        return false;
    }

    const SensorPattern cfg = kSensorPatterns[groupIndex][sensorIndex];
    if (!cfg.enabled) {
        memset(globalOut, 0, TACTILE_AXIS_COUNT);
        if (rawOut) {
            memset(rawOut, 0, kForceSummaryLen);
        }
        return false;
    }

    uint8_t payload[kForceSummaryLen] = {0};
    const uint8_t selectorPattern = cfg.pattern;

    bool ok = false;
    for (uint8_t attempt = 0; attempt < kReadRetryCount; attempt++) {
        ok = readRegisters(selectorPattern, payload, kForceSummaryLen);
        if (ok) {
            break;
        }
        delay(kRetryDelayMs);
    }
    if (ok) {
        globalOut[0] = payload[0];
        globalOut[1] = payload[1];
        globalOut[2] = payload[2];
        if (rawOut) {
            memcpy(rawOut, payload, kForceSummaryLen);
        }
    } else {
        globalOut[0] = 0;
        globalOut[1] = 0;
        globalOut[2] = 0;
        if (rawOut) {
            memset(rawOut, 0, kForceSummaryLen);
        }
    }
    return ok;
}

void HalTactile::update() {
    TactileData next{};
    TactileReadDiag nextDiag = _diag;
    nextDiag.frameCount++;
    nextDiag.enabledMask = 0;
    nextDiag.okReads = 0;
    nextDiag.failedReads = 0;
    nextDiag.okMask = 0;

    if (_spiReady) {
        const bool locked = (g_sensorMuxMutex == NULL) ||
            (xSemaphoreTake(g_sensorMuxMutex, portMAX_DELAY) == pdTRUE);
        for (uint8_t g = 0; g < TACTILE_GROUP_NUM; g++) {
            TacGroup& grp = next.groups[g];
            uint8_t* globals[TACTILE_SENSOR_PER_GROUP] = {
                grp.sensor_A.global,
                grp.sensor_B.global,
                grp.sensor_C.global,
            };
            uint8_t* rawSummaries[TACTILE_SENSOR_PER_GROUP] = {
                grp.sensor_A.rawSummary,
                grp.sensor_B.rawSummary,
                grp.sensor_C.rawSummary,
            };
            for (uint8_t s = 0; s < TACTILE_SENSOR_PER_GROUP; s++) {
                if (!kSensorPatterns[g][s].enabled) {
                    memset(globals[s], 0, TACTILE_AXIS_COUNT);
                    memset(rawSummaries[s], 0, kForceSummaryLen);
                    continue;
                }
                nextDiag.enabledMask |= (uint16_t)(1U << (g * TACTILE_SENSOR_PER_GROUP + s));
                if (readSensorSummary(g, s, globals[s], rawSummaries[s])) {
                    nextDiag.okReads++;
                    nextDiag.okMask |= (uint16_t)(1U << (g * TACTILE_SENSOR_PER_GROUP + s));
                } else {
                    nextDiag.failedReads++;
                }
            }
        }
        if (locked && g_sensorMuxMutex != NULL) {
            xSemaphoreGive(g_sensorMuxMutex);
        }
    } else {
        nextDiag.failedReads = TACTILE_GROUP_NUM * TACTILE_SENSOR_PER_GROUP;
    }

    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(10))) {
        for (uint8_t g = 0; g < TACTILE_GROUP_NUM; g++) {
            for (uint8_t s = 0; s < TACTILE_SENSOR_PER_GROUP; s++) {
                const uint16_t bit = (uint16_t)(1U << (g * TACTILE_SENSOR_PER_GROUP + s));
                if ((nextDiag.enabledMask & bit) == 0) {
                    _zeroHoldCounts[g][s] = 0;
                    continue;
                }

                uint8_t* currGlobal = nullptr;
                uint8_t* currRaw = nullptr;
                uint8_t* prevGlobal = nullptr;
                uint8_t* prevRaw = nullptr;
                sensorPointers(next, g, s, &currGlobal, &currRaw);
                sensorPointers(*_data, g, s, &prevGlobal, &prevRaw);

                const bool readOk = (nextDiag.okMask & bit) != 0;
                const bool currZero = axisMagnitude(currGlobal) <= kZeroNoiseFloor;
                const uint8_t prevMag = axisMagnitude(prevGlobal);

                if (!readOk) {
                    memcpy(currGlobal, prevGlobal, TACTILE_AXIS_COUNT);
                    memcpy(currRaw, prevRaw, kForceSummaryLen);
                    continue;
                }

                if (currZero && prevMag >= kHoldMinMagnitude && _zeroHoldCounts[g][s] < kZeroHoldFrames) {
                    _zeroHoldCounts[g][s]++;
                    memcpy(currGlobal, prevGlobal, TACTILE_AXIS_COUNT);
                    memcpy(currRaw, prevRaw, kForceSummaryLen);
                } else if (currZero) {
                    if (_zeroHoldCounts[g][s] < 0xFF) {
                        _zeroHoldCounts[g][s]++;
                    }
                } else {
                    _zeroHoldCounts[g][s] = 0;
                }
            }
        }
        *_data = next;
        _diag.frameCount = nextDiag.frameCount;
        _diag.enabledMask = nextDiag.enabledMask;
        _diag.okReads = nextDiag.okReads;
        _diag.failedReads = nextDiag.failedReads;
        _diag.okMask = nextDiag.okMask;
        xSemaphoreGive(_mutex);
    }
}

TactileData HalTactile::getData() {
    static TactileData ret;

    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(10))) {
        ret = *_data;
        xSemaphoreGive(_mutex);
    }
    return ret;
}

TactileReadDiag HalTactile::getDiagnostics() {
    TactileReadDiag ret{};
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(10))) {
        ret = _diag;
        xSemaphoreGive(_mutex);
    }
    return ret;
}
