#include "HalTactile.h"
#include "HalSensorSpi.h"

#include <SPI.h>
#include <cstring>

namespace {
// 触觉模块寄存器读取协议：5字节命令头、1字节状态、负载、1字节CRC8。
constexpr uint8_t kCmdRead = 0xFB;
constexpr uint16_t kLenCmdHead = 5;
constexpr uint16_t kLenStatus = 1;
constexpr uint16_t kLenCrc = 1;
constexpr uint16_t kForceSummaryAddr = 1008;
constexpr uint16_t kForceSummaryLen = 3;
constexpr uint16_t kForceDistributionAddr = 1038;
constexpr uint16_t kForceDistributionMaxLen = TACTILE_PAD_TAXELS * TACTILE_AXIS_COUNT;
// 板级片选建立、模块准备响应及释放片选所需延时。
constexpr uint16_t kDelayCsHoldUs = 30;
constexpr uint16_t kDelayResponseUs = 100;
constexpr uint16_t kDelaySelectSettleUs = 80;
constexpr uint8_t kSummaryReadAttempts = 2;
constexpr uint8_t kFullReadAttempts = 1;
constexpr uint16_t kRetryDelayUs = 200;

struct SensorPattern {
    bool enabled;  ///< 是否扫描该物理模块。
    uint8_t pattern;///< GPIO17/14/13/6/7组成的五位路由编码。
};

/** @brief 合成“手指组高三位+组内触觉模块低两位”的选择器编码。 */
constexpr uint8_t makeSelectorPattern(uint8_t group, uint8_t sensorBits) {
    return (uint8_t)((TACTILE_GROUP_CHANNEL_MAP[group] << 2) | (sensorBits & 0x03));
}

// 显式表保存实机调通得到的“逻辑手指×模块”到选择器编码映射。
// CS_A/CS_B是组内地址位：S0指尖=11、S1中段=10、S2近端=01。
constexpr SensorPattern kSensorPatterns[TACTILE_GROUP_NUM][TACTILE_SENSOR_PER_GROUP] = {
    {{true, makeSelectorPattern(0, 0x03)}, {true, makeSelectorPattern(0, 0x02)}, {true, makeSelectorPattern(0, 0x01)}},
    {{true, makeSelectorPattern(1, 0x03)}, {true, makeSelectorPattern(1, 0x02)}, {true, makeSelectorPattern(1, 0x01)}},
    {{true, makeSelectorPattern(2, 0x03)}, {true, makeSelectorPattern(2, 0x02)}, {true, makeSelectorPattern(2, 0x01)}},
    {{true, makeSelectorPattern(3, 0x03)}, {true, makeSelectorPattern(3, 0x02)}, {true, makeSelectorPattern(3, 0x01)}},
    {{true, makeSelectorPattern(4, 0x03)}, {true, makeSelectorPattern(4, 0x02)}, {true, makeSelectorPattern(4, 0x01)}},
};

/** @brief 先启用内部上拉，再切成高电平输出，降低上电片选毛刺风险。 */
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

/** @brief 返回不同触觉模块对应的全量力数组字节数。 */
uint16_t sensorForceLen(uint8_t sensor) {
    switch (sensor) {
        case 0:
            return TACTILE_SENSOR0_TAXELS * TACTILE_AXIS_COUNT;
        case 1:
            return TACTILE_SENSOR1_TAXELS * TACTILE_AXIS_COUNT;
        case 2:
            return TACTILE_SENSOR2_TAXELS * TACTILE_AXIS_COUNT;
        default:
            return 0;
    }
}

// [待确认是否启用或删除] 当前未调用；可从全量力数据推导摘要，理论上可省去一次摘要寄存器读取，
// 但推导规则是否等价于模块固件的摘要算法需要实机确认。
void deriveGlobalFromForces(const uint8_t* forces, uint16_t forceLen, uint8_t* globalOut, uint8_t* rawOut) {
    if (!forces || !globalOut || forceLen < TACTILE_AXIS_COUNT) {
        return;
    }

    int8_t bestX = 0;
    int8_t bestY = 0;
    uint8_t bestXMag = 0;
    uint8_t bestYMag = 0;
    uint8_t bestZ = 0;

    for (uint16_t idx = 0; idx + 2 < forceLen; idx += TACTILE_AXIS_COUNT) {
        const int8_t x = (int8_t)forces[idx];
        const int8_t y = (int8_t)forces[idx + 1];
        const uint8_t z = forces[idx + 2];
        const uint8_t xMag = (uint8_t)abs((int)x);
        const uint8_t yMag = (uint8_t)abs((int)y);

        if (xMag > bestXMag) {
            bestXMag = xMag;
            bestX = x;
        }
        if (yMag > bestYMag) {
            bestYMag = yMag;
            bestY = y;
        }
        if (z > bestZ) {
            bestZ = z;
        }
    }

    globalOut[0] = (uint8_t)bestX;
    globalOut[1] = (uint8_t)bestY;
    globalOut[2] = bestZ;
    if (rawOut) {
        rawOut[0] = globalOut[0];
        rawOut[1] = globalOut[1];
        rawOut[2] = globalOut[2];
    }
}

// CRC8查找表：以256字节Flash换取逐字节O(1)更新，适合每轮大量触觉数据校验。
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

// 全局唯一触觉HAL对象。
HalTactile tactile;

/**
 * @brief 创建快照互斥量和数据存储。
 * @note [待确认是否简化] _data只在全局对象生命周期使用，可改为类内直接成员以避免new。
 */
HalTactile::HalTactile()
    : _mutex(xSemaphoreCreateMutex()),
      _data(new TactileData()),
      _spi(nullptr),
      _spiReady(false),
      _diag{},
      _lastReadResult{TactileReadFailReason::None, 0, 0, 0},
      _lastReadAttempts(0) {
    memset(_data, 0, sizeof(TactileData));
}

/** @brief 在10秒启动等待前释放所有触觉路由，避免同时选中多个模块。 */
void HalTactile::prepareBootSelectPins() {
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

/** @brief 初始化选择线和SPI；失败时保留_spiReady=false供update上报全失败。 */
void HalTactile::begin() {
    prepareBootSelectPins();
    _spiReady = setupSPI();
    if (!_spiReady) {
        Serial.println("[Tactile] SPI init failed");
    }
}

/** @brief 获取公共SPI模块创建的触觉设备句柄。 */
bool HalTactile::setupSPI() {
    _spi = beginTactileSensorSpi() ? tactileSpiDevice : nullptr;
    setAllSelectPinsHigh();
    return _spi != nullptr;
}

/** @brief 使用查表法计算协议CRC8，初值0xFF。 */
uint8_t HalTactile::crc8(const uint8_t* data, uint32_t len) {
    uint8_t crc = 0xFF;
    while (len--) {
        crc = kCrc8Table[crc ^ *data++];
    }
    return crc;
}

/** @brief 将所有地址/使能线恢复为非选中状态，并等待硬件稳定。 */
void HalTactile::setAllSelectPinsHigh() {
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

/** @brief 输出五位物理选择编码，选中一个手指上的一个触觉模块。 */
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

/**
 * @brief 向指定模块发送寄存器读取命令，再读取状态、负载和CRC8。
 * @note 命令和响应拆成两次SPI事务，但在此期间保持同一物理路由被选中。
 */
bool HalTactile::readRegisters(uint8_t selectorPattern, uint16_t startAddr, uint8_t* payload, uint16_t payloadLen) {
    _lastReadResult = {TactileReadFailReason::None, 0, 0, 0};
    if (!payload || payloadLen == 0 || payloadLen > kForceDistributionMaxLen) {
        _lastReadResult.reason = TactileReadFailReason::InvalidArgument;
        return false;
    }
    if (!_spiReady || _spi == nullptr) {
        _lastReadResult.reason = TactileReadFailReason::SpiNotReady;
        return false;
    }

    // 触觉使用独立SPI2和GPIO7/6/13/14/17/18选路线，可与磁编SPI3并行工作。
    // 栈上按最大52触元帧分配DMA对齐缓冲；摘要读取也复用同一大小。
    uint8_t raw[kLenCmdHead + kLenStatus + kForceDistributionMaxLen + kLenCrc] __attribute__((aligned(4))) = {0};
    uint8_t zeros[kLenStatus + kForceDistributionMaxLen + kLenCrc] __attribute__((aligned(4))) = {0};
    uint8_t response[kLenStatus + kForceDistributionMaxLen + kLenCrc] __attribute__((aligned(4))) = {0};
    const uint16_t totalLen = kLenCmdHead + kLenStatus + payloadLen + kLenCrc;
    const uint16_t responseLen = kLenStatus + payloadLen + kLenCrc;

    raw[0] = kCmdRead;
    raw[1] = (uint8_t)(startAddr & 0xFF);
    raw[2] = (uint8_t)(startAddr >> 8);
    raw[3] = (uint8_t)(payloadLen & 0xFF);
    raw[4] = (uint8_t)(payloadLen >> 8);

    setAllSelectPinsHigh();
    setSelectorPattern(selectorPattern);
    delayMicroseconds(kDelayCsHoldUs);

    // 第一阶段只发送5字节读取命令。
    spi_transaction_t commandTransaction = {};
    commandTransaction.length = kLenCmdHead * 8U;
    commandTransaction.tx_buffer = raw;
    const bool commandOk = (spi_device_transmit(_spi, &commandTransaction) == ESP_OK);

    delayMicroseconds(kDelayResponseUs);

    // 第二阶段发送0填充时钟，同时接收状态+负载+CRC。
    spi_transaction_t responseTransaction = {};
    responseTransaction.length = responseLen * 8U;
    responseTransaction.tx_buffer = zeros;
    responseTransaction.rx_buffer = response;
    const bool responseOk = (spi_device_transmit(_spi, &responseTransaction) == ESP_OK);

    delayMicroseconds(kDelayCsHoldUs);
    setAllSelectPinsHigh();

    if (!commandOk || !responseOk) {
        _lastReadResult.reason = commandOk
            ? TactileReadFailReason::ResponseTransport
            : TactileReadFailReason::CommandTransport;
        memset(payload, 0, payloadLen);
        return false;
    }
    memcpy(raw + kLenCmdHead, response, responseLen);

    // CRC覆盖命令头、状态和负载，不包含末尾CRC本身。
    const uint8_t recvCrc = raw[totalLen - 1];
    const uint8_t calcCrc = crc8(raw, totalLen - 1);
    _lastReadResult.status = raw[kLenCmdHead];
    _lastReadResult.recvCrc = recvCrc;
    _lastReadResult.calcCrc = calcCrc;

    if (recvCrc != calcCrc) {
        _lastReadResult.reason = TactileReadFailReason::CrcMismatch;
        memset(payload, 0, payloadLen);
        return false;
    }

    memcpy(payload, raw + kLenCmdHead + kLenStatus, payloadLen);
    return true;
}

/** @brief 读取单模块3字节摘要寄存器，失败时清零输出。 */
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
    _lastReadAttempts = 0;
    for (uint8_t attempt = 0; attempt < kSummaryReadAttempts; attempt++) {
        _lastReadAttempts++;
        ok = readRegisters(selectorPattern, kForceSummaryAddr, payload, kForceSummaryLen);
        if (ok) {
            break;
        }
        if (attempt + 1 < kSummaryReadAttempts) {
            delayMicroseconds(kRetryDelayUs);
        }
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

/** @brief 读取单模块完整触元力分布，失败时清零整块输出。 */
bool HalTactile::readSensorForces(uint8_t groupIndex, uint8_t sensorIndex, uint8_t* forcesOut, uint16_t forcesLen) {
    if (
        !forcesOut ||
        groupIndex >= TACTILE_GROUP_NUM ||
        sensorIndex >= TACTILE_SENSOR_PER_GROUP ||
        forcesLen == 0 ||
        forcesLen > kForceDistributionMaxLen
    ) {
        return false;
    }

    const SensorPattern cfg = kSensorPatterns[groupIndex][sensorIndex];
    if (!cfg.enabled) {
        memset(forcesOut, 0, forcesLen);
        return false;
    }

    const uint8_t selectorPattern = cfg.pattern;
    bool ok = false;
    _lastReadAttempts = 0;
    for (uint8_t attempt = 0; attempt < kFullReadAttempts; attempt++) {
        _lastReadAttempts++;
        ok = readRegisters(selectorPattern, kForceDistributionAddr, forcesOut, forcesLen);
        if (ok) {
            break;
        }
        if (attempt + 1 < kFullReadAttempts) {
            delayMicroseconds(kRetryDelayUs);
        }
    }
    if (!ok) {
        memset(forcesOut, 0, forcesLen);
    }
    return ok;
}

/**
 * @brief 扫描5组×3模块，每个模块分别读取摘要和全量力分布。
 * @note 一轮共最多30次寄存器读取；这是33ms任务周期中的主要耗时来源。
 */
void HalTactile::update() {
    TactileData next{};
    TactileReadDiag nextDiag = _diag;
    nextDiag.frameCount++;
    nextDiag.enabledMask = 0;
    nextDiag.okReads = 0;
    nextDiag.failedReads = 0;
    nextDiag.summaryOkMask = 0;
    nextDiag.fullOkMask = 0;
    nextDiag.okMask = 0;
    nextDiag.commandFailMask = 0;
    nextDiag.responseFailMask = 0;
    nextDiag.crcFailMask = 0;
    nextDiag.statusNonzeroMask = 0;
    nextDiag.summaryRetryRecoveredMask = 0;

    auto incrementSaturated = [](uint16_t& value) {
        if (value != UINT16_MAX) value++;
    };
    auto recordRead = [&](uint8_t moduleIndex, bool summary, bool ok) {
        const uint16_t moduleBit = (uint16_t)(1U << moduleIndex);
        nextDiag.lastStatus = _lastReadResult.status;
        nextDiag.lastRecvCrc = _lastReadResult.recvCrc;
        nextDiag.lastCalcCrc = _lastReadResult.calcCrc;
        nextDiag.moduleLastStatus[moduleIndex] = _lastReadResult.status;
        if (_lastReadResult.status != 0) nextDiag.statusNonzeroMask |= moduleBit;
        if (summary) {
            incrementSaturated(ok
                ? nextDiag.summarySuccessCount[moduleIndex]
                : nextDiag.summaryFailureCount[moduleIndex]);
            if (ok && _lastReadAttempts > 1) nextDiag.summaryRetryRecoveredMask |= moduleBit;
            if (!ok) nextDiag.summaryLastFailReason[moduleIndex] = (uint8_t)_lastReadResult.reason;
        } else {
            incrementSaturated(ok
                ? nextDiag.fullSuccessCount[moduleIndex]
                : nextDiag.fullFailureCount[moduleIndex]);
            if (!ok) nextDiag.fullLastFailReason[moduleIndex] = (uint8_t)_lastReadResult.reason;
        }
        if (ok) return;
        switch (_lastReadResult.reason) {
            case TactileReadFailReason::CommandTransport:
                nextDiag.commandFailMask |= moduleBit;
                break;
            case TactileReadFailReason::ResponseTransport:
                nextDiag.responseFailMask |= moduleBit;
                break;
            case TactileReadFailReason::CrcMismatch:
                nextDiag.crcFailMask |= moduleBit;
                break;
            default:
                break;
        }
        nextDiag.lastFailedPattern = kSensorPatterns[moduleIndex / TACTILE_SENSOR_PER_GROUP]
            [moduleIndex % TACTILE_SENSOR_PER_GROUP].pattern;
        nextDiag.lastFailStatus = _lastReadResult.status;
        nextDiag.lastFailRecvCrc = _lastReadResult.recvCrc;
        nextDiag.lastFailCalcCrc = _lastReadResult.calcCrc;
    };

    if (_spiReady) {
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
            uint8_t* forces[TACTILE_SENSOR_PER_GROUP] = {
                &grp.sensor_A.forces[0][0],
                &grp.sensor_B.forces[0][0],
                &grp.sensor_C.forces[0][0],
            };
            for (uint8_t s = 0; s < TACTILE_SENSOR_PER_GROUP; s++) {
                const uint16_t forceLen = sensorForceLen(s);
                if (!kSensorPatterns[g][s].enabled) {
                    memset(globals[s], 0, TACTILE_AXIS_COUNT);
                    memset(rawSummaries[s], 0, kForceSummaryLen);
                    memset(forces[s], 0, forceLen);
                    continue;
                }
                nextDiag.enabledMask |= (uint16_t)(1U << (g * TACTILE_SENSOR_PER_GROUP + s));

                // [待确认是否优化] 摘要与全量分开读取；若确认deriveGlobalFromForces等价，可减少一半事务数。
                const bool summaryOk = readSensorSummary(g, s, globals[s], rawSummaries[s]);
                const uint8_t moduleIndex = g * TACTILE_SENSOR_PER_GROUP + s;
                recordRead(moduleIndex, true, summaryOk);
                const bool forcesOk = readSensorForces(g, s, forces[s], forceLen);
                recordRead(moduleIndex, false, forcesOk);
                const uint16_t moduleBit = (uint16_t)(1U << moduleIndex);

                // 摘要与全量有效性必须独立记录：摘要为零可能是正常无受力，
                // 只有SPI事务及CRC8失败时才清除summaryOkMask对应位。
                if (summaryOk) {
                    nextDiag.summaryOkMask |= moduleBit;
                }
                if (forcesOk) {
                    nextDiag.fullOkMask |= moduleBit;
                }

                if (summaryOk && forcesOk) {
                    nextDiag.okReads++;
                    nextDiag.okMask |= moduleBit;
                } else {
                    nextDiag.failedReads++;
                }
            }
        }
    } else {
        nextDiag.failedReads = TACTILE_GROUP_NUM * TACTILE_SENSOR_PER_GROUP;
    }

    // 先在局部next中构建完整帧，再在短临界区内一次替换，避免消费者读到半帧。
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(10))) {
        *_data = next;
        _diag = nextDiag;
        xSemaphoreGive(_mutex);
    }
}

/** @brief 在互斥保护下复制最新触觉快照。 */
TactileData HalTactile::getData() {
    static TactileData ret;

    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(10))) {
        ret = *_data;
        xSemaphoreGive(_mutex);
    }
    return ret;
}

/** @brief 在互斥保护下复制诊断统计。 */
TactileReadDiag HalTactile::getDiagnostics() {
    TactileReadDiag ret{};
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(10))) {
        ret = _diag;
        xSemaphoreGive(_mutex);
    }
    return ret;
}
