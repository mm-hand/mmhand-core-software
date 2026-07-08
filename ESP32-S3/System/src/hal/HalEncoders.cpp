#include "HalEncoders.h"
#include "HalSensorSpi.h"
#include <cstring>

#if ENC_HW_MODE != ENC_HW_MODE_138_CS_DEMUX && ENC_HW_MODE != ENC_HW_MODE_151_MISO_MUX
#error "ENC_HW_MODE must be ENC_HW_MODE_138_CS_DEMUX or ENC_HW_MODE_151_MISO_MUX"
#endif

#if ENC_151_DIAG_MODE != ENC_151_DIAG_MODE_SAFE && ENC_151_DIAG_MODE != ENC_151_DIAG_MODE_FULL
#error "ENC_151_DIAG_MODE must be ENC_151_DIAG_MODE_SAFE or ENC_151_DIAG_MODE_FULL"
#endif

HalEncoders &encoders = HalEncoders::getInstance();

HalEncoders::HalEncoders() : _spi(nullptr) {
    for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
        _fsmStates[i] = FSM_READ_ANGLE;
        _cmdPipeline[i] = AS5047P_REG_ANGLECOM;
        _latchedErrorCodes[i] = 0;
        _recoveryAttempts[i] = 0;
    }
}

void HalEncoders::begin() {
#if ENC_HW_MODE == ENC_HW_MODE_151_MISO_MUX
#if PIN_ENC_CS >= 0
    pinMode(PIN_ENC_CS, OUTPUT);
    digitalWrite(PIN_ENC_CS, HIGH);
#endif
#endif
    pinMode(PIN_MUX_A, OUTPUT);
    pinMode(PIN_MUX_B, OUTPUT);
    pinMode(PIN_MUX_C, OUTPUT);
    setMux(ENC_151_IDLE_CHANNEL);
    setupSPI();
}

void HalEncoders::setupSPI() {
    _spi = beginEncoderSensorSpi() ? encoderSpiDevice : nullptr;
}

void HalEncoders::setMux(uint8_t ch) {
    digitalWrite(PIN_MUX_A, (ch & 0x01));
    digitalWrite(PIN_MUX_B, (ch >> 1) & 0x01);
    digitalWrite(PIN_MUX_C, (ch >> 2) & 0x01);
    delayMicroseconds(5);
}

bool HalEncoders::transferOnce(uint8_t muxChannel, uint8_t count, const uint16_t *txBuf, uint16_t *rxBuf) {
    if (count == 0 || count > MAX_GROUP_SIZE || txBuf == nullptr || rxBuf == nullptr) {
        return false;
    }

    if (_spi == nullptr) {
        setupSPI();
    }
    if (_spi == nullptr) {
        return false;
    }

    const bool locked =
        (g_sensorSelectMutex == NULL) ||
        (xSemaphoreTake(g_sensorSelectMutex, pdMS_TO_TICKS(20)) == pdTRUE);
    if (!locked) {
        return false;
    }

    setMux(muxChannel);

    uint8_t txBytes[MAX_GROUP_SIZE * 2] __attribute__((aligned(4))) = {0};
    uint8_t rxBytes[MAX_GROUP_SIZE * 2] __attribute__((aligned(4))) = {0};
    const size_t byteCount = (size_t)count * 2U;
    for (uint8_t i = 0; i < count; i++) {
        txBytes[i * 2] = (uint8_t)(txBuf[i] >> 8);
        txBytes[i * 2 + 1] = (uint8_t)(txBuf[i] & 0xFF);
    }

#if ENC_HW_MODE == ENC_HW_MODE_151_MISO_MUX
#if PIN_ENC_CS >= 0
    digitalWrite(PIN_ENC_CS, LOW);
    delayMicroseconds(1);
#endif
#endif

    spi_transaction_t transaction = {};
    transaction.length = byteCount * 8U;
    transaction.tx_buffer = txBytes;
    transaction.rx_buffer = rxBytes;
    const bool ok = (spi_device_transmit(_spi, &transaction) == ESP_OK);

    if (ok) {
        for (uint8_t i = 0; i < count; i++) {
            rxBuf[i] = ((uint16_t)rxBytes[i * 2] << 8) | rxBytes[i * 2 + 1];
        }
    }

#if ENC_HW_MODE == ENC_HW_MODE_151_MISO_MUX
#if PIN_ENC_CS >= 0
    digitalWrite(PIN_ENC_CS, HIGH);
    delayMicroseconds(2);
#endif
#endif

    setMux(ENC_151_IDLE_CHANNEL);
    if (g_sensorSelectMutex != NULL) {
        xSemaphoreGive(g_sensorSelectMutex);
    }
    return ok;
}

uint8_t HalEncoders::get_parity(uint16_t n) {
    uint8_t p = 0;
    while (n) {
        p ^= 1;
        n &= (n - 1);
    }
    return p;
}

uint16_t HalEncoders::build_command_frame(uint16_t addr, bool is_read) {
    if (is_read) {
        addr |= 0x4000;
    }
    if (get_parity(addr & 0x7FFF)) {
        addr |= 0x8000;
    }
    return addr;
}

uint8_t HalEncoders::remapGroupIndex(uint8_t groupCount, uint8_t transferIndex) const {
    uint8_t originalIndex = groupCount - 1 - transferIndex;

    if (groupCount == 4) {
        static constexpr uint8_t kOldToNew[4] = {0, 1, 3, 2};
        return kOldToNew[originalIndex];
    }

    if (groupCount == 5) {
        // Thumb physical encoder chain order is fingertip to root:
        // J19, J18, J17, J16, J20. Keep uploaded indices in logical J16..J20 slots.
        // Verified on hardware: the J18/J19 encoder slots are crossed relative to
        // the first-pass thumb remap, so only those two logical slots are swapped.
        static constexpr uint8_t kOldToNew[5] = {4, 0, 1, 3, 2};
        return kOldToNew[originalIndex];
    }

    return originalIndex;
}

int HalEncoders::mapGlobalEncoderIndex(int groupStart, uint8_t groupCount, uint8_t transferIndex) const {
    return groupStart + remapGroupIndex(groupCount, transferIndex);
}

const char *HalEncoders::getErrorString(uint16_t errorCode) {
    if (errorCode == ERR_CODE_NONE) {
        return "OK";
    }
    if (errorCode == ERR_CODE_LINK_LOST) {
        return "LINK_LOST";
    }
    if (errorCode & ERR_CODE_DIAG_BASE) {
        if (errorCode & 0x0800) {
            return "MAG_LOW";
        }
        if (errorCode & 0x0400) {
            return "MAG_HIGH";
        }
        return "DIAG_ERR";
    }
    if (errorCode & ERR_CODE_ERRFL_BASE) {
        if (errorCode & 0x01) {
            return "FRAME_ERR";
        }
        if (errorCode & 0x02) {
            return "INV_CMD";
        }
        if (errorCode & 0x04) {
            return "PAR_ERR";
        }
    }
    return "UNKNOWN";
}

EncoderData HalEncoders::getData() {
    EncoderData d{};
    getData(d);
    return d;
}

uint16_t HalEncoders::pickRegisterForState(int encoderId, bool fullDiag) const {
    switch (_fsmStates[encoderId]) {
    case FSM_READ_ERRFL:
        return AS5047P_REG_ERRFL;
    case FSM_READ_DIAAGC:
        return fullDiag ? AS5047P_REG_DIAAGC : AS5047P_REG_ANGLECOM;
    default:
        return AS5047P_REG_ANGLECOM;
    }
}

uint8_t HalEncoders::getGroupMuxChannel(uint8_t groupIndex) const {
#if ENC_HW_MODE == ENC_HW_MODE_151_MISO_MUX
    if (groupIndex < GROUP_COUNT) {
        return ENC_151_GROUP_CHANNEL_MAP[groupIndex];
    }
#else
    if (groupIndex < GROUP_COUNT) {
        return ENC_138_GROUP_CHANNEL_MAP[groupIndex];
    }
#endif
    return static_cast<uint8_t>(ENC_MUX_BASE_CHANNEL + groupIndex);
}

void HalEncoders::markLinkLost(EncoderData &outData, int encoderId) {
    outData.errorFlags[encoderId] = 1;
    _latchedErrorCodes[encoderId] = ERR_CODE_LINK_LOST;
    _recoveryAttempts[encoderId]++;
    if (_recoveryAttempts[encoderId] >= MAX_RECOVERY) {
        _fsmStates[encoderId] = FSM_READ_ANGLE;
    }
}

void HalEncoders::decodeFrame(
    EncoderData &outData,
    int encoderId,
    uint16_t rawVal,
    uint16_t issuedCmd,
    bool fullDiag
) {
    // 0x0000 is a valid AS5047P ANGLECOM response at mechanical zero
    // (parity=0, error=0, angle=0). Do not treat zero crossing as link loss.
    if (rawVal == 0xFFFF) {
        markLinkLost(outData, encoderId);
        return;
    }

    if (get_parity(rawVal & 0x7FFF) != ((rawVal >> 15) & 0x01)) {
        outData.errorFlags[encoderId] = 1;
        _latchedErrorCodes[encoderId] = ERR_CODE_ERRFL_BASE | AS5047P_ERR_PARERR;
        _fsmStates[encoderId] = FSM_READ_ERRFL;
        return;
    }

    const bool errorBit = (rawVal & 0x4000) != 0;
    const uint16_t payload = rawVal & 0x3FFF;

    switch (issuedCmd) {
    case AS5047P_REG_ANGLECOM:
        if (errorBit) {
            outData.errorFlags[encoderId] = 1;
            _fsmStates[encoderId] = FSM_READ_ERRFL;
        } else {
            outData.rawAngles[encoderId] = payload;
            outData.errorFlags[encoderId] = 0;
            _recoveryAttempts[encoderId] = 0;
            if (_latchedErrorCodes[encoderId] != ERR_CODE_LINK_LOST) {
                _latchedErrorCodes[encoderId] = 0;
            }
            _fsmStates[encoderId] = FSM_READ_ANGLE;
        }
        break;

    case AS5047P_REG_ERRFL:
        outData.errorFlags[encoderId] = 1;
        if (payload & 0x07) {
            _latchedErrorCodes[encoderId] = ERR_CODE_ERRFL_BASE | (payload & 0x0F);
        }
        _fsmStates[encoderId] = fullDiag ? FSM_READ_DIAAGC : FSM_READ_ANGLE;
        break;

    case AS5047P_REG_DIAAGC:
        outData.errorFlags[encoderId] = 1;
        if (payload & 0x0F00) {
            _latchedErrorCodes[encoderId] = ERR_CODE_DIAG_BASE | (payload & 0x0FFF);
        }
        _recoveryAttempts[encoderId]++;
        _fsmStates[encoderId] = FSM_READ_ANGLE;
        break;

    default:
        _fsmStates[encoderId] = FSM_READ_ANGLE;
        break;
    }
}

void HalEncoders::collectGroup138(uint8_t groupIndex, int groupStart, uint8_t count, EncoderData &outData) {
    uint16_t txBuf[MAX_GROUP_SIZE] = {0};
    uint16_t rxBuf[MAX_GROUP_SIZE] = {0};
    uint16_t pendingCmds[MAX_GROUP_SIZE] = {0};

    for (uint8_t k = 0; k < count; k++) {
        int id = mapGlobalEncoderIndex(groupStart, count, k);
        if (id >= ENCODER_TOTAL_NUM) {
            id = ENCODER_TOTAL_NUM - 1;
        }

        const uint16_t reg = pickRegisterForState(id, true);
        pendingCmds[k] = reg;
        const uint16_t frame = build_command_frame(reg, true);
        txBuf[k] = frame;
    }

    const uint8_t muxChannel = getGroupMuxChannel(groupIndex);
    const bool transferOk = transferOnce(muxChannel, count, txBuf, rxBuf);

    for (uint8_t k = 0; k < count; k++) {
        const int id = mapGlobalEncoderIndex(groupStart, count, k);
        if (id >= ENCODER_TOTAL_NUM) {
            continue;
        }

        const uint16_t prevCmd = _cmdPipeline[id];
        _cmdPipeline[id] = pendingCmds[k];

        if (!transferOk) {
            markLinkLost(outData, id);
            continue;
        }

        const uint16_t rawVal = rxBuf[k];
        decodeFrame(outData, id, rawVal, prevCmd, true);
    }
}

void HalEncoders::collectGroup151(uint8_t groupIndex, int groupStart, uint8_t count, EncoderData &outData) {
    uint16_t txBuf[MAX_GROUP_SIZE] = {0};
    uint16_t primeRx[MAX_GROUP_SIZE] = {0};
    uint16_t rxBuf[MAX_GROUP_SIZE] = {0};
    uint16_t issuedCmds[MAX_GROUP_SIZE] = {0};

#if ENC_151_DIAG_MODE == ENC_151_DIAG_MODE_FULL
    constexpr bool fullDiag = true;
#else
    constexpr bool fullDiag = false;
#endif

    for (uint8_t k = 0; k < count; k++) {
        int id = mapGlobalEncoderIndex(groupStart, count, k);
        if (id >= ENCODER_TOTAL_NUM) {
            id = ENCODER_TOTAL_NUM - 1;
        }

        const uint16_t reg = pickRegisterForState(id, fullDiag);
        issuedCmds[k] = reg;
        const uint16_t frame = build_command_frame(reg, true);
        txBuf[k] = frame;
    }

    const uint8_t muxChannel = getGroupMuxChannel(groupIndex);
    const bool primeOk = transferOnce(muxChannel, count, txBuf, primeRx);
    const bool captureOk = transferOnce(muxChannel, count, txBuf, rxBuf);

    for (uint8_t k = 0; k < count; k++) {
        const int id = mapGlobalEncoderIndex(groupStart, count, k);
        if (id >= ENCODER_TOTAL_NUM) {
            continue;
        }

        if (!primeOk || !captureOk) {
            markLinkLost(outData, id);
            continue;
        }

        const uint16_t rawVal = rxBuf[k];
        decodeFrame(outData, id, rawVal, issuedCmds[k], fullDiag);
    }
}

void HalEncoders::getData(EncoderData &outData) {
    int globalIdx = 0;
    outData = {};

    for (uint8_t grp = 0; grp < GROUP_COUNT; grp++) {
        const uint8_t count = group_sizes_[grp];
        if ((ENC_ACTIVE_GROUP_MASK & (1U << grp)) == 0) {
            for (uint8_t k = 0; k < count; k++) {
                const int id = globalIdx + k;
                if (id >= ENCODER_TOTAL_NUM) {
                    continue;
                }
                outData.rawAngles[id] = 0;
                outData.errorFlags[id] = 1;
                _latchedErrorCodes[id] = ERR_CODE_LINK_LOST;
                _fsmStates[id] = FSM_READ_ANGLE;
            }
            globalIdx += count;
            continue;
        }

#if ENC_HW_MODE == ENC_HW_MODE_151_MISO_MUX
        collectGroup151(grp, globalIdx, count, outData);
#else
        collectGroup138(grp, globalIdx, count, outData);
#endif

        globalIdx += count;
    }

    memcpy(outData.latchedErrors, _latchedErrorCodes, sizeof(_latchedErrorCodes));
}
