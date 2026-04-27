#include "HalEncoders.h"
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
    pinMode(PIN_ENC_CS, OUTPUT);
    digitalWrite(PIN_ENC_CS, HIGH);
    pinMode(PIN_MUX_A, OUTPUT);
    pinMode(PIN_MUX_B, OUTPUT);
    pinMode(PIN_MUX_C, OUTPUT);
    setupSPI();
}

void HalEncoders::setupSPI() {
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_ENC_MOSI,
        .miso_io_num = PIN_ENC_MISO,
        .sclk_io_num = PIN_ENC_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);

    spi_device_interface_config_t devcfg = {
        .mode = 1,
        .clock_speed_hz = 1000 * 1000,
        .spics_io_num = -1,
        .queue_size = 1,
    };
    spi_bus_add_device(SPI2_HOST, &devcfg, &_spi);
}

void HalEncoders::setMux(uint8_t ch) {
    digitalWrite(PIN_MUX_A, (ch & 0x01));
    digitalWrite(PIN_MUX_B, (ch >> 1) & 0x01);
    digitalWrite(PIN_MUX_C, (ch >> 2) & 0x01);
    delayMicroseconds(5);
}

bool HalEncoders::transferOnce(uint8_t muxChannel, uint8_t count, const uint16_t *txBuf, uint16_t *rxBuf) {
    setMux(muxChannel);

    spi_transaction_t t = {};
    t.length = static_cast<size_t>(count) * 16;
    t.tx_buffer = txBuf;
    t.rx_buffer = rxBuf;

    digitalWrite(PIN_ENC_CS, LOW);
    delayMicroseconds(1);
    const esp_err_t err = spi_device_polling_transmit(_spi, &t);
    digitalWrite(PIN_ENC_CS, HIGH);
    delayMicroseconds(2);

    return err == ESP_OK;
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
        static constexpr uint8_t kOldToNew[5] = {0, 1, 2, 4, 3};
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
    if (rawVal == 0x0000 || rawVal == 0xFFFF) {
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
        txBuf[k] = static_cast<uint16_t>((frame >> 8) | (frame << 8));
    }

    const uint8_t muxChannel = static_cast<uint8_t>(ENC_MUX_BASE_CHANNEL + groupIndex);
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

        const uint16_t rawVal = static_cast<uint16_t>((rxBuf[k] << 8) | (rxBuf[k] >> 8));
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
        txBuf[k] = static_cast<uint16_t>((frame >> 8) | (frame << 8));
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

        const uint16_t rawVal = static_cast<uint16_t>((rxBuf[k] << 8) | (rxBuf[k] >> 8));
        decodeFrame(outData, id, rawVal, issuedCmds[k], fullDiag);
    }
}

void HalEncoders::getData(EncoderData &outData) {
    int globalIdx = 0;
    outData = {};

    for (uint8_t grp = 0; grp < GROUP_COUNT; grp++) {
        const uint8_t count = group_sizes_[grp];

#if ENC_HW_MODE == ENC_HW_MODE_151_MISO_MUX
        collectGroup151(grp, globalIdx, count, outData);
#else
        collectGroup138(grp, globalIdx, count, outData);
#endif

        globalIdx += count;
    }

    memcpy(outData.latchedErrors, _latchedErrorCodes, sizeof(_latchedErrorCodes));
}
