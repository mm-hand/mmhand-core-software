#include "HalTWAI.h"
#include <cstring>

// Global instance
HalTWAI twaiBus;

namespace {
constexpr uint8_t kErrorCodesPerFrame = 7;
constexpr uint8_t kErrorFrameCount =
    (ENCODER_TOTAL_NUM + kErrorCodesPerFrame - 1) / kErrorCodesPerFrame;

constexpr uint16_t kDiagMagLowBit = 0x0800;
constexpr uint16_t kDiagMagHighBit = 0x0400;
} // namespace

static_assert(
    kErrorFrameCount ==
        (CAN_ID_ERROR_DETAIL_LAST - CAN_ID_ERROR_DETAIL_BASE + 1),
    "Error detail frame ID range does not match frame count.");

HalTWAI::HalTWAI() : _is_initialized(false) {}

bool HalTWAI::begin() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)PIN_TWAI_TX,
        (gpio_num_t)PIN_TWAI_RX,
        TWAI_MODE_NORMAL
    );
    g_config.tx_queue_len = 50;
    g_config.rx_queue_len = 10;

    twai_timing_config_t t_config = CAN_BAUD_RATE;
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        return false;
    }
    if (twai_start() != ESP_OK) {
        return false;
    }

    _is_initialized = true;
    return true;
}

void HalTWAI::sendFrame(uint32_t id, const uint8_t *data, uint8_t len) {
    if (!_is_initialized) return;

    twai_message_t message;
    message.identifier = id;
    message.extd = 0;
    message.rtr = 0;
    message.data_length_code = len;
    memcpy(message.data, data, len);

    twai_transmit(&message, 0);
}

void HalTWAI::sendEncoderData(const EncoderData &encData) {
    uint8_t buffer[8];
    int idx = 0;

    // First 5 frames contain 20 encoder values.
    for (int frame = 0; frame < 5; frame++) {
        for (int i = 0; i < 4; i++) {
            uint16_t val = encData.rawAngles[idx + i];
            buffer[i * 2] = (uint8_t)(val >> 8);
            buffer[i * 2 + 1] = (uint8_t)(val & 0xFF);
        }

        sendFrame(CAN_ID_ENC_BASE + frame, buffer, 8);
        idx += 4;
    }

    // Last frame contains encoder #21 in byte[0..1].
    memset(buffer, 0, 8);
    uint16_t lastVal = encData.rawAngles[idx];
    buffer[0] = (uint8_t)(lastVal >> 8);
    buffer[1] = (uint8_t)(lastVal & 0xFF);

    sendFrame(CAN_ID_ENC_BASE + 5, buffer, 8);
}

void HalTWAI::sendTactileSummary(const TactileData &tacData) {
    uint8_t buffer[8];
    uint8_t bufPtr = 0;
    uint32_t frameId = 0x200;

    for (int g = 0; g < TACTILE_GROUP_NUM; g++) {
        const TacGroup &grp = tacData.groups[g];
        const uint8_t *ptrs[3] = {
            grp.sensor_A.global,
            grp.sensor_B.global,
            grp.sensor_C.global
        };

        for (int s = 0; s < 3; s++) {
            for (int k = 0; k < 3; k++) {
                buffer[bufPtr++] = ptrs[s][k];
                if (bufPtr == 8) {
                    sendFrame(frameId++, buffer, 8);
                    bufPtr = 0;
                    memset(buffer, 0, 8);
                }
            }
        }
    }
    if (bufPtr > 0) {
        sendFrame(frameId, buffer, bufPtr);
    }
}

void HalTWAI::sendTactileFullDump(const TactileData &tacData) {
    (void)tacData;
    // Placeholder
}

bool HalTWAI::sendCalibrationAck(bool success) {
    if (!_is_initialized) return false;

    twai_message_t message;
    message.identifier = 0x300;
    message.extd = 0;
    message.rtr = 0;
    message.data_length_code = 1;
    message.data[0] = success ? 0x01 : 0x00;

    return (twai_transmit(&message, 0) == ESP_OK);
}

bool HalTWAI::receiveMonitor(RemoteCommand *outCmd) {
    if (!_is_initialized) return false;

    twai_message_t message;
    if (twai_receive(&message, 0) == ESP_OK) {
        if (outCmd) {
            outCmd->cmd_type = message.data[0];
            outCmd->is_new = true;
        }
        return true;
    }
    return false;
}

uint8_t HalTWAI::encodeFaultCode(uint16_t rawLatchedError, uint16_t errorFlag) {
    if (errorFlag == 0) {
        return 0x00; // OK
    }

    if (rawLatchedError == ERR_CODE_LINK_LOST) {
        return 0x01; // LINK_LOST
    }

    if (rawLatchedError & ERR_CODE_ERRFL_BASE) {
        const uint16_t errflBits = rawLatchedError & 0x000F;
        if (errflBits & 0x0008) {
            return 0x1F; // ERRFL_OTHER
        }

        switch (errflBits & 0x0007) {
        case 0x0001:
            return 0x10; // ERRFL_FRERR
        case 0x0002:
            return 0x11; // ERRFL_INVCOMM
        case 0x0004:
            return 0x12; // ERRFL_PARERR
        case 0x0003:
            return 0x13; // ERRFL_FRERR_INVCOMM
        case 0x0005:
            return 0x14; // ERRFL_FRERR_PARERR
        case 0x0006:
            return 0x15; // ERRFL_INVCOMM_PARERR
        case 0x0007:
            return 0x16; // ERRFL_FRERR_INVCOMM_PARERR
        default:
            return 0x1F; // ERRFL_OTHER
        }
    }

    if (rawLatchedError & ERR_CODE_DIAG_BASE) {
        const uint16_t diagBits = rawLatchedError & 0x0FFF;
        const bool magLow = (diagBits & kDiagMagLowBit) != 0;
        const bool magHigh = (diagBits & kDiagMagHighBit) != 0;
        const bool hasOther = (diagBits & ~(kDiagMagLowBit | kDiagMagHighBit)) != 0;

        if (magLow && magHigh) {
            return hasOther ? 0x26 : 0x22; // DIA_MAG_LOW_HIGH(_PLUS_OTHER)
        }
        if (magLow) {
            return hasOther ? 0x24 : 0x20; // DIA_MAG_LOW(_PLUS_OTHER)
        }
        if (magHigh) {
            return hasOther ? 0x25 : 0x21; // DIA_MAG_HIGH(_PLUS_OTHER)
        }
        if (hasOther) {
            return 0x23; // DIA_OTHER_ONLY
        }
        return 0x2F; // DIA_RESERVED
    }

    return 0xF0; // UNKNOWN_RAW
}

// [seq + 7 routes] x 3 frames:
// 0x1F0 => idx 0..6, 0x1F1 => idx 7..13, 0x1F2 => idx 14..20
void HalTWAI::sendErrorStatus(const EncoderData &data) {
    bool hasError = false;
    for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
        if (data.errorFlags[i]) {
            hasError = true;
            break;
        }
    }

    if (!hasError) {
        return;
    }

    static uint8_t seq = 0;
    uint8_t buf[8] = {0};

    for (uint8_t frame = 0; frame < kErrorFrameCount; frame++) {
        buf[0] = seq;
        for (uint8_t slot = 0; slot < kErrorCodesPerFrame; slot++) {
            const int idx = frame * kErrorCodesPerFrame + slot;
            if (idx < ENCODER_TOTAL_NUM) {
                buf[slot + 1] = encodeFaultCode(data.latchedErrors[idx], data.errorFlags[idx]);
            } else {
                buf[slot + 1] = 0xFF; // RESERVED
            }
        }
        sendFrame(CAN_ID_ERROR_DETAIL_BASE + frame, buf, 8);
    }

    seq++;
}

bool HalTWAI::maintain() {
    twai_status_info_t status_info;

    if (twai_get_status_info(&status_info) != ESP_OK) {
        return false;
    }

    if (status_info.state == TWAI_STATE_BUS_OFF) {
        if (twai_initiate_recovery() != ESP_OK) {
            // Keep silent here; caller handles retry rhythm.
        }
        return false;
    }

    if (status_info.state == TWAI_STATE_STOPPED) {
        if (twai_start() != ESP_OK) {
            // Keep silent here; caller handles retry rhythm.
        }
        return false;
    }

    if (status_info.tx_error_counter > 100 || status_info.rx_error_counter > 100) {
        Serial.printf(
            "[CAN WARN] High Error Count! Tx:%d Rx:%d\n",
            status_info.tx_error_counter,
            status_info.rx_error_counter
        );
    }

    return (status_info.state == TWAI_STATE_RUNNING);
}
