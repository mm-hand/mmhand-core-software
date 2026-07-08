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

uint8_t tactileAxisMagnitude(const uint8_t* axes) {
    const int x = abs((int8_t)axes[0]);
    const int y = abs((int8_t)axes[1]);
    const int z = axes[2];
    int value = x;
    if (y > value) value = y;
    if (z > value) value = z;
    return (uint8_t)value;
}

void writeFilteredTactileSummary(uint8_t* dst, const uint8_t* src) {
    if (tactileAxisMagnitude(src) <= TACTILE_SUMMARY_NOISE_FLOOR) {
        dst[0] = 0;
        dst[1] = 0;
        dst[2] = 0;
        return;
    }
    dst[0] = src[0];
    dst[1] = src[1];
    dst[2] = src[2];
}

uint16_t crc16Ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}
} // namespace

static_assert(
    kErrorFrameCount ==
        (CAN_ID_ERROR_DETAIL_LAST - CAN_ID_ERROR_DETAIL_BASE + 1),
    "Error detail frame ID range does not match frame count.");
static_assert(
    CAN_ENC_FRAME_COUNT == (CAN_ID_ENC_LAST - CAN_ID_ENC_BASE + 1),
    "Encoder frame ID range does not match frame count.");
static_assert(CAN_ID_ENC_CRC <= 0x7FF, "Encoder CRC CAN ID must fit standard CAN.");
static_assert(
    CAN_TAC_SUMMARY_PACKET_BYTES <=
        (CAN_TAC_SUMMARY_FRAME_COUNT * CAN_TAC_SUMMARY_PAYLOAD_PER_FRAME),
    "Tactile summary payload plus CRC budget too small.");
static_assert(
    CAN_TAC_FULL_DLC == (1 + CAN_TAC_FULL_PAYLOAD_PER_FRAME),
    "Tactile full DLC must be seq(1) + payload bytes.");
static_assert(
    TACTILE_FULL_BYTES <= (CAN_TAC_FULL_FRAME_COUNT * CAN_TAC_FULL_PAYLOAD_PER_FRAME),
    "Tactile full payload budget too small.");
static_assert(CAN_ID_TAC_FULL_DUMP_LAST <= 0x7FF, "Tactile full CAN IDs must fit standard CAN.");

HalTWAI::HalTWAI() : _is_initialized(false) {}

bool HalTWAI::begin() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)PIN_TWAI_TX,
        (gpio_num_t)PIN_TWAI_RX,
        TWAI_MODE_NORMAL
    );
    g_config.tx_queue_len = 512;
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

bool HalTWAI::sendFrame(uint32_t id, const uint8_t *data, uint8_t len) {
    if (!_is_initialized) return false;
    if (!data || len > 8) return false;

    twai_message_t message = {};
    message.identifier = id;
    message.extd = 0;
    message.rtr = 0;
    message.data_length_code = len;
    memcpy(message.data, data, len);

    return (twai_transmit(&message, pdMS_TO_TICKS(5)) == ESP_OK);
}

uint8_t HalTWAI::sendEncoderData(const EncoderData &encData) {
    uint8_t packet[CAN_ENC_PAYLOAD_BYTES] = {0};
    for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
        const uint16_t val = encData.rawAngles[i];
        packet[i * 2] = (uint8_t)(val >> 8);
        packet[i * 2 + 1] = (uint8_t)(val & 0xFF);
    }

    uint8_t buffer[8];
    uint8_t sent = 0;

    for (int frame = 0; frame < CAN_ENC_FRAME_COUNT; frame++) {
        memset(buffer, 0, sizeof(buffer));
        for (int i = 0; i < 4; i++) {
            const int realIdx = frame * 4 + i;
            if (realIdx >= ENCODER_TOTAL_NUM) {
                break;
            }
            buffer[i * 2] = packet[realIdx * 2];
            buffer[i * 2 + 1] = packet[realIdx * 2 + 1];
        }

        if (sendFrame(CAN_ID_ENC_BASE + frame, buffer, 8)) {
            sent++;
        }
    }

    static uint8_t seq = 0;
    const uint16_t crc = crc16Ccitt(packet, sizeof(packet));
    memset(buffer, 0, 8);
    buffer[0] = seq++;
    buffer[1] = (uint8_t)(crc >> 8);
    buffer[2] = (uint8_t)(crc & 0xFF);
    buffer[3] = CAN_ENC_FRAME_COUNT;
    buffer[4] = CAN_ENC_PAYLOAD_BYTES;
    if (sendFrame(CAN_ID_ENC_CRC, buffer, CAN_ENC_CRC_DLC)) {
        sent++;
    }
    return sent;
}

uint8_t HalTWAI::sendTactileSummary(const TactileData &tacData) {
    uint8_t packet[CAN_TAC_SUMMARY_PACKET_BYTES] = {0};
    uint8_t cursor = 0;

    for (int g = 0; g < TACTILE_GROUP_NUM; g++) {
        const TacGroup &grp = tacData.groups[g];
        const uint8_t *ptrs[3] = {
            grp.sensor_A.global,
            grp.sensor_B.global,
            grp.sensor_C.global
        };

        for (int s = 0; s < 3; s++) {
            writeFilteredTactileSummary(packet + cursor, ptrs[s]);
            cursor += TACTILE_AXIS_COUNT;
        }
    }

    const uint16_t crc = crc16Ccitt(packet, TACTILE_SUMMARY_BYTES);
    packet[TACTILE_SUMMARY_BYTES] = (uint8_t)(crc >> 8);
    packet[TACTILE_SUMMARY_BYTES + 1] = (uint8_t)(crc & 0xFF);

    static uint8_t seq = 0;
    const uint8_t thisSeq = seq++;
    uint8_t buffer[CAN_TAC_SUMMARY_DLC] = {0};
    for (uint8_t frame = 0; frame < CAN_TAC_SUMMARY_FRAME_COUNT; frame++) {
        buffer[0] = thisSeq;
        for (uint8_t slot = 0; slot < CAN_TAC_SUMMARY_PAYLOAD_PER_FRAME; slot++) {
            const uint8_t idx = frame * CAN_TAC_SUMMARY_PAYLOAD_PER_FRAME + slot;
            buffer[slot + 1] = (idx < CAN_TAC_SUMMARY_PACKET_BYTES) ? packet[idx] : 0;
        }
        sendFrame(CAN_ID_TAC_SUMMARY_BASE + frame, buffer, CAN_TAC_SUMMARY_DLC);
    }
    return thisSeq;
}

void HalTWAI::sendTactileFullDump(const TactileData &tacData, uint8_t seq) {
    uint8_t buffer[CAN_TAC_FULL_DLC] = {0};
    uint16_t frameIdx = 0;
    uint8_t slot = 0;

    auto beginFrame = [&]() {
        memset(buffer, 0, sizeof(buffer));
        buffer[0] = seq;
    };

    auto flushFrame = [&]() {
        sendFrame(CAN_ID_TAC_FULL_DUMP_BASE + frameIdx, buffer, CAN_TAC_FULL_DLC);
        frameIdx++;
        slot = 0;
        if ((frameIdx & 0x3F) == 0) {
            delay(1);
        }
    };

    auto pushByte = [&](uint8_t value) {
        if (slot == 0) {
            beginFrame();
        }
        buffer[1 + slot] = value;
        slot++;
        if (slot >= CAN_TAC_FULL_PAYLOAD_PER_FRAME) {
            flushFrame();
        }
    };

    auto pushBlock = [&](const uint8_t* bytes, uint16_t len) {
        for (uint16_t i = 0; i < len; i++) {
            pushByte(bytes[i]);
        }
    };

    for (int g = 0; g < TACTILE_GROUP_NUM; g++) {
        const TacGroup& grp = tacData.groups[g];
        pushBlock(&grp.sensor_A.forces[0][0], TACTILE_SENSOR0_TAXELS * TACTILE_AXIS_COUNT);
        pushBlock(&grp.sensor_B.forces[0][0], TACTILE_SENSOR1_TAXELS * TACTILE_AXIS_COUNT);
        pushBlock(&grp.sensor_C.forces[0][0], TACTILE_SENSOR2_TAXELS * TACTILE_AXIS_COUNT);
    }

    if (slot != 0) {
        flushFrame();
    }
}

void HalTWAI::sendTactileRawDiag(const TactileData &tacData) {
    static uint8_t seq = 0;
    const uint8_t thisSeq = seq++;

    auto sendSensorStats = [&](uint8_t group, uint8_t sensor, const uint8_t* forces, uint16_t forceLen) {
        int minZ = 127;
        int maxZ = -128;
        uint16_t absSum = 0;
        uint8_t nonZero = 0;
        for (uint16_t idx = 2; idx < forceLen; idx += TACTILE_AXIS_COUNT) {
            const int z = (int8_t)forces[idx];
            if (z < minZ) minZ = z;
            if (z > maxZ) maxZ = z;
            const int mag = abs(z);
            const uint32_t nextSum = (uint32_t)absSum + (uint32_t)mag;
            absSum = (nextSum > 0xFFFFU) ? 0xFFFFU : (uint16_t)nextSum;
            if (mag > 3 && nonZero < 0xFF) {
                nonZero++;
            }
        }
        if (absSum <= 50 && nonZero <= 4) {
            return;
        }

        uint8_t payload[8] = {
            thisSeq,
            group,
            sensor,
            (uint8_t)(int8_t)minZ,
            (uint8_t)(int8_t)maxZ,
            (uint8_t)(absSum & 0xFF),
            (uint8_t)((absSum >> 8) & 0xFF),
            nonZero,
        };
        sendFrame(CAN_ID_TAC_RAW_DIAG, payload, sizeof(payload));
    };

    for (uint8_t g = 0; g < TACTILE_GROUP_NUM; g++) {
        const TacGroup& grp = tacData.groups[g];
        sendSensorStats(g, 0, &grp.sensor_A.forces[0][0], TACTILE_SENSOR0_TAXELS * TACTILE_AXIS_COUNT);
        sendSensorStats(g, 1, &grp.sensor_B.forces[0][0], TACTILE_SENSOR1_TAXELS * TACTILE_AXIS_COUNT);
        sendSensorStats(g, 2, &grp.sensor_C.forces[0][0], TACTILE_SENSOR2_TAXELS * TACTILE_AXIS_COUNT);
    }
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
