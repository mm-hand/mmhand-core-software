#include "HalTWAI.h"
#include "Crc16.h"
#include <cstring>

// 全局唯一TWAI对象，任务层直接调用。
HalTWAI twaiBus;

namespace {
// 错误详情每帧最多7路，21路恰好拆成3帧。
constexpr uint8_t kErrorCodesPerFrame = 7;
constexpr uint8_t kErrorFrameCount =
    (ENCODER_TOTAL_NUM + kErrorCodesPerFrame - 1) / kErrorCodesPerFrame;

// DIAAGC bit11=MAGL、bit10=MAGH；命名与AS5047P手册一致。
constexpr uint16_t kDiagMagLowBit = 0x0800;
constexpr uint16_t kDiagMagHighBit = 0x0400;

/** @brief 取触觉三轴最大幅值，用于摘要噪声门限。 */
uint8_t tactileAxisMagnitude(const uint8_t* axes) {
    const int x = abs((int8_t)axes[0]);
    const int y = abs((int8_t)axes[1]);
    const int z = axes[2];
    int value = x;
    if (y > value) value = y;
    if (z > value) value = z;
    return (uint8_t)value;
}

/** @brief 按配置门限过滤一组三轴触觉摘要。 */
void writeFilteredTactileSummary(uint8_t* dst, const uint8_t* src) {
    // [待确认是否简化] 当前门限为0，除全零外不会过滤任何值。
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

} // namespace

// 编译期检查CAN ID范围、分帧数量和标准帧DLC，防止改规模后静默越界。
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

/** @brief 初始状态为未安装TWAI驱动。 */
HalTWAI::HalTWAI() : _is_initialized(false) {}

/** @brief 安装并启动普通模式TWAI；收发均使用标准11位ID。 */
bool HalTWAI::begin() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)PIN_TWAI_TX,
        (gpio_num_t)PIN_TWAI_RX,
        TWAI_MODE_NORMAL
    );
    // 触觉全量调试一次可产生219帧，因此发送队列预留512深度。
    g_config.tx_queue_len = kTxQueueLength;
    g_config.rx_queue_len = 10;

    twai_timing_config_t t_config = CAN_BAUD_RATE;
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        return false;
    }
    if (twai_start() != ESP_OK) {
        // [待确认是否修正] 此处没有卸载已安装驱动，后续再次begin可能返回已安装错误。
        return false;
    }

    _is_initialized = true;
    return true;
}

/** @brief 复制负载并阻塞等待最多5ms加入TWAI发送队列。 */
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

/**
 * @brief 将21个uint16按大端序拆成6个数据帧，再发送CRC元数据帧。
 * @note 数据帧本身不带序号；P4依赖完整帧集合和末尾CRC拒绝跨批混帧。
 */
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

    // 序号放在CRC帧中，uint8自然回绕。
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

/** @brief 展平15组三轴摘要和模块有效位图，附加CRC16后拆成7个CAN帧。 */
uint8_t HalTWAI::sendTactileSummary(const TactileData &tacData, uint16_t moduleValidMask) {
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

    // 高字节在前，和P4侧其他16位协议字段保持一致；bit0~bit14对应G0S0~G4S2。
    packet[TACTILE_SUMMARY_BYTES] = (uint8_t)(moduleValidMask >> 8);
    packet[TACTILE_SUMMARY_BYTES + 1] = (uint8_t)(moduleValidMask & 0xFF);
    const uint16_t crc = crc16Ccitt(packet, CAN_TAC_SUMMARY_CRC_INPUT_BYTES);
    packet[CAN_TAC_SUMMARY_CRC_INPUT_BYTES] = (uint8_t)(crc >> 8);
    packet[CAN_TAC_SUMMARY_CRC_INPUT_BYTES + 1] = (uint8_t)(crc & 0xFF);

    static uint8_t seq = 0;
    const uint8_t thisSeq = seq++;
    uint8_t buffer[CAN_TAC_SUMMARY_DLC] = {0};
    for (uint8_t frame = 0; frame < CAN_TAC_SUMMARY_FRAME_COUNT; frame++) {
        buffer[0] = thisSeq;
        for (uint8_t slot = 0; slot < CAN_TAC_SUMMARY_PAYLOAD_PER_FRAME; slot++) {
            const uint8_t idx = frame * CAN_TAC_SUMMARY_PAYLOAD_PER_FRAME + slot;
            buffer[slot + 1] = (idx < CAN_TAC_SUMMARY_PACKET_BYTES) ? packet[idx] : 0;
        }
        // [待确认是否增强] 当前忽略单帧发送失败，调用者仍会收到并使用该批次序号。
        sendFrame(CAN_ID_TAC_SUMMARY_BASE + frame, buffer, CAN_TAC_SUMMARY_DLC);
    }
    return thisSeq;
}

/**
 * @brief 将1530字节全量触觉流拆成7字节负载CAN帧。
 * @warning 一批约219帧，默认关闭；开启后会显著占用1Mbit/s总线。
 */
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
        // 每64帧主动让出1ms，降低长批次对其他任务的连续阻塞。
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

/** @brief 仅对Z轴活动明显的模块发送最小值、最大值、绝对和和非零数。 */
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

/** @brief 把内部错误位组合压缩为P4协议定义的单字节错误码。 */
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

// 三帧格式均为[seq + 7路错误码]：0x1F0覆盖J00~J06，依次到J20。
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

/** @brief 维护TWAI状态机：bus-off发起恢复、STOPPED尝试重新启动。 */
bool HalTWAI::maintain(twai_status_info_t& status_info, bool& statusValid) {
    statusValid = twai_get_status_info(&status_info) == ESP_OK;
    if (!statusValid) {
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

    return (status_info.state == TWAI_STATE_RUNNING);
}
