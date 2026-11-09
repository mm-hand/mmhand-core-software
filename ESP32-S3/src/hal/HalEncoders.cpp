#include "HalEncoders.h"
#include "HalSensorSpi.h"
#include <cstring>

// 任务层使用的全局单例引用；实际对象由getInstance内的静态变量持有。
HalEncoders &encoders = HalEncoders::getInstance();

namespace {
// S3 只发映射后的 raw：DIP 原始 raw 本来就随正方向增大，直接透传；
// AA/MCP/PIP 原始 raw 随正方向减小，取 16384-raw。上位机看到的 raw 全部正方向增大。
// 掩码是左右手末端板共同验证的硬件事实，不是协议字段；换板或改链序必须同步改。
// 拇指 J20 是根部，不在 DIP 集合。
// ponytail: 固定掩码 0x88888；若末端板拓扑变化，改这里并重新验证 raw 方向。
constexpr uint32_t kDipRawMask =
    (1u << 3) | (1u << 7) | (1u << 11) | (1u << 15) | (1u << 19);
constexpr uint16_t flip14(uint16_t raw) {
    return static_cast<uint16_t>((16384u - (raw & 0x3FFFu)) & 0x3FFFu);
}
static_assert(flip14(0) == 0, "raw 0 must wrap to 0");
static_assert(flip14(1) == 16383, "raw 1 must wrap to 16383");
static_assert(flip14(16383) == 1, "raw 16383 must wrap to 1");
} // namespace

/** @brief 初始化21路状态机、命令流水线和错误锁存表。 */
HalEncoders::HalEncoders() : _spi(nullptr) {
    for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
        _fsmStates[i] = FSM_READ_ANGLE;
        _cmdPipeline[i] = AS5047P_REG_ANGLECOM;
        _latchedErrorCodes[i] = 0;
        _recoveryAttempts[i] = 0;
    }
}

/** @brief 初始化138板CS译码地址脚及底层SPI，并在任务启动前做一次接线自检。 */
void HalEncoders::begin() {
    pinMode(PIN_MUX_A, OUTPUT);
    pinMode(PIN_MUX_B, OUTPUT);
    pinMode(PIN_MUX_C, OUTPUT);
    // 7号通道未连接到有效磁编组，用作所有组的非选中空闲态。
    setDemuxChannel(ENC_DEMUX_IDLE_CHANNEL);
    setupSPI();
    _wire.seq = 1;
    probeWires(_wire);
    _wireLastMs = millis();
}

/** @brief 选手指：-1=全不选；下一代 MISO 数据选择器也在这里切换。 */
void HalEncoders::selectFinger(int8_t finger) {
    setDemuxChannel(finger < 0 ? ENC_DEMUX_IDLE_CHANNEL : getGroupDemuxChannel((uint8_t)finger));
}

/** @brief 通过公共SPI初始化模块取得磁编设备句柄。 */
void HalEncoders::setupSPI() {
    _spi = beginEncoderSensorSpi() ? encoderSpiDevice : nullptr;
}

/** @brief 输出三位二进制CS译码地址，并留5us给译码器稳定。 */
void HalEncoders::setDemuxChannel(uint8_t ch) {
    digitalWrite(PIN_MUX_A, (ch & 0x01));
    digitalWrite(PIN_MUX_B, (ch >> 1) & 0x01);
    digitalWrite(PIN_MUX_C, (ch >> 2) & 0x01);
    delayMicroseconds(5);
}

/**
 * @brief 选中一组磁编并完成一次count×16位同步SPI事务。
 * @note SPI驱动返回ESP_OK只代表控制器事务完成，不代表每颗AS5047P真实在线。
 */
bool HalEncoders::transferOnce(int8_t finger, uint8_t count, const uint16_t *txBuf, uint16_t *rxBuf) {
    if (count == 0 || count > MAX_GROUP_SIZE || txBuf == nullptr || rxBuf == nullptr) {
        return false;
    }

    if (_spi == nullptr) {
        setupSPI();
    }
    if (_spi == nullptr) {
        return false;
    }

    selectFinger(finger);

    // DMA缓冲按4字节对齐；协议线上采用高字节在前。
    uint8_t txBytes[MAX_GROUP_SIZE * 2] __attribute__((aligned(4))) = {0};
    uint8_t rxBytes[MAX_GROUP_SIZE * 2] __attribute__((aligned(4))) = {0};
    const size_t byteCount = (size_t)count * 2U;
    for (uint8_t i = 0; i < count; i++) {
        txBytes[i * 2] = (uint8_t)(txBuf[i] >> 8);
        txBytes[i * 2 + 1] = (uint8_t)(txBuf[i] & 0xFF);
    }

    // 一个事务保持同一组处于选中态，连续移动count个16位字。
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

    // 事务结束立即切回空闲通道，避免未选中器件继续驱动公共线。
    selectFinger(-1);
    return ok;
}

/** @brief 返回输入值中1的数量是奇数(1)还是偶数(0)。 */
uint8_t HalEncoders::get_parity(uint16_t n) {
    uint8_t p = 0;
    while (n) {
        p ^= 1;
        n &= (n - 1);
    }
    return p;
}

/** @brief 构造AS5047P 16位命令：地址+R/W位+偶校验位。 */
uint16_t HalEncoders::build_command_frame(uint16_t addr, bool is_read) {
    if (is_read) {
        addr |= 0x4000;
    }
    if (get_parity(addr & 0x7FFF)) {
        addr |= 0x8000;
    }
    return addr;
}

/**
 * @brief 将菊花链移位顺序换算为组内逻辑顺序。
 * @note 先整体反序，再应用实机验证得到的PCB槽位交换表。
 */
uint8_t HalEncoders::remapGroupIndex(uint8_t groupCount, uint8_t transferIndex) const {
    uint8_t originalIndex = groupCount - 1 - transferIndex;

    if (groupCount == 4) {
        static constexpr uint8_t kOldToNew[4] = {0, 1, 3, 2};
        return kOldToNew[originalIndex];
    }

    if (groupCount == 5) {
        // 拇指物理链从指尖到根部为J19、J18、J17、J16、J20。
        // 实机确认J18/J19相对初版映射交叉，因此仅交换这两个逻辑槽位。
        static constexpr uint8_t kOldToNew[5] = {4, 0, 1, 3, 2};
        return kOldToNew[originalIndex];
    }

    return originalIndex;
}

/** @brief 在组起始下标基础上叠加链序映射，得到J00~J20全局编号。 */
int HalEncoders::mapGlobalEncoderIndex(int groupStart, uint8_t groupCount, uint8_t transferIndex) const {
    return groupStart + remapGroupIndex(groupCount, transferIndex);
}

/**
 * @brief 把内部锁存错误码转换为便于串口阅读的短字符串。
 * @note [待确认是否删除] 当前没有调用者。
 */
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

/** @brief 创建零初始化快照并委托引用版本完成扫描。 */
EncoderData HalEncoders::getData() {
    EncoderData d{};
    getData(d);
    return d;
}

/** @brief 根据单路FSM决定下一拍读取ANGLE、ERRFL还是DIAAGC。 */
uint16_t HalEncoders::pickRegisterForState(int encoderId) const {
    switch (_fsmStates[encoderId]) {
    case FSM_READ_ERRFL:
        return AS5047P_REG_ERRFL;
    case FSM_READ_DIAAGC:
        return AS5047P_REG_DIAAGC;
    default:
        return AS5047P_REG_ANGLECOM;
    }
}

/** @brief 返回当前逻辑手指组对应的138板CS译码通道。 */
uint8_t HalEncoders::getGroupDemuxChannel(uint8_t groupIndex) const {
    if (groupIndex < GROUP_COUNT) {
        return kFingerChannelMap[groupIndex];
    }
    return ENC_DEMUX_IDLE_CHANNEL;
}

/** @brief 标记链路断开；连续失败达到上限后强制重新从ANGLE状态尝试。 */
void HalEncoders::markLinkLost(EncoderData &outData, int encoderId) {
    outData.errorFlags[encoderId] = 1;
    _latchedErrorCodes[encoderId] = ERR_CODE_LINK_LOST;
    _recoveryAttempts[encoderId]++;
    if (_recoveryAttempts[encoderId] >= MAX_RECOVERY) {
        _fsmStates[encoderId] = FSM_READ_ANGLE;
    }
}

/**
 * @brief 校验并解析一颗AS5047P的流水线响应。
 * @param issuedCmd 产生本次响应的上一拍寄存器命令，而非本拍刚发送的命令。
 */
void HalEncoders::decodeFrame(
    EncoderData &outData,
    int encoderId,
    uint16_t rawVal,
    uint16_t issuedCmd
) {
    // 0x0000在ANGLECOM机械零位时合法：parity=0、error=0、angle=0。
    // 因此不能按单路零值判掉线；公共MISO短地导致的21路全零目前由上位机额外识别。
    if (rawVal == 0xFFFF) {
        markLinkLost(outData, encoderId);
        return;
    }

    // bit15应使完整16位帧满足偶校验；不匹配时下一拍读取ERRFL。
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
            outData.rawAngles[encoderId] =
                (kDipRawMask & (1u << encoderId)) ? payload : flip14(payload);
            outData.errorFlags[encoderId] = 0;
            _recoveryAttempts[encoderId] = 0;
            if (_latchedErrorCodes[encoderId] != ERR_CODE_LINK_LOST) {
                _latchedErrorCodes[encoderId] = 0;
            }
            _fsmStates[encoderId] = FSM_READ_ANGLE;
        }
        break;

    case AS5047P_REG_ERRFL:
        // 错误诊断拍不提供可用角度，因此本帧仍保持errorFlags=1。
        outData.errorFlags[encoderId] = 1;
        if (payload & 0x07) {
            _latchedErrorCodes[encoderId] = ERR_CODE_ERRFL_BASE | (payload & 0x0F);
        }
        _fsmStates[encoderId] = FSM_READ_DIAAGC;
        break;

    case AS5047P_REG_DIAAGC:
        // [需进一步确认] 当前把bit11~8任一置位都锁存为诊断错误；
        // AS5047P的LF(bit8)=1通常表示内部补偿完成，不应单独当作故障。
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

/**
 * @brief 138板采集：MUX选择某组CS，一次事务发送整条菊花链，并用上一拍命令解释响应。
 */
void HalEncoders::collectGroup138(uint8_t groupIndex, int groupStart, uint8_t count, EncoderData &outData) {
    uint16_t txBuf[MAX_GROUP_SIZE] = {0};
    uint16_t rxBuf[MAX_GROUP_SIZE] = {0};
    uint16_t pendingCmds[MAX_GROUP_SIZE] = {0};

    for (uint8_t k = 0; k < count; k++) {
        int id = mapGlobalEncoderIndex(groupStart, count, k);
        if (id >= ENCODER_TOTAL_NUM) {
            id = ENCODER_TOTAL_NUM - 1;
        }

        const uint16_t reg = pickRegisterForState(id);
        pendingCmds[k] = reg;
        const uint16_t frame = build_command_frame(reg, true);
        txBuf[k] = frame;
    }

    const bool transferOk = transferOnce((int8_t)groupIndex, count, txBuf, rxBuf);

    for (uint8_t k = 0; k < count; k++) {
        const int id = mapGlobalEncoderIndex(groupStart, count, k);
        if (id >= ENCODER_TOTAL_NUM) {
            continue;
        }

        // AS5047P返回上一拍命令的结果；先取旧命令，再保存本拍命令。
        const uint16_t prevCmd = _cmdPipeline[id];
        _cmdPipeline[id] = pendingCmds[k];

        if (!transferOk) {
            markLinkLost(outData, id);
            continue;
        }

        const uint16_t rawVal = rxBuf[k];
        decodeFrame(outData, id, rawVal, prevCmd);
    }
}

/** @brief 按{4,4,4,4,5}扫描五组，并发布21路一致快照。 */
void HalEncoders::getData(EncoderData &outData) {
    int globalIdx = 0;
    outData = {};

    for (uint8_t grp = 0; grp < GROUP_COUNT; grp++) {
        const uint8_t count = group_sizes_[grp];
        // 被编译掩码关闭的组仍占据固定Jxx槽位，但明确标为LINK_LOST。
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

        collectGroup138(grp, globalIdx, count, outData);

        // 自检判为无驱动/短路的手指：整组写分类哨兵并置无效，不许浮空值冒充角度。
        if (_wire.seq != 0 && _wire.finger[grp] != 'O') {
            const uint16_t sentinel = _wire.finger[grp] == 'N' ? ENCODER_OPEN_RAW : ENCODER_SHORT_RAW;
            for (uint8_t k = 0; k < count; k++) {
                const int id = globalIdx + k;
                outData.rawAngles[id] = sentinel;
                outData.errorFlags[id] = 1;
                _latchedErrorCodes[id] = ERR_CODE_LINK_LOST;
            }
        }

        globalIdx += count;
    }

    // 最后统一复制锁存表，确保调用者得到与本轮扫描一致的错误详情。
    memcpy(outData.latchedErrors, _latchedErrorCodes, sizeof(_latchedErrorCodes));
}

/** @brief 设置 MISO 内部上下拉；0=无，1=上拉，2=下拉。 */
void HalEncoders::setMisoPull(uint8_t mode) {
    gpio_set_pull_mode((gpio_num_t)PIN_ENC_MISO,
        mode == 1 ? GPIO_PULLUP_ONLY : mode == 2 ? GPIO_PULLDOWN_ONLY : GPIO_FLOATING);
    delayMicroseconds(5);
}

/** @brief 选中一根手指、在指定拉电平下读回整链；不解释响应，只用于驱动判定。 */
bool HalEncoders::probeFinger(uint8_t finger, bool pullUp, uint16_t* rxBuf) {
    uint16_t txBuf[MAX_GROUP_SIZE] = {0};
    const uint16_t command = build_command_frame(AS5047P_REG_ANGLECOM, true);
    for (uint8_t i = 0; i < group_sizes_[finger]; i++) {
        txBuf[i] = command;
    }
    setMisoPull(pullUp ? 1 : 2);
    const bool ok = transferOnce((int8_t)finger, group_sizes_[finger], txBuf, rxBuf);
    setMisoPull(0);
    return ok;
}

/**
 * @brief 逐指接线自检：同一位置字在上拉/下拉各读一次，得到 O/N/S/H。
 * @note 探测命令固定为 ANGLE，结束后复位 FSM/流水线，避免诊断读打乱正常扫描。
 */
void HalEncoders::probeWires(EncoderWireResult& out) {
    const uint32_t seq = out.seq;
    out = {};
    out.seq = seq;
    for (uint8_t finger = 0; finger < GROUP_COUNT; finger++) {
        uint16_t up[MAX_GROUP_SIZE] = {0};
        uint16_t down[MAX_GROUP_SIZE] = {0};
        probeFinger(finger, true, up);
        probeFinger(finger, false, down);
        out.finger[finger] = wireFingerChar(up, down, group_sizes_[finger]);
        out.up[finger] = up[0];
        out.down[finger] = down[0];
    }
    for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
        _fsmStates[i] = FSM_READ_ANGLE;
        _cmdPipeline[i] = AS5047P_REG_ANGLECOM;
    }
}

/** @brief 节流执行自检；开机已测一次，之后每 2 s 一次。 */
void HalEncoders::serviceWireProbe(uint32_t nowMs) {
    if (_wireLastMs != 0 && (uint32_t)(nowMs - _wireLastMs) < ENCODER_WIRE_PROBE_PERIOD_MS) {
        return;
    }
    _wireLastMs = nowMs;
    _wire.seq++;
    probeWires(_wire);
}
