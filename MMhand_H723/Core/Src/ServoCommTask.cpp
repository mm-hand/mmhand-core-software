#include "ServoCommTask.h"

#include "HardwareMap.h"

#include <cstdlib>
#include <cstring>

extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart9;
extern UART_HandleTypeDef huart1;

#ifndef SERVO_CONFIGURE_MULTI_TURN_ON_BOOT
#define SERVO_CONFIGURE_MULTI_TURN_ON_BOOT 1
#endif

#ifndef SERVO_BOOT_RECOVER_LIMIT_TO_RAW
#define SERVO_BOOT_RECOVER_LIMIT_TO_RAW 0
#endif

static constexpr uint8_t INST_READ = 0x02U;
static constexpr uint8_t INST_WRITE = 0x03U;
static constexpr uint8_t INST_SYNC_READ = 0x82U;
static constexpr uint8_t INST_SYNC_WRITE = 0x83U;
static constexpr uint8_t SMS_STS_MIN_ANGLE_LIMIT_L = 9U;
static constexpr uint8_t SMS_STS_TORQUE_ENABLE = 40U;
static constexpr uint8_t SMS_STS_ACC = 41U;
static constexpr uint8_t SMS_STS_LOCK = 55U;
static constexpr uint8_t SMS_STS_PRESENT_POSITION_L = 56U;
static constexpr uint8_t SMS_STS_PRESENT_READ_LEN = 8U;
static constexpr uint32_t SERVO_MULTI_TURN_CONFIG_DELAY_MS = 5U;
static constexpr uint32_t SERVO_TASK_PERIOD_MS = 10U;
static constexpr uint32_t SERVO_ACK_TIMEOUT_MS = 8U;
static constexpr uint32_t SERVO_READ_TIMEOUT_MS = 8U;
static constexpr uint32_t kBootRecoverLimitToRawWindowMs = 8000U;
static constexpr int32_t kServoAbsLimit = 30719;
static constexpr int32_t kServoAbsLimitRecoverMargin = 16;

ServoDebugBus servo1[SERVO_DEBUG_BUS_COUNT + 1U];
ServoComm servoComm;

static ServoDebugId *getServoDebugId(uint8_t busIndex, uint8_t id)
{
    const uint8_t debugBusIndex = static_cast<uint8_t>(busIndex + 1U);
    if ((debugBusIndex == 0U) ||
        (debugBusIndex > SERVO_DEBUG_BUS_COUNT) ||
        (id == 0U) ||
        (id > SERVO_DEBUG_ID_COUNT)) {
        return nullptr;
    }
    return &servo1[debugBusIndex].ID[id];
}

static void initServoDebugTable()
{
    static bool initialized = false;
    if (initialized) {
        return;
    }

    for (uint8_t bus = 1U; bus <= SERVO_DEBUG_BUS_COUNT; bus++) {
        servo1[bus].initialized = 1U;
        servo1[bus].busIndex = bus;
        servo1[bus].onlineCount = 0U;

        for (uint8_t id = 1U; id <= SERVO_DEBUG_ID_COUNT; id++) {
            ServoDebugId &item = servo1[bus].ID[id];
            item.initialized = 1U;
            item.busIndex = bus;
            item.servoId = id;
            item.online = 0U;
            item.lastStatus = 0U;
            item.lastError = 0U;
            item.send = 0U;
            item.absolutePosition = 0;
            item.rawPosition = 0;
            item.speed = 0;
            item.load = 0;
            item.voltage = 0U;
            item.temperature = 0U;
            item.lastUpdate = 0U;
            item.readOkCount = 0U;
            item.timeoutCount = 0U;
            item.writeOkCount = 0U;
            item.targetPosition = 0;
            item.targetSpeed = SERVO_TARGET_SPEED_DEFAULT;
            item.targetAcc = SERVO_TARGET_ACC_DEFAULT;
        }
    }

    initialized = true;
}

static void markServoDebugOffline(uint8_t busIndex, uint8_t id, uint8_t error)
{
    ServoDebugId *item = getServoDebugId(busIndex, id);
    if (item == nullptr) {
        return;
    }

    item->online = 0U;
    item->lastError = error;
    item->timeoutCount++;
    item->lastUpdate = HAL_GetTick();
}

struct ServoFeedbackInternal {
    int16_t rawPosition;
    int16_t speed;
    int16_t load;
    uint8_t voltage;
    uint8_t temperature;
    int32_t absolutePosition;
    int16_t turnCount;
    int16_t lastRawPosition;
    bool initialized;
    bool online;
    uint32_t lastUpdate;
};

static bool isServoAbsAtLimit(int32_t value)
{
    return value >= (kServoAbsLimit - kServoAbsLimitRecoverMargin) ||
           value <= (-kServoAbsLimit + kServoAbsLimitRecoverMargin);
}

static int16_t clampServoAbsLocal(int32_t value)
{
    if (value > kServoAbsLimit) {
        return static_cast<int16_t>(kServoAbsLimit);
    }
    if (value < -kServoAbsLimit) {
        return static_cast<int16_t>(-kServoAbsLimit);
    }
    return static_cast<int16_t>(value);
}

static int16_t normalizeServoRawSingleTurn(int32_t value)
{
    int32_t raw = value % 4096;
    if (raw < 0) {
        raw += 4096;
    }
    return static_cast<int16_t>(raw);
}

static uint16_t singleTurnDistanceCounts(int16_t a, int16_t b)
{
    int32_t diff = std::abs(static_cast<int32_t>(normalizeServoRawSingleTurn(a)) -
                            static_cast<int32_t>(normalizeServoRawSingleTurn(b))) % 4096;
    if (diff > 2048) {
        diff = 4096 - diff;
    }
    return static_cast<uint16_t>(diff);
}

class ServoBusDriver {
public:
    ServoBusDriver()
        : uart(nullptr),
          busIndex(0U),
          writeCount(0U),
          syncReadTimeoutMs(SERVO_SYNC_READ_TIMEOUT_MS_DEFAULT),
          txCount(0U),
          rxCount(0U),
          timeoutCount(0U),
          checksumErrorCount(0U),
          invalidFrameCount(0U)
    {
        std::memset(feedback, 0, sizeof(feedback));
        std::memset(torqueDisabled, 0, sizeof(torqueDisabled));
        std::memset(writeIDs, 0, sizeof(writeIDs));
        std::memset(writePos, 0, sizeof(writePos));
        std::memset(writeSpd, 0, sizeof(writeSpd));
        std::memset(writeAcc, 0, sizeof(writeAcc));
    }

    void begin(uint8_t index, UART_HandleTypeDef *handle)
    {
        busIndex = index;
        uart = handle;
    }

    bool configureMultiTurnMode(uint8_t id)
    {
        if (!isUsableId(id)) {
            return false;
        }

        uint8_t zeroLimits[4] = {0U, 0U, 0U, 0U};
        bool ok = true;
        ok = writeByte(id, SMS_STS_LOCK, 0U) && ok;
        vTaskDelay(pdMS_TO_TICKS(SERVO_MULTI_TURN_CONFIG_DELAY_MS));
        ok = writeRegister(id, SMS_STS_MIN_ANGLE_LIMIT_L, zeroLimits, sizeof(zeroLimits)) && ok;
        vTaskDelay(pdMS_TO_TICKS(SERVO_MULTI_TURN_CONFIG_DELAY_MS));
        ok = writeByte(id, SMS_STS_LOCK, 1U) && ok;
        vTaskDelay(pdMS_TO_TICKS(SERVO_MULTI_TURN_CONFIG_DELAY_MS));
        return ok;
    }

    void setTarget(uint8_t id, int16_t position, uint16_t targetSpeed, uint8_t acc)
    {
        if (!isUsableId(id) || (writeCount >= MAX_SERVOS_PER_BUS)) {
            return;
        }
        if (torqueDisabled[id]) {
            (void)setTorqueEnabled(id, true);
        }

        writeIDs[writeCount] = id;
        writePos[writeCount] = clampServoAbsLocal(position);
        writeSpd[writeCount] = targetSpeed;
        writeAcc[writeCount] = acc;
        writeCount++;
    }

    void syncWriteAll()
    {
        if ((writeCount == 0U) || (uart == nullptr)) {
            return;
        }

        uint8_t payload[MAX_SERVOS_PER_BUS * 8U] = {};
        for (uint8_t i = 0U; i < writeCount; i++) {
            uint16_t pos = encodeSigned15(writePos[i]);
            const uint8_t base = static_cast<uint8_t>(i * 8U);
            payload[base] = writeIDs[i];
            payload[base + 1U] = writeAcc[i];
            payload[base + 2U] = static_cast<uint8_t>(pos & 0xFFU);
            payload[base + 3U] = static_cast<uint8_t>((pos >> 8) & 0xFFU);
            payload[base + 4U] = 0U;
            payload[base + 5U] = 0U;
            payload[base + 6U] = static_cast<uint8_t>(writeSpd[i] & 0xFFU);
            payload[base + 7U] = static_cast<uint8_t>((writeSpd[i] >> 8) & 0xFFU);
        }

        const uint8_t paramLen = 7U;
        const uint8_t packetLen = static_cast<uint8_t>(((paramLen + 1U) * writeCount) + 4U);
        uint8_t frame[80] = {};
        uint16_t idx = 0U;
        frame[idx++] = 0xFFU;
        frame[idx++] = 0xFFU;
        frame[idx++] = 0xFEU;
        frame[idx++] = packetLen;
        frame[idx++] = INST_SYNC_WRITE;
        frame[idx++] = SMS_STS_ACC;
        frame[idx++] = paramLen;
        for (uint8_t i = 0U; i < (writeCount * 8U); i++) {
            frame[idx++] = payload[i];
        }
        const uint8_t sumLen = static_cast<uint8_t>(idx - 2U);
        frame[idx++] = checksum(&frame[2], sumLen);

        clearRx();
        if (HAL_UART_Transmit(uart, frame, idx, SERVO_ACK_TIMEOUT_MS) == HAL_OK) {
            txCount++;
            servoComm.txFrameCount++;
            servoComm.recordTx(frame, idx);
        }
        writeCount = 0U;
    }

    int syncReadPositions(const uint8_t *ids, uint8_t count)
    {
        if ((uart == nullptr) || (ids == nullptr) || (count == 0U)) {
            return 0;
        }

        if (count > MAX_SERVOS_PER_BUS) {
            count = MAX_SERVOS_PER_BUS;
        }

        uint8_t frame[16] = {};
        uint16_t idx = 0U;
        frame[idx++] = 0xFFU;
        frame[idx++] = 0xFFU;
        frame[idx++] = 0xFEU;
        frame[idx++] = static_cast<uint8_t>(count + 4U);
        frame[idx++] = INST_SYNC_READ;
        frame[idx++] = SMS_STS_PRESENT_POSITION_L;
        frame[idx++] = SMS_STS_PRESENT_READ_LEN;
        for (uint8_t i = 0U; i < count; i++) {
            frame[idx++] = ids[i];
        }
        const uint8_t sumLen = static_cast<uint8_t>(idx - 2U);
        frame[idx++] = checksum(&frame[2], sumLen);

        clearRx();
        if (HAL_UART_Transmit(uart, frame, idx, SERVO_ACK_TIMEOUT_MS) != HAL_OK) {
            timeoutCount++;
            servoComm.timeoutCount++;
            return 0;
        }
        txCount++;
        servoComm.txFrameCount++;
        servoComm.recordTx(frame, idx);

        uint8_t rxBuffer[MAX_SERVOS_PER_BUS * (SMS_STS_PRESENT_READ_LEN + 6U)] = {};
        const uint16_t rxLen = receiveBuffer(rxBuffer,
                                             static_cast<uint16_t>(count * (SMS_STS_PRESENT_READ_LEN + 6U)),
                                             syncReadTimeoutMs);
        servoComm.recordRx(rxBuffer, rxLen);
        if (rxLen == 0U) {
            timeoutCount++;
            servoComm.timeoutCount++;
            for (uint8_t i = 0U; i < count; i++) {
                if (ids[i] <= MAX_SERVO_ID) {
                    feedback[ids[i]].online = false;
                    markServoDebugOffline(busIndex, ids[i], 1U);
                    if ((busIndex == servoComm.selectedBus) && (ids[i] == servoComm.selectedId)) {
                        servoComm.online = 0U;
                    }
                }
            }
            return 0;
        }

        int successCount = 0;
        for (uint8_t i = 0U; i < count; i++) {
            const uint8_t id = ids[i];
            uint8_t payload[SMS_STS_PRESENT_READ_LEN] = {};
            uint8_t status = 0U;
            if ((id <= MAX_SERVO_ID) &&
                parseStatusFrame(rxBuffer, rxLen, id, SMS_STS_PRESENT_READ_LEN, payload, &status)) {
                const int16_t rawPos = decodeSigned15(payload[0], payload[1]);
                updateMultiTurnPosition(id, rawPos);
                feedback[id].speed = decodeSigned15(payload[2], payload[3]);
                feedback[id].load = decodeLoad(payload[4], payload[5]);
                feedback[id].voltage = payload[6];
                feedback[id].temperature = payload[7];
                feedback[id].online = (status == 0U);
                feedback[id].lastUpdate = HAL_GetTick();
                if (feedback[id].online) {
                    rxCount++;
                    servoComm.rxFrameCount++;
                    servoComm.readOkCount++;
                    servoComm.recordFeedback(busIndex,
                                          id,
                                          feedback[id].rawPosition,
                                          feedback[id].absolutePosition,
                                          feedback[id].speed,
                                          feedback[id].load,
                                          feedback[id].voltage,
                                          feedback[id].temperature);
                    successCount++;
                }
            } else if (id <= MAX_SERVO_ID) {
                feedback[id].online = false;
                markServoDebugOffline(busIndex, id, 2U);
                if ((busIndex == servoComm.selectedBus) && (id == servoComm.selectedId)) {
                    servoComm.online = 0U;
                }
            }
        }

        return successCount;
    }

    int16_t getRawPosition(uint8_t id) const
    {
        if (id > MAX_SERVO_ID) {
            return -1;
        }
        return feedback[id].rawPosition;
    }

    int32_t getAbsolutePosition(uint8_t id) const
    {
        if (id > MAX_SERVO_ID) {
            return 0;
        }
        return feedback[id].absolutePosition;
    }

    const ServoFeedbackInternal &getFeedback(uint8_t id) const
    {
        static ServoFeedbackInternal dummy = {};
        if (id > MAX_SERVO_ID) {
            return dummy;
        }
        return feedback[id];
    }

    bool isOnline(uint8_t id) const
    {
        return (id <= MAX_SERVO_ID) && feedback[id].online;
    }

    bool setTorqueEnabled(uint8_t id, bool enabled)
    {
        if (!isUsableId(id)) {
            return false;
        }
        const bool ok = writeByte(id, SMS_STS_TORQUE_ENABLE, enabled ? 1U : 0U);
        if (ok) {
            torqueDisabled[id] = !enabled;
        }
        return ok;
    }

    bool restoreAbsolutePosition(uint8_t id, int32_t absPosition, uint16_t rawToleranceCounts)
    {
        if ((id == 0U) || (id > MAX_SERVO_ID)) {
            return false;
        }

        ServoFeedbackInternal &fb = feedback[id];
        if (!fb.online) {
            return false;
        }

        const int16_t currentRaw = normalizeServoRawSingleTurn(fb.rawPosition);
        const int16_t restoredRaw = normalizeServoRawSingleTurn(absPosition);
        if (singleTurnDistanceCounts(currentRaw, restoredRaw) > rawToleranceCounts) {
            return false;
        }

        const int32_t absClamped = clampServoAbsLocal(absPosition);
        fb.rawPosition = currentRaw;
        fb.lastRawPosition = currentRaw;
        fb.absolutePosition = absClamped;
        fb.turnCount = static_cast<int16_t>((absClamped - currentRaw) / 4096);
        fb.initialized = true;
        fb.lastUpdate = HAL_GetTick();
        return true;
    }

    uint8_t onlineCountForMappedChannels(void) const
    {
        uint8_t count = 0U;
        for (uint8_t ch = 0U; ch < SERVO_TOTAL_NUM; ch++) {
            if (isMotorChannelValid(ch) &&
                motorMap[ch].busIndex == busIndex &&
                isOnline(motorMap[ch].servoID)) {
                count++;
            }
        }
        return count;
    }

private:
    bool isUsableId(uint8_t id) const
    {
        return (uart != nullptr) && (id != 0U) && (id <= MAX_SERVO_ID);
    }

    static uint8_t checksum(const uint8_t *data, uint8_t len)
    {
        uint8_t sum = 0U;
        for (uint8_t i = 0U; i < len; i++) {
            sum = static_cast<uint8_t>(sum + data[i]);
        }
        return static_cast<uint8_t>(~sum);
    }

    static uint16_t encodeSigned15(int16_t value)
    {
        if (value < 0) {
            return static_cast<uint16_t>((-value) | 0x8000U);
        }
        return static_cast<uint16_t>(value);
    }

    static int16_t decodeSigned15(uint8_t low, uint8_t high)
    {
        const uint16_t value = static_cast<uint16_t>(low) | (static_cast<uint16_t>(high) << 8);
        if ((value & 0x8000U) != 0U) {
            return static_cast<int16_t>(-static_cast<int16_t>(value & 0x7FFFU));
        }
        return static_cast<int16_t>(value);
    }

    static int16_t decodeLoad(uint8_t low, uint8_t high)
    {
        const uint16_t value = static_cast<uint16_t>(low) | (static_cast<uint16_t>(high) << 8);
        if ((value & 0x0400U) != 0U) {
            return static_cast<int16_t>(-static_cast<int16_t>(value & 0x03FFU));
        }
        return static_cast<int16_t>(value & 0x03FFU);
    }

    void clearRx()
    {
        if (uart == nullptr) {
            return;
        }
        uint8_t drop = 0U;
        while (HAL_UART_Receive(uart, &drop, 1U, 0U) == HAL_OK) {
        }
        __HAL_UART_CLEAR_OREFLAG(uart);
    }

    uint16_t receiveBuffer(uint8_t *buffer, uint16_t maxLen, uint32_t timeoutMs)
    {
        if ((uart == nullptr) || (buffer == nullptr) || (maxLen == 0U)) {
            return 0U;
        }

        uint16_t len = 0U;
        const uint32_t start = HAL_GetTick();
        while (((HAL_GetTick() - start) <= timeoutMs) && (len < maxLen)) {
            if (HAL_UART_Receive(uart, &buffer[len], 1U, 1U) == HAL_OK) {
                len++;
            }
        }
        return len;
    }

    bool writeRegister(uint8_t id, uint8_t address, const uint8_t *data, uint8_t len)
    {
        if (!isUsableId(id) || (data == nullptr && len > 0U)) {
            return false;
        }

        uint8_t frame[48] = {};
        uint16_t idx = 0U;
        frame[idx++] = 0xFFU;
        frame[idx++] = 0xFFU;
        frame[idx++] = id;
        frame[idx++] = static_cast<uint8_t>(len + 3U);
        frame[idx++] = INST_WRITE;
        frame[idx++] = address;
        for (uint8_t i = 0U; i < len; i++) {
            frame[idx++] = data[i];
        }
        const uint8_t sumLen = static_cast<uint8_t>(idx - 2U);
        frame[idx++] = checksum(&frame[2], sumLen);

        clearRx();
        if (HAL_UART_Transmit(uart, frame, idx, SERVO_ACK_TIMEOUT_MS) != HAL_OK) {
            timeoutCount++;
            servoComm.timeoutCount++;
            return false;
        }
        txCount++;
        servoComm.txFrameCount++;
        servoComm.recordTx(frame, idx);
        return readAck(id);
    }

    bool writeByte(uint8_t id, uint8_t address, uint8_t value)
    {
        return writeRegister(id, address, &value, 1U);
    }

    bool readAck(uint8_t id)
    {
        uint8_t rx[8] = {};
        const uint16_t rxLen = receiveBuffer(rx, sizeof(rx), SERVO_ACK_TIMEOUT_MS);
        servoComm.recordRx(rx, rxLen);
        if (rxLen == 0U) {
            timeoutCount++;
            servoComm.timeoutCount++;
            return false;
        }

        uint8_t payload[1] = {};
        uint8_t status = 0U;
        const bool ok = parseStatusFrame(rx, rxLen, id, 0U, payload, &status) && (status == 0U);
        if (ok) {
            rxCount++;
            servoComm.rxFrameCount++;
        }
        return ok;
    }

    bool parseStatusFrame(const uint8_t *rx,
                          uint16_t rxLen,
                          uint8_t id,
                          uint8_t payloadLen,
                          uint8_t *payload,
                          uint8_t *status)
    {
        const uint16_t frameLen = static_cast<uint16_t>(payloadLen + 6U);
        for (uint16_t offset = 0U; (offset + frameLen) <= rxLen; offset++) {
            if ((rx[offset] != 0xFFU) || (rx[offset + 1U] != 0xFFU)) {
                continue;
            }
            if ((rx[offset + 2U] != id) || (rx[offset + 3U] != static_cast<uint8_t>(payloadLen + 2U))) {
                continue;
            }

            const uint8_t calc = checksum(&rx[offset + 2U], static_cast<uint8_t>(payloadLen + 3U));
            if (calc != rx[offset + frameLen - 1U]) {
                checksumErrorCount++;
                servoComm.checksumErrorCount++;
                return false;
            }

            if (status != nullptr) {
                *status = rx[offset + 4U];
            }
            servoComm.lastStatus = rx[offset + 4U];
            servoComm.lastError = rx[offset + 4U];

            if ((payload != nullptr) && (payloadLen > 0U)) {
                std::memcpy(payload, &rx[offset + 5U], payloadLen);
            }
            return true;
        }

        invalidFrameCount++;
        servoComm.invalidFrameCount++;
        return false;
    }

    static int32_t expandSingleTurnRawNearAnchor(int16_t rawSingleTurn, int32_t anchor)
    {
        int32_t candidate = rawSingleTurn;
        while ((candidate - anchor) > 2048) {
            candidate -= 4096;
        }
        while ((anchor - candidate) > 2048) {
            candidate += 4096;
        }
        return clampServoAbsLocal(candidate);
    }

    void updateMultiTurnPosition(uint8_t id, int16_t newRawPos)
    {
        ServoFeedbackInternal &fb = feedback[id];
        int32_t absPos = 0;
        int16_t rawSingleTurn = 0;

        if ((newRawPos < 0) || (newRawPos >= 4096)) {
            absPos = clampServoAbsLocal(static_cast<int32_t>(newRawPos));
            rawSingleTurn = normalizeServoRawSingleTurn(absPos);
        } else {
            rawSingleTurn = normalizeServoRawSingleTurn(newRawPos);
            const int32_t anchor = fb.initialized ? fb.absolutePosition : static_cast<int32_t>(rawSingleTurn);
            absPos = expandSingleTurnRawNearAnchor(rawSingleTurn, anchor);
        }

        fb.rawPosition = rawSingleTurn;
        fb.lastRawPosition = rawSingleTurn;
        fb.absolutePosition = absPos;
        fb.turnCount = static_cast<int16_t>((absPos - rawSingleTurn) / 4096);
        fb.initialized = true;
    }

    UART_HandleTypeDef *uart;
    uint8_t busIndex;
    uint8_t writeIDs[MAX_SERVOS_PER_BUS];
    int16_t writePos[MAX_SERVOS_PER_BUS];
    uint16_t writeSpd[MAX_SERVOS_PER_BUS];
    uint8_t writeAcc[MAX_SERVOS_PER_BUS];
    uint8_t writeCount;
    ServoFeedbackInternal feedback[MAX_SERVO_ID + 1U];
    bool torqueDisabled[MAX_SERVO_ID + 1U];
    uint16_t syncReadTimeoutMs;
    uint32_t txCount;
    uint32_t rxCount;
    uint32_t timeoutCount;
    uint32_t checksumErrorCount;
    uint32_t invalidFrameCount;
};

static ServoBusDriver g_servoBuses[NUM_BUSES];

static ServoBusDriver *getBusByIndex(uint8_t busIndex)
{
    if (busIndex >= NUM_BUSES) {
        return nullptr;
    }
    return &g_servoBuses[busIndex];
}

static bool addReadTarget(uint8_t busIds[NUM_BUSES][MAX_SERVOS_PER_BUS],
                          uint8_t busCounts[NUM_BUSES],
                          uint8_t bus,
                          uint8_t id)
{
    if (bus >= NUM_BUSES) {
        return false;
    }
    for (uint8_t i = 0U; i < busCounts[bus]; i++) {
        if (busIds[bus][i] == id) {
            return true;
        }
    }
    if (busCounts[bus] >= MAX_SERVOS_PER_BUS) {
        return false;
    }
    busIds[bus][busCounts[bus]++] = id;
    return true;
}

static bool restoreServoMultiTurnFromHost(TaskSharedData_t *sharedData,
                                          uint32_t motorMask,
                                          const int32_t *motorAbs,
                                          uint16_t rawToleranceCounts,
                                          uint32_t *restoredMask,
                                          uint32_t *rejectedMask)
{
    uint32_t restored = 0U;
    uint32_t rejected = 0U;
    if ((sharedData == nullptr) || (motorAbs == nullptr)) {
        if (restoredMask != nullptr) {
            *restoredMask = 0U;
        }
        if (rejectedMask != nullptr) {
            *rejectedMask = motorMask;
        }
        return false;
    }

    const uint32_t requestMask = motorMask & validMotorChannelMask();
    for (uint8_t ch = 0U; ch < SERVO_TOTAL_NUM; ch++) {
        const uint32_t bit = (1UL << ch);
        if ((requestMask & bit) == 0U) {
            continue;
        }

        const uint8_t busIndex = motorMap[ch].busIndex;
        const uint8_t id = motorMap[ch].servoID;
        ServoBusDriver *bus = getBusByIndex(busIndex);
        if ((bus == nullptr) ||
            (id == 0U) ||
            (id > MAX_SERVO_ID) ||
            !bus->restoreAbsolutePosition(id, motorAbs[ch], rawToleranceCounts)) {
            rejected |= bit;
            continue;
        }
        restored |= bit;
    }

    if (restored != 0U) {
        sharedData->multi_turn_unknown_bitmap &= ~restored;
    }

    if (restoredMask != nullptr) {
        *restoredMask = restored;
    }
    if (rejectedMask != nullptr) {
        *rejectedMask = rejected;
    }
    return (requestMask != 0U) && ((requestMask & ~restored) == 0U);
}

static void beginServoBuses()
{
    static bool initialized = false;
    if (initialized) {
        return;
    }

    initServoDebugTable();
    g_servoBuses[0].begin(0U, &huart7);
    g_servoBuses[1].begin(1U, &huart9);
    g_servoBuses[2].begin(2U, &huart1);
    initialized = true;
}

static void configureMappedServosForMultiTurn()
{
#if SERVO_CONFIGURE_MULTI_TURN_ON_BOOT
    static bool configuredOnce = false;
    if (configuredOnce) {
        return;
    }

    uint8_t configuredIds[NUM_BUSES][MAX_SERVOS_PER_BUS] = {};
    uint8_t configuredCounts[NUM_BUSES] = {};

    for (uint8_t ch = 0U; ch < SERVO_TOTAL_NUM; ch++) {
        if (!isMotorChannelValid(ch)) {
            continue;
        }

        const uint8_t busIndex = motorMap[ch].busIndex;
        const uint8_t id = motorMap[ch].servoID;
        bool alreadyConfigured = false;
        for (uint8_t i = 0U; i < configuredCounts[busIndex]; i++) {
            if (configuredIds[busIndex][i] == id) {
                alreadyConfigured = true;
                break;
            }
        }
        if (alreadyConfigured || (configuredCounts[busIndex] >= MAX_SERVOS_PER_BUS)) {
            continue;
        }

        ServoBusDriver *bus = getBusByIndex(busIndex);
        if ((bus != nullptr) && bus->configureMultiTurnMode(id)) {
            configuredIds[busIndex][configuredCounts[busIndex]++] = id;
        }
    }
    configuredOnce = true;
#endif
}

static void applyServoTargetBatch(const ServoTargetBatch_t &batch)
{
    bool busWritePending[NUM_BUSES] = {};

    for (uint8_t i = 0U; (i < batch.count) && (i < SERVO_TARGET_BATCH_MAX); i++) {
        const ServoTargetCommand_t &cmd = batch.commands[i];
        ServoBusDriver *bus = getBusByIndex(cmd.busIndex);
        if (bus == nullptr) {
            continue;
        }
        bus->setTarget(cmd.servoId, cmd.position, cmd.speed, cmd.acc);
        busWritePending[cmd.busIndex] = true;
    }

    for (uint8_t busIndex = 0U; busIndex < NUM_BUSES; busIndex++) {
        if (busWritePending[busIndex]) {
            g_servoBuses[busIndex].syncWriteAll();
            servoComm.writeBatchCount++;
        }
    }
}

static void processDebugServoCommands()
{
    bool busWritePending[NUM_BUSES] = {};

    for (uint8_t debugBus = 1U; debugBus <= SERVO_DEBUG_BUS_COUNT; debugBus++) {
        const uint8_t busIndex = static_cast<uint8_t>(debugBus - 1U);
        ServoBusDriver *bus = getBusByIndex(busIndex);
        if (bus == nullptr) {
            continue;
        }

        for (uint8_t id = 1U; id <= SERVO_DEBUG_ID_COUNT; id++) {
            ServoDebugId &item = servo1[debugBus].ID[id];
            if (item.send == 0U) {
                continue;
            }

            const uint16_t speed = (item.targetSpeed == 0U) ? SERVO_TARGET_SPEED_DEFAULT : item.targetSpeed;
            const uint8_t acc = (item.targetAcc == 0U) ? SERVO_TARGET_ACC_DEFAULT : item.targetAcc;
            bus->setTarget(id, clampServoAbsLocal(item.targetPosition), speed, acc);
            item.send = 0U;
            item.writeOkCount++;
            busWritePending[busIndex] = true;
        }
    }

    for (uint8_t busIndex = 0U; busIndex < NUM_BUSES; busIndex++) {
        if (busWritePending[busIndex]) {
            g_servoBuses[busIndex].syncWriteAll();
            servoComm.writeBatchCount++;
        }
    }
}

static void emergencyHoldMappedServosAtFeedback(TaskSharedData_t *sharedData)
{
    if ((sharedData != nullptr) && (sharedData->servoTargetQueue != nullptr)) {
        xQueueReset(sharedData->servoTargetQueue);
    }

    uint8_t heldIds[NUM_BUSES][MAX_SERVOS_PER_BUS] = {};
    uint8_t heldCounts[NUM_BUSES] = {};
    bool busWritePending[NUM_BUSES] = {};
    const uint32_t unknownMask = (sharedData != nullptr) ? sharedData->multi_turn_unknown_bitmap : 0U;

    for (uint8_t ch = 0U; ch < SERVO_TOTAL_NUM; ch++) {
        const uint32_t bit = (1UL << ch);
        if (!isMotorChannelValid(ch)) {
            continue;
        }

        const uint8_t busIndex = motorMap[ch].busIndex;
        const uint8_t id = motorMap[ch].servoID;
        bool alreadyHeld = false;
        for (uint8_t i = 0U; i < heldCounts[busIndex]; i++) {
            if (heldIds[busIndex][i] == id) {
                alreadyHeld = true;
                break;
            }
        }
        if (alreadyHeld || (heldCounts[busIndex] >= MAX_SERVOS_PER_BUS)) {
            continue;
        }

        ServoBusDriver *bus = getBusByIndex(busIndex);
        if ((bus == nullptr) || !bus->isOnline(id)) {
            continue;
        }

        if ((unknownMask & bit) != 0U) {
            (void)bus->setTorqueEnabled(id, false);
            continue;
        }

        const int32_t holdPos32 = bus->getAbsolutePosition(id);
        const int16_t rawSingleTurn = normalizeServoRawSingleTurn(bus->getRawPosition(id));
        const int16_t holdPos = isServoAbsAtLimit(holdPos32)
            ? rawSingleTurn
            : clampServoAbsLocal(holdPos32);
        bus->setTarget(id, holdPos, SERVO_TARGET_SPEED_DEFAULT, SERVO_TARGET_ACC_DEFAULT);
        heldIds[busIndex][heldCounts[busIndex]++] = id;
        busWritePending[busIndex] = true;
    }

    for (uint8_t busIndex = 0U; busIndex < NUM_BUSES; busIndex++) {
        if (busWritePending[busIndex]) {
            g_servoBuses[busIndex].syncWriteAll();
            servoComm.emergencyHoldCount++;
        }
    }
}

static void bootRecoverLimitServosToRaw(uint32_t bootRecoverDeadlineMs)
{
#if SERVO_BOOT_RECOVER_LIMIT_TO_RAW
    static bool attempted[NUM_BUSES][MAX_SERVO_ID + 1U] = {};

    const uint32_t nowMs = HAL_GetTick();
    if ((int32_t)(nowMs - bootRecoverDeadlineMs) > 0) {
        return;
    }

    uint8_t heldIds[NUM_BUSES][MAX_SERVOS_PER_BUS] = {};
    uint8_t heldCounts[NUM_BUSES] = {};
    bool busWritePending[NUM_BUSES] = {};

    for (uint8_t ch = 0U; ch < SERVO_TOTAL_NUM; ch++) {
        if (!isMotorChannelValid(ch)) {
            continue;
        }

        const uint8_t busIndex = motorMap[ch].busIndex;
        const uint8_t id = motorMap[ch].servoID;
        if (attempted[busIndex][id]) {
            continue;
        }

        bool alreadyHeld = false;
        for (uint8_t i = 0U; i < heldCounts[busIndex]; i++) {
            if (heldIds[busIndex][i] == id) {
                alreadyHeld = true;
                break;
            }
        }
        if (alreadyHeld || (heldCounts[busIndex] >= MAX_SERVOS_PER_BUS)) {
            continue;
        }

        ServoBusDriver *bus = getBusByIndex(busIndex);
        if ((bus == nullptr) || !bus->isOnline(id)) {
            continue;
        }

        const int32_t absPos = bus->getAbsolutePosition(id);
        if (!isServoAbsAtLimit(absPos)) {
            attempted[busIndex][id] = true;
            continue;
        }

        const int16_t rawSingleTurn = normalizeServoRawSingleTurn(bus->getRawPosition(id));
        bus->setTarget(id, rawSingleTurn, SERVO_TARGET_SPEED_DEFAULT, SERVO_TARGET_ACC_DEFAULT);
        attempted[busIndex][id] = true;
        heldIds[busIndex][heldCounts[busIndex]++] = id;
        busWritePending[busIndex] = true;
    }

    for (uint8_t busIndex = 0U; busIndex < NUM_BUSES; busIndex++) {
        if (busWritePending[busIndex]) {
            g_servoBuses[busIndex].syncWriteAll();
        }
    }
#else
    (void)bootRecoverDeadlineMs;
#endif
}

static void publishServoFeedback(TaskSharedData_t *sharedData)
{
    if (sharedData == nullptr) {
        return;
    }

    ServoAngleData_t servoData = {};
    ServoAngleData_t servoRawData = {};
    ServoTelemetryData_t telemetryData = {};
    servoData.timestamp = HAL_GetTick();
    servoRawData.timestamp = servoData.timestamp;
    telemetryData.timestamp = servoData.timestamp;

    for (uint8_t busIndex = 0U; busIndex < NUM_BUSES; busIndex++) {
        const uint8_t onlineCount = g_servoBuses[busIndex].onlineCountForMappedChannels();
        servoComm.busOnlineCount[busIndex] = onlineCount;
        if ((busIndex + 1U) <= SERVO_DEBUG_BUS_COUNT) {
            servo1[busIndex + 1U].onlineCount = onlineCount;
        }
    }

    for (uint8_t ch = 0U; ch < SERVO_TOTAL_NUM; ch++) {
        if (!isMotorChannelValid(ch)) {
            continue;
        }

        const uint8_t busIndex = motorMap[ch].busIndex;
        const uint8_t id = motorMap[ch].servoID;
        ServoBusDriver *bus = getBusByIndex(busIndex);
        if ((bus != nullptr) && bus->isOnline(id)) {
            const ServoFeedbackInternal &fb = bus->getFeedback(id);
            servoData.servoAngles[ch] = bus->getAbsolutePosition(id);
            servoData.servoRawPositions[ch] = bus->getRawPosition(id);
            servoRawData.servoRawPositions[ch] = bus->getRawPosition(id);
            servoData.onlineStatus[ch] = 1U;
            servoRawData.onlineStatus[ch] = 1U;
            telemetryData.speed[ch] = fb.speed;
            telemetryData.load[ch] = fb.load;
            telemetryData.voltage[ch] = fb.voltage;
            telemetryData.temperature[ch] = fb.temperature;
            telemetryData.onlineStatus[ch] = 1U;
        }
    }

    if (sharedData->servoFeedbackQueue != nullptr) {
        xQueueOverwrite(sharedData->servoFeedbackQueue, &servoData);
    }
    if (sharedData->servoAngleQueue != nullptr) {
        xQueueOverwrite(sharedData->servoAngleQueue, &servoData);
    }
    if (sharedData->servoRawQueue != nullptr) {
        xQueueOverwrite(sharedData->servoRawQueue, &servoRawData);
    }
    if (sharedData->servoTelemetryQueue != nullptr) {
        xQueueOverwrite(sharedData->servoTelemetryQueue, &telemetryData);
    }
    if (sharedData->servoTelemetrySnapshotQueue != nullptr) {
        xQueueOverwrite(sharedData->servoTelemetrySnapshotQueue, &telemetryData);
    }
}

static void processHostMultiTurnRestore(TaskSharedData_t *sharedData, uint32_t *lastToken)
{
    if ((sharedData == nullptr) || (lastToken == nullptr)) {
        return;
    }

    const uint32_t token = sharedData->restore_multi_turn_request_token;
    if (token == *lastToken) {
        return;
    }
    *lastToken = token;
    servoComm.restoreRequestCount++;

    uint32_t motorMask = 0U;
    int32_t motorAbs[SERVO_TOTAL_NUM] = {};
    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    const bool locked = (lock != nullptr) && (xSemaphoreTake(lock, pdMS_TO_TICKS(5)) == pdTRUE);
    motorMask = sharedData->restore_multi_turn_mask;
    std::memcpy(motorAbs, sharedData->restore_multi_turn_abs, sizeof(motorAbs));
    if (locked) {
        xSemaphoreGive(lock);
    }

    uint32_t restoredMask = 0U;
    uint32_t rejectedMask = 0U;
    (void)restoreServoMultiTurnFromHost(sharedData,
                                        motorMask,
                                        motorAbs,
                                        256U,
                                        &restoredMask,
                                        &rejectedMask);
    sharedData->restore_multi_turn_restored_mask = restoredMask;
    sharedData->restore_multi_turn_rejected_mask = rejectedMask;
    sharedData->restore_multi_turn_result_token = token;
}

ServoComm::ServoComm()
    : shared(nullptr),
      taskRunning(0U),
      selectedBus(0U),
      selectedId(1U),
      online(0U),
      lastStatus(0U),
      lastError(0U),
      loopCount(0U),
      txFrameCount(0U),
      rxFrameCount(0U),
      readOkCount(0U),
      timeoutCount(0U),
      checksumErrorCount(0U),
      invalidFrameCount(0U),
      writeBatchCount(0U),
      emergencyHoldCount(0U),
      restoreRequestCount(0U),
      lastPollDurationMs(0U),
      absolutePosition(0),
      rawPosition(0),
      speed(0),
      load(0),
      voltage(0U),
      temperature(0U),
      lastTxLen(0U),
      lastRxLen(0U)
{
    std::memset(lastTxFrame, 0, sizeof(lastTxFrame));
    std::memset(lastRxFrame, 0, sizeof(lastRxFrame));
    std::memset(busOnlineCount, 0, sizeof(busOnlineCount));
}

void ServoComm::recordTx(const uint8_t *data, uint16_t len)
{
    const uint16_t copyLen = (len > sizeof(lastTxFrame)) ? sizeof(lastTxFrame) : len;
    if ((data != nullptr) && (copyLen > 0U)) {
        std::memcpy(lastTxFrame, data, copyLen);
    }
    lastTxLen = static_cast<uint8_t>(copyLen);
}

void ServoComm::recordRx(const uint8_t *data, uint16_t len)
{
    const uint16_t copyLen = (len > sizeof(lastRxFrame)) ? sizeof(lastRxFrame) : len;
    if ((data != nullptr) && (copyLen > 0U)) {
        std::memcpy(lastRxFrame, data, copyLen);
    }
    lastRxLen = static_cast<uint8_t>(copyLen);
}

void ServoComm::recordFeedback(uint8_t busIndex,
                               uint8_t id,
                               int16_t raw,
                               int32_t absolute,
                               int16_t currentSpeed,
                               int16_t currentLoad,
                               uint8_t currentVoltage,
                               uint8_t currentTemperature)
{
    ServoDebugId *item = getServoDebugId(busIndex, id);
    if (item != nullptr) {
        item->online = 1U;
        item->lastStatus = lastStatus;
        item->lastError = lastError;
        item->rawPosition = raw;
        item->absolutePosition = absolute;
        item->speed = currentSpeed;
        item->load = currentLoad;
        item->voltage = currentVoltage;
        item->temperature = currentTemperature;
        item->lastUpdate = HAL_GetTick();
        if ((item->readOkCount == 0U) && (item->writeOkCount == 0U)) {
            item->targetPosition = clampServoAbsLocal(absolute);
        }
        item->readOkCount++;
    }

    if ((busIndex == selectedBus) && (id == selectedId)) {
        online = 1U;
        rawPosition = raw;
        absolutePosition = absolute;
        speed = currentSpeed;
        load = currentLoad;
        voltage = currentVoltage;
        temperature = currentTemperature;
    }
}

void ServoComm::run(void *parameter)
{
    shared = static_cast<TaskSharedData_t *>(parameter);
    if (shared == nullptr) {
        vTaskDelete(nullptr);
        return;
    }

    beginServoBuses();
    configureMappedServosForMultiTurn();
    taskRunning = 1U;

    TickType_t lastWakeTime = xTaskGetTickCount();
    uint8_t readBusPhase = 0U;
    uint32_t lastEmergencyStopToken = shared->servo_emergency_stop_token;
    uint32_t lastRestoreToken = shared->restore_multi_turn_request_token;
    const uint32_t bootRecoverDeadlineMs = HAL_GetTick() + kBootRecoverLimitToRawWindowMs;

    for (;;) {
        const uint32_t loopStart = HAL_GetTick();
        loopCount++;

        const uint32_t emergencyStopToken = shared->servo_emergency_stop_token;
        if (emergencyStopToken != lastEmergencyStopToken) {
            lastEmergencyStopToken = emergencyStopToken;
            emergencyHoldMappedServosAtFeedback(shared);
        }

        processHostMultiTurnRestore(shared, &lastRestoreToken);

        ServoTargetBatch_t batch = {};
        while ((shared->servoTargetQueue != nullptr) &&
               (xQueueReceive(shared->servoTargetQueue, &batch, 0) == pdTRUE)) {
            const uint8_t owner = shared->servo_target_owner;
            if ((owner != SERVO_TARGET_OWNER_NONE) && (batch.source == owner)) {
                applyServoTargetBatch(batch);
            }
        }
        processDebugServoCommands();

        uint8_t readIds[NUM_BUSES][MAX_SERVOS_PER_BUS] = {};
        uint8_t readCounts[NUM_BUSES] = {};
        for (uint8_t ch = 0U; ch < SERVO_TOTAL_NUM; ch++) {
            if (isMotorChannelValid(ch)) {
                (void)addReadTarget(readIds, readCounts, motorMap[ch].busIndex, motorMap[ch].servoID);
            }
        }

        const uint8_t readStartBus = (readBusPhase == 0U) ? 0U : 2U;
        for (uint8_t offset = 0U; offset < 2U; offset++) {
            const uint8_t busIndex = static_cast<uint8_t>(readStartBus + offset);
            if ((busIndex >= NUM_BUSES) || (readCounts[busIndex] == 0U)) {
                continue;
            }
            (void)g_servoBuses[busIndex].syncReadPositions(readIds[busIndex], readCounts[busIndex]);
        }
        readBusPhase ^= 1U;

        bootRecoverLimitServosToRaw(bootRecoverDeadlineMs);
        publishServoFeedback(shared);
        lastPollDurationMs = HAL_GetTick() - loopStart;
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(SERVO_TASK_PERIOD_MS));
    }
}

extern "C" void ServoCommTask_Run(void *parameter)
{
    servoComm.run(parameter);
}
