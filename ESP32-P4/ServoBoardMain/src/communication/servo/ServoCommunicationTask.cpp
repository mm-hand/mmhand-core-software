#include "ServoCommunicationTask.h"

#include <string.h>

#include "../../hardware/HardwareMap.h"
#if MULTITURN_NVS_PERSISTENCE_ENABLE
#include "../../persistence/MultiTurnNvsStore.h"
#endif
#include "../libraries/FTServo_Arduino/src/SMS_STS.h"

#ifndef FTSERVO_ARDUINO_EXTERNAL_BUILD
#include "../libraries/FTServo_Arduino/src/SCS.cpp"
#include "../libraries/FTServo_Arduino/src/SCSerial.cpp"
#include "../libraries/FTServo_Arduino/src/SMS_STS.cpp"
#endif

#ifndef SERVO_CONFIGURE_MULTI_TURN_ON_BOOT
#define SERVO_CONFIGURE_MULTI_TURN_ON_BOOT 1
#endif

#ifndef SERVO_BOOT_RECOVER_LIMIT_TO_RAW
#define SERVO_BOOT_RECOVER_LIMIT_TO_RAW 0
#endif

#define SERVO_MULTI_TURN_CONFIG_DELAY_MS 5
static const uint32_t kBootRecoverLimitToRawWindowMs = 8000;
#if MULTITURN_NVS_PERSISTENCE_ENABLE
// NVS 恢复会先短暂等待舵机上线，只恢复当前 raw 与快照 raw 足够接近的通道。
static const uint32_t kNvsRestoreWaitWindowMs = 3000;
// 上位机每 500ms 发送 heartbeat；连续丢两次就认为上位机离线并立即保存本地快照。
static const uint32_t kHostHeartbeatOfflineMs = 1000;
// 上位机离线后仍低频保存；接近跨圈边界时缩短间隔，降低断电时圈数过期风险。
static const uint32_t kNvsSaveMinIntervalMs = 500;
static const uint32_t kNvsOfflinePeriodicSaveMs = 2000;
static const uint16_t kNvsRestoreRawToleranceCounts = 96;
static const int16_t kNvsTurnEdgeRawMargin = 256;
static const int32_t kNvsMoveAbsThreshold = 64;
#endif
static const int32_t kServoAbsLimit = 30719;
static const int32_t kServoAbsLimitRecoverMargin = 16;

static bool isServoAbsAtLimit(int32_t value)
{
    return value >= (kServoAbsLimit - kServoAbsLimitRecoverMargin) ||
           value <= (-kServoAbsLimit + kServoAbsLimitRecoverMargin);
}

static int16_t clampServoAbsLocal(int32_t value)
{
    if (value > kServoAbsLimit) return (int16_t)kServoAbsLimit;
    if (value < -kServoAbsLimit) return (int16_t)-kServoAbsLimit;
    return (int16_t)value;
}

static int16_t normalizeServoRawSingleTurn(int32_t value)
{
    int32_t raw = value % 4096;
    if (raw < 0) {
        raw += 4096;
    }
    return (int16_t)raw;
}

static uint16_t singleTurnDistanceCounts(int16_t a, int16_t b)
{
    int32_t diff = abs((int32_t)normalizeServoRawSingleTurn(a) -
                       (int32_t)normalizeServoRawSingleTurn(b)) % 4096;
    if (diff > 2048) {
        diff = 4096 - diff;
    }
    return (uint16_t)diff;
}

// Per-servo feedback cache used inside the servo communication layer.
struct ServoFeedbackInternal {
    int16_t rawPosition;
    int16_t speed;
    int16_t load;
    uint8_t voltage;
    uint8_t temperature;
    // absolutePosition is the servo's current power-session coordinate. It may
    // reset to single-turn raw after servo power loss.
    int32_t absolutePosition;
    // logicalOffset maps the current power-session coordinate to firmware logical
    // multi-turn ABS restored from host/NVS: logical = hardware + offset.
    int32_t logicalOffset;
    int16_t turnCount;
    int16_t lastRawPosition;
    bool initialized;
    bool absoluteReliable;
    bool online;
    uint32_t lastUpdate;
};

// Private wrapper for one Feetech UART servo bus.
class ServoBusDriver {
public:
    ServoBusDriver() : _serial(NULL), _writeCount(0), _syncReadTimeoutMs(SERVO_SYNC_READ_TIMEOUT_MS_DEFAULT)
    {
        memset(_feedback, 0, sizeof(_feedback));
        memset(_torqueDisabled, 0, sizeof(_torqueDisabled));
    }

    void begin(uint8_t busIndex, int rxPin, int txPin, uint32_t baud)
    {
        HardwareSerial* serial = NULL;
        switch (busIndex) {
            case 0: serial = &Serial1; break;
            case 1: serial = &Serial2; break;
            case 2: serial = &Serial3; break;
            case 3: serial = &Serial4; break;
            default: return;
        }

        _serial = serial;
        _serial->begin(baud, SERIAL_8N1, rxPin, txPin);
        _sms.pSerial = _serial;
    }

    bool configureMultiTurnMode(uint8_t id)
    {
        if (!_serial || id > MAX_SERVO_ID) {
            return false;
        }

        uint8_t zeroLimits[4] = {0, 0, 0, 0};
        bool ok = true;
        ok = (_sms.unLockEprom(id) >= 0) && ok;
        delay(SERVO_MULTI_TURN_CONFIG_DELAY_MS);
        ok = (_sms.genWrite(id, SMS_STS_MIN_ANGLE_LIMIT_L, zeroLimits, sizeof(zeroLimits)) >= 0) && ok;
        delay(SERVO_MULTI_TURN_CONFIG_DELAY_MS);
        ok = (_sms.LockEprom(id) >= 0) && ok;
        delay(SERVO_MULTI_TURN_CONFIG_DELAY_MS);
        return ok;
    }

    void setTarget(uint8_t id, int16_t position, uint16_t speed, uint8_t acc)
    {
        // 上位机/状态机传进来的是逻辑多圈 ABS；真正写舵机前要换算回本次上电的硬件坐标。
        queueTarget(id, logicalToHardwareTarget(id, position), speed, acc);
    }

    void setHardwareTarget(uint8_t id, int16_t position, uint16_t speed, uint8_t acc)
    {
        // 少数保护路径需要直接写当前上电会话的 raw/硬件坐标，不能再扣 logicalOffset。
        queueTarget(id, clampServoAbsLocal(position), speed, acc);
    }

    void queueTarget(uint8_t id, int16_t position, uint16_t speed, uint8_t acc)
    {
        if (id > MAX_SERVO_ID || _writeCount >= MAX_SERVOS_PER_BUS) {
            return;
        }
        if (_torqueDisabled[id]) {
            _sms.EnableTorque(id, 1);
            _torqueDisabled[id] = false;
        }

        _writeIDs[_writeCount] = id;
        _writePos[_writeCount] = position;
        _writeSpd[_writeCount] = speed;
        _writeAcc[_writeCount] = acc;
        _writeCount++;
    }

    void syncWriteAll()
    {
        if (_writeCount == 0 || !_serial) {
            return;
        }
        _sms.SyncWritePosEx(_writeIDs, _writeCount, _writePos, _writeSpd, _writeAcc);
        _writeCount = 0;
    }

    int syncReadPositions(const uint8_t* ids, uint8_t count)
    {
        if (!_serial || !ids || count == 0) {
            return 0;
        }

        const uint8_t addrPresentPosition = 56;
        const uint8_t feedbackLen = 8;
        const uint16_t timeoutMs = (_syncReadTimeoutMs == 0) ? 1 : _syncReadTimeoutMs;
        int successCount = 0;

        _sms.syncReadBegin(count, feedbackLen, timeoutMs);
        const int txResult = _sms.syncReadPacketTx((uint8_t*)ids, count, addrPresentPosition, feedbackLen);
        if (txResult <= 0) {
            for (uint8_t i = 0; i < count; i++) {
                if (ids[i] <= MAX_SERVO_ID) {
                    _feedback[ids[i]].online = false;
                }
            }
            _sms.syncReadEnd();
            return 0;
        }

        for (uint8_t i = 0; i < count; i++) {
            const uint8_t id = ids[i];
            uint8_t rxBuf[feedbackLen];
            const int rxLen = _sms.syncReadPacketRx(id, rxBuf);
            if (id > MAX_SERVO_ID) {
                continue;
            }
            if (rxLen == feedbackLen) {
                const int16_t rawPos = decodeSigned(rxBuf[0], rxBuf[1]);
                _updateMultiTurnPosition(id, rawPos);
                _feedback[id].speed = decodeSigned(rxBuf[2], rxBuf[3]);
                _feedback[id].load = decodeServoLoad(rxBuf[4], rxBuf[5]);
                _feedback[id].voltage = rxBuf[6];
                _feedback[id].temperature = rxBuf[7];
                _feedback[id].online = true;
                _feedback[id].lastUpdate = millis();
                successCount++;
            } else {
                _feedback[id].online = false;
            }
        }

        _sms.syncReadEnd();
        return successCount;
    }

    int16_t getRawPosition(uint8_t id) const
    {
        if (id > MAX_SERVO_ID) return -1;
        return _feedback[id].rawPosition;
    }

    int32_t getAbsolutePosition(uint8_t id) const
    {
        if (id > MAX_SERVO_ID) return 0;
        const ServoFeedbackInternal& fb = _feedback[id];
        return fb.absolutePosition + fb.logicalOffset;
    }

    const ServoFeedbackInternal& getFeedback(uint8_t id) const
    {
        static ServoFeedbackInternal dummy;
        if (id > MAX_SERVO_ID) return dummy;
        return _feedback[id];
    }

    bool isOnline(uint8_t id) const
    {
        return id <= MAX_SERVO_ID && _feedback[id].online;
    }

    bool isAbsoluteReliable(uint8_t id) const
    {
        return id <= MAX_SERVO_ID && _feedback[id].absoluteReliable;
    }

    bool setTorqueEnabled(uint8_t id, bool enabled)
    {
        if (!_serial || id > MAX_SERVO_ID) {
            return false;
        }
        const bool ok = (_sms.EnableTorque(id, enabled ? 1 : 0) >= 0);
        if (ok) {
            _torqueDisabled[id] = !enabled;
        }
        return ok;
    }

    bool restoreAbsolutePosition(uint8_t id, int32_t absPosition, uint16_t rawToleranceCounts)
    {
        if (id > MAX_SERVO_ID) {
            return false;
        }
        ServoFeedbackInternal& fb = _feedback[id];
        if (!fb.online) {
            return false;
        }

        const int16_t currentRaw = normalizeServoRawSingleTurn(fb.rawPosition);
        const int16_t restoredRaw = normalizeServoRawSingleTurn(absPosition);
        if (singleTurnDistanceCounts(currentRaw, restoredRaw) > rawToleranceCounts) {
            return false;
        }

        const int32_t restoredLogicalAbs = clampServoAbsLocal(absPosition);
        const int32_t hardwareAbs = fb.initialized
            ? fb.absolutePosition
            : (int32_t)currentRaw;
        fb.rawPosition = currentRaw;
        fb.lastRawPosition = currentRaw;
        fb.absolutePosition = hardwareAbs;
        fb.logicalOffset = restoredLogicalAbs - hardwareAbs;
        fb.turnCount = (int16_t)((restoredLogicalAbs - currentRaw) / 4096);
        fb.initialized = true;
        fb.absoluteReliable = true;
        fb.lastUpdate = millis();
        Serial.printf("[SERVO RESTORE OFFSET] id=%u logical=%ld hardware=%ld offset=%ld raw=%d\r\n",
                      (unsigned)id,
                      (long)restoredLogicalAbs,
                      (long)hardwareAbs,
                      (long)fb.logicalOffset,
                      (int)currentRaw);
        return true;
    }

private:
    int16_t logicalToHardwareTarget(uint8_t id, int32_t logicalPosition) const
    {
        if (id > MAX_SERVO_ID) {
            return 0;
        }
        const ServoFeedbackInternal& fb = _feedback[id];
        return clampServoAbsLocal(logicalPosition - fb.logicalOffset);
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

    static int16_t decodeSigned(uint8_t lo, uint8_t hi)
    {
        const uint16_t raw = ((uint16_t)hi << 8) | lo;
        return (raw & (1 << 15)) ? -(int16_t)(raw & ~(1 << 15)) : (int16_t)raw;
    }

    static int16_t decodeServoLoad(uint8_t lo, uint8_t hi)
    {
        const uint16_t raw = ((uint16_t)hi << 8) | lo;
        const int16_t magnitude = (int16_t)(raw & 0x03FF);
        return (raw & (1 << 10)) ? (int16_t)-magnitude : magnitude;
    }

    void _updateMultiTurnPosition(uint8_t id, int16_t newRawPos)
    {
        ServoFeedbackInternal& fb = _feedback[id];
        int32_t absPos = 0;
        int16_t rawSingleTurn = 0;

        const bool servoProvidedMultiTurnAbs = (newRawPos < 0 || newRawPos >= 4096);
        if (servoProvidedMultiTurnAbs) {
            absPos = clampServoAbsLocal((int32_t)newRawPos);
            rawSingleTurn = normalizeServoRawSingleTurn(absPos);
        } else {
            rawSingleTurn = normalizeServoRawSingleTurn(newRawPos);
            const int32_t anchor = fb.initialized
                ? fb.absolutePosition
                : (int32_t)rawSingleTurn;
            absPos = expandSingleTurnRawNearAnchor(rawSingleTurn, anchor);
        }

        fb.rawPosition = rawSingleTurn;
        fb.lastRawPosition = rawSingleTurn;
        fb.absolutePosition = absPos;
        const int32_t logicalAbs = absPos + fb.logicalOffset;
        fb.turnCount = (int16_t)((logicalAbs - rawSingleTurn) / 4096);
        fb.initialized = true;
    }

    SMS_STS _sms;
    HardwareSerial* _serial;
    uint8_t _writeIDs[MAX_SERVOS_PER_BUS];
    int16_t _writePos[MAX_SERVOS_PER_BUS];
    uint16_t _writeSpd[MAX_SERVOS_PER_BUS];
    uint8_t _writeAcc[MAX_SERVOS_PER_BUS];
    uint8_t _writeCount;
    ServoFeedbackInternal _feedback[MAX_SERVO_ID + 1];
    bool _torqueDisabled[MAX_SERVO_ID + 1];
    uint16_t _syncReadTimeoutMs;
};

static ServoBusDriver g_servoBuses[NUM_BUSES];

static ServoBusDriver* getBusByIndex(uint8_t busIndex)
{
    if (busIndex >= NUM_BUSES) {
        return NULL;
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
    for (uint8_t i = 0; i < busCounts[bus]; i++) {
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

static bool restoreServoMultiTurnFromHost(TaskSharedData_t* sharedData,
                                          uint32_t motorMask,
                                          const int32_t* motorAbs,
                                          uint16_t rawToleranceCounts,
                                          uint32_t* restoredMask,
                                          uint32_t* rejectedMask)
{
    uint32_t restored = 0;
    uint32_t rejected = 0;
    if (!sharedData || !motorAbs) {
        if (restoredMask) *restoredMask = 0;
        if (rejectedMask) *rejectedMask = motorMask;
        return false;
    }

    const uint32_t requestMask = motorMask & validMotorChannelMask();
    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        const uint32_t bit = (1UL << ch);
        if ((requestMask & bit) == 0) {
            continue;
        }

        const uint8_t busIndex = motorMap[ch].busIndex;
        const uint8_t id = motorMap[ch].servoID;
        ServoBusDriver* bus = getBusByIndex(busIndex);
        if (!bus || id > MAX_SERVO_ID ||
            !bus->restoreAbsolutePosition(id, motorAbs[ch], rawToleranceCounts)) {
            rejected |= bit;
            continue;
        }
        restored |= bit;
    }

    if (restored != 0) {
        sharedData->multi_turn_unknown_bitmap &= ~restored;
    }

    if (restoredMask) *restoredMask = restored;
    if (rejectedMask) *rejectedMask = rejected;
    Serial.printf("[SERVO RESTORE] request=0x%08lX restored=0x%08lX rejected=0x%08lX unknown=0x%08lX tol=%u\r\n",
                  (unsigned long)requestMask,
                  (unsigned long)restored,
                  (unsigned long)rejected,
                  (unsigned long)sharedData->multi_turn_unknown_bitmap,
                  (unsigned)rawToleranceCounts);
    return (requestMask != 0) && ((requestMask & ~restored) == 0);
}

static void beginServoBuses()
{
    static bool initialized = false;
    if (initialized) {
        return;
    }

    g_servoBuses[0].begin(0, 21, 20, 1000000);
    g_servoBuses[1].begin(1, 23, 22, 1000000);
    g_servoBuses[2].begin(2, 27, 26, 1000000);
    g_servoBuses[3].begin(3, 33, 32, 1000000);
    initialized = true;
}

static void configureMappedServosForMultiTurn()
{
#if SERVO_CONFIGURE_MULTI_TURN_ON_BOOT
    static bool configuredOnce = false;
    if (configuredOnce) {
        return;
    }

    uint8_t configuredIds[NUM_BUSES][MAX_SERVOS_PER_BUS] = {0};
    uint8_t configuredCounts[NUM_BUSES] = {0};

    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        const uint8_t busIndex = motorMap[ch].busIndex;
        const uint8_t id = motorMap[ch].servoID;
        if (busIndex >= NUM_BUSES || id > MAX_SERVO_ID) {
            continue;
        }

        bool alreadyConfigured = false;
        for (uint8_t i = 0; i < configuredCounts[busIndex]; i++) {
            if (configuredIds[busIndex][i] == id) {
                alreadyConfigured = true;
                break;
            }
        }
        if (alreadyConfigured || configuredCounts[busIndex] >= MAX_SERVOS_PER_BUS) {
            continue;
        }

        ServoBusDriver* bus = getBusByIndex(busIndex);
        if (bus && bus->configureMultiTurnMode(id)) {
            configuredIds[busIndex][configuredCounts[busIndex]++] = id;
        }
    }
    configuredOnce = true;
#endif
}

static void applyServoTargetBatch(const ServoTargetBatch_t& batch)
{
    bool busWritePending[NUM_BUSES] = {false};

    for (uint8_t i = 0; i < batch.count && i < SERVO_TARGET_BATCH_MAX; i++) {
        const ServoTargetCommand_t& cmd = batch.commands[i];
        ServoBusDriver* bus = getBusByIndex(cmd.busIndex);
        if (!bus) {
            continue;
        }
        bus->setTarget(cmd.servoId, cmd.position, cmd.speed, cmd.acc);
        busWritePending[cmd.busIndex] = true;
    }

    for (uint8_t busIndex = 0; busIndex < NUM_BUSES; busIndex++) {
        if (busWritePending[busIndex]) {
            g_servoBuses[busIndex].syncWriteAll();
        }
    }
}

static void emergencyHoldMappedServosAtFeedback(TaskSharedData_t* sharedData)
{
    if (sharedData && sharedData->servoTargetQueue) {
        xQueueReset(sharedData->servoTargetQueue);
    }

    uint8_t heldIds[NUM_BUSES][MAX_SERVOS_PER_BUS] = {0};
    uint8_t heldCounts[NUM_BUSES] = {0};
    bool busWritePending[NUM_BUSES] = {false};
    uint32_t skippedUnknownMask = 0;
    uint32_t torqueDisabledMask = 0;
    const uint32_t unknownMask = sharedData ? sharedData->multi_turn_unknown_bitmap : 0;

    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        const uint32_t bit = (1UL << ch);
        const uint8_t busIndex = motorMap[ch].busIndex;
        const uint8_t id = motorMap[ch].servoID;
        if (busIndex >= NUM_BUSES || id > MAX_SERVO_ID) {
            continue;
        }

        bool alreadyHeld = false;
        for (uint8_t i = 0; i < heldCounts[busIndex]; i++) {
            if (heldIds[busIndex][i] == id) {
                alreadyHeld = true;
                break;
            }
        }
        if (alreadyHeld || heldCounts[busIndex] >= MAX_SERVOS_PER_BUS) {
            continue;
        }

        ServoBusDriver* bus = getBusByIndex(busIndex);
        if (!bus || !bus->isOnline(id)) {
            continue;
        }

        if ((unknownMask & bit) != 0) {
            skippedUnknownMask |= bit;
            if (bus->setTorqueEnabled(id, false)) {
                torqueDisabledMask |= bit;
            }
            continue;
        }

        const int32_t holdPos32 = bus->getAbsolutePosition(id);
        const int16_t rawSingleTurn = normalizeServoRawSingleTurn(bus->getRawPosition(id));
        const int16_t holdPos = isServoAbsAtLimit(holdPos32)
            ? rawSingleTurn
            : clampServoAbsLocal(holdPos32);
        if (isServoAbsAtLimit(holdPos32)) {
            bus->setHardwareTarget(id, holdPos, SERVO_TARGET_SPEED_DEFAULT, SERVO_TARGET_ACC_DEFAULT);
        } else {
            bus->setTarget(id, holdPos, SERVO_TARGET_SPEED_DEFAULT, SERVO_TARGET_ACC_DEFAULT);
        }
        heldIds[busIndex][heldCounts[busIndex]++] = id;
        busWritePending[busIndex] = true;
    }

    if (skippedUnknownMask != 0) {
        Serial.printf("[SERVO HOLD] unknown motors torque-off skip-position mask=0x%08lX disabled=0x%08lX\r\n",
                      (unsigned long)skippedUnknownMask,
                      (unsigned long)torqueDisabledMask);
    }

    for (uint8_t busIndex = 0; busIndex < NUM_BUSES; busIndex++) {
        if (busWritePending[busIndex]) {
            g_servoBuses[busIndex].syncWriteAll();
        }
    }
}

static void bootRecoverLimitServosToRaw(uint32_t bootRecoverDeadlineMs)
{
#if SERVO_BOOT_RECOVER_LIMIT_TO_RAW
    static bool attempted[NUM_BUSES][MAX_SERVO_ID + 1] = {false};

    const uint32_t nowMs = millis();
    if ((int32_t)(nowMs - bootRecoverDeadlineMs) > 0) {
        return;
    }

    uint8_t heldIds[NUM_BUSES][MAX_SERVOS_PER_BUS] = {0};
    uint8_t heldCounts[NUM_BUSES] = {0};
    bool busWritePending[NUM_BUSES] = {false};

    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        const uint8_t busIndex = motorMap[ch].busIndex;
        const uint8_t id = motorMap[ch].servoID;
        if (busIndex >= NUM_BUSES || id > MAX_SERVO_ID) {
            continue;
        }
        if (attempted[busIndex][id]) {
            continue;
        }

        bool alreadyHeld = false;
        for (uint8_t i = 0; i < heldCounts[busIndex]; i++) {
            if (heldIds[busIndex][i] == id) {
                alreadyHeld = true;
                break;
            }
        }
        if (alreadyHeld || heldCounts[busIndex] >= MAX_SERVOS_PER_BUS) {
            continue;
        }

        ServoBusDriver* bus = getBusByIndex(busIndex);
        if (!bus || !bus->isOnline(id)) {
            continue;
        }

        const int32_t absPos = bus->getAbsolutePosition(id);
        if (!isServoAbsAtLimit(absPos)) {
            attempted[busIndex][id] = true;
            continue;
        }

        const int16_t rawSingleTurn = normalizeServoRawSingleTurn(bus->getRawPosition(id));
        bus->setHardwareTarget(id, rawSingleTurn, SERVO_TARGET_SPEED_DEFAULT, SERVO_TARGET_ACC_DEFAULT);
        attempted[busIndex][id] = true;
        heldIds[busIndex][heldCounts[busIndex]++] = id;
        busWritePending[busIndex] = true;
        Serial.printf("[SERVO BOOT RECOVER] bus=%u id=%u abs=%ld raw=%d\r\n",
                      (unsigned)busIndex,
                      (unsigned)id,
                      (long)absPos,
                      (int)rawSingleTurn);
    }

    for (uint8_t busIndex = 0; busIndex < NUM_BUSES; busIndex++) {
        if (busWritePending[busIndex]) {
            g_servoBuses[busIndex].syncWriteAll();
        }
    }
#else
    (void)bootRecoverDeadlineMs;
#endif
}

static void publishServoFeedback(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return;
    }

    ServoAngleData_t servoData;
    ServoAngleData_t servoRawData;
    ServoTelemetryData_t telemetryData;
    memset(&servoData, 0, sizeof(servoData));
    memset(&servoRawData, 0, sizeof(servoRawData));
    memset(&telemetryData, 0, sizeof(telemetryData));

    servoData.timestamp = millis();
    servoRawData.timestamp = servoData.timestamp;
    telemetryData.timestamp = servoData.timestamp;

    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        const uint8_t busIndex = motorMap[ch].busIndex;
        const uint8_t id = motorMap[ch].servoID;
        ServoBusDriver* bus = getBusByIndex(busIndex);
        if (bus && bus->isOnline(id)) {
            const ServoFeedbackInternal& fb = bus->getFeedback(id);
            servoData.servoAngles[ch] = bus->getAbsolutePosition(id);
            servoData.servoRawPositions[ch] = bus->getRawPosition(id);
            servoRawData.servoRawPositions[ch] = bus->getRawPosition(id);
            servoData.onlineStatus[ch] = 1;
            servoRawData.onlineStatus[ch] = 1;
            telemetryData.speed[ch] = fb.speed;
            telemetryData.load[ch] = fb.load;
            telemetryData.voltage[ch] = fb.voltage;
            telemetryData.temperature[ch] = fb.temperature;
            telemetryData.onlineStatus[ch] = 1;
            if (!sharedData->multiturn_nvs_enabled) {
                const uint32_t bit = (1UL << ch);
                if ((sharedData->multi_turn_unknown_bitmap & bit) != 0) {
                    sharedData->multi_turn_unknown_bitmap &= ~bit;
                    Serial.printf("[SERVO RESTORE] M%02u NVS off, using current raw as turn0 abs=%ld raw=%d unknown=0x%08lX\r\n",
                                  (unsigned)ch,
                                  (long)servoData.servoAngles[ch],
                                  (int)servoData.servoRawPositions[ch],
                                  (unsigned long)sharedData->multi_turn_unknown_bitmap);
                }
                continue;
            }
            if (bus->isAbsoluteReliable(id)) {
                const uint32_t bit = (1UL << ch);
                if ((sharedData->multi_turn_unknown_bitmap & bit) != 0) {
                    sharedData->multi_turn_unknown_bitmap &= ~bit;
                    Serial.printf("[SERVO RESTORE] M%02u absolute feedback trusted abs=%ld raw=%d unknown=0x%08lX\r\n",
                                  (unsigned)ch,
                                  (long)servoData.servoAngles[ch],
                                  (int)servoData.servoRawPositions[ch],
                                  (unsigned long)sharedData->multi_turn_unknown_bitmap);
                }
            }
        }
    }

    if (sharedData->servoFeedbackQueue) {
        xQueueOverwrite(sharedData->servoFeedbackQueue, &servoData);
    }
    if (sharedData->servoAngleQueue) {
        xQueueOverwrite(sharedData->servoAngleQueue, &servoData);
    }
    if (sharedData->servoRawQueue) {
        xQueueOverwrite(sharedData->servoRawQueue, &servoRawData);
    }
    if (sharedData->servoTelemetryQueue) {
        xQueueOverwrite(sharedData->servoTelemetryQueue, &telemetryData);
    }
    if (sharedData->servoTelemetrySnapshotQueue) {
        xQueueOverwrite(sharedData->servoTelemetrySnapshotQueue, &telemetryData);
    }
}

static void processHostMultiTurnRestore(TaskSharedData_t* sharedData, uint32_t* lastToken)
{
    if (!sharedData || !lastToken) {
        return;
    }
    const uint32_t token = sharedData->restore_multi_turn_request_token;
    if (token == *lastToken) {
        return;
    }
    *lastToken = token;

    uint32_t motorMask = 0;
    int32_t motorAbs[SERVO_TOTAL_NUM] = {0};
    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(5)) != pdTRUE) {
        return;
    }
    motorMask = sharedData->restore_multi_turn_mask;
    memcpy(motorAbs, sharedData->restore_multi_turn_abs, sizeof(motorAbs));
    if (lock) {
        xSemaphoreGive(lock);
    }

    uint32_t restoredMask = 0;
    uint32_t rejectedMask = 0;
    restoreServoMultiTurnFromHost(sharedData,
                                  motorMask,
                                  motorAbs,
                                  256,
                                  &restoredMask,
                                  &rejectedMask);
    sharedData->restore_multi_turn_restored_mask = restoredMask;
    sharedData->restore_multi_turn_rejected_mask = rejectedMask;
    sharedData->restore_multi_turn_result_token = token;
#if MULTITURN_NVS_PERSISTENCE_ENABLE
    if (restoredMask != 0) {
        sharedData->multiturn_nvs_save_reason = MULTITURN_NVS_SAVE_REASON_HOST_RESTORE;
        sharedData->multiturn_nvs_save_request_token++;
    }
#endif
}

#if MULTITURN_NVS_PERSISTENCE_ENABLE
struct MultiTurnNvsRuntimeState {
    // RAM 中镜像上一份快照，用于避免重复写入，并检测离线运动/接近跨圈边界。
    bool initialized;
    bool hostWasOnline;
    uint32_t lastSaveMs;
    uint32_t lastValidMask;
    int32_t lastAbs[SERVO_TOTAL_NUM];
    int16_t lastRaw[SERVO_TOTAL_NUM];
    int16_t lastTurn[SERVO_TOTAL_NUM];
};

static bool isRawNearTurnEdge(int16_t raw)
{
    const int16_t normalized = normalizeServoRawSingleTurn(raw);
    return normalized <= kNvsTurnEdgeRawMargin ||
           normalized >= (4096 - kNvsTurnEdgeRawMargin);
}

static uint32_t collectOnlineMotorMask()
{
    uint32_t mask = 0;
    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        const uint8_t busIndex = motorMap[ch].busIndex;
        const uint8_t id = motorMap[ch].servoID;
        ServoBusDriver* bus = getBusByIndex(busIndex);
        if (bus && bus->isOnline(id)) {
            mask |= (1UL << ch);
        }
    }
    return mask & validMotorChannelMask();
}

static bool collectTrustedMultiTurnSnapshot(TaskSharedData_t* sharedData,
                                            int32_t* motorAbs,
                                            int16_t* motorRaw,
                                            int16_t* motorTurn,
                                            uint32_t* validMask)
{
    if (!sharedData || !motorAbs || !motorRaw || !motorTurn || !validMask) {
        return false;
    }

    uint32_t mask = 0;
    // 多圈未知的通道不能保存，否则会把不安全的开机状态固化成错误参考。
    const uint32_t allowedMask = validMotorChannelMask() & ~sharedData->multi_turn_unknown_bitmap;
    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        const uint32_t bit = (1UL << ch);
        if ((allowedMask & bit) == 0) {
            continue;
        }
        const uint8_t busIndex = motorMap[ch].busIndex;
        const uint8_t id = motorMap[ch].servoID;
        ServoBusDriver* bus = getBusByIndex(busIndex);
        if (!bus || !bus->isOnline(id)) {
            continue;
        }
        const int16_t raw = normalizeServoRawSingleTurn(bus->getRawPosition(id));
        const int32_t absPos = bus->getAbsolutePosition(id);
        motorAbs[ch] = absPos;
        motorRaw[ch] = raw;
        motorTurn[ch] = (int16_t)((absPos - raw) / 4096);
        mask |= bit;
    }
    *validMask = mask;
    return mask != 0;
}

static void seedNvsRuntimeStateFromSnapshot(MultiTurnNvsRuntimeState* state,
                                            const MultiTurnNvsSnapshot& snapshot)
{
    if (!state) {
        return;
    }
    memset(state, 0, sizeof(*state));
    state->initialized = true;
    state->hostWasOnline = true;
    state->lastValidMask = snapshot.validMask & validMotorChannelMask();
    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        state->lastRaw[ch] = snapshot.raw[ch];
        state->lastTurn[ch] = snapshot.turns[ch];
        state->lastAbs[ch] = multiTurnSnapshotAbsForChannel(snapshot, ch);
    }
}

static bool snapshotChanged(const MultiTurnNvsRuntimeState& state,
                            uint32_t validMask,
                            const int32_t* motorAbs,
                            const int16_t* motorRaw,
                            const int16_t* motorTurn,
                            bool* turnChanged,
                            bool* edgeChanged)
{
    bool changed = !state.initialized || state.lastValidMask != validMask;
    bool anyTurnChanged = false;
    bool anyEdgeChanged = false;
    const uint32_t compareMask = validMask & state.lastValidMask;

    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        const uint32_t bit = (1UL << ch);
        if ((compareMask & bit) == 0) {
            continue;
        }
        if (state.lastTurn[ch] != motorTurn[ch]) {
            changed = true;
            anyTurnChanged = true;
        }
        if (state.lastRaw[ch] != motorRaw[ch] ||
            state.lastAbs[ch] != motorAbs[ch]) {
            changed = true;
        }
        if (isRawNearTurnEdge(motorRaw[ch]) &&
            abs((int32_t)motorAbs[ch] - (int32_t)state.lastAbs[ch]) >= kNvsMoveAbsThreshold) {
            anyEdgeChanged = true;
        }
    }

    if (turnChanged) {
        *turnChanged = anyTurnChanged;
    }
    if (edgeChanged) {
        *edgeChanged = anyEdgeChanged;
    }
    return changed;
}

static bool saveCurrentMultiTurnSnapshot(TaskSharedData_t* sharedData,
                                         MultiTurnNvsRuntimeState* state,
                                         uint8_t reason,
                                         bool force)
{
    if (!sharedData || !state) {
        return false;
    }
    if (!sharedData->multiturn_nvs_enabled) {
        return false;
    }

    int32_t motorAbs[SERVO_TOTAL_NUM] = {0};
    int16_t motorRaw[SERVO_TOTAL_NUM] = {0};
    int16_t motorTurn[SERVO_TOTAL_NUM] = {0};
    uint32_t validMask = 0;
    if (!collectTrustedMultiTurnSnapshot(sharedData, motorAbs, motorRaw, motorTurn, &validMask)) {
        return false;
    }

    bool turnChanged = false;
    bool edgeChanged = false;
    const bool changed = snapshotChanged(*state,
                                         validMask,
                                         motorAbs,
                                         motorRaw,
                                         motorTurn,
                                         &turnChanged,
                                         &edgeChanged);
    const uint32_t nowMs = millis();
    if (!force && !changed) {
        return false;
    }
    if (!force && state->initialized &&
        (uint32_t)(nowMs - state->lastSaveMs) < kNvsSaveMinIntervalMs) {
        return false;
    }

    uint32_t sequence = 0;
    // 所有 flash 写入集中在这里；其它任务只发 token，避免状态机/控制逻辑阻塞在 NVS。
    if (!multiTurnNvsSaveSnapshot(validMask, motorAbs, motorRaw, &sequence)) {
        sharedData->multiturn_nvs_save_fail_count++;
        Serial.printf("[NVS MULTITURN] save failed reason=%u mask=0x%08lX fail=%lu\r\n",
                      (unsigned)reason,
                      (unsigned long)validMask,
                      (unsigned long)sharedData->multiturn_nvs_save_fail_count);
        return false;
    }

    state->initialized = true;
    state->lastSaveMs = nowMs;
    state->lastValidMask = validMask;
    memcpy(state->lastAbs, motorAbs, sizeof(state->lastAbs));
    memcpy(state->lastRaw, motorRaw, sizeof(state->lastRaw));
    memcpy(state->lastTurn, motorTurn, sizeof(state->lastTurn));
    sharedData->multiturn_nvs_save_valid_mask = validMask;
    sharedData->multiturn_nvs_snapshot_valid = 1;
    Serial.printf("[NVS MULTITURN] saved seq=%lu reason=%u mask=0x%08lX turnChanged=%u edge=%u\r\n",
                  (unsigned long)sequence,
                  (unsigned)reason,
                  (unsigned long)validMask,
                  turnChanged ? 1U : 0U,
                  edgeChanged ? 1U : 0U);
    return true;
}

static void processMultiTurnNvsSave(TaskSharedData_t* sharedData,
                                    MultiTurnNvsRuntimeState* state,
                                    uint32_t* lastSaveToken)
{
    if (!sharedData || !state || !lastSaveToken) {
        return;
    }

    const uint32_t nowMs = millis();
    const uint32_t hostHeartbeatMs = sharedData->host_rx_heartbeat_ms;
    const bool hostOnline = hostHeartbeatMs != 0 &&
        (uint32_t)(nowMs - hostHeartbeatMs) <= kHostHeartbeatOfflineMs;

    // 上位机在线时主要依赖桌面缓存；heartbeat 停止后，NVS 兜底才变成关键保护。
    if (state->hostWasOnline && !hostOnline) {
        saveCurrentMultiTurnSnapshot(sharedData,
                                     state,
                                     MULTITURN_NVS_SAVE_REASON_HOST_OFFLINE,
                                     true);
    }
    state->hostWasOnline = hostOnline;

    const uint32_t token = sharedData->multiturn_nvs_save_request_token;
    if (token != *lastSaveToken) {
        *lastSaveToken = token;
        const uint8_t reason = sharedData->multiturn_nvs_save_reason;
        saveCurrentMultiTurnSnapshot(sharedData, state, reason, true);
        sharedData->multiturn_nvs_save_result_token = token;
    }

    if (hostOnline) {
        return;
    }

    int32_t motorAbs[SERVO_TOTAL_NUM] = {0};
    int16_t motorRaw[SERVO_TOTAL_NUM] = {0};
    int16_t motorTurn[SERVO_TOTAL_NUM] = {0};
    uint32_t validMask = 0;
    if (!collectTrustedMultiTurnSnapshot(sharedData, motorAbs, motorRaw, motorTurn, &validMask)) {
        return;
    }

    bool turnChanged = false;
    bool edgeChanged = false;
    const bool changed = snapshotChanged(*state,
                                         validMask,
                                         motorAbs,
                                         motorRaw,
                                         motorTurn,
                                         &turnChanged,
                                         &edgeChanged);
    if (!changed) {
        return;
    }

    const uint32_t elapsed = state->initialized ? (uint32_t)(nowMs - state->lastSaveMs) : 0xFFFFFFFFUL;
    if (turnChanged || edgeChanged) {
        if (elapsed >= kNvsSaveMinIntervalMs) {
            saveCurrentMultiTurnSnapshot(sharedData,
                                         state,
                                         MULTITURN_NVS_SAVE_REASON_OFFLINE_TURN_EDGE,
                                         true);
        }
    } else if (elapsed >= kNvsOfflinePeriodicSaveMs) {
        saveCurrentMultiTurnSnapshot(sharedData,
                                     state,
                                     MULTITURN_NVS_SAVE_REASON_OFFLINE_MOVE,
                                     true);
    }
}

static void processBootNvsRestore(TaskSharedData_t* sharedData,
                                  const MultiTurnNvsSnapshot* snapshot,
                                  bool* restoreDone,
                                  uint32_t restoreDeadlineMs)
{
    if (!sharedData || !snapshot || !restoreDone || *restoreDone) {
        return;
    }

    const uint32_t requestMask = snapshot->validMask & validMotorChannelMask();
    if (requestMask == 0) {
        *restoreDone = true;
        return;
    }

    const uint32_t onlineMask = collectOnlineMotorMask();
    const uint32_t nowMs = millis();
    if ((onlineMask & requestMask) != requestMask &&
        (int32_t)(nowMs - restoreDeadlineMs) <= 0) {
        return;
    }

    int32_t motorAbs[SERVO_TOTAL_NUM] = {0};
    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        motorAbs[ch] = multiTurnSnapshotAbsForChannel(*snapshot, ch);
    }

    uint32_t restoredMask = 0;
    uint32_t rejectedMask = 0;
    // NVS 恢复复用上位机恢复的底层路径，但 raw 容差更严；校验失败则保持 multi-turn unknown。
    restoreServoMultiTurnFromHost(sharedData,
                                  requestMask & onlineMask,
                                  motorAbs,
                                  kNvsRestoreRawToleranceCounts,
                                  &restoredMask,
                                  &rejectedMask);
    Serial.printf("[NVS MULTITURN] boot restore seq=%lu request=0x%08lX online=0x%08lX restored=0x%08lX rejected=0x%08lX\r\n",
                  (unsigned long)snapshot->sequence,
                  (unsigned long)requestMask,
                  (unsigned long)onlineMask,
                  (unsigned long)restoredMask,
                  (unsigned long)rejectedMask);
    *restoreDone = true;
}
#endif

void servoCommunicationTask(void* parameter)
{
    TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;
    if (!sharedData) {
        vTaskDelete(NULL);
        return;
    }

    beginServoBuses();
    configureMappedServosForMultiTurn();

    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t taskPeriodTicks = pdMS_TO_TICKS(10);
    uint8_t readBusPhase = 0;
    uint32_t lastEmergencyStopToken = sharedData->servo_emergency_stop_token;
    uint32_t lastRestoreToken = sharedData->restore_multi_turn_request_token;
#if MULTITURN_NVS_PERSISTENCE_ENABLE
    uint32_t lastNvsSaveToken = sharedData->multiturn_nvs_save_request_token;
#endif
    const uint32_t bootRecoverDeadlineMs = millis() + kBootRecoverLimitToRawWindowMs;
#if MULTITURN_NVS_PERSISTENCE_ENABLE
    const uint32_t nvsRestoreDeadlineMs = millis() + kNvsRestoreWaitWindowMs;
    bool nvsEnabled = true;
    if (!multiTurnNvsLoadEnabled(&nvsEnabled)) {
        Serial.println("[NVS MULTITURN] enabled flag load failed, default enabled");
        nvsEnabled = true;
    }
    sharedData->multiturn_nvs_enabled = nvsEnabled ? 1 : 0;
    MultiTurnNvsSnapshot bootNvsSnapshot;
    const bool hasBootNvsSnapshot = nvsEnabled && multiTurnNvsLoadLatest(&bootNvsSnapshot);
    sharedData->multiturn_nvs_snapshot_valid = hasBootNvsSnapshot ? 1 : 0;
    bool bootNvsRestoreDone = !hasBootNvsSnapshot;
    MultiTurnNvsRuntimeState nvsSaveState;
    memset(&nvsSaveState, 0, sizeof(nvsSaveState));
    nvsSaveState.hostWasOnline = true;
    if (!nvsEnabled) {
        Serial.println("[NVS MULTITURN] disabled by user, boot restore skipped");
    } else if (hasBootNvsSnapshot) {
        seedNvsRuntimeStateFromSnapshot(&nvsSaveState, bootNvsSnapshot);
        Serial.printf("[NVS MULTITURN] loaded seq=%lu mask=0x%08lX\r\n",
                      (unsigned long)bootNvsSnapshot.sequence,
                      (unsigned long)(bootNvsSnapshot.validMask & validMotorChannelMask()));
    } else {
        Serial.println("[NVS MULTITURN] no valid boot snapshot");
    }
#else
    // Flash/NVS 多圈持久化调试期关闭：不开机读 NVS、不写 NVS、不做本地恢复。
    Serial.println("[NVS MULTITURN] persistence disabled");
#endif

    for (;;)
    {
        const uint32_t emergencyStopToken = sharedData->servo_emergency_stop_token;
        if (emergencyStopToken != lastEmergencyStopToken) {
            lastEmergencyStopToken = emergencyStopToken;
            emergencyHoldMappedServosAtFeedback(sharedData);
        }

        processHostMultiTurnRestore(sharedData, &lastRestoreToken);

        ServoTargetBatch_t batch;
        while (sharedData->servoTargetQueue &&
               xQueueReceive(sharedData->servoTargetQueue, &batch, 0) == pdTRUE) {
            const uint8_t owner = sharedData->servo_target_owner;
            if (owner != SERVO_TARGET_OWNER_NONE && batch.source == owner) {
                applyServoTargetBatch(batch);
            }
        }

        uint8_t readIds[NUM_BUSES][MAX_SERVOS_PER_BUS] = {0};
        uint8_t readCounts[NUM_BUSES] = {0};
        for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
            addReadTarget(readIds, readCounts, motorMap[ch].busIndex, motorMap[ch].servoID);
        }

        const uint8_t readStartBus = (readBusPhase == 0) ? 0 : 2;
        for (uint8_t offset = 0; offset < 2; offset++) {
            const uint8_t busIndex = (uint8_t)(readStartBus + offset);
            if (busIndex >= NUM_BUSES || readCounts[busIndex] == 0) {
                continue;
            }
            g_servoBuses[busIndex].syncReadPositions(readIds[busIndex], readCounts[busIndex]);
        }
        readBusPhase ^= 1;

#if MULTITURN_NVS_PERSISTENCE_ENABLE
        processBootNvsRestore(sharedData,
                              hasBootNvsSnapshot ? &bootNvsSnapshot : NULL,
                              &bootNvsRestoreDone,
                              nvsRestoreDeadlineMs);
#endif
        bootRecoverLimitServosToRaw(bootRecoverDeadlineMs);
        publishServoFeedback(sharedData);
#if MULTITURN_NVS_PERSISTENCE_ENABLE
        processMultiTurnNvsSave(sharedData, &nvsSaveState, &lastNvsSaveToken);
#endif
        vTaskDelayUntil(&lastWakeTime, taskPeriodTicks);
    }
}
