#include "ServoCommunicationTask.h"

#include <string.h>

#include "../../hardware/HardwareMap.h"
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
    int32_t absolutePosition;
    int16_t turnCount;
    int16_t lastRawPosition;
    bool initialized;
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
        if (!_serial || id == 0 || id > MAX_SERVO_ID) {
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
        if (id == 0 || id > MAX_SERVO_ID || _writeCount >= MAX_SERVOS_PER_BUS) {
            return;
        }
        if (_torqueDisabled[id]) {
            _sms.EnableTorque(id, 1);
            _torqueDisabled[id] = false;
        }
        position = clampServoAbsLocal(position);

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
                _feedback[id].load = decodeSigned(rxBuf[4], rxBuf[5]);
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
        return _feedback[id].absolutePosition;
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

    bool setTorqueEnabled(uint8_t id, bool enabled)
    {
        if (!_serial || id == 0 || id > MAX_SERVO_ID) {
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
        if (id == 0 || id > MAX_SERVO_ID) {
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

        const int32_t absClamped = clampServoAbsLocal(absPosition);
        fb.rawPosition = currentRaw;
        fb.lastRawPosition = currentRaw;
        fb.absolutePosition = absClamped;
        fb.turnCount = (int16_t)((absClamped - currentRaw) / 4096);
        fb.initialized = true;
        fb.lastUpdate = millis();
        return true;
    }

private:
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

    void _updateMultiTurnPosition(uint8_t id, int16_t newRawPos)
    {
        ServoFeedbackInternal& fb = _feedback[id];
        int32_t absPos = 0;
        int16_t rawSingleTurn = 0;

        if (newRawPos < 0 || newRawPos >= 4096) {
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
        fb.turnCount = (int16_t)((absPos - rawSingleTurn) / 4096);
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
        if (!bus || id == 0 || id > MAX_SERVO_ID ||
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
        if (busIndex >= NUM_BUSES || id == 0 || id > MAX_SERVO_ID) {
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
        if (busIndex >= NUM_BUSES || id == 0 || id > MAX_SERVO_ID) {
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
        bus->setTarget(id, holdPos, SERVO_TARGET_SPEED_DEFAULT, SERVO_TARGET_ACC_DEFAULT);
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
        if (busIndex >= NUM_BUSES || id == 0 || id > MAX_SERVO_ID) {
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
        bus->setTarget(id, rawSingleTurn, SERVO_TARGET_SPEED_DEFAULT, SERVO_TARGET_ACC_DEFAULT);
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
}

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
    const uint32_t bootRecoverDeadlineMs = millis() + kBootRecoverLimitToRawWindowMs;

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

        bootRecoverLimitServosToRaw(bootRecoverDeadlineMs);
        publishServoFeedback(sharedData);
        vTaskDelayUntil(&lastWakeTime, taskPeriodTicks);
    }
}
