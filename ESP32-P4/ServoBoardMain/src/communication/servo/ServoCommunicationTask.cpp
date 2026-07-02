#include "ServoCommunicationTask.h"

#include <string.h>

#include "../../hardware/HardwareMap.h"
#include "../../shared/SharedSafetyState.h"
#include "../../system/FirmwareLog.h"
#include "../../system/TaskRuntimeMonitor.h"
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
static const uint8_t kServoOfflineInvalidateMisses = 3;
static const uint8_t kServoOnlineConfirmReads = 2;
static const uint32_t kServoOfflineProbeIntervalMs = 50;
#if MULTITURN_NVS_PERSISTENCE_ENABLE
static const uint32_t kNvsRestoreWaitWindowMs = 3000;
static const uint32_t kHostHeartbeatOfflineMs = 1000;
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
    int16_t presentPosition;
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
    uint8_t missedReadCount;
    uint8_t successfulReadCount;
    bool onlineTransitionPending;
    uint32_t lastUpdate;
};

// Private wrapper for one Feetech UART servo bus.
class ServoBusDriver {
public:
    ServoBusDriver() : _serial(NULL), _writeCount(0), _syncReadTimeoutMs(SERVO_SYNC_READ_TIMEOUT_MS_DEFAULT)
    {
        memset(_feedback, 0, sizeof(_feedback));
        memset(_torqueDisabled, 0, sizeof(_torqueDisabled));
        memset(_multiTurnConfigured, 0, sizeof(_multiTurnConfigured));
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
        if (ok) {
            _multiTurnConfigured[id] = true;
        }
        return ok;
    }

    void setTarget(uint8_t id, int16_t position, uint16_t speed, uint8_t acc)
    {
        queueTarget(id, logicalToHardwareTarget(id, position), speed, acc);
    }

    void setHardwareTarget(uint8_t id, int16_t position, uint16_t speed, uint8_t acc)
    {
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
                    markReadFailure(ids[i]);
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
                recordFeedbackSuccess(id,
                                      decodeSigned(rxBuf[0], rxBuf[1]),
                                      decodeSigned(rxBuf[2], rxBuf[3]),
                                      decodeServoLoad(rxBuf[4], rxBuf[5]),
                                      rxBuf[6],
                                      rxBuf[7]);
                successCount++;
            } else {
                markReadFailure(id);
            }
        }

        _sms.syncReadEnd();
        return successCount;
    }

    bool probeFeedback(uint8_t id)
    {
        if (!_serial || id > MAX_SERVO_ID) {
            return false;
        }
        if (_sms.FeedBack(id) < 0) {
            markReadFailure(id);
            return false;
        }
        recordFeedbackSuccess(id,
                              (int16_t)_sms.ReadPos(-1),
                              (int16_t)_sms.ReadSpeed(-1),
                              (int16_t)_sms.ReadLoad(-1),
                              (uint8_t)_sms.ReadVoltage(-1),
                              (uint8_t)_sms.ReadTemper(-1));
        return true;
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

    int16_t getPresentPosition(uint8_t id) const
    {
        if (id > MAX_SERVO_ID) return 0;
        return _feedback[id].presentPosition;
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

    bool hadMultiTurnTrust(uint8_t id) const
    {
        if (id > MAX_SERVO_ID) {
            return false;
        }
        const ServoFeedbackInternal& fb = _feedback[id];
        return fb.initialized || fb.absoluteReliable || fb.logicalOffset != 0;
    }

    bool isOfflineInvalidated(uint8_t id) const
    {
        return id <= MAX_SERVO_ID &&
               !_feedback[id].online &&
               _feedback[id].missedReadCount >= kServoOfflineInvalidateMisses;
    }

    bool consumeOnlineTransition(uint8_t id)
    {
        if (id > MAX_SERVO_ID || !_feedback[id].onlineTransitionPending) {
            return false;
        }
        _feedback[id].onlineTransitionPending = false;
        return true;
    }

    bool isMultiTurnConfigured(uint8_t id) const
    {
        return id <= MAX_SERVO_ID && _multiTurnConfigured[id];
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
        const uint16_t rawDistance = singleTurnDistanceCounts(currentRaw, restoredRaw);
        if (rawDistance > rawToleranceCounts) {
            FW_LOG_INFO_PRINTF("[SERVO RESTORE REJECT] id=%u currentRaw=%d restoredRaw=%d distance=%u tol=%u logical=%ld\r\n",
                          (unsigned)id,
                          (int)currentRaw,
                          (int)restoredRaw,
                          (unsigned)rawDistance,
                          (unsigned)rawToleranceCounts,
                          (long)absPosition);
            return false;
        }

        const int32_t restoredLogicalAbs = clampServoAbsLocal(absPosition);
        // New-device reset sends the current single-turn raw as turn0. If the
        // servo power was cycled while P4 stayed on, the previous software
        // multi-turn anchor is stale and must not be reused here.
        const bool restoreToCurrentSingleTurn =
            (restoredLogicalAbs >= 0 && restoredLogicalAbs < 4096);
        const int32_t hardwareAbs = restoreToCurrentSingleTurn
            ? (int32_t)currentRaw
            : (fb.initialized ? fb.absolutePosition : (int32_t)currentRaw);
        fb.rawPosition = currentRaw;
        fb.lastRawPosition = currentRaw;
        fb.absolutePosition = hardwareAbs;
        fb.logicalOffset = restoredLogicalAbs - hardwareAbs;
        fb.turnCount = (int16_t)((restoredLogicalAbs - currentRaw) / 4096);
        fb.initialized = true;
        fb.absoluteReliable = true;
        fb.lastUpdate = millis();
        FW_LOG_INFO_PRINTF("[SERVO RESTORE OFFSET] id=%u logical=%ld hardware=%ld offset=%ld raw=%d\r\n",
                      (unsigned)id,
                      (long)restoredLogicalAbs,
                      (long)hardwareAbs,
                      (long)fb.logicalOffset,
                      (int)currentRaw);
        return true;
    }

private:
    void recordFeedbackSuccess(uint8_t id,
                               int16_t rawPos,
                               int16_t speed,
                               int16_t load,
                               uint8_t voltage,
                               uint8_t temperature)
    {
        ServoFeedbackInternal& fb = _feedback[id];
        const bool wasOnline = fb.online;
        _updateMultiTurnPosition(id, rawPos);
        fb.speed = speed;
        fb.load = load;
        fb.voltage = voltage;
        fb.temperature = temperature;
        fb.missedReadCount = 0;
        if (fb.successfulReadCount < kServoOnlineConfirmReads) {
            fb.successfulReadCount++;
        }
        if (!wasOnline && fb.successfulReadCount >= kServoOnlineConfirmReads) {
            fb.online = true;
            fb.onlineTransitionPending = true;
            FW_LOG_INFO_PRINTF("[SERVO HOTPLUG] id=%u communication restored\r\n", (unsigned)id);
        }
        fb.lastUpdate = millis();
    }

    void markReadFailure(uint8_t id)
    {
        if (id > MAX_SERVO_ID) {
            return;
        }
        ServoFeedbackInternal& fb = _feedback[id];
        fb.successfulReadCount = 0;
        if (fb.missedReadCount < 0xFF) {
            fb.missedReadCount++;
        }
        if (fb.missedReadCount >= kServoOfflineInvalidateMisses && fb.online) {
            fb.online = false;
            fb.onlineTransitionPending = false;
            _multiTurnConfigured[id] = false;
            invalidateMultiTurnTrust(id);
        }
    }

    void invalidateMultiTurnTrust(uint8_t id)
    {
        if (id > MAX_SERVO_ID) {
            return;
        }
        ServoFeedbackInternal& fb = _feedback[id];
        const bool hadTrust = fb.initialized || fb.absoluteReliable || fb.logicalOffset != 0;
        fb.initialized = false;
        fb.absoluteReliable = false;
        fb.logicalOffset = 0;
        fb.turnCount = 0;
        if (hadTrust) {
            FW_LOG_INFO_PRINTF("[SERVO OFFLINE] id=%u multi-turn trust invalidated after %u missed reads\r\n",
                          (unsigned)id,
                          (unsigned)fb.missedReadCount);
        }
    }

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

        fb.presentPosition = newRawPos;
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
    bool _multiTurnConfigured[MAX_SERVO_ID + 1];
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

static void probeOneOfflineMappedServo(uint32_t nowMs)
{
    static uint32_t lastProbeMs = 0;
    static uint8_t nextChannel = 0;
    if ((uint32_t)(nowMs - lastProbeMs) < kServoOfflineProbeIntervalMs) {
        return;
    }
    lastProbeMs = nowMs;

    for (uint8_t offset = 0; offset < SERVO_TOTAL_NUM; offset++) {
        const uint8_t ch = (uint8_t)((nextChannel + offset) % SERVO_TOTAL_NUM);
        const uint8_t busIndex = motorMap[ch].busIndex;
        const uint8_t id = motorMap[ch].servoID;
        ServoBusDriver* bus = getBusByIndex(busIndex);
        if (!bus || id > MAX_SERVO_ID || bus->isOnline(id)) {
            continue;
        }
        nextChannel = (uint8_t)((ch + 1) % SERVO_TOTAL_NUM);
        bus->probeFeedback(id);
        return;
    }
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
            !bus->isMultiTurnConfigured(id) ||
            !bus->restoreAbsolutePosition(id, motorAbs[ch], rawToleranceCounts)) {
            rejected |= bit;
            continue;
        }
        restored |= bit;
    }

    if (restored != 0) {
        sharedAtomicFetchAnd(&sharedData->multi_turn_unknown_bitmap, ~restored);
    }

    if (restoredMask) *restoredMask = restored;
    if (rejectedMask) *rejectedMask = rejected;
    FW_LOG_INFO_PRINTF("[SERVO RESTORE] request=0x%08lX restored=0x%08lX rejected=0x%08lX unknown=0x%08lX tol=%u\r\n",
                  (unsigned long)requestMask,
                  (unsigned long)restored,
                  (unsigned long)rejected,
                  (unsigned long)sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap),
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

static void emergencyTorqueOffMappedServos(TaskSharedData_t* sharedData)
{
    if (sharedData && sharedData->servoTargetQueue) {
        xQueueReset(sharedData->servoTargetQueue);
    }

    uint8_t disabledIds[NUM_BUSES][MAX_SERVOS_PER_BUS] = {0};
    uint8_t disabledCounts[NUM_BUSES] = {0};
    uint32_t requestedMask = 0;
    uint32_t torqueDisabledMask = 0;
    uint32_t torqueDisableFailedMask = 0;

    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        const uint32_t bit = (1UL << ch);
        const uint8_t busIndex = motorMap[ch].busIndex;
        const uint8_t id = motorMap[ch].servoID;
        if (busIndex >= NUM_BUSES || id > MAX_SERVO_ID) {
            continue;
        }

        requestedMask |= bit;
        bool alreadyDisabled = false;
        for (uint8_t i = 0; i < disabledCounts[busIndex]; i++) {
            if (disabledIds[busIndex][i] == id) {
                alreadyDisabled = true;
                break;
            }
        }
        if (alreadyDisabled) {
            torqueDisabledMask |= bit;
            continue;
        }
        if (disabledCounts[busIndex] >= MAX_SERVOS_PER_BUS) {
            torqueDisableFailedMask |= bit;
            continue;
        }

        ServoBusDriver* bus = getBusByIndex(busIndex);
        if (!bus) {
            torqueDisableFailedMask |= bit;
            continue;
        }

        if (bus->setTorqueEnabled(id, false)) {
            torqueDisabledMask |= bit;
            disabledIds[busIndex][disabledCounts[busIndex]++] = id;
        } else {
            torqueDisableFailedMask |= bit;
        }
    }

    FW_LOG_INFO_PRINTF("[SERVO E-STOP] torque-off request=0x%08lX disabled=0x%08lX failed=0x%08lX\r\n",
                  (unsigned long)requestedMask,
                  (unsigned long)torqueDisabledMask,
                  (unsigned long)torqueDisableFailedMask);
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
        FW_LOG_INFO_PRINTF("[SERVO BOOT RECOVER] bus=%u id=%u abs=%ld raw=%d\r\n",
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
    uint32_t onlineMask = 0;

    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        const uint8_t busIndex = motorMap[ch].busIndex;
        const uint8_t id = motorMap[ch].servoID;
        ServoBusDriver* bus = getBusByIndex(busIndex);
        if (bus && bus->isOnline(id)) {
            const ServoFeedbackInternal& fb = bus->getFeedback(id);
            servoData.servoAngles[ch] = bus->getAbsolutePosition(id);
            servoData.servoRawPositions[ch] = bus->getRawPosition(id);
            servoData.servoHardwarePositions[ch] = bus->getPresentPosition(id);
            servoRawData.servoRawPositions[ch] = bus->getRawPosition(id);
            servoRawData.servoHardwarePositions[ch] = bus->getPresentPosition(id);
            servoData.onlineStatus[ch] = 1;
            servoRawData.onlineStatus[ch] = 1;
            onlineMask |= (1UL << ch);
            telemetryData.speed[ch] = fb.speed;
            telemetryData.load[ch] = fb.load;
            telemetryData.voltage[ch] = fb.voltage;
            telemetryData.temperature[ch] = fb.temperature;
            telemetryData.onlineStatus[ch] = 1;
            if (bus->isAbsoluteReliable(id)) {
                const uint32_t bit = (1UL << ch);
                if ((sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap) & bit) != 0) {
                    sharedAtomicFetchAnd(&sharedData->multi_turn_unknown_bitmap, ~bit);
                    FW_LOG_INFO_PRINTF("[SERVO RESTORE] M%02u absolute feedback trusted abs=%ld raw=%d unknown=0x%08lX\r\n",
                                  (unsigned)ch,
                                  (long)servoData.servoAngles[ch],
                                  (int)servoData.servoRawPositions[ch],
                                  (unsigned long)sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap));
                }
            }
        } else if (!bus || bus->isOfflineInvalidated(id)) {
            const uint32_t bit = (1UL << ch);
            if ((sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap) & bit) == 0) {
                sharedAtomicFetchOr(&sharedData->multi_turn_unknown_bitmap, bit);
                FW_LOG_INFO_PRINTF("[SERVO OFFLINE] M%02u bus=%u id=%u marked Turn UNKNOWN unknown=0x%08lX\r\n",
                              (unsigned)ch,
                              (unsigned)busIndex,
                              (unsigned)id,
                              (unsigned long)sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap));
            }
        }
    }
    sharedAtomicStore(&sharedData->servo_online_bitmap,
                      onlineMask & validMotorChannelMask());

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
    const uint32_t token =
        sharedAtomicLoad(&sharedData->restore_multi_turn_request_token);
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
    sharedAtomicStore(&sharedData->restore_multi_turn_result_token, token);
#if MULTITURN_NVS_PERSISTENCE_ENABLE
    if (restoredMask != 0) {
        sharedData->multiturn_nvs_save_reason = MULTITURN_NVS_SAVE_REASON_HOST_RESTORE;
        sharedAtomicAddFetch(&sharedData->multiturn_nvs_save_request_token, (uint32_t)1);
    }
#endif
}

#if MULTITURN_NVS_PERSISTENCE_ENABLE
struct MultiTurnNvsRuntimeState {
    bool initialized;
    bool hostWasOnline;
    bool saveQueued;
    uint32_t lastSaveMs;
    uint32_t lastValidMask;
    uint32_t pendingRequestToken;
    uint32_t pendingQueuedAtMs;
    uint32_t pendingValidMask;
    int32_t lastAbs[SERVO_TOTAL_NUM];
    int32_t pendingAbs[SERVO_TOTAL_NUM];
    int16_t lastRaw[SERVO_TOTAL_NUM];
    int16_t pendingRaw[SERVO_TOTAL_NUM];
    int16_t lastTurn[SERVO_TOTAL_NUM];
    int16_t pendingTurn[SERVO_TOTAL_NUM];
    uint8_t pendingReason;
    uint8_t pendingTurnChanged;
    uint8_t pendingEdgeChanged;
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
    const uint32_t allowedMask =
        validMotorChannelMask() & ~sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap);
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

static void processServoOnlineTransitions(TaskSharedData_t* sharedData,
                                          const MultiTurnNvsRuntimeState* nvsState,
                                          bool nvsEnabled)
{
    if (!sharedData) {
        return;
    }
    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        const uint32_t bit = (1UL << ch);
        const uint8_t busIndex = motorMap[ch].busIndex;
        const uint8_t id = motorMap[ch].servoID;
        ServoBusDriver* bus = getBusByIndex(busIndex);
        if (!bus || !bus->consumeOnlineTransition(id)) {
            continue;
        }

        const bool torqueOffOk = bus->setTorqueEnabled(id, false);
        bool configured = bus->isMultiTurnConfigured(id);
        if (!configured) {
            configured = bus->configureMultiTurnMode(id);
        }

        bool trusted = bus->isAbsoluteReliable(id);
        const bool hasAnchor = nvsEnabled && nvsState && nvsState->initialized &&
            ((nvsState->lastValidMask & bit) != 0);
        if (!trusted && torqueOffOk && configured && hasAnchor) {
            trusted = bus->restoreAbsolutePosition(id,
                                                   nvsState->lastAbs[ch],
                                                   kNvsRestoreRawToleranceCounts);
        }

        if (trusted) {
            sharedAtomicFetchAnd(&sharedData->multi_turn_unknown_bitmap, ~bit);
        } else {
            sharedAtomicFetchOr(&sharedData->multi_turn_unknown_bitmap, bit);
        }
        FW_LOG_INFO_PRINTF("[SERVO HOTPLUG] M%02u bus=%u id=%u torque_off=%u configured=%u anchor=%u trusted=%u raw=%d unknown=0x%08lX\r\n",
                      (unsigned)ch,
                      (unsigned)busIndex,
                      (unsigned)id,
                      torqueOffOk ? 1U : 0U,
                      configured ? 1U : 0U,
                      hasAnchor ? 1U : 0U,
                      trusted ? 1U : 0U,
                      (int)bus->getRawPosition(id),
                      (unsigned long)sharedAtomicLoad(&sharedData->multi_turn_unknown_bitmap));
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

static bool snapshotMatchesPending(const MultiTurnNvsRuntimeState& state,
                                   uint32_t validMask,
                                   const int32_t* motorAbs,
                                   const int16_t* motorRaw,
                                   const int16_t* motorTurn)
{
    if (!state.saveQueued || state.pendingValidMask != validMask) {
        return false;
    }
    return memcmp(state.pendingAbs, motorAbs, sizeof(state.pendingAbs)) == 0 &&
        memcmp(state.pendingRaw, motorRaw, sizeof(state.pendingRaw)) == 0 &&
        memcmp(state.pendingTurn, motorTurn, sizeof(state.pendingTurn)) == 0;
}

static void queueLatestMultiTurnNvsJob(QueueHandle_t queue, const MultiTurnNvsSaveJob_t* job)
{
    if (!queue || !job) {
        return;
    }
    xQueueOverwrite(queue, job);
}

static void applyCompletedNvsSave(MultiTurnNvsRuntimeState* state,
                                  const MultiTurnNvsSaveResult_t& result)
{
    if (!state || !result.success) {
        return;
    }
    state->initialized = true;
    state->lastSaveMs = result.completedAtMs;
    state->lastValidMask = result.validMask;
    memcpy(state->lastAbs, result.motorAbs, sizeof(state->lastAbs));
    memcpy(state->lastRaw, result.motorRaw, sizeof(state->lastRaw));
    memcpy(state->lastTurn, result.motorTurn, sizeof(state->lastTurn));
}

static void processMultiTurnNvsSaveResults(TaskSharedData_t* sharedData,
                                           MultiTurnNvsRuntimeState* state)
{
    if (!sharedData || !state || !sharedData->multiturnNvsResultQueue) {
        return;
    }

    MultiTurnNvsSaveResult_t result;
    while (xQueueReceive(sharedData->multiturnNvsResultQueue, &result, 0) == pdTRUE) {
        if (state->saveQueued && result.queuedAtMs == state->pendingQueuedAtMs) {
            state->saveQueued = false;
        }

        if (result.success) {
            applyCompletedNvsSave(state, result);
            FW_LOG_INFO_PRINTF("[NVS MULTITURN] saved seq=%lu reason=%u mask=0x%08lX turnChanged=%u edge=%u\r\n",
                          (unsigned long)result.sequence,
                          (unsigned)result.reason,
                          (unsigned long)result.validMask,
                          result.turnChanged ? 1U : 0U,
                          result.edgeChanged ? 1U : 0U);
        } else {
            FW_LOG_INFO_PRINTF("[NVS MULTITURN] save failed reason=%u mask=0x%08lX fail=%lu\r\n",
                          (unsigned)result.reason,
                          (unsigned long)result.validMask,
                          (unsigned long)sharedData->multiturn_nvs_save_fail_count);
        }
    }
}

static bool saveCurrentMultiTurnSnapshot(TaskSharedData_t* sharedData,
                                         MultiTurnNvsRuntimeState* state,
                                         uint8_t reason,
                                         bool force,
                                         uint32_t requestToken)
{
    if (!sharedData || !state) {
        return false;
    }
    if (!sharedData->multiturn_nvs_enabled || !sharedData->multiturnNvsSaveQueue) {
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

    if (snapshotMatchesPending(*state, validMask, motorAbs, motorRaw, motorTurn) &&
        (requestToken == 0 || requestToken == state->pendingRequestToken)) {
        return false;
    }

    MultiTurnNvsSaveJob_t job;
    memset(&job, 0, sizeof(job));
    job.requestToken = requestToken;
    job.validMask = validMask;
    job.queuedAtMs = nowMs;
    job.reason = reason;
    job.turnChanged = turnChanged ? 1U : 0U;
    job.edgeChanged = edgeChanged ? 1U : 0U;
    memcpy(job.motorAbs, motorAbs, sizeof(job.motorAbs));
    memcpy(job.motorRaw, motorRaw, sizeof(job.motorRaw));
    memcpy(job.motorTurn, motorTurn, sizeof(job.motorTurn));

    queueLatestMultiTurnNvsJob(sharedData->multiturnNvsSaveQueue, &job);
    state->saveQueued = true;
    state->pendingRequestToken = requestToken;
    state->pendingQueuedAtMs = nowMs;
    state->pendingValidMask = validMask;
    state->pendingReason = reason;
    state->pendingTurnChanged = job.turnChanged;
    state->pendingEdgeChanged = job.edgeChanged;
    memcpy(state->pendingAbs, motorAbs, sizeof(state->pendingAbs));
    memcpy(state->pendingRaw, motorRaw, sizeof(state->pendingRaw));
    memcpy(state->pendingTurn, motorTurn, sizeof(state->pendingTurn));
    return true;

#if 0
    uint32_t sequence = 0;
    if (!multiTurnNvsSaveSnapshot(validMask, motorAbs, motorRaw, &sequence)) {
        sharedData->multiturn_nvs_save_fail_count++;
        FW_LOG_INFO_PRINTF("[NVS MULTITURN] save failed reason=%u mask=0x%08lX fail=%lu\r\n",
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
    FW_LOG_INFO_PRINTF("[NVS MULTITURN] saved seq=%lu reason=%u mask=0x%08lX turnChanged=%u edge=%u\r\n",
                  (unsigned long)sequence,
                  (unsigned)reason,
                  (unsigned long)validMask,
                  turnChanged ? 1U : 0U,
                  edgeChanged ? 1U : 0U);
    return true;
#endif
}

static void processMultiTurnNvsSave(TaskSharedData_t* sharedData,
                                    MultiTurnNvsRuntimeState* state,
                                    uint32_t* lastSaveToken)
{
    if (!sharedData || !state || !lastSaveToken) {
        return;
    }

    processMultiTurnNvsSaveResults(sharedData, state);

    const uint32_t nowMs = millis();
    const uint32_t hostHeartbeatMs = sharedData->host_rx_heartbeat_ms;
    const bool hostOnline = hostHeartbeatMs != 0 &&
        (uint32_t)(nowMs - hostHeartbeatMs) <= kHostHeartbeatOfflineMs;

    if (state->hostWasOnline && !hostOnline) {
        saveCurrentMultiTurnSnapshot(sharedData,
                                     state,
                                     MULTITURN_NVS_SAVE_REASON_HOST_OFFLINE,
                                     true,
                                     0);
    }
    state->hostWasOnline = hostOnline;

    const uint32_t token =
        sharedAtomicLoad(&sharedData->multiturn_nvs_save_request_token);
    if (token != *lastSaveToken) {
        *lastSaveToken = token;
        const uint8_t reason = sharedData->multiturn_nvs_save_reason;
        saveCurrentMultiTurnSnapshot(sharedData, state, reason, true, token);
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
    // Keep turn/edge anchors fresh even while the desktop host is online. Without
    // this, a servo power cycle after crossing a turn boundary can only see a stale
    // NVS raw/turn pair and the hotplug restore will correctly reject it.
    if (turnChanged || edgeChanged) {
        if (turnChanged || elapsed >= kNvsSaveMinIntervalMs) {
            saveCurrentMultiTurnSnapshot(sharedData,
                                         state,
                                         MULTITURN_NVS_SAVE_REASON_OFFLINE_TURN_EDGE,
                                         true,
                                         0);
        }
        return;
    }
    if (hostOnline) {
        return;
    }
    if (elapsed >= kNvsOfflinePeriodicSaveMs) {
        saveCurrentMultiTurnSnapshot(sharedData,
                                     state,
                                     MULTITURN_NVS_SAVE_REASON_OFFLINE_MOVE,
                                     true,
                                     0);
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
        restoreServoMultiTurnFromHost(sharedData,
                                  requestMask & onlineMask,
                                  motorAbs,
                                  kNvsRestoreRawToleranceCounts,
                                  &restoredMask,
                                  &rejectedMask);
    FW_LOG_INFO_PRINTF("[NVS MULTITURN] boot restore seq=%lu request=0x%08lX online=0x%08lX restored=0x%08lX rejected=0x%08lX\r\n",
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
    uint32_t lastEmergencyStopToken =
        sharedAtomicLoad(&sharedData->servo_emergency_stop_token);
    uint32_t lastRestoreToken =
        sharedAtomicLoad(&sharedData->restore_multi_turn_request_token);
#if MULTITURN_NVS_PERSISTENCE_ENABLE
    uint32_t lastNvsSaveToken =
        sharedAtomicLoad(&sharedData->multiturn_nvs_save_request_token);
#endif
    const uint32_t bootRecoverDeadlineMs = millis() + kBootRecoverLimitToRawWindowMs;
#if MULTITURN_NVS_PERSISTENCE_ENABLE
    const uint32_t nvsRestoreDeadlineMs = millis() + kNvsRestoreWaitWindowMs;
    bool nvsEnabled = true;
    if (!multiTurnNvsLoadEnabled(&nvsEnabled)) {
        FW_LOG_INFO_PRINTLN("[NVS MULTITURN] enabled flag load failed, default enabled");
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
        FW_LOG_INFO_PRINTLN("[NVS MULTITURN] disabled by user, boot restore skipped");
    } else if (hasBootNvsSnapshot) {
        seedNvsRuntimeStateFromSnapshot(&nvsSaveState, bootNvsSnapshot);
        FW_LOG_INFO_PRINTF("[NVS MULTITURN] loaded seq=%lu mask=0x%08lX\r\n",
                      (unsigned long)bootNvsSnapshot.sequence,
                      (unsigned long)(bootNvsSnapshot.validMask & validMotorChannelMask()));
    } else {
        FW_LOG_INFO_PRINTLN("[NVS MULTITURN] no valid boot snapshot");
    }
#else
    FW_LOG_INFO_PRINTLN("[NVS MULTITURN] persistence disabled");
#endif

    for (;;)
    {
        taskRuntimeMarkLoopStart(TASK_RUNTIME_SERVO_COMM);
        const uint32_t emergencyStopToken =
            sharedAtomicLoad(&sharedData->servo_emergency_stop_token);
        if (emergencyStopToken != lastEmergencyStopToken) {
            lastEmergencyStopToken = emergencyStopToken;
            emergencyTorqueOffMappedServos(sharedData);
            taskRuntimeMarkStopTorqueOffComplete();
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
        probeOneOfflineMappedServo(millis());

#if MULTITURN_NVS_PERSISTENCE_ENABLE
        processBootNvsRestore(sharedData,
                              hasBootNvsSnapshot ? &bootNvsSnapshot : NULL,
                              &bootNvsRestoreDone,
                              nvsRestoreDeadlineMs);
        processServoOnlineTransitions(sharedData, &nvsSaveState, nvsEnabled);
#endif
        bootRecoverLimitServosToRaw(bootRecoverDeadlineMs);
        publishServoFeedback(sharedData);
#if MULTITURN_NVS_PERSISTENCE_ENABLE
        processMultiTurnNvsSave(sharedData, &nvsSaveState, &lastNvsSaveToken);
#endif
        taskRuntimeMarkLoopEnd(TASK_RUNTIME_SERVO_COMM);
        const TickType_t nextWakeTime = lastWakeTime + taskPeriodTicks;
        const bool periodOverrun = (int32_t)(xTaskGetTickCount() - nextWakeTime) >= 0;
        taskRuntimeMarkPeriodResult(TASK_RUNTIME_SERVO_COMM, periodOverrun);
        vTaskDelayUntil(&lastWakeTime, taskPeriodTicks);
        if (periodOverrun) {
            // Offline bus scans can overrun the 10 ms period. Force one blocked
            // tick so lower-priority status/desktop communication still runs.
            vTaskDelay(1);
        }
    }
}
