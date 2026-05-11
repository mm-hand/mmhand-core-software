#include "ServoCommunicationTask.h"

#include <string.h>

#include "../libraries/FTServo_Arduino/src/SMS_STS.h"

#ifndef FTSERVO_ARDUINO_EXTERNAL_BUILD
#include "../libraries/FTServo_Arduino/src/SCS.cpp"
#include "../libraries/FTServo_Arduino/src/SCSerial.cpp"
#include "../libraries/FTServo_Arduino/src/SMS_STS.cpp"
#endif

extern MotorMapItem motorMap[SERVO_TOTAL_NUM];

#ifndef SERVO_CONFIGURE_MULTI_TURN_ON_BOOT
#define SERVO_CONFIGURE_MULTI_TURN_ON_BOOT 1
#endif

#define SERVO_MULTI_TURN_CONFIG_DELAY_MS 5

// 单个舵机在通信层内部维护的反馈缓存。
// rawPosition 是单圈读数，absolutePosition 是根据跨圈变化推导的多圈位置。
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

// ServoBusDriver 是 ServoCommunicationTask.cpp 私有的舵机总线驱动封装。
// 它负责一条 UART 上的批量写入、同步读取和多圈位置缓存。
class ServoBusDriver {
public:
    ServoBusDriver() : _serial(NULL), _writeCount(0), _syncReadTimeoutMs(SERVO_SYNC_READ_TIMEOUT_MS_DEFAULT)
    {
        memset(_feedback, 0, sizeof(_feedback));
    }

    void begin(uint8_t busIndex, int rxPin, int txPin, uint32_t baud)
    {
        // ESP32-P4 当前使用 Serial1~Serial4 分别绑定四条飞特舵机总线。
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
        // 先缓存到本总线的批量写数组，syncWriteAll 再一次性下发。
        if (_writeCount >= MAX_SERVOS_PER_BUS) {
            return;
        }
        if (position < -30719) position = -30719;
        if (position > 30719) position = 30719;

        _writeIDs[_writeCount] = id;
        _writePos[_writeCount] = position;
        _writeSpd[_writeCount] = speed;
        _writeAcc[_writeCount] = acc;
        _writeCount++;
    }

    void syncWriteAll()
    {
        // 飞特同步写要求同一总线上的 ID/位置/速度/加速度数组并行传入。
        if (_writeCount == 0 || !_serial) {
            return;
        }
        _sms.SyncWritePosEx(_writeIDs, _writeCount, _writePos, _writeSpd, _writeAcc);
        _writeCount = 0;
    }

    int syncReadPositions(const uint8_t* ids, uint8_t count)
    {
        // 读取当前位置开始的 8 字节：位置、速度、负载、电压、温度。
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
            // TX 失败说明该轮同步读无效，先将目标 ID 标记离线。
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
                // 返回长度正确才更新缓存；否则保留其他字段并标记离线。
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

private:
    static int16_t decodeSigned(uint8_t lo, uint8_t hi)
    {
        // 飞特返回值使用最高位表示符号，其余位表示绝对值。
        const uint16_t raw = ((uint16_t)hi << 8) | lo;
        return (raw & (1 << 15)) ? -(int16_t)(raw & ~(1 << 15)) : (int16_t)raw;
    }

    void _updateMultiTurnPosition(uint8_t id, int16_t newRawPos)
    {
        // 根据单圈 raw 跨过半圈的方向推断圈数，得到连续多圈位置。
        ServoFeedbackInternal& fb = _feedback[id];
        if (newRawPos < 0 || newRawPos >= 4096) {
            int16_t rawSingleTurn = (int16_t)(newRawPos % 4096);
            if (rawSingleTurn < 0) {
                rawSingleTurn += 4096;
            }
            fb.rawPosition = rawSingleTurn;
            fb.lastRawPosition = rawSingleTurn;
            fb.absolutePosition = newRawPos;
            if (fb.absolutePosition > 30719) {
                fb.absolutePosition = 30719;
            } else if (fb.absolutePosition < -30719) {
                fb.absolutePosition = -30719;
            }
            fb.turnCount = (int16_t)((fb.absolutePosition - rawSingleTurn) / 4096);
            fb.initialized = true;
            return;
        }

        if (!fb.initialized) {
            fb.rawPosition = newRawPos;
            fb.lastRawPosition = newRawPos;
            fb.absolutePosition = newRawPos;
            fb.turnCount = 0;
            fb.initialized = true;
            return;
        }

        const int16_t delta = newRawPos - fb.lastRawPosition;
        const int16_t halfTurn = 2048;
        if (delta > halfTurn) {
            fb.turnCount--;
        } else if (delta < -halfTurn) {
            fb.turnCount++;
        }

        fb.rawPosition = newRawPos;
        fb.lastRawPosition = newRawPos;
        fb.absolutePosition = (int32_t)fb.turnCount * 4096 + newRawPos;
        if (fb.absolutePosition > 30719) {
            fb.absolutePosition = 30719;
        } else if (fb.absolutePosition < -30719) {
            fb.absolutePosition = -30719;
        }
    }

    SMS_STS _sms;
    HardwareSerial* _serial;
    uint8_t _writeIDs[MAX_SERVOS_PER_BUS];
    int16_t _writePos[MAX_SERVOS_PER_BUS];
    uint16_t _writeSpd[MAX_SERVOS_PER_BUS];
    uint8_t _writeAcc[MAX_SERVOS_PER_BUS];
    uint8_t _writeCount;
    ServoFeedbackInternal _feedback[MAX_SERVO_ID + 1];
    uint16_t _syncReadTimeoutMs;
};

static ServoBusDriver g_servoBuses[NUM_BUSES];

static ServoBusDriver* getBusByIndex(uint8_t busIndex)
{
    // 所有外部目标都必须先通过 busIndex 映射到内部驱动对象。
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
    // 同一条总线上同一个 ID 只加入一次，避免重复同步读。
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

static void beginServoBuses()
{
    // 防止任务意外重复初始化 UART。
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
    // 按总线聚合目标：先填每条总线缓存，再分别执行同步写。
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

    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
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

        const int32_t holdPos32 = bus->getAbsolutePosition(id);
        int16_t holdPos = 0;
        if (holdPos32 > 30719) {
            holdPos = 30719;
        } else if (holdPos32 < -30719) {
            holdPos = -30719;
        } else {
            holdPos = (int16_t)holdPos32;
        }
        bus->setTarget(id, holdPos, SERVO_TARGET_SPEED_DEFAULT, SERVO_TARGET_ACC_DEFAULT);
        heldIds[busIndex][heldCounts[busIndex]++] = id;
        busWritePending[busIndex] = true;
    }

    for (uint8_t busIndex = 0; busIndex < NUM_BUSES; busIndex++) {
        if (busWritePending[busIndex]) {
            g_servoBuses[busIndex].syncWriteAll();
        }
    }
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

    // 将物理 bus/id 反馈按 motorMap 转换成上层统一的 0..21 motor 通道顺序。
    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        const uint8_t busIndex = motorMap[ch].busIndex;
        const uint8_t id = motorMap[ch].servoID;
        ServoBusDriver* bus = getBusByIndex(busIndex);
        if (bus && bus->isOnline(id)) {
            const ServoFeedbackInternal& fb = bus->getFeedback(id);
            servoData.servoAngles[ch] = bus->getAbsolutePosition(id);
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

    for (;;)
    {
        const uint32_t emergencyStopToken = sharedData->servo_emergency_stop_token;
        if (emergencyStopToken != lastEmergencyStopToken) {
            lastEmergencyStopToken = emergencyStopToken;
            emergencyHoldMappedServosAtFeedback(sharedData);
        }

        ServoTargetBatch_t batch;
        while (sharedData->servoTargetQueue &&
               xQueueReceive(sharedData->servoTargetQueue, &batch, 0) == pdTRUE) {
            // owner/source 双重匹配，保证校准目标和普通控制目标不会互相覆盖。
            const uint8_t owner = sharedData->servo_target_owner;
            if (owner != SERVO_TARGET_OWNER_NONE && batch.source == owner) {
                applyServoTargetBatch(batch);
            }
        }

        uint8_t readIds[NUM_BUSES][MAX_SERVOS_PER_BUS] = {0};
        uint8_t readCounts[NUM_BUSES] = {0};
        // 每轮都从 motorMap 生成读取列表，确保映射表是唯一硬件拓扑来源。
        for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
            addReadTarget(readIds, readCounts, motorMap[ch].busIndex, motorMap[ch].servoID);
        }

        const uint8_t readStartBus = (readBusPhase == 0) ? 0 : 2;
        // 10ms 周期内只读两条总线，0/1 与 2/3 交替，降低单周期串口阻塞时间。
        for (uint8_t offset = 0; offset < 2; offset++) {
            const uint8_t busIndex = (uint8_t)(readStartBus + offset);
            if (busIndex >= NUM_BUSES || readCounts[busIndex] == 0) {
                continue;
            }
            g_servoBuses[busIndex].syncReadPositions(readIds[busIndex], readCounts[busIndex]);
        }
        readBusPhase ^= 1;

        publishServoFeedback(sharedData);
        vTaskDelayUntil(&lastWakeTime, taskPeriodTicks);
    }
}
