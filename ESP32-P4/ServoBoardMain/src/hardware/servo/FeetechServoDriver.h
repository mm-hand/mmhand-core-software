/**
 * @file FeetechServoDriver.h
 * @brief 封装飞特 ST3215 串口协议、同步收发、反馈缓存和硬件多圈坐标。
 *
 * 这是 P4 中允许包含飞特 SDK 的唯一业务头文件。上层任务不得接触 SMS_STS，
 * 只能通过本类提交硬件目标、读取反馈或执行受控恢复操作。
 */
#ifndef P4_FEETECH_SERVO_DRIVER_H
#define P4_FEETECH_SERVO_DRIVER_H

#include <Arduino.h>
#include <string.h>

#include "ServoValue.h"
#include "../libraries/FTServo_Arduino/src/SMS_STS.h"
#include "../../communication/servo/ServoPositionRange.h"
#include "../../config/ServoConfig.h"
#include "../HardwareMap.h"
#include "../../system/FirmwareLog.h"

using mmhand::config::servo::kMultiTurnConfigDelayMs;
using mmhand::config::servo::kOfflineInvalidateMisses;
using mmhand::config::servo::kOnlineConfirmReads;
using mmhand::config::servo::kRecoveryMaxAttempts;
using mmhand::config::servo::kRecoveryRetryIntervalMs;

inline int16_t normalizeServoRawSingleTurn(int32_t value)
{
    return mmhand::servo::normalizeRawSingleTurn(value);
}

inline uint16_t singleTurnDistanceCounts(int16_t a, int16_t b)
{
    return static_cast<uint16_t>(mmhand::servo::circularRawDistance(a, b));
}

/** 单台舵机的驱动内部反馈状态；不得直接暴露为跨任务共享可写数据。 */
struct ServoFeedbackInternal {
    int16_t rawPosition; ///< 归一化后的 0..4095 hardware raw。
    int16_t speed;       ///< ST3215 Present Speed 原始值。
    int16_t load;        ///< ST3215 Present Load 有符号值。
    int16_t current;     ///< ST3215 Present Current 原始值。
    uint8_t voltage;     ///< ST3215 Present Voltage 原始值。
    uint8_t temperature; ///< 摄氏温度。
    // 当前舵机上电会话中的硬件多圈坐标；舵机独立掉电后可能退回单圈 raw。
    int32_t absolutePosition;
    // 将当前硬件坐标映射到 P4 恢复的逻辑多圈坐标：
    // logical ABS = hardware ABS + logicalOffset。
    int32_t logicalOffset;
    int16_t turnCount; ///< logical ABS 对应的诊断圈数。
    int16_t lastRawPosition; ///< 最近一次参与展开的 raw。
    bool initialized; ///< hardware ABS 展开器是否已有锚点。
    bool absoluteReliable; ///< logical ABS 是否可供运动控制。
    bool online; ///< 已通过连续成功反馈确认在线。
    uint8_t missedReadCount; ///< 连续同步读失败计数。
    uint8_t successfulReadCount; ///< 连续同步读成功计数。
    bool onlineTransitionPending; ///< 上线恢复事务尚未完成。
    // 热插拔恢复失败不会立即丢弃 pending；通信任务按间隔重试，
    // 达到上限后设置 recoveryFailed，等待下一次重新上线再清零。
    uint8_t recoveryAttemptCount; ///< 当前上线事件已尝试恢复次数。
    bool recoveryFailed; ///< 达到最大重试次数后的锁存状态。
    uint32_t nextRecoveryAttemptMs; ///< 下一次允许重试的绝对毫秒时间。
    // 舵机离线前最后一次可信的逻辑多圈位置。掉线时先冻结到这里，
    // 后台 NVS 任务随后写 Flash，避免离线通道从在线快照中消失。
    int32_t offlineLastTrustedAbs; ///< 离线前最后可信 logical ABS。
    int16_t offlineLastTrustedRaw; ///< 离线前最后可信 hardware raw。
    int16_t offlineLastTrustedTurn; ///< 离线前最后可信诊断圈数。
    uint32_t offlineLastTrustedTimestampMs; ///< 上述快照的采样时间。
    bool offlineTrustedSnapshotPending; ///< 后台 NVS 尚未消费冻结快照。
    uint32_t lastUpdate; ///< 最近一次有效反馈时间。
};

/** 一条飞特 UART 总线的唯一驱动实例。 */
class FeetechServoDriver {
public:
    /** 构造空驱动；必须先调用 begin() 绑定一条硬件 UART。 */
    FeetechServoDriver()
        : _serial(NULL),
          _writeCount(0),
          _syncReadTimeoutMs(SERVO_SYNC_READ_TIMEOUT_MS_DEFAULT),
          _currentReadDivider(0)
    {
        memset(_feedback, 0, sizeof(_feedback));
        memset(_torqueDisabled, 0, sizeof(_torqueDisabled));
        memset(_multiTurnConfigured, 0, sizeof(_multiTurnConfigured));
    }

    /** 绑定总线索引和串口引脚，不扫描舵机，也不改变舵机 EEPROM。 */
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

    /** 清除角度限制以启用 ST3215 多圈模式；会解锁并写入舵机 EEPROM。 */
    bool configureMultiTurnMode(uint8_t id)
    {
        if (!_serial || id > MAX_SERVO_ID) {
            return false;
        }

        uint8_t zeroLimits[4] = {0, 0, 0, 0};
        bool ok = true;
        ok = (_sms.unLockEprom(id) >= 0) && ok;
        delay(kMultiTurnConfigDelayMs);
        ok = (_sms.genWrite(id, SMS_STS_MIN_ANGLE_LIMIT_L, zeroLimits, sizeof(zeroLimits)) >= 0) && ok;
        delay(kMultiTurnConfigDelayMs);
        ok = (_sms.LockEprom(id) >= 0) && ok;
        delay(kMultiTurnConfigDelayMs);
        if (ok) {
            _multiTurnConfigured[id] = true;
        }
        return ok;
    }

    /** 将 logical ABS 转换为本次上电会话 hardware ABS 后加入同步写批次。 */
    bool setTarget(uint8_t id, int32_t logicalPosition, uint16_t speed, uint8_t acc)
    {
        int16_t hardwarePosition = 0;
        if (!logicalToHardwareTarget(id, logicalPosition, &hardwarePosition)) {
            return false;
        }
        queueTarget(id, hardwarePosition, speed, acc);
        return true;
    }

    /** 仅供恢复事务使用：绕过 logicalOffset，直接提交 hardware ABS。 */
    bool setHardwareTarget(uint8_t id, int32_t position, uint16_t speed, uint8_t acc)
    {
        if (position < kServoHardwareCommandMin ||
            position > kServoHardwareCommandMax) {
            return false;
        }
        queueTarget(id, (int16_t)position, speed, acc);
        return true;
    }

    /** 追加一台舵机到本总线同步写缓存；必要时自动重新使能力矩。 */
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

    /** 一次性发送当前缓存并清空批次；空批次不访问 UART。 */
    void syncWriteAll()
    {
        if (_writeCount == 0 || !_serial) {
            return;
        }
        _sms.SyncWritePosEx(_writeIDs, _writeCount, _writePos, _writeSpd, _writeAcc);
        _writeCount = 0;
    }

    /** 同步读取指定 ID 的位置/速度/负载，返回成功反馈的舵机数量。 */
    int syncReadPositions(const uint8_t* ids, uint8_t count)
    {
        if (!_serial || !ids || count == 0) {
            return 0;
        }

        const uint8_t addrPresentPosition = 56;
        // 电流只用于记录和诊断，不参与当前控制。每条总线约100 ms读取一次，
        // 其余周期维持原来的8字节反馈，避免扩大串口报文后拖慢10 ms控制链路。
        static const uint8_t kCurrentReadEverySyncCalls = 5;
        const bool includeCurrent = (_currentReadDivider == 0);
        _currentReadDivider =
            (uint8_t)((_currentReadDivider + 1U) % kCurrentReadEverySyncCalls);
        const uint8_t feedbackLen = includeCurrent ? 15 : 8;
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
                const int16_t current = includeCurrent
                    ? decodeSigned(rxBuf[13], rxBuf[14])
                    : _feedback[id].current;
                recordFeedbackSuccess(id,
                                      decodeSigned(rxBuf[0], rxBuf[1]),
                                      decodeSigned(rxBuf[2], rxBuf[3]),
                                      decodeServoLoad(rxBuf[4], rxBuf[5]),
                                      current,
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

    /** 对一台离线舵机执行低频单播探测，成功时更新上线去抖状态。 */
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
                              (int16_t)_sms.ReadCurrent(-1),
                              (uint8_t)_sms.ReadVoltage(-1),
                              (uint8_t)_sms.ReadTemper(-1));
        return true;
    }

    /** 返回 0..4095 hardware raw；非法 ID 返回 -1。 */
    int16_t getRawPosition(uint8_t id) const
    {
        if (id > MAX_SERVO_ID) return -1;
        return _feedback[id].rawPosition;
    }

    /** 返回控制使用的 logical ABS，即 hardware ABS + logicalOffset。 */
    int32_t getAbsolutePosition(uint8_t id) const
    {
        if (id > MAX_SERVO_ID) return 0;
        const ServoFeedbackInternal& fb = _feedback[id];
        return fb.absolutePosition + fb.logicalOffset;
    }

    // hardware ABS 是舵机本次上电会话直接返回的坐标，不叠加 P4 logicalOffset。
    int32_t getHardwareAbsolutePosition(uint8_t id) const
    {
        return id <= MAX_SERVO_ID ? _feedback[id].absolutePosition : 0;
    }

    /** 返回只读驱动反馈；非法 ID 返回静态零值对象。 */
    const ServoFeedbackInternal& getFeedback(uint8_t id) const
    {
        static ServoFeedbackInternal dummy;
        if (id > MAX_SERVO_ID) return dummy;
        return _feedback[id];
    }

    /** 表示通信在线，不等价于多圈可信。 */
    bool isOnline(uint8_t id) const
    {
        return id <= MAX_SERVO_ID && _feedback[id].online;
    }

    /** 表示 logical ABS 已由 NVS、New Device 或标定重新建立可信基准。 */
    bool isAbsoluteReliable(uint8_t id) const
    {
        return id <= MAX_SERVO_ID && _feedback[id].absoluteReliable;
    }

    /** 判断该舵机在掉线前是否曾建立过多圈基准。 */
    bool hadMultiTurnTrust(uint8_t id) const
    {
        if (id > MAX_SERVO_ID) {
            return false;
        }
        const ServoFeedbackInternal& fb = _feedback[id];
        return fb.initialized || fb.absoluteReliable || fb.logicalOffset != 0;
    }

    /** 判断连续读失败是否已达到离线失效门槛。 */
    bool isOfflineInvalidated(uint8_t id) const
    {
        return id <= MAX_SERVO_ID &&
               !_feedback[id].online &&
               _feedback[id].missedReadCount >= kOfflineInvalidateMisses;
    }

    /** 舵机刚恢复通信但尚未完成扭矩关闭/多圈配置/NVS 对齐。 */
    bool hasOnlineTransitionPending(uint8_t id) const
    {
        return id <= MAX_SERVO_ID &&
               _feedback[id].online &&
               _feedback[id].onlineTransitionPending;
    }

    /** 返回 hardware ABS 到 logical ABS 的平移量。 */
    int32_t getLogicalOffset(uint8_t id) const
    {
        return id <= MAX_SERVO_ID ? _feedback[id].logicalOffset : 0;
    }

    /** 根据 logicalOffset 计算本次上电会话可安全下发的 logical 范围。 */
    ServoLogicalRange getLogicalRange(uint8_t id) const
    {
        return servoLogicalRangeFromOffset(getLogicalOffset(id));
    }

    /** 判断上线恢复事务是否达到下一次重试时间。 */
    bool isRecoveryAttemptDue(uint8_t id, uint32_t nowMs) const
    {
        if (!hasOnlineTransitionPending(id) || _feedback[id].recoveryFailed) {
            return false;
        }
        return (int32_t)(nowMs - _feedback[id].nextRecoveryAttemptMs) >= 0;
    }

    bool isRecoveryFailed(uint8_t id) const
    {
        return id <= MAX_SERVO_ID && _feedback[id].recoveryFailed;
    }

    uint8_t recoveryAttemptCount(uint8_t id) const
    {
        return id <= MAX_SERVO_ID ? _feedback[id].recoveryAttemptCount : 0;
    }

    // 只有扭矩关闭和多圈模式配置都成功后才消费上线事件。失败时保留
    // pending 并延迟重试，避免一次串口瞬态错误永久跳过恢复。
    void finishRecoveryAttempt(uint8_t id, bool success, uint32_t nowMs)
    {
        if (id > MAX_SERVO_ID) {
            return;
        }
        ServoFeedbackInternal& fb = _feedback[id];
        if (success) {
            fb.onlineTransitionPending = false;
            fb.recoveryAttemptCount = 0;
            fb.recoveryFailed = false;
            fb.nextRecoveryAttemptMs = 0;
            return;
        }

        if (fb.recoveryAttemptCount < 0xFF) {
            fb.recoveryAttemptCount++;
        }
        fb.nextRecoveryAttemptMs = nowMs + kRecoveryRetryIntervalMs;
        if (fb.recoveryAttemptCount >= kRecoveryMaxAttempts) {
            fb.recoveryFailed = true;
        }
    }

    bool isMultiTurnConfigured(uint8_t id) const
    {
        return id <= MAX_SERVO_ID && _multiTurnConfigured[id];
    }

    /** 开关单台舵机扭矩，并同步维护驱动侧扭矩缓存。 */
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

    // 重置一台 ST3215 的内部圈数，并在保持力矩关闭的前提下重新建立 turn0。
    // RESET 绝不使用广播 ID；复位后重新配置多圈模式，防止状态重置同时恢复角度限位。
    bool resetTurnCounter(uint8_t id, int16_t* outRaw)
    {
        if (!_serial || id > MAX_SERVO_ID || !outRaw ||
            !_feedback[id].online) {
            return false;
        }
        if (!setTorqueEnabled(id, false)) {
            return false;
        }
        if (_sms.Reset(id) <= 0) {
            return false;
        }
        delay(kMultiTurnConfigDelayMs);

        ServoFeedbackInternal& fb = _feedback[id];
        fb.initialized = false;
        fb.absoluteReliable = false;
        fb.logicalOffset = 0;
        fb.turnCount = 0;
        _multiTurnConfigured[id] = false;
        if (!configureMultiTurnMode(id)) {
            return false;
        }

        // 必须用 RESET 后重新读取的反馈作为新 turn0，不能沿用命令前缓存。
        bool feedbackOk = false;
        for (uint8_t attempt = 0; attempt < 3; attempt++) {
            if (probeFeedback(id)) {
                feedbackOk = true;
                break;
            }
            delay(kMultiTurnConfigDelayMs);
        }
        if (!feedbackOk) {
            return false;
        }

        const int16_t raw = normalizeServoRawSingleTurn(fb.rawPosition);
        fb.rawPosition = raw;
        fb.lastRawPosition = raw;
        fb.absolutePosition = raw;
        fb.logicalOffset = 0;
        fb.turnCount = 0;
        fb.initialized = true;
        fb.absoluteReliable = true;
        fb.onlineTransitionPending = false;
        fb.recoveryAttemptCount = 0;
        fb.recoveryFailed = false;
        fb.nextRecoveryAttemptMs = 0;
        fb.lastUpdate = millis();
        *outRaw = raw;
        return true;
    }

    /** 使用 NVS logical ABS 和当前 raw 建立 logicalOffset；raw 差值超限则拒绝。 */
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

        const int32_t restoredLogicalAbs = absPosition;
        // New Device Reset 会把当前单圈 raw 作为 turn0。若仅舵机掉电而
        // P4 持续运行，旧的软件多圈锚点已经失效，这里禁止继续复用。
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

    // 自动标定已经重新建立机械端点后，当前 power-session ABS 可作为新的
    // 可信基准。这里不平移坐标，也不修改 logicalOffset，只闭合驱动内部
    // absoluteReliable 与共享 Turn Known 状态。
    bool acceptCurrentSessionAsTrusted(uint8_t id)
    {
        if (id > MAX_SERVO_ID || !_feedback[id].online ||
            !_multiTurnConfigured[id] ||
            _feedback[id].onlineTransitionPending ||
            _feedback[id].recoveryFailed) {
            return false;
        }
        ServoFeedbackInternal& fb = _feedback[id];
        fb.initialized = true;
        fb.absoluteReliable = true;
        fb.lastUpdate = millis();
        return true;
    }

    // 通信任务消费一次掉线冻结快照。读取后清 pending，确保同一次掉电
    // 只产生一条 NVS 保存请求。
    bool consumeOfflineTrustedSnapshot(uint8_t id,
                                       int32_t* logicalAbs,
                                       int16_t* raw,
                                       int16_t* turn,
                                       uint32_t* timestampMs)
    {
        if (id > MAX_SERVO_ID || !logicalAbs || !raw || !turn ||
            !timestampMs || !_feedback[id].offlineTrustedSnapshotPending) {
            return false;
        }
        ServoFeedbackInternal& fb = _feedback[id];
        *logicalAbs = fb.offlineLastTrustedAbs;
        *raw = fb.offlineLastTrustedRaw;
        *turn = fb.offlineLastTrustedTurn;
        *timestampMs = fb.offlineLastTrustedTimestampMs;
        fb.offlineTrustedSnapshotPending = false;
        return true;
    }

private:
    /** 合并一帧有效反馈，并执行上线去抖和掉电前可信快照更新。 */
    void recordFeedbackSuccess(uint8_t id,
                               int16_t rawPos,
                               int16_t speed,
                               int16_t load,
                               int16_t current,
                               uint8_t voltage,
                               uint8_t temperature)
    {
        ServoFeedbackInternal& fb = _feedback[id];
        const bool wasOnline = fb.online;
        _updateMultiTurnPosition(id, rawPos);
        fb.speed = speed;
        fb.load = load;
        fb.current = current;
        fb.voltage = voltage;
        fb.temperature = temperature;
        fb.missedReadCount = 0;
        if (fb.successfulReadCount < kOnlineConfirmReads) {
            fb.successfulReadCount++;
        }
        if (!wasOnline && fb.successfulReadCount >= kOnlineConfirmReads) {
            fb.online = true;
            fb.onlineTransitionPending = true;
            fb.recoveryAttemptCount = 0;
            fb.recoveryFailed = false;
            fb.nextRecoveryAttemptMs = 0;
            FW_LOG_INFO_PRINTF("[SERVO HOTPLUG] id=%u communication restored\r\n", (unsigned)id);
        }
        // 只有已经建立过可信多圈基准的反馈，才能作为掉电恢复锚点。
        // 初次发现的新舵机仍保持 Turn Unknown，不能把未知圈数写入 Flash。
        if (fb.absoluteReliable) {
            const int32_t logicalAbs = fb.absolutePosition + fb.logicalOffset;
            const int16_t trustedRaw =
                normalizeServoRawSingleTurn(fb.rawPosition);
            fb.offlineLastTrustedAbs = logicalAbs;
            fb.offlineLastTrustedRaw = trustedRaw;
            fb.offlineLastTrustedTurn =
                (int16_t)((logicalAbs - trustedRaw) / 4096);
            fb.offlineLastTrustedTimestampMs = millis();
        }
        fb.lastUpdate = millis();
    }

    /** 累计读失败；达到阈值时冻结最后可信位置并撤销多圈可信状态。 */
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
        if (fb.missedReadCount >= kOfflineInvalidateMisses && fb.online) {
            // 在清除 logicalOffset/turnCount 之前冻结最后可信反馈。这里只写
            // RAM，不在舵机总线任务里直接写 Flash。
            if (fb.absoluteReliable) {
                const int32_t logicalAbs =
                    fb.absolutePosition + fb.logicalOffset;
                const int16_t trustedRaw =
                    normalizeServoRawSingleTurn(fb.rawPosition);
                fb.offlineLastTrustedAbs = logicalAbs;
                fb.offlineLastTrustedRaw = trustedRaw;
                fb.offlineLastTrustedTurn =
                    (int16_t)((logicalAbs - trustedRaw) / 4096);
                fb.offlineLastTrustedTimestampMs = fb.lastUpdate;
                fb.offlineTrustedSnapshotPending = true;
            }
            fb.online = false;
            fb.onlineTransitionPending = false;
            fb.recoveryAttemptCount = 0;
            fb.recoveryFailed = false;
            fb.nextRecoveryAttemptMs = 0;
            _multiTurnConfigured[id] = false;
            invalidateMultiTurnTrust(id);
        }
    }

    /** 清除本次上电会话的软件多圈锚点，但保留最后掉线快照。 */
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

    /** 把业务 logical ABS 反算为 ST3215 可接受的 int16 hardware ABS。 */
    bool logicalToHardwareTarget(uint8_t id,
                                 int32_t logicalPosition,
                                 int16_t* outHardwarePosition) const
    {
        if (id > MAX_SERVO_ID || !outHardwarePosition) {
            return false;
        }
        const ServoFeedbackInternal& fb = _feedback[id];
        return servoLogicalToHardwareAbs(logicalPosition,
                                         fb.logicalOffset,
                                         outHardwarePosition);
    }

    /** 将单圈 raw 展开到最接近 anchor 的多圈坐标，避免跨零点跳变。 */
    static int32_t expandSingleTurnRawNearAnchor(int16_t rawSingleTurn, int32_t anchor)
    {
        int32_t candidate = rawSingleTurn;
        while ((candidate - anchor) > 2048) {
            candidate -= 4096;
        }
        while ((anchor - candidate) > 2048) {
            candidate += 4096;
        }
        return candidate;
    }

    /** 解码飞特“最高位为方向、其余位为幅值”的通用有符号字段。 */
    static int16_t decodeSigned(uint8_t lo, uint8_t hi)
    {
        const uint16_t raw = ((uint16_t)hi << 8) | lo;
        return (raw & (1 << 15)) ? -(int16_t)(raw & ~(1 << 15)) : (int16_t)raw;
    }

    /** 解码 ST3215 10 bit 幅值 + bit10 方向的负载字段。 */
    static int16_t decodeServoLoad(uint8_t lo, uint8_t hi)
    {
        const uint16_t raw = ((uint16_t)hi << 8) | lo;
        const int16_t magnitude = (int16_t)(raw & 0x03FF);
        return (raw & (1 << 10)) ? (int16_t)-magnitude : magnitude;
    }

    /** 用最新 raw 更新 hardware ABS，并据 logicalOffset 维护诊断圈数。 */
    void _updateMultiTurnPosition(uint8_t id, int16_t newRawPos)
    {
        ServoFeedbackInternal& fb = _feedback[id];
        int32_t absPos = 0;
        int16_t rawSingleTurn = 0;

        const bool servoProvidedMultiTurnAbs = (newRawPos < 0 || newRawPos >= 4096);
        if (servoProvidedMultiTurnAbs) {
            absPos = (int32_t)newRawPos;
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

    SMS_STS _sms; ///< 唯一允许接触的飞特 SDK 实例。
    HardwareSerial* _serial; ///< 当前驱动绑定的 UART，未 begin 时为空。
    uint8_t _writeIDs[MAX_SERVOS_PER_BUS]; ///< 同步写 ID 缓存。
    int16_t _writePos[MAX_SERVOS_PER_BUS]; ///< 同步写 hardware ABS 缓存。
    uint16_t _writeSpd[MAX_SERVOS_PER_BUS]; ///< 同步写速度缓存。
    uint8_t _writeAcc[MAX_SERVOS_PER_BUS]; ///< 同步写加速度缓存。
    uint8_t _writeCount; ///< 当前同步写缓存中的有效命令数。
    ServoFeedbackInternal _feedback[MAX_SERVO_ID + 1]; ///< 按物理 ID 索引的驱动私有状态。
    bool _torqueDisabled[MAX_SERVO_ID + 1]; ///< 最近一次成功设置的扭矩关闭状态。
    bool _multiTurnConfigured[MAX_SERVO_ID + 1]; ///< 本次上线是否已完成多圈配置。
    uint16_t _syncReadTimeoutMs; ///< 单次同步读响应时限。
    uint8_t _currentReadDivider; ///< 降频读取 Present Current 的分频计数器。
};



#endif // P4_FEETECH_SERVO_DRIVER_H
