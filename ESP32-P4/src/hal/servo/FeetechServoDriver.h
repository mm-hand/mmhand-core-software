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
#include "FTServo_Arduino/src/SMS_STS.h"
#include "../../config/safety/ServoPositionRange.h"
#include "../../config/BoardConfig.h"
#include "../../config/ServoConfig.h"
#include "../../config/topology/HardwareMap.h"

using mmhand::config::servo::kMultiTurnConfigDelayMs;
using mmhand::config::servo::kOfflineInvalidateMisses;
using mmhand::config::servo::kOnlineConfirmReads;
using mmhand::config::servo::kRecoveryMaxAttempts;
using mmhand::config::servo::kRecoveryRetryIntervalMs;

static_assert(MAX_SERVOS_PER_BUS * (15U + 6U) <=
                  SCS::kSyncReadRxBufferCapacity,
              "Feetech sync-read buffer is smaller than one full BUS response");

inline int16_t normalizeServoRawSingleTurn(int32_t value)
{
    return mmhand::servo::normalizeRawSingleTurn(value);
}

enum class ServoOnlineCheck : uint8_t { Pending, Ready, UnsupportedFirmware, VersionReadFailed, ConfigurationFailed };

/** 单台舵机的驱动内部反馈状态；不得直接暴露为跨任务共享可写数据。 */
struct ServoFeedbackInternal {
    uint8_t firmwareMajor, firmwareMinor;
    ServoOnlineCheck onlineCheck;
    int16_t speed;       ///< ST3215 Present Speed 原始值。
    int16_t load;        ///< ST3215 Present Load 有符号值。
    int16_t current;     ///< ST3215 Present Current 原始值。
    uint8_t voltage;     ///< ST3215 Present Voltage 原始值。
    uint8_t temperature; ///< 摄氏温度。
    // 电机直接报告的有符号原生ABS，不按数值范围切换解释。
    int32_t absolutePosition;
    bool absoluteReliable; ///< logical ABS 是否可供运动控制。
    bool online; ///< 已通过连续成功反馈确认在线。
    uint32_t onlineSessionGeneration; ///< 每次确认重新上线递增，用于识别跨掉线的未决置数。
    uint8_t missedReadCount; ///< 连续同步读失败计数。
    uint8_t successfulReadCount; ///< 连续同步读成功计数。
    bool onlineTransitionPending; ///< 上线恢复事务尚未完成。
    // 热插拔恢复失败不会立即丢弃 pending；通信任务按间隔重试，
    // 达到上限后设置 recoveryFailed，等待下一次重新上线再清零。
    uint8_t recoveryAttemptCount; ///< 当前上线事件已尝试恢复次数。
    bool recoveryFailed; ///< 达到最大重试次数后的锁存状态。
    uint32_t nextRecoveryAttemptMs; ///< 下一次允许重试的绝对毫秒时间。
    // 舵机离线前最后一次可信的逻辑多圈位置。掉线时先冻结到这里，
    // 供本次 P4 会话内的热插拔恢复使用；跨重启恢复由桌面端 JSON 负责。
    int32_t offlineLastTrustedAbs; ///< 离线前最后可信 logical ABS。
    bool offlineTrustedSnapshotPending; ///< ServoBus 尚未消费的本次会话冻结快照。
    uint32_t feedbackSequence; ///< 每份真实反馈递增，恢复只接受写入后的新样本。
    bool restorePending; ///< 已发送过0x0B，结果尚未由原生反馈确认。
    int32_t restoreTarget; ///< 未决写入冻结首次目标，禁止按改变后的相位重算。
    uint32_t lastUpdate; ///< 最近一次有效反馈时间。
};

/** 单条 BUS 非阻塞同步读事务状态。 */
enum class ServoSyncReadState : uint8_t {
    Idle = 0,
    Pending,
    Complete,
    TimedOut,
    ProtocolError,
    Aborted,
};

/** 一次 BUS 同步读的终态或当前进度，供四路反馈协调器统一汇总。 */
struct ServoSyncReadResult {
    ServoSyncReadState state;
    uint8_t requestedCount;
    uint8_t successCount;
    uint16_t bufferedBytes;
    uint16_t expectedBytes;
    uint32_t sequence;
    uint64_t startedUs;
    uint64_t completedUs;
};

/** 一条飞特 UART 总线的唯一驱动实例。 */
class FeetechServoDriver {
public:
    /** 构造空驱动；必须先调用 begin() 绑定一条硬件 UART。 */
    FeetechServoDriver();

    /** 绑定总线索引和串口引脚，不扫描舵机，也不改变舵机 EEPROM。 */
    void begin(uint8_t busIndex, int rxPin, int txPin, uint32_t baud);

    /**
     * 重建当前 BUS 的 UART 传输层，并撤销该 BUS 上全部旧反馈的在线/多圈可信状态。
     * HardwareSerial::end() 会删除底层 UART 驱动及事件任务，因此同时清除了
     * 残留 RX 数据和 UART 事件；重新 begin 后必须把飞特 SDK 重新绑定到该串口。
     */
    bool restartTransport()
    {
        if (!_serial || _busIndex >= NUM_BUSES || _baud == 0) {
            return false;
        }

        // 丢弃尚未完成的读写事务，禁止 UART 重启后误用旧响应或误发旧目标。
        abortSyncRead(false);
        _writeCount = 0;
        memset(_writeIDs, 0, sizeof(_writeIDs));
        memset(_writePos, 0, sizeof(_writePos));
        memset(_writeSpd, 0, sizeof(_writeSpd));
        memset(_writeAcc, 0, sizeof(_writeAcc));
        // UART 会话已经中断，旧反馈即使数值看似正常也不能继续作为在线证据。
        // 冻结锚点后统一等待连续新反馈和恢复事务重新建立可信坐标。
        for (uint8_t id = 0; id <= MAX_SERVO_ID; ++id) {
            if (_feedback[id].online || _feedback[id].absoluteReliable) {
                invalidateFeedbackSession(id);
            }
        }
        // flush(false) 等待硬件 TX 空闲并清除底层 RX FIFO；end() 随后删除
        // Arduino UART 事件任务，保证旧响应和旧事件都不会进入新会话。
        _serial->flush(false);
        _sms.pSerial = NULL;
        _serial->end();
        _serial->setRxBufferSize(kServoBusUartRxBufferBytes);
        _serial->begin(_baud, SERIAL_8N1, _rxPin, _txPin);
        _sms.pSerial = _serial;
        return true;
    }

    /** 每次上线统一核验版本和配置，配置不符才写EEPROM。 */
    bool checkOnlineConfiguration(uint8_t id);

    /** 检查实际 ABS 的范围和信任状态后加入同步写批次。 */
    bool setTarget(uint8_t id, int32_t logicalPosition, uint16_t speed, uint8_t acc);

    /** 追加一台舵机到本总线同步写缓存；必要时自动重新使能力矩。 */
    bool queueTarget(uint8_t id, int16_t position, uint16_t speed, uint8_t acc)
    {
        if (id > MAX_SERVO_ID || _writeCount >= MAX_SERVOS_PER_BUS) {
            return false;
        }
        if (_torqueDisabled[id]) {
            // 单台舵机偶发丢一次 ACK 不应否决整批目标；确认成功后才继续。
            bool enabled = false;
            for (uint8_t attempt = 0; attempt < 3 && !enabled; ++attempt) {
                if (attempt != 0) delay(kMultiTurnConfigDelayMs);
                enabled = setTorqueEnabled(id, true);
            }
            if (!enabled) return false;
        }

        _writeIDs[_writeCount] = id;
        _writePos[_writeCount] = position;
        _writeSpd[_writeCount] = speed;
        _writeAcc[_writeCount] = acc;
        _writeCount++;
        return true;
    }

    /** 放弃尚未同步写出的目标；用于整批事务异常时保证不会只发送半批。 */
    void clearQueuedTargets() { _writeCount = 0; }

    /** 一次性发送当前缓存并清空批次；空批次不访问 UART。 */
    void syncWriteAll();

    /**
     * 启动一次非阻塞同步读，只向当前 UART 发送广播请求。
     * 四条 BUS 依次调用本函数后由硬件并行接收，调用方稍后统一 collect。
     */
    bool beginSyncRead(const uint8_t* ids,
                       uint8_t count,
                       uint32_t sequence,
                       uint64_t nowUs)
    {
        if (!_serial || !ids || count == 0 ||
            count > MAX_SERVOS_PER_BUS || _syncReadActive) {
            return false;
        }

        const uint8_t addrPresentPosition = 56;
        // 位置反馈提升到100 Hz后使用10分频，保持电流约10 Hz，避免扩大总线占用。
        const bool includeCurrent = (_currentReadDivider == 0);
        _currentReadDivider =
            (uint8_t)((_currentReadDivider + 1U) %
                      mmhand::config::servo::kServoCurrentReadDivider);
        const uint8_t feedbackLen = includeCurrent ? 15 : 8;
        memcpy(_syncReadIds, ids, count);
        _syncReadCount = count;
        _syncReadFeedbackLen = feedbackLen;
        _syncReadIncludeCurrent = includeCurrent;
        _syncReadSequence = sequence;
        _syncReadStartedUs = nowUs;
        _syncReadDeadlineUs =
            nowUs + mmhand::config::servo::kServoFeedbackDeadlineUs;

        // 正常链路使用 TxOnly + CollectAvailable；SDK 阻塞超时在此不参与调度。
        _sms.syncReadBegin(count, feedbackLen, 0);
        const int txResult = _sms.syncReadPacketTxOnly(
            _syncReadIds, count, addrPresentPosition, feedbackLen);
        if (txResult <= 0) {
            for (uint8_t i = 0; i < count; i++) {
                if (_syncReadIds[i] <= MAX_SERVO_ID) {
                    markReadFailure(_syncReadIds[i]);
                }
            }
            _sms.syncReadEnd();
            _syncReadCount = 0;
            return false;
        }
        _syncReadActive = true;
        return true;
    }

    /** 搬运当前已到达字节；完整到包或截止后才解析并更新反馈可信状态。 */
    ServoSyncReadResult collectSyncRead(uint64_t nowUs);

    /** 取消当前事务；E-STOP/UART重启不把主动取消累计为舵机读失败。 */
    void abortSyncRead(bool markFailure);

    // Maintenance uses raw addresses, independent of product Mxx feedback arrays.
    bool probeMaintenanceId(uint8_t id) {
        if (!_serial || id >= 254) return false;
        const auto saved = _sms.IOTimeOut;
        _sms.IOTimeOut = 3;
        // A fault-status reply still occupies this address.
        const bool found = _sms.Ping(id) == id;
        _sms.IOTimeOut = saved;
        return found;
    }

    bool maintenanceIdLocked(uint8_t id) {
        if (!_serial || id >= 254) return false;
        const auto saved=_sms.IOTimeOut; _sms.IOTimeOut=3;
        const bool locked=_sms.readByte(id,SMS_STS_LOCK)==1;
        _sms.IOTimeOut=saved; return locked;
    }

    void discardMaintenanceState(uint8_t id) {
        if(id>MAX_SERVO_ID) return;
        _feedback[id]=ServoFeedbackInternal{};
        _torqueDisabled[id]=false; _multiTurnConfigured[id]=false;
    }

    uint8_t renameMaintenanceId(uint8_t oldId, uint8_t newId, uint8_t* stage) {
        if (!_serial || oldId >= 254 || newId >= 254 || oldId == newId) return 3;
        const auto saved = _sms.IOTimeOut;
        _sms.IOTimeOut = 20;
        *stage = 1;
        if (_sms.readByte(oldId, SMS_STS_ID) != oldId || _sms.getState() || probeMaintenanceId(newId)) {
            _sms.IOTimeOut = saved; return 3;
        }
        *stage = 2;
        _sms.EnableTorque(oldId, 0);
        if (_sms.readByte(oldId, SMS_STS_TORQUE_ENABLE) != 0) {
            _sms.IOTimeOut = saved; return 4;
        }
        _sms.unLockEprom(oldId);
        if (_sms.readByte(oldId, SMS_STS_LOCK) != 0) {
            _sms.LockEprom(oldId); _sms.IOTimeOut = saved; return 4;
        }
        *stage = 3;
        // ID-write ACK may carry either address; readback determines the outcome.
        _sms.writeByte(oldId, SMS_STS_ID, newId);
        delay(10);
        const int newValue = _sms.readByte(newId, SMS_STS_ID);
        const int oldValue = _sms.readByte(oldId, SMS_STS_ID);
        *stage = 4;
        bool locked = false;
        if (newValue == newId && oldValue < 0) {
            locked = _sms.LockEprom(newId) >= 0 && _sms.getState() == 0;
        } else if (oldValue == oldId && newValue < 0) {
            _sms.LockEprom(oldId);
        }
        *stage = 5;
        const bool verified = locked && _sms.readByte(newId, SMS_STS_ID) == newId
                              && _sms.readByte(newId, SMS_STS_LOCK) == 1;
        _sms.IOTimeOut = saved;
        return verified ? 0 : 5;
    }

    bool syncReadActive() const { return _syncReadActive; }

    uint64_t syncReadDeadlineUs() const { return _syncReadDeadlineUs; }

    /** 对一台离线舵机执行低频单播探测，成功时更新上线去抖状态。 */
    bool probeFeedback(uint8_t id,
                       uint16_t timeoutMs = SERVO_PROBE_TIMEOUT_MS)
    {
        if (!_serial || id > MAX_SERVO_ID) {
            return false;
        }
        // 扫描阶段临时缩短单播超时，避免一个不存在的 ID 长时间阻塞四条 BUS。
        const unsigned long previousTimeout = _sms.IOTimeOut;
        _sms.IOTimeOut = timeoutMs == 0 ? 1 : timeoutMs;
        const int feedbackResult = _sms.FeedBack(id);
        _sms.IOTimeOut = previousTimeout;
        if (feedbackResult < 0 || _sms.getState() != 0) {
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

    /** 返回未经补圈的原生ABS；坐标基准是否可信由isAbsoluteReliable单独表示。 */
    int32_t getAbsolutePosition(uint8_t id) const
    {
        if (id > MAX_SERVO_ID) return 0;
        return _feedback[id].absolutePosition;
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

    /** 表示 logical ABS 已由桌面 JSON、ABS设零或标定重新建立可信基准。 */
    bool isAbsoluteReliable(uint8_t id) const
    {
        return id <= MAX_SERVO_ID && _feedback[id].absoluteReliable;
    }

    /** 判断连续读失败是否已达到离线失效门槛。 */
    bool isOfflineInvalidated(uint8_t id) const
    {
        return id <= MAX_SERVO_ID &&
               !_feedback[id].online &&
               _feedback[id].missedReadCount >= kOfflineInvalidateMisses;
    }

    /**
     * 反馈时间超过上限时强制判离线。
     *
     * 这是连续读失败之外的独立兜底，专门覆盖路由扫描占用 UART、任务长间隔
     * 等没有机会调用 markReadFailure() 的场景。返回 true 表示本次发生了边沿。
     */
    bool invalidateIfFeedbackStale(uint8_t id,
                                   uint32_t nowMs,
                                   uint32_t timeoutMs)
    {
        if (id > MAX_SERVO_ID || timeoutMs == 0) {
            return false;
        }
        ServoFeedbackInternal& fb = _feedback[id];
        if (!fb.online || fb.lastUpdate == 0) {
            return false;
        }
        const uint32_t ageMs = nowMs - fb.lastUpdate;
        // 调用方可能在本周期开始时采样 nowMs，而反馈在本周期稍后更新。
        // 此时 ageMs 会发生无符号下溢；把半圈时间范围外的差值视为“未来时间戳”，
        // 禁止将刚收到的新反馈误判为长期陈旧。
        if ((int32_t)ageMs < 0 || ageMs < timeoutMs) {
            return false;
        }
        invalidateFeedbackSession(id);
        return true;
    }

    /** 舵机刚恢复通信但尚未完成扭矩关闭、多圈配置和 JSON/RAM 锚点对齐。 */
    bool hasOnlineTransitionPending(uint8_t id) const
    {
        return id <= MAX_SERVO_ID &&
               _feedback[id].online &&
               _feedback[id].onlineTransitionPending;
    }

    /** 返回舵机实际 ABS 可安全下发的范围，可信/在线状态由发布者核验。 */
    ServoLogicalRange getLogicalRange(uint8_t id) const
    {
        (void)id;
        return servoPositionRange();
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
        if (fb.onlineCheck == ServoOnlineCheck::UnsupportedFirmware) fb.recoveryFailed = true;
        if (fb.recoveryAttemptCount >= kRecoveryMaxAttempts) {
            fb.recoveryFailed = true;
        }
    }

    bool isMultiTurnConfigured(uint8_t id) const
    {
        return id <= MAX_SERVO_ID && _multiTurnConfigured[id];
    }

    bool isTorqueDisabled(uint8_t id) const
    {
        return id <= MAX_SERVO_ID && _torqueDisabled[id];
    }

    /** 开关单台舵机扭矩，并同步维护驱动侧扭矩缓存。 */
    bool setTorqueEnabled(uint8_t id, bool enabled)
    {
        if (!_serial || id > MAX_SERVO_ID) {
            return false;
        }
        const bool ok = (_sms.EnableTorque(id, enabled ? 1 : 0) > 0 && _sms.getState() == 0);
        if (ok) {
            _torqueDisabled[id] = !enabled;
        }
        return ok;
    }

    /** 只发送一次位置校正；结果由下一轮正常原生反馈验收。 */
    bool beginAbsoluteRestore(uint8_t id, int32_t position)
    {
        if (!_serial || id > MAX_SERVO_ID || !_feedback[id].online ||
            !_multiTurnConfigured[id] || position < kServoHardwareCommandMin ||
            position > kServoHardwareCommandMax) return false;
        ServoFeedbackInternal& fb = _feedback[id];
        if (fb.restorePending) return true; // 保留第一次写入，绝不重算或重发。
        abortSyncRead(false);
        clearQueuedTargets();
        if (!setTorqueEnabled(id, false)) return false;
        fb.absoluteReliable = false;
        fb.offlineTrustedSnapshotPending = false;
        fb.restoreTarget = position;
        fb.restorePending = true;
        // 丢ACK可能发生在写入之后；后续同样等待新的原生反馈，不重复置数。
        const bool acknowledged = _sms.CalibratePosition(id, (int16_t)position) > 0;
        if (!acknowledged && _sms.getLastError() == 0) {
            // 电机明确返回错误，与应答丢失不同。
            fb.restorePending = false;
            return false;
        }
        return true;
    }

    /** 完整ABS必须匹配；只容忍关扭矩回弹的少量计数，禁止取模/整圈兼容。 */
    bool confirmAbsoluteRestore(uint8_t id, uint32_t baselineSequence)
    {
        if (id > MAX_SERVO_ID) return false;
        ServoFeedbackInternal& fb = _feedback[id];
        if (!fb.restorePending || !fb.online || !_multiTurnConfigured[id] ||
            fb.feedbackSequence == baselineSequence) return false;
        // 0x0B 写入后偶尔回弹 1–3 个计数（约 0.09°–0.26°）；整圈差是 4096，
        // 远大于该容差，所以仍不会放过单圈相位或差整圈的值。
        constexpr int32_t kRestoreToleranceCounts = 4;
        const int32_t restoreError = fb.absolutePosition - fb.restoreTarget;
        if (restoreError < -kRestoreToleranceCounts ||
            restoreError > kRestoreToleranceCounts) return false;
        fb.restorePending = false;
        fb.absoluteReliable = true;
        fb.offlineLastTrustedAbs = fb.absolutePosition;

        finishRecoveryAttempt(id, true, millis());
        return true;
    }

    /** 显式归零会建立新基准，必须先撤销旧未决恢复。 */
    void discardAbsoluteRestore(uint8_t id)
    {
        if (id <= MAX_SERVO_ID) {
            _feedback[id].restorePending = false;
            _feedback[id].offlineTrustedSnapshotPending = false;
            _feedback[id].absoluteReliable = false;
        }
    }

    // ServoBus 消费一次掉线冻结快照。读取后清 pending，确保同一次掉电
    // 仅建立一份会话内热插拔恢复锚点。
    bool consumeOfflineTrustedSnapshot(uint8_t id,
                                       int32_t* logicalAbs)
    {
        if (id > MAX_SERVO_ID || !logicalAbs || !_feedback[id].offlineTrustedSnapshotPending) {
            return false;
        }
        ServoFeedbackInternal& fb = _feedback[id];
        *logicalAbs = fb.offlineLastTrustedAbs;
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
        fb.absolutePosition = rawPos;

        ++fb.feedbackSequence;
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
            ++fb.onlineSessionGeneration;
            fb.online = true;
            fb.onlineTransitionPending = true;
            fb.recoveryAttemptCount = 0;
            fb.recoveryFailed = false;
            fb.nextRecoveryAttemptMs = 0;
        }
        // 只有已经建立过可信多圈基准的反馈，才能作为掉电恢复锚点。
        // 初次发现的新舵机仍保持 Turn Unknown，不能把未知圈数写入 Flash。
        if (fb.absoluteReliable) {
            const int32_t logicalAbs = fb.absolutePosition;
            fb.offlineLastTrustedAbs = logicalAbs;

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
            invalidateFeedbackSession(id);
        }
    }

    /** 冻结最后可信位置，并统一撤销在线、多圈配置及逻辑 ABS 信任。 */
    void invalidateFeedbackSession(uint8_t id)
    {
        if (id > MAX_SERVO_ID) {
            return;
        }
        ServoFeedbackInternal& fb = _feedback[id];
        // 在清除多圈信任之前冻结最后可信反馈。这里只写
        // RAM，不在舵机总线任务里直接写 Flash。
        if (fb.absoluteReliable) {
            const int32_t logicalAbs = fb.absolutePosition;
            fb.offlineLastTrustedAbs = logicalAbs;

            fb.offlineTrustedSnapshotPending = true;
        }
        fb.online = false;
        fb.successfulReadCount = 0;
        fb.missedReadCount = kOfflineInvalidateMisses;
        fb.onlineTransitionPending = false;
        fb.recoveryAttemptCount = 0;
        fb.recoveryFailed = false;
        fb.nextRecoveryAttemptMs = 0;
        _multiTurnConfigured[id] = false;
        fb.onlineCheck = ServoOnlineCheck::Pending;
        fb.firmwareMajor = fb.firmwareMinor = 0;
        invalidateMultiTurnTrust(id);
    }

    /** 清除本次上电会话的软件多圈锚点，但保留最后掉线快照。 */
    void invalidateMultiTurnTrust(uint8_t id)
    {
        if (id > MAX_SERVO_ID) {
            return;
        }
        ServoFeedbackInternal& fb = _feedback[id];
        // 保留原生反馈用于诊断，撤销其作为标定坐标的授权。
        fb.absoluteReliable = false;

    }

    /** 在线且坐标可信时，检查实际 ABS 的硬件范围后生成命令。 */
    bool validateAbsoluteTarget(uint8_t id,
                                 int32_t logicalPosition,
                                 int16_t* outHardwarePosition) const
    {
        if (id > MAX_SERVO_ID || !outHardwarePosition ||
            !_feedback[id].online || !_feedback[id].absoluteReliable || !_multiTurnConfigured[id]) {
            return false;
        }
        return servoHardwareCommandFromAbs(logicalPosition, outHardwarePosition);
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

    SMS_STS _sms; ///< 唯一允许接触的飞特 SDK 实例。
    HardwareSerial* _serial; ///< 当前驱动绑定的 UART，未 begin 时为空。
    uint8_t _busIndex; ///< 当前 HardwareSerial 对应的物理 BUS 编号。
    int _rxPin; ///< restartTransport() 复用的 RX 引脚。
    int _txPin; ///< restartTransport() 复用的 TX 引脚。
    uint32_t _baud; ///< restartTransport() 复用的总线波特率。
    uint8_t _writeIDs[MAX_SERVOS_PER_BUS]; ///< 同步写 ID 缓存。
    int16_t _writePos[MAX_SERVOS_PER_BUS]; ///< 同步写 hardware ABS 缓存。
    uint16_t _writeSpd[MAX_SERVOS_PER_BUS]; ///< 同步写速度缓存。
    uint8_t _writeAcc[MAX_SERVOS_PER_BUS]; ///< 同步写加速度缓存。
    uint8_t _writeCount; ///< 当前同步写缓存中的有效命令数。
    ServoFeedbackInternal _feedback[MAX_SERVO_ID + 1]; ///< 按物理 ID 索引的驱动私有状态。
    bool _torqueDisabled[MAX_SERVO_ID + 1]; ///< 最近一次成功设置的扭矩关闭状态。
    bool _multiTurnConfigured[MAX_SERVO_ID + 1]; ///< 本次上线是否已完成多圈配置。
    uint8_t _currentReadDivider; ///< 降频读取 Present Current 的分频计数器。
    bool _syncReadActive; ///< 当前 BUS 是否正在等待同步读回包。
    bool _syncReadIncludeCurrent; ///< 本轮是否读取扩展15字节遥测。
    uint8_t _syncReadIds[MAX_SERVOS_PER_BUS]; ///< 本轮请求的物理ID。
    uint8_t _syncReadCount; ///< 本轮请求ID数量。
    uint8_t _syncReadFeedbackLen; ///< 单台舵机期望反馈数据长度。
    uint32_t _syncReadSequence; ///< 四路统一反馈轮次。
    uint64_t _syncReadStartedUs; ///< 请求开始时间。
    uint64_t _syncReadDeadlineUs; ///< 最晚收集时间。

};


#endif // P4_FEETECH_SERVO_DRIVER_H
