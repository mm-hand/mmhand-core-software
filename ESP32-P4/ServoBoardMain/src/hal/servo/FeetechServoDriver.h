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
using mmhand::config::servo::kResetZeroFeedbackAttempts;
using mmhand::config::servo::kResetZeroTransactionAttempts;

static_assert(MAX_SERVOS_PER_BUS * (15U + 6U) <=
                  SCS::kSyncReadRxBufferCapacity,
              "Feetech sync-read buffer is smaller than one full BUS response");

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
    int16_t turnCount; ///< 实际 ABS 对应的诊断圈数。
    int16_t lastRawPosition; ///< 最近一次参与展开的 raw。
    bool initialized; ///< hardware ABS 展开器是否已有锚点。
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
    int16_t offlineLastTrustedRaw; ///< 离线前最后可信 hardware raw。
    int16_t offlineLastTrustedTurn; ///< 离线前最后可信诊断圈数。
    uint32_t offlineLastTrustedTimestampMs; ///< 上述快照的采样时间。
    bool offlineTrustedSnapshotPending; ///< ServoBus 尚未消费的本次会话冻结快照。
    uint32_t lastUpdate; ///< 最近一次有效反馈时间。
};

/** 多圈模式按需配置事务的单步结果；Pending 表示等待下一次 10 ms 服务周期继续。 */
enum class ServoMultiTurnConfigResult : uint8_t {
    Pending = 0,
    Complete = 1,
    Failed = 2,
};

/** 多圈配置内部阶段；每次 step 最多执行一笔飞特协议事务，禁止在任务内 delay。 */
enum class ServoMultiTurnConfigPhase : uint8_t {
    Idle = 0,
    VerifyLimits,
    UnlockEeprom,
    WriteZeroLimits,
    LockEeprom,
    Finalize,
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
    FeetechServoDriver()
        : _serial(NULL),
          _busIndex(0xFF),
          _rxPin(-1),
          _txPin(-1),
          _baud(0),
          _writeCount(0),
          _currentReadDivider(0),
          _syncReadActive(false),
          _syncReadIncludeCurrent(false),
          _syncReadCount(0),
          _syncReadFeedbackLen(0),
          _syncReadSequence(0),
          _syncReadStartedUs(0),
          _syncReadDeadlineUs(0),
          _multiTurnConfigId(0xFF),
          _multiTurnConfigPhase(ServoMultiTurnConfigPhase::Idle),
          _multiTurnConfigNotBeforeMs(0),
          _multiTurnConfigFailed(false)
    {
        memset(_feedback, 0, sizeof(_feedback));
        memset(_torqueDisabled, 0, sizeof(_torqueDisabled));
        memset(_multiTurnConfigured, 0, sizeof(_multiTurnConfigured));
        memset(_syncReadIds, 0, sizeof(_syncReadIds));
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
        _busIndex = busIndex;
        _rxPin = rxPin;
        _txPin = txPin;
        _baud = baud;
        // 必须在 begin() 前扩大 Arduino UART RX ring；不设置 TX ring，命令立即下发。
        _serial->setRxBufferSize(kServoBusUartRxBufferBytes);
        _serial->begin(baud, SERIAL_8N1, rxPin, txPin);
        _sms.pSerial = _serial;
    }

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
        resetMultiTurnConfigurationJob();
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

    /**
     * 按需推进一台舵机的多圈配置。
     *
     * 首先读取 9..12 号角度限位寄存器；四字节已经为 0 时只登记本次会话已验证，
     * 不再重复擦写 EEPROM。只有配置确实不符时，才把解锁、写入、加锁和三段等待
     * 分散到后续服务周期，消除 27 台舵机集中配置造成的数百毫秒阻塞。
     */
    ServoMultiTurnConfigResult stepMultiTurnConfiguration(uint8_t id,
                                                           uint32_t nowMs)
    {
        if (!_serial || id > MAX_SERVO_ID) {
            return ServoMultiTurnConfigResult::Failed;
        }
        if (_multiTurnConfigured[id]) {
            return ServoMultiTurnConfigResult::Complete;
        }
        if (_multiTurnConfigId == 0xFF) {
            _multiTurnConfigId = id;
            _multiTurnConfigPhase = ServoMultiTurnConfigPhase::VerifyLimits;
            _multiTurnConfigNotBeforeMs = 0;
            _multiTurnConfigFailed = false;
        } else if (_multiTurnConfigId != id) {
            // 一条物理 BUS 同时只允许一个 EEPROM 事务；调用方下周期继续当前舵机。
            return ServoMultiTurnConfigResult::Pending;
        }
        if ((int32_t)(nowMs - _multiTurnConfigNotBeforeMs) < 0) {
            return ServoMultiTurnConfigResult::Pending;
        }

        switch (_multiTurnConfigPhase) {
            case ServoMultiTurnConfigPhase::VerifyLimits: {
                uint8_t limits[4] = {0, 0, 0, 0};
                if (_sms.Read(id,
                              SMS_STS_MIN_ANGLE_LIMIT_L,
                              limits,
                              sizeof(limits)) != (int)sizeof(limits)) {
                    return failMultiTurnConfiguration();
                }
                if (limits[0] == 0 && limits[1] == 0 &&
                    limits[2] == 0 && limits[3] == 0) {
                    _multiTurnConfigured[id] = true;
                    resetMultiTurnConfigurationJob();
                    return ServoMultiTurnConfigResult::Complete;
                }
                _multiTurnConfigPhase =
                    ServoMultiTurnConfigPhase::UnlockEeprom;
                return ServoMultiTurnConfigResult::Pending;
            }
            case ServoMultiTurnConfigPhase::UnlockEeprom:
                if (_sms.unLockEprom(id) < 0) {
                    return failMultiTurnConfiguration();
                }
                _multiTurnConfigPhase =
                    ServoMultiTurnConfigPhase::WriteZeroLimits;
                _multiTurnConfigNotBeforeMs = nowMs + kMultiTurnConfigDelayMs;
                return ServoMultiTurnConfigResult::Pending;

            case ServoMultiTurnConfigPhase::WriteZeroLimits: {
                uint8_t zeroLimits[4] = {0, 0, 0, 0};
                if (_sms.genWrite(id,
                                  SMS_STS_MIN_ANGLE_LIMIT_L,
                                  zeroLimits,
                                  sizeof(zeroLimits)) < 0) {
                    // EEPROM 已经解锁，即使写入失败也必须继续执行 LockEeprom。
                    _multiTurnConfigFailed = true;
                }
                _multiTurnConfigPhase = ServoMultiTurnConfigPhase::LockEeprom;
                _multiTurnConfigNotBeforeMs = nowMs + kMultiTurnConfigDelayMs;
                return ServoMultiTurnConfigResult::Pending;
            }
            case ServoMultiTurnConfigPhase::LockEeprom:
                if (_sms.LockEprom(id) < 0) {
                    return failMultiTurnConfiguration();
                }
                _multiTurnConfigPhase = ServoMultiTurnConfigPhase::Finalize;
                _multiTurnConfigNotBeforeMs = nowMs + kMultiTurnConfigDelayMs;
                return ServoMultiTurnConfigResult::Pending;

            case ServoMultiTurnConfigPhase::Finalize:
                if (_multiTurnConfigFailed) {
                    return failMultiTurnConfiguration();
                }
                _multiTurnConfigured[id] = true;
                resetMultiTurnConfigurationJob();
                return ServoMultiTurnConfigResult::Complete;

            case ServoMultiTurnConfigPhase::Idle:
            default:
                return failMultiTurnConfiguration();
        }
    }

    /** 检查实际 ABS 的范围和信任状态后加入同步写批次。 */
    bool setTarget(uint8_t id, int32_t logicalPosition, uint16_t speed, uint8_t acc)
    {
        int16_t hardwarePosition = 0;
        if (!validateAbsoluteTarget(id, logicalPosition, &hardwarePosition)) {
            return false;
        }
        return queueTarget(id, hardwarePosition, speed, acc);
    }

    /** 追加一台舵机到本总线同步写缓存；必要时自动重新使能力矩。 */
    bool queueTarget(uint8_t id, int16_t position, uint16_t speed, uint8_t acc)
    {
        if (id > MAX_SERVO_ID || _writeCount >= MAX_SERVOS_PER_BUS) {
            return false;
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
        return true;
    }

    /** 放弃尚未同步写出的目标；用于整批事务异常时保证不会只发送半批。 */
    void clearQueuedTargets() { _writeCount = 0; }

    /** 一次性发送当前缓存并清空批次；空批次不访问 UART。 */
    void syncWriteAll()
    {
        if (_writeCount == 0 || !_serial) {
            return;
        }
        _sms.SyncWritePosEx(_writeIDs, _writeCount, _writePos, _writeSpd, _writeAcc);
        _writeCount = 0;
    }

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
    ServoSyncReadResult collectSyncRead(uint64_t nowUs)
    {
        ServoSyncReadResult result = {
            _syncReadActive ? ServoSyncReadState::Pending
                            : ServoSyncReadState::Idle,
            _syncReadCount,
            0,
            _sms.syncReadBufferedBytes(),
            _sms.syncReadExpectedBytes(),
            _syncReadSequence,
            _syncReadStartedUs,
            0,
        };
        if (!_syncReadActive) {
            return result;
        }

        _sms.syncReadCollectAvailable();
        result.bufferedBytes = _sms.syncReadBufferedBytes();
        result.expectedBytes = _sms.syncReadExpectedBytes();
        const bool allBytesReceived =
            result.expectedBytes != 0 &&
            result.bufferedBytes >= result.expectedBytes;
        const bool deadlineReached = nowUs >= _syncReadDeadlineUs;
        if (!allBytesReceived && !deadlineReached) {
            return result;
        }

        uint8_t successCount = 0;
        for (uint8_t i = 0; i < _syncReadCount; i++) {
            const uint8_t id = _syncReadIds[i];
            uint8_t rxBuf[15] = {0};
            const int rxLen = _sms.syncReadPacketRx(id, rxBuf);
            if (id > MAX_SERVO_ID) {
                continue;
            }
            if (rxLen == _syncReadFeedbackLen) {
                const int16_t current = _syncReadIncludeCurrent
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
        _syncReadActive = false;
        result.successCount = successCount;
        result.completedUs = nowUs;
        result.state = successCount == _syncReadCount
            ? ServoSyncReadState::Complete
            : (deadlineReached ? ServoSyncReadState::TimedOut
                               : ServoSyncReadState::ProtocolError);
        _syncReadCount = 0;
        return result;
    }

    /** 取消当前事务；E-STOP/UART重启不把主动取消累计为舵机读失败。 */
    void abortSyncRead(bool markFailure)
    {
        if (!_syncReadActive) {
            return;
        }
        if (markFailure) {
            for (uint8_t i = 0; i < _syncReadCount; ++i) {
                if (_syncReadIds[i] <= MAX_SERVO_ID) {
                    markReadFailure(_syncReadIds[i]);
                }
            }
        }
        _sms.syncReadCollectAvailable();
        _sms.syncReadEnd();
        _syncReadActive = false;
        _syncReadCount = 0;
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
        if (feedbackResult < 0) {
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

    /** 返回控制使用的实际 ABS，与舵机反馈使用同一坐标。 */
    int32_t getAbsolutePosition(uint8_t id) const
    {
        if (id > MAX_SERVO_ID) return 0;
        return _feedback[id].absolutePosition;
    }

    // 供硬件范围诊断使用，与控制 ABS 使用同一坐标。
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

    /** 表示 logical ABS 已由桌面 JSON、Reset Zero 或标定重新建立可信基准。 */
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

    /**
     * 使用 0x0B 自定义位置校正，直接改写舵机 hardware ABS（STS 固件 >= 3.10）。
     * 单播 ACK 和校正后的新反馈均确认成功，才建立新的可信坐标。
     * 不使用 RESET 降级：不支持校正或应答不确定时保持 Unknown，禁止运动。
     */
    bool calibrateHardwareAbsolutePosition(uint8_t id,
                                           int32_t position,
                                           int16_t* outRaw,
                                           uint8_t* outAttempts = nullptr,
                                           bool* outCorrectionAttempted = nullptr)
    {
        if (outAttempts) *outAttempts = 0;
        if (outCorrectionAttempted) *outCorrectionAttempted = false;
        if (!_serial || id > MAX_SERVO_ID || !outRaw ||
            !_feedback[id].online ||
            position < kServoHardwareCommandMin ||
            position > kServoHardwareCommandMax) {
            return false;
        }

        // 旧读事务与旧目标使用校正前坐标，不能跨越此次坐标改写。
        abortSyncRead(false);
        clearQueuedTargets();
        ServoFeedbackInternal& fb = _feedback[id];
        invalidateMultiTurnTrust(id);
        fb.offlineTrustedSnapshotPending = false;
        for (uint8_t attempt = 1;
             attempt <= kResetZeroTransactionAttempts;
             ++attempt) {
            if (outAttempts) *outAttempts = attempt;
            // SDK 的普通写接口返回 0 表示未收到 ACK，不能作为关力矩成功。
            if (_sms.EnableTorque(id, 0) <= 0 || _sms.getState() != 0) {
                delay(kMultiTurnConfigDelayMs);
                continue;
            }
            _torqueDisabled[id] = true;
            if (!_multiTurnConfigured[id]) {
                uint8_t limits[4] = {0xFF, 0xFF, 0xFF, 0xFF};
                const bool configured = configureMultiTurnMode(id) &&
                    _sms.Read(id, SMS_STS_MIN_ANGLE_LIMIT_L,
                              limits, sizeof(limits)) == (int)sizeof(limits) &&
                    _sms.getState() == 0 &&
                    limits[0] == 0 && limits[1] == 0 &&
                    limits[2] == 0 && limits[3] == 0;
                _multiTurnConfigured[id] = configured;
                if (!configured) {
                    delay(kMultiTurnConfigDelayMs);
                    continue;
                }
            }

            // 即使 ACK 丢失，舵机也可能已经改写坐标，不能继续相信旧展开锚点。
            invalidateMultiTurnTrust(id);
            if (outCorrectionAttempted) *outCorrectionAttempted = true;
            if (_sms.CalibratePosition(id, (int16_t)position) <= 0) {
                delay(kMultiTurnConfigDelayMs);
                continue;
            }
            delay(kMultiTurnConfigDelayMs);
            for (uint8_t readAttempt = 0;
                 readAttempt < kResetZeroFeedbackAttempts;
                 ++readAttempt) {
                fb.initialized = false;
                if (probeFeedback(id) && _sms.getState() == 0 &&
                    (int32_t)_sms.ReadPos(-1) == position &&
                    fb.absolutePosition == position && fb.online &&
                    fb.lastUpdate != 0) {
                    const int16_t raw = normalizeServoRawSingleTurn(position);
                    fb.turnCount = (int16_t)((position - raw) / 4096);
                    fb.absoluteReliable = true;
                    fb.onlineTransitionPending = false;
                    fb.recoveryAttemptCount = 0;
                    fb.recoveryFailed = false;
                    fb.nextRecoveryAttemptMs = 0;
                    fb.offlineLastTrustedAbs = position;
                    fb.offlineLastTrustedRaw = raw;
                    fb.offlineLastTrustedTurn = fb.turnCount;
                    fb.offlineLastTrustedTimestampMs = fb.lastUpdate;
                    fb.offlineTrustedSnapshotPending = false;
                    *outRaw = raw;
                    return true;
                }
                delay(kMultiTurnConfigDelayMs);
            }
        }
        invalidateMultiTurnTrust(id);
        fb.offlineTrustedSnapshotPending = false;
        return false;
    }

    // 重置一台 ST3215 的内部圈数，并在保持力矩关闭的前提下重新建立 turn0。
    // RESET 绝不使用广播 ID；复位后重新配置多圈模式，防止状态重置同时恢复角度限位。
    bool resetTurnCounter(uint8_t id, int16_t* outRaw,
                          bool* outResetAttempted = nullptr)
    {
        if (outResetAttempted) *outResetAttempted = false;
        if (!_serial || id > MAX_SERVO_ID || !outRaw ||
            !_feedback[id].online) {
            return false;
        }
        abortSyncRead(false);
        clearQueuedTargets();
        if (_sms.EnableTorque(id, 0) <= 0 || _sms.getState() != 0) {
            return false;
        }
        _torqueDisabled[id] = true;
        ServoFeedbackInternal& fb = _feedback[id];
        // RESET 即使没有应答也可能已生效；发送前撤销旧坐标及恢复锚点。
        invalidateMultiTurnTrust(id);
        fb.offlineTrustedSnapshotPending = false;
        _multiTurnConfigured[id] = false;
        if (outResetAttempted) *outResetAttempted = true;
        if (_sms.Reset(id) <= 0 || _sms.getState() != 0) {
            return false;
        }
        delay(kMultiTurnConfigDelayMs);

        if (!configureMultiTurnMode(id)) {
            return false;
        }

        // 必须用 RESET 后重新读取的反馈作为新 turn0，不能沿用命令前缓存。
        bool feedbackOk = false;
        for (uint8_t attempt = 0;
             attempt < kResetZeroFeedbackAttempts;
             attempt++) {
            fb.initialized = false;
            if (probeFeedback(id) && _sms.getState() == 0 &&
                fb.online && fb.lastUpdate != 0) {
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
        // 保留 RESET 后真实读回的 ABS，不能把原生多圈反馈强制改成单圈 raw。
        fb.turnCount = (int16_t)((fb.absolutePosition - raw) / 4096);
        fb.initialized = true;
        fb.absoluteReliable = true;
        fb.onlineTransitionPending = false;
        fb.recoveryAttemptCount = 0;
        fb.recoveryFailed = false;
        fb.nextRecoveryAttemptMs = 0;
        fb.lastUpdate = millis();
        fb.offlineLastTrustedAbs = fb.absolutePosition;
        fb.offlineLastTrustedRaw = raw;
        fb.offlineLastTrustedTurn = fb.turnCount;
        fb.offlineLastTrustedTimestampMs = fb.lastUpdate;
        fb.offlineTrustedSnapshotPending = false;
        *outRaw = raw;
        return true;
    }


    // ServoBus 消费一次掉线冻结快照。读取后清 pending，确保同一次掉电
    // 仅建立一份会话内热插拔恢复锚点。
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
    /** 清理总线内唯一的多圈配置事务，不修改任何舵机的已验证标志。 */
    void resetMultiTurnConfigurationJob()
    {
        _multiTurnConfigId = 0xFF;
        _multiTurnConfigPhase = ServoMultiTurnConfigPhase::Idle;
        _multiTurnConfigNotBeforeMs = 0;
        _multiTurnConfigFailed = false;
    }

    /** 统一结束失败事务，确保后续恢复重试可以从寄存器核验阶段重新开始。 */
    ServoMultiTurnConfigResult failMultiTurnConfiguration()
    {
        resetMultiTurnConfigurationJob();
        return ServoMultiTurnConfigResult::Failed;
    }

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
        fb.successfulReadCount = 0;
        fb.missedReadCount = kOfflineInvalidateMisses;
        fb.onlineTransitionPending = false;
        fb.recoveryAttemptCount = 0;
        fb.recoveryFailed = false;
        fb.nextRecoveryAttemptMs = 0;
        _multiTurnConfigured[id] = false;
        if (_multiTurnConfigId == id) {
            resetMultiTurnConfigurationJob();
        }
        invalidateMultiTurnTrust(id);
    }

    /** 清除本次上电会话的软件多圈锚点，但保留最后掉线快照。 */
    void invalidateMultiTurnTrust(uint8_t id)
    {
        if (id > MAX_SERVO_ID) {
            return;
        }
        ServoFeedbackInternal& fb = _feedback[id];
        fb.initialized = false;
        fb.absoluteReliable = false;
        fb.turnCount = 0;
    }

    /** 在线且坐标可信时，检查实际 ABS 的硬件范围后生成命令。 */
    bool validateAbsoluteTarget(uint8_t id,
                                 int32_t logicalPosition,
                                 int16_t* outHardwarePosition) const
    {
        if (id > MAX_SERVO_ID || !outHardwarePosition ||
            !_feedback[id].online || !_feedback[id].absoluteReliable) {
            return false;
        }
        return servoHardwareCommandFromAbs(logicalPosition, outHardwarePosition);
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

    /** 用最新 raw 更新实际 ABS，并维护诊断圈数。 */
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
        fb.turnCount = (int16_t)((absPos - rawSingleTurn) / 4096);
        fb.initialized = true;
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
    uint8_t _multiTurnConfigId; ///< 当前 BUS 正在配置的舵机 ID，0xFF 表示空闲。
    ServoMultiTurnConfigPhase _multiTurnConfigPhase; ///< EEPROM 非阻塞配置阶段。
    uint32_t _multiTurnConfigNotBeforeMs; ///< 下一阶段允许执行的单调时钟时间。
    bool _multiTurnConfigFailed; ///< 写入失败后仍先执行 EEPROM 加锁，再向上层报告失败。
};



#endif // P4_FEETECH_SERVO_DRIVER_H
