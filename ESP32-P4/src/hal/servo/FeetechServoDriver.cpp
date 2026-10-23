/**
 * @file FeetechServoDriver.cpp
 * @brief 飞特驱动翻译单元；SDK 实现随本目录 FTServo_Arduino 一起编译。
 */
#include "FeetechServoDriver.h"

/* FeetechServoDriver::FeetechServoDriver */
FeetechServoDriver::FeetechServoDriver()
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

/* void FeetechServoDriver::begin */
void FeetechServoDriver::begin(uint8_t busIndex, int rxPin, int txPin, uint32_t baud)
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

/* bool FeetechServoDriver::configureMultiTurnMode */
bool FeetechServoDriver::configureMultiTurnMode(uint8_t id)
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

/* bool FeetechServoDriver::setTarget */
bool FeetechServoDriver::setTarget(uint8_t id, int32_t logicalPosition, uint16_t speed, uint8_t acc)
{
    int16_t hardwarePosition = 0;
    if (!validateAbsoluteTarget(id, logicalPosition, &hardwarePosition)) {
        return false;
    }
    return queueTarget(id, hardwarePosition, speed, acc);
}

/* void FeetechServoDriver::syncWriteAll */
void FeetechServoDriver::syncWriteAll()
{
    if (_writeCount == 0 || !_serial) {
        return;
    }
    _sms.SyncWritePosEx(_writeIDs, _writeCount, _writePos, _writeSpd, _writeAcc);
    _writeCount = 0;
}

/* ServoSyncReadResult FeetechServoDriver::collectSyncRead */
ServoSyncReadResult FeetechServoDriver::collectSyncRead(uint64_t nowUs)
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

/* void FeetechServoDriver::abortSyncRead */
void FeetechServoDriver::abortSyncRead(bool markFailure)
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
