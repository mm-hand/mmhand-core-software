#include "UpperCommTask.h"
#include "../can/CanCommTask.h"
#include "../../shared/TaskSharedData.h"
#include "../../calibration/CalibrationTask.h"
#include "../../hardware/HardwareMap.h"
#include "../../system/FirmwareLog.h"
#include "../../system/StateMachineTask.h"
#if MULTITURN_NVS_PERSISTENCE_ENABLE
#include "../../persistence/MultiTurnNvsStore.h"
#endif
#include "UpperCommCommandRouter.h"
#include "UpperCommProtocol.h"

#include <math.h>
#include <string.h>

extern volatile uint8_t g_calibrationStatus;
extern volatile uint8_t g_calibrationAckStatus;
extern volatile uint8_t g_calibrationAckRepeatRemaining;

// UpperCommTask responsibilities:
// 1) parse host commands and update shared command state;
// 2) publish sensor, servo, telemetry, debug, and fault packets;
// 3) keep protocol ACK and fault heartbeat reporting synchronized.
static uint8_t g_sensorStreamMode = SENSOR_STREAM_MODE_SIGNED_I16;
static const uint16_t kEncoderDisconnectRaw = 0xFFFF;
static const uint8_t kTactileFullPacketMagic0 = 0x54; // 'T'
static const uint8_t kTactileFullPacketMagic1 = 0x32; // '2'

// Append a float using big-endian wire order.
static void appendFloatBigEndian(uint8_t* buffer, size_t* idx, float value)
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof(bits));
    buffer[(*idx)++] = (uint8_t)((bits >> 24) & 0xFF);
    buffer[(*idx)++] = (uint8_t)((bits >> 16) & 0xFF);
    buffer[(*idx)++] = (uint8_t)((bits >> 8) & 0xFF);
    buffer[(*idx)++] = (uint8_t)(bits & 0xFF);
}

static void appendI32BigEndian(uint8_t* buffer, size_t* idx, int32_t value)
{
    const uint32_t v = (uint32_t)value;
    buffer[(*idx)++] = (uint8_t)((v >> 24) & 0xFF);
    buffer[(*idx)++] = (uint8_t)((v >> 16) & 0xFF);
    buffer[(*idx)++] = (uint8_t)((v >> 8) & 0xFF);
    buffer[(*idx)++] = (uint8_t)(v & 0xFF);
}

static void appendI16BigEndian(uint8_t* buffer, size_t* idx, int16_t value)
{
    const uint16_t v = (uint16_t)value;
    buffer[(*idx)++] = (uint8_t)((v >> 8) & 0xFF);
    buffer[(*idx)++] = (uint8_t)(v & 0xFF);
}

static int16_t clampI16Local(int32_t value)
{
    if (value > 32767) return 32767;
    if (value < -32768) return -32768;
    return (int16_t)value;
}

// Encode raw CAN magnetic encoder values for the sensor packet.
static uint16_t encodeRawEncoderForWire(uint16_t rawValue, bool channelValid)
{
    if (!channelValid) {
        return (uint16_t)PROTOCOL_DISCONNECT_SENTINEL;
    }

    if (rawValue == kEncoderDisconnectRaw || rawValue == (uint16_t)PROTOCOL_DISCONNECT_SENTINEL) {
        return (uint16_t)PROTOCOL_DISCONNECT_SENTINEL;
    }

    return rawValue;
}

static uint16_t crc16Ccitt(const uint8_t* data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

static bool calibrationAckPending()
{
    return g_calibrationAckStatus != CALIB_STATUS_IDLE &&
           g_calibrationAckRepeatRemaining > 0;
}

static uint8_t takeCalibrationAckStatusForPacket()
{
    if (!calibrationAckPending()) {
        return CALIB_STATUS_IDLE;
    }
    const uint8_t status = g_calibrationAckStatus;
    g_calibrationAckRepeatRemaining--;
    if (g_calibrationAckRepeatRemaining == 0) {
        g_calibrationAckStatus = CALIB_STATUS_IDLE;
    }
    return status;
}

// Send a protocol ACK packet.
static void sendProtoAckPacket(uint8_t cmd, uint8_t appliedMode, uint8_t status)
{
    uint8_t buffer[8];
    size_t idx = 0;

    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00;
    buffer[idx++] = PACKET_TYPE_PROTO_ACK;
    buffer[idx++] = cmd;
    buffer[idx++] = appliedMode;
    buffer[idx++] = status;
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = (uint8_t)(idx - 2);
    FW_SERIAL_WRITE(buffer, idx);
}

// Send the current state-machine status.
static void sendSystemStatusPacket(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return;
    }

    uint8_t buffer[56];
    size_t idx = 0;
    const uint32_t faultBitmap = sharedData->system_fault_bitmap;
    const uint32_t transitionCount = sharedData->state_transition_count;

    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00;
    buffer[idx++] = PACKET_TYPE_SYSTEM_STATUS;
    buffer[idx++] = sharedData->system_state;
    buffer[idx++] = sharedData->control_enabled;
    buffer[idx++] = sharedData->servo_target_owner;
    buffer[idx++] = sharedData->control_mode;
    buffer[idx++] = g_calibrationStatus;
    buffer[idx++] = (uint8_t)((faultBitmap >> 24) & 0xFF);
    buffer[idx++] = (uint8_t)((faultBitmap >> 16) & 0xFF);
    buffer[idx++] = (uint8_t)((faultBitmap >> 8) & 0xFF);
    buffer[idx++] = (uint8_t)(faultBitmap & 0xFF);
    buffer[idx++] = (uint8_t)((transitionCount >> 24) & 0xFF);
    buffer[idx++] = (uint8_t)((transitionCount >> 16) & 0xFF);
    buffer[idx++] = (uint8_t)((transitionCount >> 8) & 0xFF);
    buffer[idx++] = (uint8_t)(transitionCount & 0xFF);
    const uint32_t unknownBitmap = sharedData->multi_turn_unknown_bitmap;
    buffer[idx++] = (uint8_t)((unknownBitmap >> 24) & 0xFF);
    buffer[idx++] = (uint8_t)((unknownBitmap >> 16) & 0xFF);
    buffer[idx++] = (uint8_t)((unknownBitmap >> 8) & 0xFF);
    buffer[idx++] = (uint8_t)(unknownBitmap & 0xFF);
    buffer[idx++] = sharedData->multiturn_nvs_enabled ? 1 : 0;
    buffer[idx++] = sharedData->multiturn_nvs_snapshot_valid ? 1 : 0;
    const uint32_t activeMotorMask = sharedData->active_motor_mask;
    buffer[idx++] = (uint8_t)((activeMotorMask >> 24) & 0xFF);
    buffer[idx++] = (uint8_t)((activeMotorMask >> 16) & 0xFF);
    buffer[idx++] = (uint8_t)((activeMotorMask >> 8) & 0xFF);
    buffer[idx++] = (uint8_t)(activeMotorMask & 0xFF);
    const uint32_t activeJointMask = sharedData->active_joint_mask;
    buffer[idx++] = (uint8_t)((activeJointMask >> 24) & 0xFF);
    buffer[idx++] = (uint8_t)((activeJointMask >> 16) & 0xFF);
    buffer[idx++] = (uint8_t)((activeJointMask >> 8) & 0xFF);
    buffer[idx++] = (uint8_t)(activeJointMask & 0xFF);
    buffer[idx++] = sharedData->require_can_feedback ? 1 : 0;
    const uint32_t servoOnlineMask = sharedData->servo_online_bitmap;
    buffer[idx++] = (uint8_t)((servoOnlineMask >> 24) & 0xFF);
    buffer[idx++] = (uint8_t)((servoOnlineMask >> 16) & 0xFF);
    buffer[idx++] = (uint8_t)((servoOnlineMask >> 8) & 0xFF);
    buffer[idx++] = (uint8_t)(servoOnlineMask & 0xFF);
    const uint32_t encoderOnlineMask = sharedData->encoder_online_bitmap;
    buffer[idx++] = (uint8_t)((encoderOnlineMask >> 24) & 0xFF);
    buffer[idx++] = (uint8_t)((encoderOnlineMask >> 16) & 0xFF);
    buffer[idx++] = (uint8_t)((encoderOnlineMask >> 8) & 0xFF);
    buffer[idx++] = (uint8_t)(encoderOnlineMask & 0xFF);
    buffer[idx++] = sharedData->can_link_online ? 1 : 0;
    const uint32_t encoderWarnMask = sharedData->encoder_warn_bitmap;
    buffer[idx++] = (uint8_t)((encoderWarnMask >> 24) & 0xFF);
    buffer[idx++] = (uint8_t)((encoderWarnMask >> 16) & 0xFF);
    buffer[idx++] = (uint8_t)((encoderWarnMask >> 8) & 0xFF);
    buffer[idx++] = (uint8_t)(encoderWarnMask & 0xFF);
    const uint32_t encoderFaultMask = sharedData->encoder_fault_bitmap;
    buffer[idx++] = (uint8_t)((encoderFaultMask >> 24) & 0xFF);
    buffer[idx++] = (uint8_t)((encoderFaultMask >> 16) & 0xFF);
    buffer[idx++] = (uint8_t)((encoderFaultMask >> 8) & 0xFF);
    buffer[idx++] = (uint8_t)(encoderFaultMask & 0xFF);
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = (uint8_t)(idx - 2);
    FW_SERIAL_WRITE(buffer, idx);
}

// Send the servo overload fault bitmap plus the latched trigger load per motor.
static void sendFaultStatusPacket(TaskSharedData_t* sharedData, uint32_t overloadFaultBitmap)
{
    uint8_t buffer[8 + SERVO_TOTAL_NUM * 2];
    size_t idx = 0;

    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00;
    buffer[idx++] = PACKET_TYPE_FAULT_STATUS;
    buffer[idx++] = (uint8_t)((overloadFaultBitmap >> 24) & 0xFF);
    buffer[idx++] = (uint8_t)((overloadFaultBitmap >> 16) & 0xFF);
    buffer[idx++] = (uint8_t)((overloadFaultBitmap >> 8) & 0xFF);
    buffer[idx++] = (uint8_t)(overloadFaultBitmap & 0xFF);
    for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
        const int16_t load = sharedData ? sharedData->overload_fault_load[ch] : 0;
        const uint16_t encodedLoad = (uint16_t)load;
        buffer[idx++] = (uint8_t)((encodedLoad >> 8) & 0xFF);
        buffer[idx++] = (uint8_t)(encodedLoad & 0xFF);
    }
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = (uint8_t)(idx - 2);
    FW_SERIAL_WRITE(buffer, idx);
}

// Send the joint reverse-release fault bitmap.
static void sendReleaseFaultPacket(uint32_t releaseFaultBitmap)
{
    uint8_t buffer[8];
    size_t idx = 0;

    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00;
    buffer[idx++] = PACKET_TYPE_RELEASE_FAULT;
    buffer[idx++] = (uint8_t)((releaseFaultBitmap >> 24) & 0xFF);
    buffer[idx++] = (uint8_t)((releaseFaultBitmap >> 16) & 0xFF);
    buffer[idx++] = (uint8_t)((releaseFaultBitmap >> 8) & 0xFF);
    buffer[idx++] = (uint8_t)(releaseFaultBitmap & 0xFF);
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = (uint8_t)(idx - 2);
    FW_SERIAL_WRITE(buffer, idx);
}

static void sendMotorAbsLimitPacket(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return;
    }

    uint8_t buffer[128];
    size_t idx = 0;
    const uint32_t validBitmap = sharedData->motor_abs_limit_valid_bitmap;
    const uint32_t lockedBitmap = sharedData->motor_abs_limit_locked_bitmap;

    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00;
    buffer[idx++] = PACKET_TYPE_MOTOR_ABS_LIMIT;
    buffer[idx++] = (uint8_t)((validBitmap >> 24) & 0xFF);
    buffer[idx++] = (uint8_t)((validBitmap >> 16) & 0xFF);
    buffer[idx++] = (uint8_t)((validBitmap >> 8) & 0xFF);
    buffer[idx++] = (uint8_t)(validBitmap & 0xFF);
    buffer[idx++] = (uint8_t)((lockedBitmap >> 24) & 0xFF);
    buffer[idx++] = (uint8_t)((lockedBitmap >> 16) & 0xFF);
    buffer[idx++] = (uint8_t)((lockedBitmap >> 8) & 0xFF);
    buffer[idx++] = (uint8_t)(lockedBitmap & 0xFF);
    for (int i = 0; i < SERVO_TOTAL_NUM; i++) {
        const int16_t minAbs = (int16_t)sharedData->motor_abs_limit_min[i];
        const int16_t maxAbs = (int16_t)sharedData->motor_abs_limit_max[i];
        buffer[idx++] = (uint8_t)(((uint16_t)minAbs >> 8) & 0xFF);
        buffer[idx++] = (uint8_t)((uint16_t)minAbs & 0xFF);
        buffer[idx++] = (uint8_t)(((uint16_t)maxAbs >> 8) & 0xFF);
        buffer[idx++] = (uint8_t)((uint16_t)maxAbs & 0xFF);
    }
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = (uint8_t)(idx - 2);
    FW_SERIAL_WRITE(buffer, idx);
}

static void sendJointCalibSnapshotPacket(uint8_t jointIndex)
{
    if (jointIndex >= ENCODER_TOTAL_NUM) {
        return;
    }

    const JointCalibrationResult& result = g_jointCalibResult[jointIndex];
    uint8_t buffer[72];
    size_t idx = 0;
    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00;
    buffer[idx++] = PACKET_TYPE_JOINT_CALIB_SNAPSHOT;
    buffer[idx++] = jointIndex;
    buffer[idx++] = result.success ? 1 : 0;
    appendI32BigEndian(buffer, &idx, result.encoderMin);
    appendI32BigEndian(buffer, &idx, result.encoderMax);
    appendI32BigEndian(buffer, &idx, result.offset);
    appendFloatBigEndian(buffer, &idx, result.angleMax);
    appendI32BigEndian(buffer, &idx, result.startEncoder);
    appendI32BigEndian(buffer, &idx, result.tightEncoder);
    appendI32BigEndian(buffer, &idx, result.zeroEncoder);
    appendI32BigEndian(buffer, &idx, result.encoderSpan);
    appendI16BigEndian(buffer, &idx, clampI16Local(result.startServoAbs));
    appendI16BigEndian(buffer, &idx, clampI16Local(result.tightServoAbs));
    appendI16BigEndian(buffer, &idx, clampI16Local(result.zeroServoAbs));
    appendFloatBigEndian(buffer, &idx, result.startDeg);
    appendFloatBigEndian(buffer, &idx, result.tightDeg);
    appendFloatBigEndian(buffer, &idx, result.totalDeg);
    buffer[idx++] = result.scaled ? 1 : 0;
    buffer[idx++] = (g_encoderDirection[jointIndex] < 0) ? 0xFF : 0x01;
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = (uint8_t)(idx - 2);
    FW_SERIAL_WRITE(buffer, idx);
}

static void sendCalibrationDiagPacket(const CalibrationDiagData_t& diag)
{
    uint8_t buffer[40];
    size_t idx = 0;
    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00;
    buffer[idx++] = PACKET_TYPE_CALIB_DIAG;
    buffer[idx++] = diag.valid ? 1 : 0;
    buffer[idx++] = diag.reason;
    buffer[idx++] = diag.phase;
    buffer[idx++] = diag.joint;
    buffer[idx++] = diag.motor;
    appendI32BigEndian(buffer, &idx, diag.valueA);
    appendI32BigEndian(buffer, &idx, diag.valueB);
    appendI32BigEndian(buffer, &idx, diag.valueC);
    buffer[idx++] = (uint8_t)((diag.bitmap >> 24) & 0xFF);
    buffer[idx++] = (uint8_t)((diag.bitmap >> 16) & 0xFF);
    buffer[idx++] = (uint8_t)((diag.bitmap >> 8) & 0xFF);
    buffer[idx++] = (uint8_t)(diag.bitmap & 0xFF);
    buffer[idx++] = (uint8_t)((diag.timestamp >> 24) & 0xFF);
    buffer[idx++] = (uint8_t)((diag.timestamp >> 16) & 0xFF);
    buffer[idx++] = (uint8_t)((diag.timestamp >> 8) & 0xFF);
    buffer[idx++] = (uint8_t)(diag.timestamp & 0xFF);
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = (uint8_t)(idx - 2);
    FW_SERIAL_WRITE(buffer, idx);
}

static void sendEncoderRawPacket(const RemoteSensorData_t& sensorRaw)
{
    uint8_t buffer[64];
    size_t idx = 0;

    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00;
    buffer[idx++] = PACKET_TYPE_ENCODER_RAW;
    for (int i = 0; i < ENCODER_TOTAL_NUM; i++)
    {
        const bool channelValid =
            sensorRaw.isValid &&
            (sensorRaw.errorFlags[i] == 0) &&
            (sensorRaw.encoderValues[i] != kEncoderDisconnectRaw);
        const uint16_t val = encodeRawEncoderForWire(sensorRaw.encoderValues[i], channelValid);
        buffer[idx++] = (uint8_t)((val >> 8) & 0xFF);
        buffer[idx++] = (uint8_t)(val & 0xFF);
    }
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = (uint8_t)(idx - 2);
    FW_SERIAL_WRITE(buffer, idx);
}

static void sendTactileFullPackets(const RemoteTactileData_t& tactile)
{
    if (!tactile.fullValid) {
        return;
    }

    static const size_t kTactileFullSerialChunkBytes = 240;
    const uint8_t fragCount = (uint8_t)((TACTILE_FULL_BYTES + kTactileFullSerialChunkBytes - 1) /
                                        kTactileFullSerialChunkBytes);
    const uint16_t fullCrc = crc16Ccitt(tactile.fullValues, TACTILE_FULL_BYTES);

    for (uint8_t fragIdx = 0; fragIdx < fragCount; fragIdx++) {
        const size_t offset = (size_t)fragIdx * kTactileFullSerialChunkBytes;
        const size_t remaining = TACTILE_FULL_BYTES - offset;
        const uint8_t chunkLen = (uint8_t)((remaining > kTactileFullSerialChunkBytes)
            ? kTactileFullSerialChunkBytes
            : remaining);

        uint8_t buffer[256];
        size_t idx = 0;
        buffer[idx++] = PROTOCOL_HEADER;
        buffer[idx++] = 0x00;
        buffer[idx++] = PACKET_TYPE_TACTILE_FULL;
        buffer[idx++] = kTactileFullPacketMagic0;
        buffer[idx++] = kTactileFullPacketMagic1;
        buffer[idx++] = tactile.fullSeq;
        buffer[idx++] = tactile.seq;
        buffer[idx++] = fragIdx;
        buffer[idx++] = fragCount;
        buffer[idx++] = chunkLen;
        buffer[idx++] = (uint8_t)(fullCrc & 0xFF);
        buffer[idx++] = (uint8_t)((fullCrc >> 8) & 0xFF);
        memcpy(buffer + idx, tactile.fullValues + offset, chunkLen);
        idx += chunkLen;
        buffer[idx++] = PROTOCOL_TAIL;
        buffer[1] = (uint8_t)(idx - 2);
        FW_SERIAL_WRITE(buffer, idx);
    }
}

// Serialize one upstream data packet.
static void sendDataPacket(ServoStatus_t* pServo,
                           RemoteSensorData_t* pSensorRaw,
                           MappedAngleData_t* pMappedAngle,
                           ServoAngleData_t* pServoAngle,
                           ServoAngleData_t* pServoRaw,
                           ServoTelemetryData_t* pTelemetry,
                           RemoteTactileData_t* pTactile,
                           JointDebugData_t* pJointDebug)
{
    (void)pServo;

    uint8_t buffer[256];
    size_t idx = 0;
    bool sendFullTactileAfter = false;
    const RemoteTactileData_t* fullTactile = NULL;

    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00; // LEN placeholder.
    const uint8_t calibrationAckStatus = takeCalibrationAckStatusForPacket();
    if (calibrationAckStatus != CALIB_STATUS_IDLE)
    {
        buffer[idx++] = PACKET_TYPE_CALIB_ACK;
        buffer[idx++] = calibrationAckStatus;
    }
    else if (pSensorRaw)
    {
        buffer[idx++] = PACKET_TYPE_SENSOR;
        for (int i = 0; i < ENCODER_TOTAL_NUM; i++)
        {
            const bool channelValid =
                pSensorRaw->isValid &&
                (pSensorRaw->errorFlags[i] == 0) &&
                (pSensorRaw->encoderValues[i] != kEncoderDisconnectRaw);
            const uint16_t val = encodeRawEncoderForWire(pSensorRaw->encoderValues[i], channelValid);
            buffer[idx++] = (uint8_t)((val >> 8) & 0xFF);
            buffer[idx++] = (uint8_t)(val & 0xFF);
        }
    }
    else if (pMappedAngle)
    {
        buffer[idx++] = PACKET_TYPE_SENSOR;
        for (int i = 0; i < ENCODER_TOTAL_NUM; i++)
        {
            const int16_t val = (pMappedAngle->isValid && pMappedAngle->validFlags[i])
                ? pMappedAngle->angleValues[i]
                : (int16_t)PROTOCOL_DISCONNECT_SENTINEL;
            const uint16_t raw = (uint16_t)val;
            buffer[idx++] = (uint8_t)((raw >> 8) & 0xFF);
            buffer[idx++] = (uint8_t)(raw & 0xFF);
        }
    }
    else if (pServoAngle)
    {
        buffer[idx++] = PACKET_TYPE_SERVO_ANGLE;
        for (int i = 0; i < SERVO_TOTAL_NUM; i++)
        {
            const int32_t angle = pServoAngle->servoAngles[i];
            buffer[idx++] = (uint8_t)((angle >> 24) & 0xFF);
            buffer[idx++] = (uint8_t)((angle >> 16) & 0xFF);
            buffer[idx++] = (uint8_t)((angle >> 8) & 0xFF);
            buffer[idx++] = (uint8_t)(angle & 0xFF);
        }
        for (int i = 0; i < SERVO_TOTAL_NUM; i++)
        {
            buffer[idx++] = pServoAngle->onlineStatus[i];
        }
    }
    else if (pServoRaw)
    {
        buffer[idx++] = PACKET_TYPE_SERVO_RAW;
        for (int i = 0; i < SERVO_TOTAL_NUM; i++)
        {
            const uint16_t raw = (uint16_t)pServoRaw->servoRawPositions[i];
            buffer[idx++] = (uint8_t)((raw >> 8) & 0xFF);
            buffer[idx++] = (uint8_t)(raw & 0xFF);
        }
        for (int i = 0; i < SERVO_TOTAL_NUM; i++)
        {
            const int32_t hw = pServoRaw->servoHardwarePositions[i];
            buffer[idx++] = (uint8_t)((hw >> 24) & 0xFF);
            buffer[idx++] = (uint8_t)((hw >> 16) & 0xFF);
            buffer[idx++] = (uint8_t)((hw >> 8) & 0xFF);
            buffer[idx++] = (uint8_t)(hw & 0xFF);
        }
        for (int i = 0; i < SERVO_TOTAL_NUM; i++)
        {
            buffer[idx++] = pServoRaw->onlineStatus[i];
        }
    }
    else if (pTelemetry)
    {
        buffer[idx++] = PACKET_TYPE_SERVO_TELEM;
        for (int i = 0; i < SERVO_TOTAL_NUM; i++)
        {
            const int16_t speed = pTelemetry->speed[i];
            const int16_t load = pTelemetry->load[i];
            buffer[idx++] = (uint8_t)(((uint16_t)speed >> 8) & 0xFF);
            buffer[idx++] = (uint8_t)((uint16_t)speed & 0xFF);
            buffer[idx++] = (uint8_t)(((uint16_t)load >> 8) & 0xFF);
            buffer[idx++] = (uint8_t)((uint16_t)load & 0xFF);
            buffer[idx++] = pTelemetry->voltage[i];
            buffer[idx++] = pTelemetry->temperature[i];
            buffer[idx++] = pTelemetry->onlineStatus[i];
        }
    }
    else if (pTactile)
    {
        buffer[idx++] = PACKET_TYPE_TACTILE;
        buffer[idx++] = pTactile->seq;
        for (int i = 0; i < TACTILE_SUMMARY_BYTES; i++)
        {
            buffer[idx++] = pTactile->values[i];
        }
        sendFullTactileAfter = pTactile->fullValid;
        fullTactile = pTactile;
    }
    else if (pJointDebug)
    {
        buffer[idx++] = PACKET_TYPE_JOINT1_DEBUG;
        buffer[idx++] = pJointDebug->jointIndex;
        buffer[idx++] = pJointDebug->valid;
        appendFloatBigEndian(buffer, &idx, pJointDebug->targetDeg);
        appendFloatBigEndian(buffer, &idx, pJointDebug->magActualDeg);
        appendFloatBigEndian(buffer, &idx, pJointDebug->loop1Output);
        appendFloatBigEndian(buffer, &idx, pJointDebug->loop2Actual);
        appendFloatBigEndian(buffer, &idx, pJointDebug->loop2Output);
        buffer[idx++] = pJointDebug->cmdValid;
        const uint16_t cmdPosRaw = (uint16_t)pJointDebug->cmdTargetPos;
        buffer[idx++] = (uint8_t)((cmdPosRaw >> 8) & 0xFF);
        buffer[idx++] = (uint8_t)(cmdPosRaw & 0xFF);
    }
    else
    {
        return;
    }

    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = (uint8_t)(idx - 2);
    FW_SERIAL_WRITE(buffer, idx);
    if (sendFullTactileAfter && fullTactile) {
        sendTactileFullPackets(*fullTactile);
    }
}

// Decode a little-endian float from command payload.
static float decodeFloatLittleEndian(const uint8_t* data)
{
    uint32_t bits = 0;
    bits |= (uint32_t)data[0];
    bits |= ((uint32_t)data[1] << 8);
    bits |= ((uint32_t)data[2] << 16);
    bits |= ((uint32_t)data[3] << 24);
    float out = 0.0f;
    memcpy(&out, &bits, sizeof(out));
    return out;
}

// Parse a little-endian float array from command payload.
static bool parseFloatArrayLittleEndian(const uint8_t* payload, size_t payloadLen, float* outValues, uint8_t count)
{
    if (!payload || !outValues) {
        return false;
    }
    const size_t required = (size_t)count * sizeof(float);
    if (payloadLen < required) {
        return false;
    }
    for (uint8_t i = 0; i < count; i++)
    {
        outValues[i] = decodeFloatLittleEndian(payload + (size_t)i * sizeof(float));
    }
    return true;
}

// Parse a big-endian int16 array from command payload.
static bool parseInt16ArrayBigEndian(const uint8_t* payload, size_t payloadLen, int32_t* outValues, uint8_t count)
{
    if (!payload || !outValues) {
        return false;
    }
    const size_t required = (size_t)count * sizeof(uint16_t);
    if (payloadLen < required) {
        return false;
    }
    for (uint8_t i = 0; i < count; i++)
    {
        const uint16_t raw = ((uint16_t)payload[(size_t)i * 2] << 8) | payload[(size_t)i * 2 + 1];
        outValues[i] = (int16_t)raw;
    }
    return true;
}

static bool parseTendonGuardPayload(const uint8_t* payload,
                                    size_t payloadLen,
                                    uint8_t* enabled,
                                    int8_t* sign,
                                    int16_t* x1Abs)
{
    if (!payload || !enabled || !sign || !x1Abs) {
        return false;
    }
    if (payloadLen < kTendonGuardPayloadBytes) {
        return false;
    }
    for (uint8_t i = 0; i < ENCODER_TOTAL_NUM; i++)
    {
        const size_t base = (size_t)i * 4;
        enabled[i] = (payload[base] != 0) ? 1 : 0;
        sign[i] = (((int8_t)payload[base + 1]) < 0) ? (int8_t)-1 : (int8_t)1;
        const uint16_t raw = ((uint16_t)payload[base + 2] << 8) | payload[base + 3];
        x1Abs[i] = clampServoAbsCommand((int16_t)raw);
    }
    return true;
}

static uint32_t readU32BigEndian(const uint8_t* p)
{
    return ((uint32_t)p[0] << 24) |
           ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) |
           (uint32_t)p[3];
}

static int32_t readI32BigEndian(const uint8_t* p)
{
    return (int32_t)readU32BigEndian(p);
}

static int16_t readI16BigEndian(const uint8_t* p)
{
    const uint16_t raw = ((uint16_t)p[0] << 8) | (uint16_t)p[1];
    return (int16_t)raw;
}

static float readFloatBigEndian(const uint8_t* p)
{
    const uint32_t bits = readU32BigEndian(p);
    float value = 0.0f;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

// Return command payload length, excluding the command byte.
static size_t getCommandPayloadLength(uint8_t cmd)
{
    switch (cmd)
    {
        case CMD_CALIBRATE:
        case CMD_START:
        case CMD_STOP:
        case CMD_RESET:
        case CMD_RUN_FEEDBACK:
        case CMD_PAUSE:
        case CMD_HOST_HEARTBEAT:
        case CMD_SIDE_SWAY_ADJUST:
        case CMD_CLEAR_CALIBRATION:
            return 0;
        case CMD_MULTITURN_NVS_CONTROL:
            return kMultiTurnNvsControlPayloadBytes;
        case CMD_ANGLE_CTRL:
            return kFloatPayloadBytes;
        case CMD_CALIB_DATA:
            return kFloatPayloadBytes;
        case CMD_MOTOR_POS:
            return kMotorPosPayloadBytes;
        case CMD_MOTOR_POS_SWEEP:
            return kMotorPosPayloadBytes;
        case CMD_MOTOR_POS_ABS:
            return kMotorPosPayloadBytes;
        case CMD_MOTOR_POS_ABS_ONE:
            return kMotorPosAbsOnePayloadBytes;
        case CMD_TENDON_GUARD:
            return kTendonGuardPayloadBytes;
        case CMD_ACTIVE_SCOPE:
            return kActiveScopePayloadBytes;
        case CMD_RESTORE_MULTITURN:
            return kRestoreMultiTurnPayloadBytes;
        case CMD_MOTOR_ABS_LIMIT_LOCK:
            return kMotorAbsLimitLockPayloadBytes;
        case CMD_CALIB_LOAD_JOINT:
            return kCalibLoadJointPayloadBytes;
        case CMD_CALIB_LOAD_LIMITS:
            return kCalibLoadLimitsPayloadBytes;
        case CMD_LOCAL_TEST_PARAMS:
            return kLocalTestParamsPayloadBytes;
        case CMD_LOCAL_TEST_START:
        case CMD_LOCAL_TEST_STOP:
            return 0;
        case CMD_SENSOR_STREAM_MODE:
            return 1;
        default:
            return (size_t)-1;
    }
}

// Handle legacy single-byte commands.
static void handleLegacySingleByteCommand(TaskSharedData_t* sharedData, uint8_t cmd)
{
    if (cmd == (uint8_t)'c') {
        postSystemEvent(sharedData, SYSTEM_EVENT_CALIBRATE);
        return;
    }
    if (cmd == (uint8_t)'b') {
        upperClearTargetAngles(sharedData);
    }
}

// Apply a parsed downstream command frame.
static void handleParsedCommand(TaskSharedData_t* sharedData, const uint8_t* frame, size_t frameLen)
{
    if (!sharedData || !frame || frameLen == 0) {
        return;
    }

    const uint8_t cmd = frame[0];
    // 任意合法上位机帧都说明 host 在线；界面空闲时由 heartbeat 帧刷新该时间戳。
    sharedData->host_rx_heartbeat_ms = millis();

    if (cmd == CMD_HOST_HEARTBEAT)
    {
        return;
    }

    if (cmd == CMD_CALIBRATE)
    {
        postSystemEvent(sharedData, SYSTEM_EVENT_CALIBRATE);
        sendProtoAckPacket(CMD_CALIBRATE, sharedData->system_state, PROTO_ACK_STATUS_QUEUED);
        return;
    }

    if (cmd == CMD_SIDE_SWAY_ADJUST)
    {
        postSystemEvent(sharedData, SYSTEM_EVENT_SIDE_SWAY_ADJUST);
        sendProtoAckPacket(CMD_SIDE_SWAY_ADJUST, sharedData->system_state, PROTO_ACK_STATUS_QUEUED);
        return;
    }

    if (cmd == CMD_START)
    {
        postSystemEvent(sharedData, SYSTEM_EVENT_START);
        sendProtoAckPacket(CMD_START, sharedData->system_state, PROTO_ACK_STATUS_QUEUED);
        return;
    }

    if (cmd == CMD_RUN_FEEDBACK)
    {
        postSystemEvent(sharedData, SYSTEM_EVENT_RUN_FEEDBACK);
        sendProtoAckPacket(CMD_RUN_FEEDBACK, sharedData->system_state, PROTO_ACK_STATUS_QUEUED);
        return;
    }

    if (cmd == CMD_PAUSE)
    {
        postSystemEvent(sharedData, SYSTEM_EVENT_PAUSE);
        sendProtoAckPacket(CMD_PAUSE, sharedData->system_state, PROTO_ACK_STATUS_QUEUED);
        return;
    }

    if (cmd == CMD_STOP)
    {
        sharedData->sine_test_enabled = 0;
        postSystemEvent(sharedData, SYSTEM_EVENT_STOP);
        sendProtoAckPacket(CMD_STOP, sharedData->system_state, PROTO_ACK_STATUS_QUEUED);
        return;
    }

    if (cmd == CMD_RESET)
    {
        sharedData->sine_test_enabled = 0;
        postSystemEvent(sharedData, SYSTEM_EVENT_RESET);
        sendProtoAckPacket(CMD_RESET, sharedData->system_state, PROTO_ACK_STATUS_QUEUED);
        return;
    }

    if (cmd == CMD_ANGLE_CTRL)
    {
        float parsedAngles[ENCODER_TOTAL_NUM] = {0.0f};
        if (parseFloatArrayLittleEndian(frame + 1, frameLen - 1, parsedAngles, ENCODER_TOTAL_NUM)) {
            upperApplyTargetAngles(sharedData, parsedAngles, ENCODER_TOTAL_NUM);
        }
        return;
    }

    if (cmd == CMD_CALIB_DATA)
    {
        float zeroRaw[ENCODER_TOTAL_NUM] = {0.0f};
        bool ok = false;
        if (parseFloatArrayLittleEndian(frame + 1, frameLen - 1, zeroRaw, ENCODER_TOTAL_NUM)) {
            upperCacheCalibZeroRaw(sharedData, zeroRaw, ENCODER_TOTAL_NUM);
            ok = true;
        }
        sendProtoAckPacket(CMD_CALIB_DATA,
                           sharedData->system_state,
                           ok ? PROTO_ACK_STATUS_OK : PROTO_ACK_STATUS_UNSUPPORTED_MODE);
        return;
    }

    if (cmd == CMD_MOTOR_POS)
    {
        int32_t motorTargets[SERVO_TOTAL_NUM] = {0};
        if (parseInt16ArrayBigEndian(frame + 1, frameLen - 1, motorTargets, SERVO_TOTAL_NUM)) {
            upperApplyMotorTargets(sharedData, motorTargets, SERVO_TOTAL_NUM, MOTOR_DIRECT_SOURCE_TARGET);
        }
        return;
    }

    if (cmd == CMD_MOTOR_POS_SWEEP)
    {
        int32_t motorTargets[SERVO_TOTAL_NUM] = {0};
        if (parseInt16ArrayBigEndian(frame + 1, frameLen - 1, motorTargets, SERVO_TOTAL_NUM)) {
            upperApplyMotorSweepTargets(sharedData, motorTargets, SERVO_TOTAL_NUM);
        }
        return;
    }

    if (cmd == CMD_MOTOR_POS_ABS)
    {
        int32_t motorTargets[SERVO_TOTAL_NUM] = {0};
        if (parseInt16ArrayBigEndian(frame + 1, frameLen - 1, motorTargets, SERVO_TOTAL_NUM)) {
            upperApplyMotorTargets(sharedData, motorTargets, SERVO_TOTAL_NUM, MOTOR_DIRECT_SOURCE_ABSOLUTE);
        }
        return;
    }

    if (cmd == CMD_MOTOR_POS_ABS_ONE)
    {
        if (frameLen >= 1 + kMotorPosAbsOnePayloadBytes) {
            const uint8_t motorIndex = frame[1];
            const uint16_t raw = ((uint16_t)frame[2] << 8) | frame[3];
            upperApplySingleMotorTarget(sharedData,
                                        motorIndex,
                                        (int16_t)raw,
                                        MOTOR_DIRECT_SOURCE_ABSOLUTE);
        }
        return;
    }

    if (cmd == CMD_TENDON_GUARD)
    {
        uint8_t enabled[ENCODER_TOTAL_NUM] = {0};
        int8_t sign[ENCODER_TOTAL_NUM] = {0};
        int16_t x1Abs[ENCODER_TOTAL_NUM] = {0};
        uint8_t status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        if (parseTendonGuardPayload(frame + 1, frameLen - 1, enabled, sign, x1Abs)) {
            upperApplyTendonGuardConfig(sharedData, enabled, sign, x1Abs, ENCODER_TOTAL_NUM);
            status = PROTO_ACK_STATUS_OK;
        }
        sendProtoAckPacket(CMD_TENDON_GUARD, sharedData->system_state, status);
        return;
    }

    if (cmd == CMD_LOCAL_TEST_PARAMS)
    {
        const float freqHz = decodeFloatLittleEndian(frame + 1);
        const float amplitudeDeg = decodeFloatLittleEndian(frame + 5);
        const bool ok = isfinite(freqHz) && isfinite(amplitudeDeg) && freqHz > 0.0f;
        if (ok) {
            SemaphoreHandle_t lock = sharedData->commandStateMutex;
            if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) == pdTRUE) {
                const uint32_t jointMask =
                    sharedData->active_joint_mask & ((1UL << ENCODER_TOTAL_NUM) - 1UL);
                sharedData->sine_test_joint_mask = jointMask;
                sharedData->sine_test_hold_ms = 0;
                for (uint8_t i = 0; i < ENCODER_TOTAL_NUM; i++) {
                    sharedData->sine_test_center_deg[i] = sharedData->targetAngles[i];
                    sharedData->sine_test_amp_deg[i] =
                        ((jointMask & (1UL << i)) != 0) ? amplitudeDeg : 0.0f;
                    sharedData->sine_test_freq_hz[i] =
                        ((jointMask & (1UL << i)) != 0) ? freqHz : 0.0f;
                    sharedData->sine_test_phase_rad[i] = 0.0f;
                }
                xSemaphoreGive(lock);
            }
        }
        Serial.printf("[UPPER] LOCAL_TEST_PARAMS freq=%.4f amp=%.3f mask=0x%08lX ok=%u\r\n",
                      (double)freqHz,
                      (double)amplitudeDeg,
                      (unsigned long)sharedData->sine_test_joint_mask,
                      ok ? 1U : 0U);
        sendProtoAckPacket(CMD_LOCAL_TEST_PARAMS,
                           sharedData->system_state,
                           ok ? PROTO_ACK_STATUS_OK : PROTO_ACK_STATUS_UNSUPPORTED_MODE);
        return;
    }

    if (cmd == CMD_LOCAL_TEST_START)
    {
        uint8_t status = PROTO_ACK_STATUS_OK;
        SemaphoreHandle_t lock = sharedData->commandStateMutex;
        if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else {
            if (sharedData->sine_test_joint_mask == 0) {
                sharedData->sine_test_joint_mask =
                    sharedData->active_joint_mask & ((1UL << ENCODER_TOTAL_NUM) - 1UL);
            }
            sharedData->sine_test_enabled = 1;
            sharedData->sine_test_start_ms = millis();
            sharedData->sine_test_request_token++;
            sharedData->joint_command_token++;
            if (lock) {
                xSemaphoreGive(lock);
            }
        }
        Serial.printf("[UPPER] LOCAL_TEST_START enabled=%u token=%lu mask=0x%08lX status=%u\r\n",
                      sharedData->sine_test_enabled ? 1U : 0U,
                      (unsigned long)sharedData->sine_test_request_token,
                      (unsigned long)sharedData->sine_test_joint_mask,
                      (unsigned)status);
        sendProtoAckPacket(cmd, sharedData->system_state, status);
        return;
    }

    if (cmd == CMD_LOCAL_TEST_STOP)
    {
        uint8_t status = PROTO_ACK_STATUS_OK;
        SemaphoreHandle_t lock = sharedData->commandStateMutex;
        if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else {
            sharedData->sine_test_enabled = 0;
            sharedData->sine_test_request_token++;
            sharedData->joint_command_token++;
            if (lock) {
                xSemaphoreGive(lock);
            }
        }
        Serial.printf("[UPPER] LOCAL_TEST_STOP status=%u\r\n", (unsigned)status);
        sendProtoAckPacket(cmd, sharedData->system_state, status);
        return;
    }

    if (cmd == CMD_ACTIVE_SCOPE)
    {
        uint8_t status = PROTO_ACK_STATUS_OK;
        if (frameLen < 1 + kActiveScopePayloadBytes) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else {
            const uint32_t activeMotorMask = readU32BigEndian(frame + 1);
            const uint32_t activeJointMask = readU32BigEndian(frame + 5);
            const uint8_t requireCanFeedback = frame[9] ? 1 : 0;
            if (activeMotorMask == 0) {
                status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
            } else {
                if (!upperApplyActiveScope(sharedData,
                                           activeMotorMask,
                                           activeJointMask,
                                           requireCanFeedback)) {
                    status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
                }
            }
        }
        if (status == PROTO_ACK_STATUS_OK) {
            postSystemEvent(sharedData, SYSTEM_EVENT_REEVALUATE_READY);
        }
        sendProtoAckPacket(CMD_ACTIVE_SCOPE, sharedData->system_state, status);
        return;
    }

    if (cmd == CMD_RESTORE_MULTITURN)
    {
        uint8_t status = PROTO_ACK_STATUS_OK;
        if (frameLen < 1 + kRestoreMultiTurnPayloadBytes) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else {
            const uint32_t motorMask = readU32BigEndian(frame + 1);
            int32_t motorAbs[SERVO_TOTAL_NUM] = {0};
            if (!parseInt16ArrayBigEndian(frame + 5,
                                          frameLen - 5,
                                          motorAbs,
                                          SERVO_TOTAL_NUM)) {
                status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
            } else {
                SemaphoreHandle_t lock = sharedData->commandStateMutex;
                if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
                    status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
                } else {
                    memcpy(sharedData->restore_multi_turn_abs,
                           motorAbs,
                           sizeof(sharedData->restore_multi_turn_abs));
                    sharedData->restore_multi_turn_mask =
                        motorMask & validMotorChannelMask();
                    sharedData->restore_multi_turn_request_token++;
                    if (lock) {
                        xSemaphoreGive(lock);
                    }
                }
            }
        }
        Serial.printf("[UPPER] RESTORE_MULTITURN request status=%u mask=0x%08lX token=%lu\r\n",
                      (unsigned)status,
                      (unsigned long)sharedData->restore_multi_turn_mask,
                      (unsigned long)sharedData->restore_multi_turn_request_token);
        sendProtoAckPacket(CMD_RESTORE_MULTITURN, sharedData->system_state, status);
        return;
    }

    if (cmd == CMD_MOTOR_ABS_LIMIT_LOCK)
    {
        uint8_t status = PROTO_ACK_STATUS_OK;
        if (frameLen < 1 + kMotorAbsLimitLockPayloadBytes) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else {
            const bool lockEnable = frame[1] != 0;
            const uint32_t requestedMask = readU32BigEndian(frame + 2) & validMotorChannelMask();
            const uint32_t validMask = sharedData->motor_abs_limit_valid_bitmap;
            if (lockEnable) {
                sharedData->motor_abs_limit_locked_bitmap |= (requestedMask & validMask);
            } else {
                sharedData->motor_abs_limit_locked_bitmap &= ~requestedMask;
            }
            Serial.printf("[UPPER] ABS_LIMIT_%s mask=0x%08lX locked=0x%08lX valid=0x%08lX\r\n",
                          lockEnable ? "LOCK" : "UNLOCK",
                          (unsigned long)requestedMask,
                          (unsigned long)sharedData->motor_abs_limit_locked_bitmap,
                          (unsigned long)validMask);
        }
        sendProtoAckPacket(CMD_MOTOR_ABS_LIMIT_LOCK, sharedData->system_state, status);
        return;
    }

    if (cmd == CMD_CALIB_LOAD_LIMITS)
    {
        uint8_t status = PROTO_ACK_STATUS_OK;
        if (frameLen < 1 + kCalibLoadLimitsPayloadBytes) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else if (sharedData->system_state == SYSTEM_STATE_RUN_DIRECT ||
                   sharedData->system_state == SYSTEM_STATE_RUN_FEEDBACK ||
                   sharedData->system_state == SYSTEM_STATE_CALIBRATION_RUNNING) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else {
            const uint32_t validMask = readU32BigEndian(frame + 1) & validMotorChannelMask();
            const uint32_t lockedMask = readU32BigEndian(frame + 5) & validMotorChannelMask();
            const uint8_t* p = frame + 9;
            for (uint8_t ch = 0; ch < SERVO_TOTAL_NUM; ch++) {
                sharedData->motor_abs_limit_min[ch] = readI16BigEndian(p);
                p += 2;
                sharedData->motor_abs_limit_max[ch] = readI16BigEndian(p);
                p += 2;
            }
            sharedData->motor_abs_limit_valid_bitmap = validMask;
            sharedData->motor_abs_limit_locked_bitmap = lockedMask & validMask;
            Serial.printf("[UPPER] CALIB_LOAD_LIMITS valid=0x%08lX locked=0x%08lX\r\n",
                          (unsigned long)sharedData->motor_abs_limit_valid_bitmap,
                          (unsigned long)sharedData->motor_abs_limit_locked_bitmap);
        }
        sendProtoAckPacket(CMD_CALIB_LOAD_LIMITS, sharedData->system_state, status);
        return;
    }

    if (cmd == CMD_CALIB_LOAD_JOINT)
    {
        uint8_t status = PROTO_ACK_STATUS_OK;
        if (frameLen < 1 + kCalibLoadJointPayloadBytes) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else if (sharedData->system_state == SYSTEM_STATE_RUN_DIRECT ||
                   sharedData->system_state == SYSTEM_STATE_RUN_FEEDBACK ||
                   sharedData->system_state == SYSTEM_STATE_CALIBRATION_RUNNING) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else {
            const uint8_t* p = frame + 1;
            const uint8_t jointIndex = *p++;
            if (jointIndex >= ENCODER_TOTAL_NUM) {
                status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
            } else {
                JointCalibrationResult result;
                memset(&result, 0, sizeof(result));
                result.success = (*p++ != 0);
                result.encoderMin = readI32BigEndian(p); p += 4;
                result.encoderMax = readI32BigEndian(p); p += 4;
                result.offset = readI32BigEndian(p); p += 4;
                result.angleMax = readFloatBigEndian(p); p += 4;
                result.startEncoder = readI32BigEndian(p); p += 4;
                result.tightEncoder = readI32BigEndian(p); p += 4;
                result.zeroEncoder = readI32BigEndian(p); p += 4;
                result.encoderSpan = readI32BigEndian(p); p += 4;
                result.startServoAbs = readI16BigEndian(p); p += 2;
                result.tightServoAbs = readI16BigEndian(p); p += 2;
                result.zeroServoAbs = readI16BigEndian(p); p += 2;
                result.startDeg = readFloatBigEndian(p); p += 4;
                result.tightDeg = readFloatBigEndian(p); p += 4;
                result.totalDeg = readFloatBigEndian(p); p += 4;
                result.scaled = (*p++ != 0);
                const int8_t direction = ((int8_t)(*p++)) < 0 ? (int8_t)-1 : (int8_t)1;
                if (result.success &&
                    (!isfinite(result.angleMax) || !isfinite(result.startDeg) ||
                     !isfinite(result.tightDeg) || !isfinite(result.totalDeg) ||
                     result.encoderSpan <= 0 || result.totalDeg <= 0.0f)) {
                    status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
                } else {
                    g_jointCalibResult[jointIndex] = result;
                    g_encoderOffsetManual[jointIndex] = result.success ? result.offset : 0;
                    setRuntimeEncoderDirection(jointIndex, direction);
                    if (result.success) {
                        g_calibrationStatus = CALIB_STATUS_SUCCESS;
                        g_calibrationAckStatus = CALIB_STATUS_SUCCESS;
                        g_calibrationAckRepeatRemaining = 5;
                        sharedData->system_fault_bitmap &= ~SYSTEM_FAULT_CALIBRATION;
                    }
                    Serial.printf("[UPPER] CALIB_LOAD_JOINT J%02u success=%u start=%ld tight=%ld span=%ld dir=%d\r\n",
                                  jointIndex,
                                  result.success ? 1U : 0U,
                                  (long)result.startEncoder,
                                  (long)result.tightEncoder,
                                  (long)result.encoderSpan,
                                  (int)direction);
                }
            }
        }
        sendProtoAckPacket(CMD_CALIB_LOAD_JOINT, sharedData->system_state, status);
        return;
    }

    if (cmd == CMD_CLEAR_CALIBRATION)
    {
        uint8_t status = PROTO_ACK_STATUS_OK;
        if (sharedData->system_state == SYSTEM_STATE_RUN_DIRECT ||
            sharedData->system_state == SYSTEM_STATE_RUN_FEEDBACK ||
            sharedData->system_state == SYSTEM_STATE_CALIBRATION_RUNNING) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else {
            sharedData->motor_abs_limit_valid_bitmap = 0;
            sharedData->motor_abs_limit_locked_bitmap = 0;
            memset((void*)sharedData->motor_abs_limit_min, 0, sizeof(sharedData->motor_abs_limit_min));
            memset((void*)sharedData->motor_abs_limit_max, 0, sizeof(sharedData->motor_abs_limit_max));
            memset(g_jointCalibResult, 0, sizeof(g_jointCalibResult));
            memset(g_encoderOffsetManual, 0, sizeof(g_encoderOffsetManual));
            g_calibrationStatus = CALIB_STATUS_IDLE;
            g_calibrationAckStatus = CALIB_STATUS_IDLE;
            g_calibrationAckRepeatRemaining = 0;
            sharedData->system_fault_bitmap &= ~SYSTEM_FAULT_CALIBRATION;
            postSystemEvent(sharedData, SYSTEM_EVENT_REEVALUATE_READY);
            Serial.println("[UPPER] CLEAR_CALIBRATION: joint calibration and ABS limits cleared");
        }
        sendProtoAckPacket(CMD_CLEAR_CALIBRATION, sharedData->system_state, status);
        return;
    }

    if (cmd == CMD_MULTITURN_NVS_CONTROL)
    {
        uint8_t status = PROTO_ACK_STATUS_OK;
        bool setOk = false;
        bool clearOk = false;
        if (frameLen < 1 + kMultiTurnNvsControlPayloadBytes) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else {
            const uint8_t action = frame[1];
#if MULTITURN_NVS_PERSISTENCE_ENABLE
            if (action == 0) {
                setOk = multiTurnNvsSetEnabled(false);
                clearOk = multiTurnNvsClearSnapshots();
                if (setOk && clearOk) {
                    sharedData->multiturn_nvs_enabled = 0;
                    sharedData->multiturn_nvs_snapshot_valid = 0;
                    sharedData->multiturn_nvs_save_valid_mask = 0;
                } else {
                    status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
                }
            } else if (action == 1) {
                setOk = multiTurnNvsSetEnabled(true);
                if (setOk) {
                    sharedData->multiturn_nvs_enabled = 1;
                    sharedData->multiturn_nvs_snapshot_valid = multiTurnNvsHasSnapshot() ? 1 : 0;
                } else {
                    status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
                }
            } else {
                status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
            }
#else
            (void)action;
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
#endif
        }
        // Prioritize the machine-readable ACK over diagnostic text. During
        // servo hotplug recovery several tasks can log at once on this port.
        sendProtoAckPacket(CMD_MULTITURN_NVS_CONTROL, sharedData->system_state, status);
        if (frameLen >= 1 + kMultiTurnNvsControlPayloadBytes && frame[1] == 0) {
            if (status == PROTO_ACK_STATUS_OK) {
                Serial.println("[NVS MULTITURN] control enabled=0 clear=1");
            } else {
                Serial.printf("[NVS MULTITURN] control disable failed set=%u clear=%u\r\n",
                              setOk ? 1U : 0U,
                              clearOk ? 1U : 0U);
            }
        } else if (frameLen >= 1 + kMultiTurnNvsControlPayloadBytes && frame[1] == 1) {
            if (status == PROTO_ACK_STATUS_OK) {
                Serial.printf("[NVS MULTITURN] control enabled=1 snapshot=%u\r\n",
                              sharedData->multiturn_nvs_snapshot_valid ? 1U : 0U);
            } else {
                Serial.println("[NVS MULTITURN] control enable failed");
            }
        }
        return;
    }

    if (cmd == CMD_SENSOR_STREAM_MODE)
    {
        uint8_t status = PROTO_ACK_STATUS_OK;
        if (frameLen < 2) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else {
            const uint8_t requestedMode = frame[1];
            if (requestedMode == SENSOR_STREAM_MODE_LEGACY_U16_WRAP ||
                requestedMode == SENSOR_STREAM_MODE_SIGNED_I16) {
                g_sensorStreamMode = requestedMode;
            } else {
                status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
            }
        }
        sendProtoAckPacket(CMD_SENSOR_STREAM_MODE, g_sensorStreamMode, status);
    }
}

// Serial receive/transmit task.
void upperCommunicationTask(void* parameter)
{
    TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;
    RemoteSensorData_t sensorRawData;
    MappedAngleData_t mappedAngleData;
    uint8_t rxBuffer[kSerialRxBufferSize];
    size_t rxLen = 0;
    uint32_t lastSensorTimestampSent = 0;
    uint32_t lastEncoderRawTimestampSent = 0;

    uint32_t lastFaultStatusSent = 0;
    uint32_t lastFaultStatusSentMs = 0;
    bool faultStatusSentInitialized = false;
    uint32_t lastReleaseFaultSent = 0;
    uint32_t lastReleaseFaultSentMs = 0;
    bool releaseFaultSentInitialized = false;
    uint32_t lastSystemStatusTransitionSent = 0;
    uint32_t lastSystemStatusSentMs = 0;
    bool systemStatusSentInitialized = false;
    uint32_t lastMotorAbsLimitSentMs = 0;
    uint32_t lastMotorAbsLimitValidSent = 0xFFFFFFFFUL;
    uint32_t lastMotorAbsLimitLockedSent = 0xFFFFFFFFUL;
    uint32_t lastJointCalibSnapshotSentMs = 0;
    uint8_t nextJointCalibSnapshotIndex = 0;
    uint32_t lastCalibrationDiagSequenceSent = 0;
    uint32_t lastCalibrationDiagSentMs = 0;

    Serial.println("<<<SYS_READY>>>");

    for (;;)
    {
        while (Serial.available())
        {
            const int byteVal = Serial.read();
            if (byteVal < 0) {
                break;
            }

            if (rxLen < sizeof(rxBuffer)) {
                rxBuffer[rxLen++] = (uint8_t)byteVal;
            } else {
                memmove(rxBuffer, rxBuffer + 1, sizeof(rxBuffer) - 1);
                rxBuffer[sizeof(rxBuffer) - 1] = (uint8_t)byteVal;
                rxLen = sizeof(rxBuffer);
            }
        }

        size_t parseOffset = 0;
        while (parseOffset < rxLen)
        {
            const uint8_t cur = rxBuffer[parseOffset];

            if (cur == (uint8_t)'c' || cur == (uint8_t)'b')
            {
                handleLegacySingleByteCommand(sharedData, cur);
                parseOffset += 1;
                continue;
            }

            if (cur != PROTOCOL_HEADER)
            {
                parseOffset += 1;
                continue;
            }

            if ((parseOffset + 4) > rxLen)
            {
                break;
            }

            const uint8_t wireLen = rxBuffer[parseOffset + 1];
            if (wireLen < 2)
            {
                parseOffset += 1;
                continue;
            }

            const size_t frameLen = (size_t)wireLen + 2;
            if ((parseOffset + frameLen) > rxLen)
            {
                break;
            }

            if (rxBuffer[parseOffset + frameLen - 1] != PROTOCOL_TAIL)
            {
                parseOffset += 1;
                continue;
            }

            const uint8_t cmd = rxBuffer[parseOffset + 2];
            const size_t payloadLen = (size_t)wireLen - 2; // CMD + payload + tail
            const size_t expectedPayloadLen = getCommandPayloadLength(cmd);
            if (expectedPayloadLen == (size_t)-1 || expectedPayloadLen != payloadLen)
            {
                parseOffset += frameLen;
                continue;
            }

            handleParsedCommand(sharedData, rxBuffer + parseOffset + 2, payloadLen + 1);
            parseOffset += frameLen;
        }

        if (parseOffset > 0)
        {
            const size_t remain = rxLen - parseOffset;
            if (remain > 0) {
                memmove(rxBuffer, rxBuffer + parseOffset, remain);
            }
            rxLen = remain;
        }

        bool sentPacket = false;
        if (sharedData->mappedAngleQueue &&
            xQueuePeek(sharedData->mappedAngleQueue, &mappedAngleData, 0) == pdTRUE &&
            mappedAngleData.timestamp != lastSensorTimestampSent)
        {
            sendDataPacket(NULL, NULL, &mappedAngleData, NULL, NULL, NULL, NULL, NULL);
            lastSensorTimestampSent = mappedAngleData.timestamp;
            sentPacket = true;
        }
        else if (sharedData->canRxQueue &&
            xQueuePeek(sharedData->canRxQueue, &sensorRawData, 0) == pdTRUE &&
            sensorRawData.timestamp != lastSensorTimestampSent)
        {
            sendDataPacket(NULL, &sensorRawData, NULL, NULL, NULL, NULL, NULL, NULL);
            lastSensorTimestampSent = sensorRawData.timestamp;
            sentPacket = true;
        }

        if (sharedData->canRxQueue &&
            xQueuePeek(sharedData->canRxQueue, &sensorRawData, 0) == pdTRUE &&
            sensorRawData.timestamp != lastEncoderRawTimestampSent)
        {
            sendEncoderRawPacket(sensorRawData);
            lastEncoderRawTimestampSent = sensorRawData.timestamp;
        }

        RemoteTactileData_t tactileData;
        if (sharedData->tactileQueue &&
            xQueueReceive(sharedData->tactileQueue, &tactileData, 0) == pdTRUE)
        {
            sendDataPacket(NULL, NULL, NULL, NULL, NULL, NULL, &tactileData, NULL);
            sentPacket = true;
        }

        JointDebugData_t jointDebugData;
        uint8_t jointDebugPacketsSent = 0;
        static const uint8_t kMaxJointDebugPacketsPerLoop = 4;
        while (jointDebugPacketsSent < kMaxJointDebugPacketsPerLoop &&
               xQueueReceive(sharedData->jointDebugQueue, &jointDebugData, 0) == pdTRUE)
        {
            sendDataPacket(NULL, NULL, NULL, NULL, NULL, NULL, NULL, &jointDebugData);
            jointDebugPacketsSent++;
            sentPacket = true;
        }

        ServoAngleData_t servoRawData;
        if (xQueueReceive(sharedData->servoRawQueue, &servoRawData, 0) == pdTRUE)
        {
            sendDataPacket(NULL, NULL, NULL, NULL, &servoRawData, NULL, NULL, NULL);
            sentPacket = true;
        }

        ServoTelemetryData_t telemetryData;
        if (xQueueReceive(sharedData->servoTelemetryQueue, &telemetryData, 0) == pdTRUE)
        {
            sendDataPacket(NULL, NULL, NULL, NULL, NULL, &telemetryData, NULL, NULL);
            sentPacket = true;
        }

        bool sentServoAngle = false;
        ServoAngleData_t servoAngleData;
        if (xQueueReceive(sharedData->servoAngleQueue, &servoAngleData, 0) == pdTRUE)
        {
            sendDataPacket(NULL, NULL, NULL, &servoAngleData, NULL, NULL, NULL, NULL);
            sentServoAngle = true;
            sentPacket = true;
        }

        if (!sentPacket && !sentServoAngle && calibrationAckPending())
        {
            sendDataPacket(NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
        }

        const uint32_t nowMs = millis();
        const uint32_t faultBitmap = sharedData->overload_fault_bitmap;
        const bool bitmapChanged =
            (!faultStatusSentInitialized) || (faultBitmap != lastFaultStatusSent);
        const bool heartbeatDue =
            (!faultStatusSentInitialized) || ((nowMs - lastFaultStatusSentMs) >= kFaultStatusHeartbeatMs);
        if (bitmapChanged || heartbeatDue) {
            sendFaultStatusPacket(sharedData, faultBitmap);
            lastFaultStatusSent = faultBitmap;
            lastFaultStatusSentMs = nowMs;
            faultStatusSentInitialized = true;
        }

        const uint32_t releaseFaultBitmap = sharedData->reverse_release_fault_bitmap;
        const bool releaseBitmapChanged =
            (!releaseFaultSentInitialized) || (releaseFaultBitmap != lastReleaseFaultSent);
        const bool releaseHeartbeatDue =
            (!releaseFaultSentInitialized) || ((nowMs - lastReleaseFaultSentMs) >= kFaultStatusHeartbeatMs);
        if (releaseBitmapChanged || releaseHeartbeatDue) {
            sendReleaseFaultPacket(releaseFaultBitmap);
            lastReleaseFaultSent = releaseFaultBitmap;
            lastReleaseFaultSentMs = nowMs;
            releaseFaultSentInitialized = true;
        }

        const uint32_t stateTransitionCount = sharedData->state_transition_count;
        const bool systemStatusChanged =
            (!systemStatusSentInitialized) ||
            (stateTransitionCount != lastSystemStatusTransitionSent);
        const bool systemStatusHeartbeatDue =
            (!systemStatusSentInitialized) ||
            ((nowMs - lastSystemStatusSentMs) >= kSystemStatusHeartbeatMs);
        if (systemStatusChanged || systemStatusHeartbeatDue) {
            sendSystemStatusPacket(sharedData);
            lastSystemStatusTransitionSent = stateTransitionCount;
            lastSystemStatusSentMs = nowMs;
            systemStatusSentInitialized = true;
        }

        const uint32_t limitValidBitmap = sharedData->motor_abs_limit_valid_bitmap;
        const uint32_t limitLockedBitmap = sharedData->motor_abs_limit_locked_bitmap;
        if (limitValidBitmap != lastMotorAbsLimitValidSent ||
            limitLockedBitmap != lastMotorAbsLimitLockedSent ||
            (uint32_t)(nowMs - lastMotorAbsLimitSentMs) >= kMotorAbsLimitHeartbeatMs) {
            sendMotorAbsLimitPacket(sharedData);
            lastMotorAbsLimitValidSent = limitValidBitmap;
            lastMotorAbsLimitLockedSent = limitLockedBitmap;
            lastMotorAbsLimitSentMs = nowMs;
        }

        if (g_calibrationStatus == CALIB_STATUS_SUCCESS &&
            (uint32_t)(nowMs - lastJointCalibSnapshotSentMs) >= kJointCalibSnapshotHeartbeatMs) {
            for (uint8_t n = 0; n < ENCODER_TOTAL_NUM; n++) {
                const uint8_t jointIndex = (uint8_t)((nextJointCalibSnapshotIndex + n) % ENCODER_TOTAL_NUM);
                if (g_jointCalibResult[jointIndex].success) {
                    sendJointCalibSnapshotPacket(jointIndex);
                }
            }
            nextJointCalibSnapshotIndex = 0;
            lastJointCalibSnapshotSentMs = nowMs;
        }

        const CalibrationDiagData_t calibDiag = sharedData->calibration_diag;
        if (calibDiag.sequence != lastCalibrationDiagSequenceSent ||
            (calibDiag.valid &&
             (uint32_t)(nowMs - lastCalibrationDiagSentMs) >= kSystemStatusHeartbeatMs)) {
            sendCalibrationDiagPacket(calibDiag);
            lastCalibrationDiagSequenceSent = calibDiag.sequence;
            lastCalibrationDiagSentMs = nowMs;
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
