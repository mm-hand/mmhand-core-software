#include "UpperCommTask.h"
#include "../can/CanCommTask.h"
#include "../../shared/TaskSharedData.h"
#include "../../calibration/CalibrationTask.h"
#include "../../hardware/HardwareMap.h"
#include "../../system/StateMachineTask.h"
#include "UpperCommCommandRouter.h"
#include "UpperCommProtocol.h"

#include <math.h>
#include <string.h>

extern volatile uint8_t g_calibrationUIStatus;

// UpperCommTask responsibilities:
// 1) parse host commands and update shared command state;
// 2) publish sensor, servo, telemetry, debug, and fault packets;
// 3) keep protocol ACK and fault heartbeat reporting synchronized.
static uint8_t g_sensorStreamMode = SENSOR_STREAM_MODE_SIGNED_I16;
static const uint16_t kEncoderDisconnectRaw = 0xFFFF;

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
    Serial.write(buffer, idx);
}

// Send the current state-machine status.
static void sendSystemStatusPacket(TaskSharedData_t* sharedData)
{
    if (!sharedData) {
        return;
    }

    uint8_t buffer[24];
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
    buffer[idx++] = g_calibrationUIStatus;
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
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = (uint8_t)(idx - 2);
    Serial.write(buffer, idx);
}

// Send the servo overload fault bitmap.
static void sendFaultStatusPacket(uint32_t overloadFaultBitmap)
{
    uint8_t buffer[8];
    size_t idx = 0;

    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00;
    buffer[idx++] = PACKET_TYPE_FAULT_STATUS;
    buffer[idx++] = (uint8_t)((overloadFaultBitmap >> 24) & 0xFF);
    buffer[idx++] = (uint8_t)((overloadFaultBitmap >> 16) & 0xFF);
    buffer[idx++] = (uint8_t)((overloadFaultBitmap >> 8) & 0xFF);
    buffer[idx++] = (uint8_t)(overloadFaultBitmap & 0xFF);
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = (uint8_t)(idx - 2);
    Serial.write(buffer, idx);
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
    Serial.write(buffer, idx);
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
    Serial.write(buffer, idx);
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
    Serial.write(buffer, idx);
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

    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00; // LEN placeholder.
    if (g_calibrationUIStatus != 0)
    {
        buffer[idx++] = PACKET_TYPE_CALIB_ACK;
        buffer[idx++] = g_calibrationUIStatus;
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
    Serial.write(buffer, idx);

    if (g_calibrationUIStatus != 0) {
        g_calibrationUIStatus = 0;
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
            return 0;
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
        case CMD_TENDON_GUARD:
            return kTendonGuardPayloadBytes;
        case CMD_ACTIVE_SCOPE:
            return kActiveScopePayloadBytes;
        case CMD_RESTORE_MULTITURN:
            return kRestoreMultiTurnPayloadBytes;
        case CMD_MOTOR_ABS_LIMIT_LOCK:
            return kMotorAbsLimitLockPayloadBytes;
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

    if (cmd == CMD_CALIBRATE)
    {
        postSystemEvent(sharedData, SYSTEM_EVENT_CALIBRATE);
        sendProtoAckPacket(CMD_CALIBRATE, sharedData->system_state, PROTO_ACK_STATUS_OK);
        return;
    }

    if (cmd == CMD_START)
    {
        postSystemEvent(sharedData, SYSTEM_EVENT_START);
        sendProtoAckPacket(CMD_START, sharedData->system_state, PROTO_ACK_STATUS_OK);
        return;
    }

    if (cmd == CMD_RUN_FEEDBACK)
    {
        postSystemEvent(sharedData, SYSTEM_EVENT_RUN_FEEDBACK);
        sendProtoAckPacket(CMD_RUN_FEEDBACK, sharedData->system_state, PROTO_ACK_STATUS_OK);
        return;
    }

    if (cmd == CMD_PAUSE)
    {
        postSystemEvent(sharedData, SYSTEM_EVENT_PAUSE);
        sendProtoAckPacket(CMD_PAUSE, sharedData->system_state, PROTO_ACK_STATUS_OK);
        return;
    }

    if (cmd == CMD_STOP)
    {
        postSystemEvent(sharedData, SYSTEM_EVENT_STOP);
        sendProtoAckPacket(CMD_STOP, sharedData->system_state, PROTO_ACK_STATUS_OK);
        return;
    }

    if (cmd == CMD_RESET)
    {
        postSystemEvent(sharedData, SYSTEM_EVENT_RESET);
        sendProtoAckPacket(CMD_RESET, sharedData->system_state, PROTO_ACK_STATUS_OK);
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

    if (cmd == CMD_TENDON_GUARD)
    {
        uint8_t enabled[ENCODER_TOTAL_NUM] = {0};
        int8_t sign[ENCODER_TOTAL_NUM] = {0};
        int16_t x1Abs[ENCODER_TOTAL_NUM] = {0};
        if (parseTendonGuardPayload(frame + 1, frameLen - 1, enabled, sign, x1Abs)) {
            upperApplyTendonGuardConfig(sharedData, enabled, sign, x1Abs, ENCODER_TOTAL_NUM);
        }
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
                upperApplyActiveScope(sharedData,
                                      activeMotorMask,
                                      activeJointMask,
                                      requireCanFeedback);
            }
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
        while (xQueueReceive(sharedData->jointDebugQueue, &jointDebugData, 0) == pdTRUE)
        {
            sendDataPacket(NULL, NULL, NULL, NULL, NULL, NULL, NULL, &jointDebugData);
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

        if (!sentPacket && !sentServoAngle && g_calibrationUIStatus != 0)
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
            sendFaultStatusPacket(faultBitmap);
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

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
