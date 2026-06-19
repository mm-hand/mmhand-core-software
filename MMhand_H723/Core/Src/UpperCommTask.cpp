#include "UpperCommTask.h"

#include "UpperCommCommandRouter.h"
#include "UsbCommTask.h"
#include "main.h"
#include "task.h"

#include <cstring>

UpperComm upper1;
volatile uint8_t g_calibrationUIStatus = 0;

static const uint16_t kEncoderDisconnectRaw = 0xFFFFU;

// 保持 freertos.c 里的 C 任务入口不变，把实际上位机通信逻辑转交给 upper1 对象。
extern "C" void UpperCommTask_Run(void *parameter)
{
    upper1.run(parameter);
}

// 返回当前系统毫秒计时，保持与 P4 侧 millis() 调用语义一致。
uint32_t UpperComm::millis(void) const
{
    return HAL_GetTick();
}

// 按大端字节序追加 float，保持与 P4 上行调试包兼容。
static void appendFloatBigEndian(uint8_t *buffer, size_t *idx, float value)
{
    uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    buffer[(*idx)++] = static_cast<uint8_t>((bits >> 24) & 0xFFU);
    buffer[(*idx)++] = static_cast<uint8_t>((bits >> 16) & 0xFFU);
    buffer[(*idx)++] = static_cast<uint8_t>((bits >> 8) & 0xFFU);
    buffer[(*idx)++] = static_cast<uint8_t>(bits & 0xFFU);
}

// 将原始磁编码器值编码成上位机协议中的断连/有效值。
static uint16_t encodeRawEncoderForWire(uint16_t rawValue, bool channelValid)
{
    if (!channelValid) {
        return static_cast<uint16_t>(PROTOCOL_DISCONNECT_SENTINEL);
    }

    if ((rawValue == kEncoderDisconnectRaw) ||
        (rawValue == static_cast<uint16_t>(PROTOCOL_DISCONNECT_SENTINEL))) {
        return static_cast<uint16_t>(PROTOCOL_DISCONNECT_SENTINEL);
    }

    return rawValue;
}

// 读取大端 uint32，供 ACTIVE_SCOPE 和位图类命令解析使用。
static uint32_t readU32BigEndian(const uint8_t *p)
{
    return (static_cast<uint32_t>(p[0]) << 24) |
           (static_cast<uint32_t>(p[1]) << 16) |
           (static_cast<uint32_t>(p[2]) << 8) |
           static_cast<uint32_t>(p[3]);
}

// 解码下位机命令中的小端 float。
static float decodeFloatLittleEndian(const uint8_t *data)
{
    uint32_t bits = 0;
    bits |= static_cast<uint32_t>(data[0]);
    bits |= (static_cast<uint32_t>(data[1]) << 8);
    bits |= (static_cast<uint32_t>(data[2]) << 16);
    bits |= (static_cast<uint32_t>(data[3]) << 24);
    float out = 0.0f;
    std::memcpy(&out, &bits, sizeof(out));
    return out;
}

// 解析下位机命令中的小端 float 数组。
static bool parseFloatArrayLittleEndian(const uint8_t *payload,
                                        size_t payloadLen,
                                        float *outValues,
                                        uint8_t count)
{
    if ((payload == nullptr) || (outValues == nullptr)) {
        return false;
    }
    const size_t required = static_cast<size_t>(count) * sizeof(float);
    if (payloadLen < required) {
        return false;
    }
    for (uint8_t i = 0; i < count; i++) {
        outValues[i] = decodeFloatLittleEndian(payload + static_cast<size_t>(i) * sizeof(float));
    }
    return true;
}

// 解析下位机命令中的大端 int16 数组，并扩展到 int32 缓存。
static bool parseInt16ArrayBigEndian(const uint8_t *payload,
                                     size_t payloadLen,
                                     int32_t *outValues,
                                     uint8_t count)
{
    if ((payload == nullptr) || (outValues == nullptr)) {
        return false;
    }
    const size_t required = static_cast<size_t>(count) * sizeof(uint16_t);
    if (payloadLen < required) {
        return false;
    }
    for (uint8_t i = 0; i < count; i++) {
        const size_t base = static_cast<size_t>(i) * 2U;
        const uint16_t raw = (static_cast<uint16_t>(payload[base]) << 8) | payload[base + 1U];
        outValues[i] = static_cast<int16_t>(raw);
    }
    return true;
}

// 解析 tendon guard 命令载荷。
static bool parseTendonGuardPayload(const uint8_t *payload,
                                    size_t payloadLen,
                                    uint8_t *enabled,
                                    int8_t *sign,
                                    int16_t *x1Abs)
{
    if ((payload == nullptr) || (enabled == nullptr) || (sign == nullptr) || (x1Abs == nullptr)) {
        return false;
    }
    if (payloadLen < kTendonGuardPayloadBytes) {
        return false;
    }
    for (uint8_t i = 0; i < ENCODER_TOTAL_NUM; i++) {
        const size_t base = static_cast<size_t>(i) * 4U;
        enabled[i] = (payload[base] != 0U) ? 1U : 0U;
        sign[i] = (static_cast<int8_t>(payload[base + 1U]) < 0) ? static_cast<int8_t>(-1) : static_cast<int8_t>(1);
        const uint16_t raw = (static_cast<uint16_t>(payload[base + 2U]) << 8) | payload[base + 3U];
        x1Abs[i] = clampServoAbsCommand(static_cast<int16_t>(raw));
    }
    return true;
}

// 查询命令载荷长度，返回值不包含命令字节本身。
static size_t getCommandPayloadLength(uint8_t cmd)
{
    switch (cmd) {
    case CMD_CALIBRATE:
    case CMD_START:
    case CMD_STOP:
    case CMD_RESET:
    case CMD_RUN_FEEDBACK:
    case CMD_PAUSE:
        return 0;
    case CMD_ANGLE_CTRL:
    case CMD_CALIB_DATA:
        return kFloatPayloadBytes;
    case CMD_MOTOR_POS:
    case CMD_MOTOR_POS_SWEEP:
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
        return static_cast<size_t>(-1);
    }
}

// 将一帧上位机协议数据排入 USB 发送队列，并同步保存到 upper1.TX* 调试字段。
bool UpperComm::sendPacket(const uint8_t *data, uint16_t len, uint8_t packetType)
{
    if ((data == nullptr) || (len == 0U) || (len > USB_COMM_MAX_DATA_SIZE)) {
        TXdropCount++;
        return false;
    }

    TXlen = len;
    TXpacketType = packetType;
    lastTxTimestamp = millis();
    std::memset(TXdata, 0, sizeof(TXdata));
    std::memcpy(TXdata, data, len);

    UsbPacket_t packet = {};
    packet.len = len;
    packet.timestamp = lastTxTimestamp;
    std::memcpy(packet.payload, data, len);

    if ((shared == nullptr) || (shared->usbTxQueue == nullptr)) {
        TXdropCount++;
        return false;
    }

    if (xQueueSendToBack(shared->usbTxQueue, &packet, 0) != pdTRUE) {
        TXqueueFullCount++;
        UsbPacket_t droppedPacket = {};
        (void)xQueueReceive(shared->usbTxQueue, &droppedPacket, 0);
        if (xQueueSendToBack(shared->usbTxQueue, &packet, 0) != pdTRUE) {
            TXdropCount++;
            return false;
        }
    }

    TXpacketCount++;
    TXbyteCount += len;
    return true;
}

// 发送协议 ACK 包。
static void sendProtoAckPacket(uint8_t cmd, uint8_t appliedMode, uint8_t status)
{
    uint8_t buffer[8] = {};
    size_t idx = 0;

    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00;
    buffer[idx++] = PACKET_TYPE_PROTO_ACK;
    buffer[idx++] = cmd;
    buffer[idx++] = appliedMode;
    buffer[idx++] = status;
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = static_cast<uint8_t>(idx - 2U);
    (void)upper1.sendPacket(buffer, static_cast<uint16_t>(idx), PACKET_TYPE_PROTO_ACK);
}

// 发送当前系统状态心跳包。
static void sendSystemStatusPacket(TaskSharedData_t *sharedData)
{
    if (sharedData == nullptr) {
        return;
    }

    uint8_t buffer[24] = {};
    size_t idx = 0;
    const uint32_t faultBitmap = sharedData->system_fault_bitmap;
    const uint32_t transitionCount = sharedData->state_transition_count;
    const uint32_t unknownBitmap = sharedData->multi_turn_unknown_bitmap;

    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00;
    buffer[idx++] = PACKET_TYPE_SYSTEM_STATUS;
    buffer[idx++] = sharedData->system_state;
    buffer[idx++] = sharedData->control_enabled;
    buffer[idx++] = sharedData->servo_target_owner;
    buffer[idx++] = sharedData->control_mode;
    buffer[idx++] = g_calibrationUIStatus;
    buffer[idx++] = static_cast<uint8_t>((faultBitmap >> 24) & 0xFFU);
    buffer[idx++] = static_cast<uint8_t>((faultBitmap >> 16) & 0xFFU);
    buffer[idx++] = static_cast<uint8_t>((faultBitmap >> 8) & 0xFFU);
    buffer[idx++] = static_cast<uint8_t>(faultBitmap & 0xFFU);
    buffer[idx++] = static_cast<uint8_t>((transitionCount >> 24) & 0xFFU);
    buffer[idx++] = static_cast<uint8_t>((transitionCount >> 16) & 0xFFU);
    buffer[idx++] = static_cast<uint8_t>((transitionCount >> 8) & 0xFFU);
    buffer[idx++] = static_cast<uint8_t>(transitionCount & 0xFFU);
    buffer[idx++] = static_cast<uint8_t>((unknownBitmap >> 24) & 0xFFU);
    buffer[idx++] = static_cast<uint8_t>((unknownBitmap >> 16) & 0xFFU);
    buffer[idx++] = static_cast<uint8_t>((unknownBitmap >> 8) & 0xFFU);
    buffer[idx++] = static_cast<uint8_t>(unknownBitmap & 0xFFU);
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = static_cast<uint8_t>(idx - 2U);
    (void)upper1.sendPacket(buffer, static_cast<uint16_t>(idx), PACKET_TYPE_SYSTEM_STATUS);
}

// 发送过载故障位图。
static void sendFaultStatusPacket(uint32_t overloadFaultBitmap)
{
    uint8_t buffer[8] = {};
    size_t idx = 0;

    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00;
    buffer[idx++] = PACKET_TYPE_FAULT_STATUS;
    buffer[idx++] = static_cast<uint8_t>((overloadFaultBitmap >> 24) & 0xFFU);
    buffer[idx++] = static_cast<uint8_t>((overloadFaultBitmap >> 16) & 0xFFU);
    buffer[idx++] = static_cast<uint8_t>((overloadFaultBitmap >> 8) & 0xFFU);
    buffer[idx++] = static_cast<uint8_t>(overloadFaultBitmap & 0xFFU);
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = static_cast<uint8_t>(idx - 2U);
    (void)upper1.sendPacket(buffer, static_cast<uint16_t>(idx), PACKET_TYPE_FAULT_STATUS);
}

// 发送反向释放保护故障位图。
static void sendReleaseFaultPacket(uint32_t releaseFaultBitmap)
{
    uint8_t buffer[8] = {};
    size_t idx = 0;

    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00;
    buffer[idx++] = PACKET_TYPE_RELEASE_FAULT;
    buffer[idx++] = static_cast<uint8_t>((releaseFaultBitmap >> 24) & 0xFFU);
    buffer[idx++] = static_cast<uint8_t>((releaseFaultBitmap >> 16) & 0xFFU);
    buffer[idx++] = static_cast<uint8_t>((releaseFaultBitmap >> 8) & 0xFFU);
    buffer[idx++] = static_cast<uint8_t>(releaseFaultBitmap & 0xFFU);
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = static_cast<uint8_t>(idx - 2U);
    (void)upper1.sendPacket(buffer, static_cast<uint16_t>(idx), PACKET_TYPE_RELEASE_FAULT);
}

// 发送电机绝对限位状态。
static void sendMotorAbsLimitPacket(TaskSharedData_t *sharedData)
{
    if (sharedData == nullptr) {
        return;
    }

    uint8_t buffer[USB_COMM_MAX_DATA_SIZE] = {};
    size_t idx = 0;
    const uint32_t validBitmap = sharedData->motor_abs_limit_valid_bitmap;
    const uint32_t lockedBitmap = sharedData->motor_abs_limit_locked_bitmap;

    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00;
    buffer[idx++] = PACKET_TYPE_MOTOR_ABS_LIMIT;
    buffer[idx++] = static_cast<uint8_t>((validBitmap >> 24) & 0xFFU);
    buffer[idx++] = static_cast<uint8_t>((validBitmap >> 16) & 0xFFU);
    buffer[idx++] = static_cast<uint8_t>((validBitmap >> 8) & 0xFFU);
    buffer[idx++] = static_cast<uint8_t>(validBitmap & 0xFFU);
    buffer[idx++] = static_cast<uint8_t>((lockedBitmap >> 24) & 0xFFU);
    buffer[idx++] = static_cast<uint8_t>((lockedBitmap >> 16) & 0xFFU);
    buffer[idx++] = static_cast<uint8_t>((lockedBitmap >> 8) & 0xFFU);
    buffer[idx++] = static_cast<uint8_t>(lockedBitmap & 0xFFU);

    for (int i = 0; i < SERVO_TOTAL_NUM; i++) {
        const int16_t minAbs = static_cast<int16_t>(sharedData->motor_abs_limit_min[i]);
        const int16_t maxAbs = static_cast<int16_t>(sharedData->motor_abs_limit_max[i]);
        buffer[idx++] = static_cast<uint8_t>((static_cast<uint16_t>(minAbs) >> 8) & 0xFFU);
        buffer[idx++] = static_cast<uint8_t>(static_cast<uint16_t>(minAbs) & 0xFFU);
        buffer[idx++] = static_cast<uint8_t>((static_cast<uint16_t>(maxAbs) >> 8) & 0xFFU);
        buffer[idx++] = static_cast<uint8_t>(static_cast<uint16_t>(maxAbs) & 0xFFU);
    }

    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = static_cast<uint8_t>(idx - 2U);
    (void)upper1.sendPacket(buffer, static_cast<uint16_t>(idx), PACKET_TYPE_MOTOR_ABS_LIMIT);
}

// 单独发送 raw 编码器包，供上位机查看未映射的 CAN 原始值。
static void sendEncoderRawPacket(const RemoteSensorData_t &sensorRaw)
{
    uint8_t buffer[64] = {};
    size_t idx = 0;

    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00;
    buffer[idx++] = PACKET_TYPE_ENCODER_RAW;
    for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
        const bool channelValid =
            sensorRaw.isValid &&
            (sensorRaw.errorFlags[i] == 0U) &&
            (sensorRaw.encoderValues[i] != kEncoderDisconnectRaw);
        const uint16_t val = encodeRawEncoderForWire(sensorRaw.encoderValues[i], channelValid);
        buffer[idx++] = static_cast<uint8_t>((val >> 8) & 0xFFU);
        buffer[idx++] = static_cast<uint8_t>(val & 0xFFU);
    }
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = static_cast<uint8_t>(idx - 2U);
    (void)upper1.sendPacket(buffer, static_cast<uint16_t>(idx), PACKET_TYPE_ENCODER_RAW);
}

// 按 P4 协议序列化一种上行数据包。
static void sendDataPacket(RemoteSensorData_t *pSensorRaw,
                           MappedAngleData_t *pMappedAngle,
                           ServoAngleData_t *pServoAngle,
                           ServoAngleData_t *pServoRaw,
                           ServoTelemetryData_t *pTelemetry,
                           RemoteTactileData_t *pTactile,
                           JointDebugData_t *pJointDebug)
{
    uint8_t buffer[USB_COMM_MAX_DATA_SIZE] = {};
    size_t idx = 0;
    uint8_t packetType = 0;

    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = 0x00;

    if (g_calibrationUIStatus != 0U) {
        packetType = PACKET_TYPE_CALIB_ACK;
        buffer[idx++] = packetType;
        buffer[idx++] = g_calibrationUIStatus;
    } else if (pSensorRaw != nullptr) {
        packetType = PACKET_TYPE_SENSOR;
        buffer[idx++] = packetType;
        for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
            const bool channelValid =
                pSensorRaw->isValid &&
                (pSensorRaw->errorFlags[i] == 0U) &&
                (pSensorRaw->encoderValues[i] != kEncoderDisconnectRaw);
            const uint16_t val = encodeRawEncoderForWire(pSensorRaw->encoderValues[i], channelValid);
            buffer[idx++] = static_cast<uint8_t>((val >> 8) & 0xFFU);
            buffer[idx++] = static_cast<uint8_t>(val & 0xFFU);
        }
    } else if (pMappedAngle != nullptr) {
        packetType = PACKET_TYPE_SENSOR;
        buffer[idx++] = packetType;
        for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
            const int16_t val = (pMappedAngle->isValid && (pMappedAngle->validFlags[i] != 0U))
                ? pMappedAngle->angleValues[i]
                : static_cast<int16_t>(PROTOCOL_DISCONNECT_SENTINEL);
            const uint16_t raw = static_cast<uint16_t>(val);
            buffer[idx++] = static_cast<uint8_t>((raw >> 8) & 0xFFU);
            buffer[idx++] = static_cast<uint8_t>(raw & 0xFFU);
        }
    } else if (pServoAngle != nullptr) {
        packetType = PACKET_TYPE_SERVO_ANGLE;
        buffer[idx++] = packetType;
        for (int i = 0; i < SERVO_TOTAL_NUM; i++) {
            const int32_t angle = pServoAngle->servoAngles[i];
            buffer[idx++] = static_cast<uint8_t>((angle >> 24) & 0xFFU);
            buffer[idx++] = static_cast<uint8_t>((angle >> 16) & 0xFFU);
            buffer[idx++] = static_cast<uint8_t>((angle >> 8) & 0xFFU);
            buffer[idx++] = static_cast<uint8_t>(angle & 0xFFU);
        }
        for (int i = 0; i < SERVO_TOTAL_NUM; i++) {
            buffer[idx++] = pServoAngle->onlineStatus[i];
        }
    } else if (pServoRaw != nullptr) {
        packetType = PACKET_TYPE_SERVO_RAW;
        buffer[idx++] = packetType;
        for (int i = 0; i < SERVO_TOTAL_NUM; i++) {
            const uint16_t raw = static_cast<uint16_t>(pServoRaw->servoRawPositions[i]);
            buffer[idx++] = static_cast<uint8_t>((raw >> 8) & 0xFFU);
            buffer[idx++] = static_cast<uint8_t>(raw & 0xFFU);
        }
        for (int i = 0; i < SERVO_TOTAL_NUM; i++) {
            buffer[idx++] = pServoRaw->onlineStatus[i];
        }
    } else if (pTelemetry != nullptr) {
        packetType = PACKET_TYPE_SERVO_TELEM;
        buffer[idx++] = packetType;
        for (int i = 0; i < SERVO_TOTAL_NUM; i++) {
            const int16_t speed = pTelemetry->speed[i];
            const int16_t load = pTelemetry->load[i];
            buffer[idx++] = static_cast<uint8_t>((static_cast<uint16_t>(speed) >> 8) & 0xFFU);
            buffer[idx++] = static_cast<uint8_t>(static_cast<uint16_t>(speed) & 0xFFU);
            buffer[idx++] = static_cast<uint8_t>((static_cast<uint16_t>(load) >> 8) & 0xFFU);
            buffer[idx++] = static_cast<uint8_t>(static_cast<uint16_t>(load) & 0xFFU);
            buffer[idx++] = pTelemetry->voltage[i];
            buffer[idx++] = pTelemetry->temperature[i];
            buffer[idx++] = pTelemetry->onlineStatus[i];
        }
    } else if (pTactile != nullptr) {
        packetType = PACKET_TYPE_TACTILE;
        buffer[idx++] = packetType;
        buffer[idx++] = pTactile->seq;
        for (int i = 0; i < TACTILE_SUMMARY_BYTES; i++) {
            buffer[idx++] = pTactile->values[i];
        }
    } else if (pJointDebug != nullptr) {
        packetType = PACKET_TYPE_JOINT1_DEBUG;
        buffer[idx++] = packetType;
        buffer[idx++] = pJointDebug->jointIndex;
        buffer[idx++] = pJointDebug->valid;
        appendFloatBigEndian(buffer, &idx, pJointDebug->targetDeg);
        appendFloatBigEndian(buffer, &idx, pJointDebug->magActualDeg);
        appendFloatBigEndian(buffer, &idx, pJointDebug->loop1Output);
        appendFloatBigEndian(buffer, &idx, pJointDebug->loop2Actual);
        appendFloatBigEndian(buffer, &idx, pJointDebug->loop2Output);
        buffer[idx++] = pJointDebug->cmdValid;
        const uint16_t cmdPosRaw = static_cast<uint16_t>(pJointDebug->cmdTargetPos);
        buffer[idx++] = static_cast<uint8_t>((cmdPosRaw >> 8) & 0xFFU);
        buffer[idx++] = static_cast<uint8_t>(cmdPosRaw & 0xFFU);
    } else {
        return;
    }

    buffer[idx++] = PROTOCOL_TAIL;
    buffer[1] = static_cast<uint8_t>(idx - 2U);
    (void)upper1.sendPacket(buffer, static_cast<uint16_t>(idx), packetType);

    if (g_calibrationUIStatus != 0U) {
        g_calibrationUIStatus = 0;
    }
}

// 清空 upper1 的调试字段和心跳去重状态。
void UpperComm::resetDebugState(void)
{
    RXbyte = 0;
    RXlen = 0;
    RXusbPacketCount = 0;
    RXbyteCount = 0;
    RXframeCount = 0;
    RXlegacyCommandCount = 0;
    RXinvalidFrameCount = 0;
    RXunsupportedCommandCount = 0;
    RXoverflowCount = 0;
    lastCommand = 0;
    lastCommandPayloadLen = 0;
    lastCommandTimestamp = 0;
    TXlen = 0;
    TXpacketType = 0;
    TXpacketCount = 0;
    TXbyteCount = 0;
    TXdropCount = 0;
    TXqueueFullCount = 0;
    lastTxTimestamp = 0;

    lastSensorTimestampSent = 0;
    lastEncoderRawTimestampSent = 0;
    lastFaultStatusSent = 0;
    lastFaultStatusSentMs = 0;
    faultStatusSentInitialized = 0;
    lastReleaseFaultSent = 0;
    lastReleaseFaultSentMs = 0;
    releaseFaultSentInitialized = 0;
    lastSystemStatusTransitionSent = 0;
    lastSystemStatusSentMs = 0;
    systemStatusSentInitialized = 0;
    lastMotorAbsLimitSentMs = 0;
    lastMotorAbsLimitValidSent = 0xFFFFFFFFUL;
    lastMotorAbsLimitLockedSent = 0xFFFFFFFFUL;

    std::memset(RXbuffer, 0, sizeof(RXbuffer));
    std::memset(TXdata, 0, sizeof(TXdata));
    std::memset(&lastSensorRaw, 0, sizeof(lastSensorRaw));
    std::memset(&lastTactile, 0, sizeof(lastTactile));
    std::memset(&lastMappedAngle, 0, sizeof(lastMappedAngle));
    std::memset(&lastServoAngle, 0, sizeof(lastServoAngle));
    std::memset(&lastServoRaw, 0, sizeof(lastServoRaw));
    std::memset(&lastTelemetry, 0, sizeof(lastTelemetry));
    std::memset(&lastJointDebug, 0, sizeof(lastJointDebug));
}

// 把 USB CDC 收到的一包原始数据追加到上位机协议解析缓存中。
void UpperComm::handleUsbPacket(const UsbPacket_t &packet)
{
    RXusbPacketCount++;
    RXbyteCount += packet.len;

    for (uint16_t i = 0; i < packet.len; i++) {
        RXbyte = packet.payload[i];
        if (RXlen < sizeof(RXbuffer)) {
            RXbuffer[RXlen++] = RXbyte;
        } else {
            std::memmove(RXbuffer, RXbuffer + 1, sizeof(RXbuffer) - 1U);
            RXbuffer[sizeof(RXbuffer) - 1U] = RXbyte;
            RXoverflowCount++;
        }
    }

    parseRxBuffer();
}

// 从累计接收缓存中拆出完整协议帧，支持历史单字节 'c'/'b' 命令。
void UpperComm::parseRxBuffer(void)
{
    size_t parseOffset = 0;

    while (parseOffset < RXlen) {
        const uint8_t cur = RXbuffer[parseOffset];

        if ((cur == static_cast<uint8_t>('c')) || (cur == static_cast<uint8_t>('b'))) {
            handleLegacySingleByteCommand(cur);
            parseOffset += 1U;
            continue;
        }

        if (cur != PROTOCOL_HEADER) {
            parseOffset += 1U;
            continue;
        }

        if ((parseOffset + 4U) > RXlen) {
            break;
        }

        const uint8_t wireLen = RXbuffer[parseOffset + 1U];
        if (wireLen < 2U) {
            RXinvalidFrameCount++;
            parseOffset += 1U;
            continue;
        }

        const size_t wholeFrameLen = static_cast<size_t>(wireLen) + 2U;
        if ((parseOffset + wholeFrameLen) > RXlen) {
            break;
        }

        if (RXbuffer[parseOffset + wholeFrameLen - 1U] != PROTOCOL_TAIL) {
            RXinvalidFrameCount++;
            parseOffset += 1U;
            continue;
        }

        const uint8_t cmd = RXbuffer[parseOffset + 2U];
        const size_t payloadLen = static_cast<size_t>(wireLen) - 2U;
        const size_t expectedPayloadLen = getCommandPayloadLength(cmd);
        if ((expectedPayloadLen == static_cast<size_t>(-1)) || (expectedPayloadLen != payloadLen)) {
            RXunsupportedCommandCount++;
            parseOffset += wholeFrameLen;
            continue;
        }

        handleParsedCommand(RXbuffer + parseOffset + 2U, static_cast<uint16_t>(payloadLen + 1U));
        parseOffset += wholeFrameLen;
    }

    if (parseOffset > 0U) {
        const size_t remain = RXlen - parseOffset;
        if (remain > 0U) {
            std::memmove(RXbuffer, RXbuffer + parseOffset, remain);
        }
        RXlen = static_cast<uint16_t>(remain);
    }
}

// 处理 P4 兼容的历史单字节命令。
void UpperComm::handleLegacySingleByteCommand(uint8_t command)
{
    RXlegacyCommandCount++;
    lastCommand = command;
    lastCommandPayloadLen = 0;
    lastCommandTimestamp = millis();

    if (command == static_cast<uint8_t>('c')) {
        postSystemEvent(shared, SYSTEM_EVENT_CALIBRATE);
        return;
    }
    if (command == static_cast<uint8_t>('b')) {
        upperClearTargetAngles(shared);
    }
}

// 按 P4 逻辑应用一帧已经校验过的下行命令。
void UpperComm::handleParsedCommand(const uint8_t *frame, uint16_t frameLen)
{
    if ((shared == nullptr) || (frame == nullptr) || (frameLen == 0U)) {
        return;
    }

    const uint8_t cmd = frame[0];
    lastCommand = cmd;
    lastCommandPayloadLen = static_cast<uint16_t>(frameLen - 1U);
    lastCommandTimestamp = millis();
    RXframeCount++;

    if (cmd == CMD_CALIBRATE) {
        postSystemEvent(shared, SYSTEM_EVENT_CALIBRATE);
        sendProtoAckPacket(CMD_CALIBRATE, shared->system_state, PROTO_ACK_STATUS_OK);
        return;
    }

    if (cmd == CMD_START) {
        postSystemEvent(shared, SYSTEM_EVENT_START);
        sendProtoAckPacket(CMD_START, shared->system_state, PROTO_ACK_STATUS_OK);
        return;
    }

    if (cmd == CMD_RUN_FEEDBACK) {
        postSystemEvent(shared, SYSTEM_EVENT_RUN_FEEDBACK);
        sendProtoAckPacket(CMD_RUN_FEEDBACK, shared->system_state, PROTO_ACK_STATUS_OK);
        return;
    }

    if (cmd == CMD_PAUSE) {
        postSystemEvent(shared, SYSTEM_EVENT_PAUSE);
        sendProtoAckPacket(CMD_PAUSE, shared->system_state, PROTO_ACK_STATUS_OK);
        return;
    }

    if (cmd == CMD_STOP) {
        postSystemEvent(shared, SYSTEM_EVENT_STOP);
        sendProtoAckPacket(CMD_STOP, shared->system_state, PROTO_ACK_STATUS_OK);
        return;
    }

    if (cmd == CMD_RESET) {
        postSystemEvent(shared, SYSTEM_EVENT_RESET);
        sendProtoAckPacket(CMD_RESET, shared->system_state, PROTO_ACK_STATUS_OK);
        return;
    }

    if (cmd == CMD_ANGLE_CTRL) {
        float parsedAngles[ENCODER_TOTAL_NUM] = {};
        if (parseFloatArrayLittleEndian(frame + 1, frameLen - 1U, parsedAngles, ENCODER_TOTAL_NUM)) {
            upperApplyTargetAngles(shared, parsedAngles, ENCODER_TOTAL_NUM);
        }
        return;
    }

    if (cmd == CMD_CALIB_DATA) {
        float zeroRaw[ENCODER_TOTAL_NUM] = {};
        bool ok = false;
        if (parseFloatArrayLittleEndian(frame + 1, frameLen - 1U, zeroRaw, ENCODER_TOTAL_NUM)) {
            upperCacheCalibZeroRaw(shared, zeroRaw, ENCODER_TOTAL_NUM);
            ok = true;
        }
        sendProtoAckPacket(CMD_CALIB_DATA,
                           shared->system_state,
                           ok ? PROTO_ACK_STATUS_OK : PROTO_ACK_STATUS_UNSUPPORTED_MODE);
        return;
    }

    if (cmd == CMD_MOTOR_POS) {
        int32_t motorTargets[SERVO_TOTAL_NUM] = {};
        if (parseInt16ArrayBigEndian(frame + 1, frameLen - 1U, motorTargets, SERVO_TOTAL_NUM)) {
            upperApplyMotorTargets(shared, motorTargets, SERVO_TOTAL_NUM, MOTOR_DIRECT_SOURCE_TARGET);
        }
        return;
    }

    if (cmd == CMD_MOTOR_POS_SWEEP) {
        int32_t motorTargets[SERVO_TOTAL_NUM] = {};
        if (parseInt16ArrayBigEndian(frame + 1, frameLen - 1U, motorTargets, SERVO_TOTAL_NUM)) {
            upperApplyMotorSweepTargets(shared, motorTargets, SERVO_TOTAL_NUM);
        }
        return;
    }

    if (cmd == CMD_MOTOR_POS_ABS) {
        int32_t motorTargets[SERVO_TOTAL_NUM] = {};
        if (parseInt16ArrayBigEndian(frame + 1, frameLen - 1U, motorTargets, SERVO_TOTAL_NUM)) {
            upperApplyMotorTargets(shared, motorTargets, SERVO_TOTAL_NUM, MOTOR_DIRECT_SOURCE_ABSOLUTE);
        }
        return;
    }

    if (cmd == CMD_TENDON_GUARD) {
        uint8_t enabled[ENCODER_TOTAL_NUM] = {};
        int8_t sign[ENCODER_TOTAL_NUM] = {};
        int16_t x1Abs[ENCODER_TOTAL_NUM] = {};
        if (parseTendonGuardPayload(frame + 1, frameLen - 1U, enabled, sign, x1Abs)) {
            upperApplyTendonGuardConfig(shared, enabled, sign, x1Abs, ENCODER_TOTAL_NUM);
        }
        return;
    }

    if (cmd == CMD_ACTIVE_SCOPE) {
        uint8_t status = PROTO_ACK_STATUS_OK;
        if (frameLen < (1U + kActiveScopePayloadBytes)) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else {
            const uint32_t activeMotorMask = readU32BigEndian(frame + 1);
            const uint32_t activeJointMask = readU32BigEndian(frame + 5);
            const uint8_t requireCanFeedback = frame[9] ? 1U : 0U;
            if (activeMotorMask == 0U) {
                status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
            } else {
                upperApplyActiveScope(shared, activeMotorMask, activeJointMask, requireCanFeedback);
            }
        }
        sendProtoAckPacket(CMD_ACTIVE_SCOPE, shared->system_state, status);
        return;
    }

    if (cmd == CMD_RESTORE_MULTITURN) {
        uint8_t status = PROTO_ACK_STATUS_OK;
        if (frameLen < (1U + kRestoreMultiTurnPayloadBytes)) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else {
            const uint32_t motorMask = readU32BigEndian(frame + 1);
            int32_t motorAbs[SERVO_TOTAL_NUM] = {};
            if (!parseInt16ArrayBigEndian(frame + 5, frameLen - 5U, motorAbs, SERVO_TOTAL_NUM)) {
                status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
            } else {
                SemaphoreHandle_t lock = shared->commandStateMutex;
                const bool locked = (lock != nullptr) && (xSemaphoreTake(lock, pdMS_TO_TICKS(10)) == pdTRUE);
                for (int i = 0; i < SERVO_TOTAL_NUM; i++) {
                    shared->restore_multi_turn_abs[i] = motorAbs[i];
                }
                shared->restore_multi_turn_mask = motorMask & validMotorChannelMask();
                shared->restore_multi_turn_request_token++;
                if (locked) {
                    xSemaphoreGive(lock);
                }
            }
        }
        sendProtoAckPacket(CMD_RESTORE_MULTITURN, shared->system_state, status);
        return;
    }

    if (cmd == CMD_MOTOR_ABS_LIMIT_LOCK) {
        uint8_t status = PROTO_ACK_STATUS_OK;
        if (frameLen < (1U + kMotorAbsLimitLockPayloadBytes)) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else {
            const bool lockEnable = frame[1] != 0U;
            const uint32_t requestedMask = readU32BigEndian(frame + 2) & validMotorChannelMask();
            const uint32_t validMask = shared->motor_abs_limit_valid_bitmap;
            if (lockEnable) {
                shared->motor_abs_limit_locked_bitmap |= (requestedMask & validMask);
            } else {
                shared->motor_abs_limit_locked_bitmap &= ~requestedMask;
            }
        }
        sendProtoAckPacket(CMD_MOTOR_ABS_LIMIT_LOCK, shared->system_state, status);
        return;
    }

    if (cmd == CMD_SENSOR_STREAM_MODE) {
        uint8_t status = PROTO_ACK_STATUS_OK;
        if (frameLen < 2U) {
            status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
        } else {
            const uint8_t requestedMode = frame[1];
            if ((requestedMode == SENSOR_STREAM_MODE_LEGACY_U16_WRAP) ||
                (requestedMode == SENSOR_STREAM_MODE_SIGNED_I16)) {
                sensorStreamMode = requestedMode;
            } else {
                status = PROTO_ACK_STATUS_UNSUPPORTED_MODE;
            }
        }
        sendProtoAckPacket(CMD_SENSOR_STREAM_MODE, sensorStreamMode, status);
    }
}

// 发布传感器、触觉、舵机和状态心跳包。
void UpperComm::publishPeriodicData(void)
{
    if (shared == nullptr) {
        return;
    }

    bool sentPacket = false;

    MappedAngleData_t mappedAngleData = {};
    if ((shared->mappedAngleQueue != nullptr) &&
        (xQueuePeek(shared->mappedAngleQueue, &mappedAngleData, 0) == pdTRUE) &&
        (mappedAngleData.timestamp != lastSensorTimestampSent)) {
        lastMappedAngle = mappedAngleData;
        sendDataPacket(nullptr, &mappedAngleData, nullptr, nullptr, nullptr, nullptr, nullptr);
        lastSensorTimestampSent = mappedAngleData.timestamp;
        sentPacket = true;
    } else {
        RemoteSensorData_t sensorRawData = {};
        if ((shared->canRxQueue != nullptr) &&
            (xQueuePeek(shared->canRxQueue, &sensorRawData, 0) == pdTRUE) &&
            (sensorRawData.timestamp != lastSensorTimestampSent)) {
            lastSensorRaw = sensorRawData;
            sendDataPacket(&sensorRawData, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
            lastSensorTimestampSent = sensorRawData.timestamp;
            sentPacket = true;
        }
    }

    RemoteSensorData_t encoderRawData = {};
    if ((shared->canRxQueue != nullptr) &&
        (xQueuePeek(shared->canRxQueue, &encoderRawData, 0) == pdTRUE) &&
        (encoderRawData.timestamp != lastEncoderRawTimestampSent)) {
        lastSensorRaw = encoderRawData;
        sendEncoderRawPacket(encoderRawData);
        lastEncoderRawTimestampSent = encoderRawData.timestamp;
    }

    RemoteTactileData_t tactileData = {};
    if ((shared->tactileQueue != nullptr) &&
        (xQueueReceive(shared->tactileQueue, &tactileData, 0) == pdTRUE)) {
        lastTactile = tactileData;
        sendDataPacket(nullptr, nullptr, nullptr, nullptr, nullptr, &tactileData, nullptr);
        sentPacket = true;
    }

    JointDebugData_t jointDebugData = {};
    while ((shared->jointDebugQueue != nullptr) &&
           (xQueueReceive(shared->jointDebugQueue, &jointDebugData, 0) == pdTRUE)) {
        lastJointDebug = jointDebugData;
        sendDataPacket(nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, &jointDebugData);
        sentPacket = true;
    }

    ServoAngleData_t servoRawData = {};
    if ((shared->servoRawQueue != nullptr) &&
        (xQueueReceive(shared->servoRawQueue, &servoRawData, 0) == pdTRUE)) {
        lastServoRaw = servoRawData;
        sendDataPacket(nullptr, nullptr, nullptr, &servoRawData, nullptr, nullptr, nullptr);
        sentPacket = true;
    }

    ServoTelemetryData_t telemetryData = {};
    if ((shared->servoTelemetryQueue != nullptr) &&
        (xQueueReceive(shared->servoTelemetryQueue, &telemetryData, 0) == pdTRUE)) {
        lastTelemetry = telemetryData;
        sendDataPacket(nullptr, nullptr, nullptr, nullptr, &telemetryData, nullptr, nullptr);
        sentPacket = true;
    }

    ServoAngleData_t servoAngleData = {};
    if ((shared->servoAngleQueue != nullptr) &&
        (xQueueReceive(shared->servoAngleQueue, &servoAngleData, 0) == pdTRUE)) {
        lastServoAngle = servoAngleData;
        sendDataPacket(nullptr, nullptr, &servoAngleData, nullptr, nullptr, nullptr, nullptr);
        sentPacket = true;
    }

    if (!sentPacket && (g_calibrationUIStatus != 0U)) {
        sendDataPacket(nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr);
    }

    const uint32_t nowMs = millis();
    const uint32_t faultBitmap = shared->overload_fault_bitmap;
    const bool faultBitmapChanged =
        (faultStatusSentInitialized == 0U) || (faultBitmap != lastFaultStatusSent);
    const bool faultHeartbeatDue =
        (faultStatusSentInitialized == 0U) ||
        ((nowMs - lastFaultStatusSentMs) >= kFaultStatusHeartbeatMs);
    if (faultBitmapChanged || faultHeartbeatDue) {
        sendFaultStatusPacket(faultBitmap);
        lastFaultStatusSent = faultBitmap;
        lastFaultStatusSentMs = nowMs;
        faultStatusSentInitialized = 1;
    }

    const uint32_t releaseFaultBitmap = shared->reverse_release_fault_bitmap;
    const bool releaseBitmapChanged =
        (releaseFaultSentInitialized == 0U) || (releaseFaultBitmap != lastReleaseFaultSent);
    const bool releaseHeartbeatDue =
        (releaseFaultSentInitialized == 0U) ||
        ((nowMs - lastReleaseFaultSentMs) >= kFaultStatusHeartbeatMs);
    if (releaseBitmapChanged || releaseHeartbeatDue) {
        sendReleaseFaultPacket(releaseFaultBitmap);
        lastReleaseFaultSent = releaseFaultBitmap;
        lastReleaseFaultSentMs = nowMs;
        releaseFaultSentInitialized = 1;
    }

    const uint32_t stateTransitionCount = shared->state_transition_count;
    const bool systemStatusChanged =
        (systemStatusSentInitialized == 0U) ||
        (stateTransitionCount != lastSystemStatusTransitionSent);
    const bool systemStatusHeartbeatDue =
        (systemStatusSentInitialized == 0U) ||
        ((nowMs - lastSystemStatusSentMs) >= kSystemStatusHeartbeatMs);
    if (systemStatusChanged || systemStatusHeartbeatDue) {
        sendSystemStatusPacket(shared);
        lastSystemStatusTransitionSent = stateTransitionCount;
        lastSystemStatusSentMs = nowMs;
        systemStatusSentInitialized = 1;
    }

    const uint32_t limitValidBitmap = shared->motor_abs_limit_valid_bitmap;
    const uint32_t limitLockedBitmap = shared->motor_abs_limit_locked_bitmap;
    if ((limitValidBitmap != lastMotorAbsLimitValidSent) ||
        (limitLockedBitmap != lastMotorAbsLimitLockedSent) ||
        ((nowMs - lastMotorAbsLimitSentMs) >= kMotorAbsLimitHeartbeatMs)) {
        sendMotorAbsLimitPacket(shared);
        lastMotorAbsLimitValidSent = limitValidBitmap;
        lastMotorAbsLimitLockedSent = limitLockedBitmap;
        lastMotorAbsLimitSentMs = nowMs;
    }
}

// 上位机通信任务主体：读取 USB RX 队列、解析命令、发布上行协议包。
void UpperComm::run(void *parameter)
{
    shared = static_cast<TaskSharedData_t *>(parameter);

    if (shared == nullptr) {
        vTaskDelete(nullptr);
        return;
    }

    taskRunning = 1;
    sensorStreamMode = SENSOR_STREAM_MODE_SIGNED_I16;
    resetDebugState();

    usb1.echoEnabled = 0;

    static const uint8_t readyBanner[] = "<<<SYS_READY>>>\r\n";
    (void)sendPacket(readyBanner, static_cast<uint16_t>(sizeof(readyBanner) - 1U), 0);

    for (;;) {
        UsbPacket_t packet = {};
        while ((shared->usbRxQueue != nullptr) &&
               (xQueueReceive(shared->usbRxQueue, &packet, 0) == pdTRUE)) {
            handleUsbPacket(packet);
        }

        publishPeriodicData();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
