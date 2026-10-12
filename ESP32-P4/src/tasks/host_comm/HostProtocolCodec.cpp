/**
 * @file HostProtocolCodec.cpp
 * @brief 实现上位机协议字段的确定性编解码和 CRC16 校验。
 */
#include "HostProtocolCodec.h"

#include <math.h>
#include <string.h>

#include "../../config/protocol/HostProtocol.h"

namespace p4::host {

size_t encodeServoPositionPackets(uint8_t* buffer,
                                  size_t capacity,
                                  const ServoAngleData_t& position)
{
    constexpr size_t rawLength = 3U + SERVO_TOTAL_NUM * 2U + 4U + 1U;
    constexpr size_t absLength = 3U + SERVO_TOTAL_NUM * 4U + 4U + 1U;
    static_assert(rawLength - 2U <= UINT8_MAX && absLength - 2U <= UINT8_MAX,
                  "servo position frame length exceeds wire field");
    if (!buffer || capacity < rawLength + absLength) {
        return 0;
    }
    size_t idx = 0;
    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = static_cast<uint8_t>(rawLength - 2U);
    buffer[idx++] = PACKET_TYPE_SERVO_RAW;
    for (uint8_t i = 0; i < SERVO_TOTAL_NUM; ++i) {
        appendI16BigEndian(buffer, &idx, position.servoRawPositions[i]);
    }
    appendU32BigEndian(buffer, &idx, position.timestamp);
    buffer[idx++] = PROTOCOL_TAIL;
    buffer[idx++] = PROTOCOL_HEADER;
    buffer[idx++] = static_cast<uint8_t>(absLength - 2U);
    buffer[idx++] = PACKET_TYPE_SERVO_ANGLE;
    for (uint8_t i = 0; i < SERVO_TOTAL_NUM; ++i) {
        appendI32BigEndian(buffer, &idx, position.servoAngles[i]);
    }
    appendU32BigEndian(buffer, &idx, position.timestamp);
    buffer[idx++] = PROTOCOL_TAIL;
    return idx;
}

// 按协议大端序追加 float 的 IEEE-754 位模式，不做数值格式转换。
void appendFloatBigEndian(uint8_t* buffer, size_t* idx, float value)
{
    uint32_t bits = 0;
    memcpy(&bits, &value, sizeof(bits));
    buffer[(*idx)++] = (uint8_t)((bits >> 24) & 0xFF);
    buffer[(*idx)++] = (uint8_t)((bits >> 16) & 0xFF);
    buffer[(*idx)++] = (uint8_t)((bits >> 8) & 0xFF);
    buffer[(*idx)++] = (uint8_t)(bits & 0xFF);
}

void appendI32BigEndian(uint8_t* buffer, size_t* idx, int32_t value)
{
    const uint32_t v = (uint32_t)value;
    buffer[(*idx)++] = (uint8_t)((v >> 24) & 0xFF);
    buffer[(*idx)++] = (uint8_t)((v >> 16) & 0xFF);
    buffer[(*idx)++] = (uint8_t)((v >> 8) & 0xFF);
    buffer[(*idx)++] = (uint8_t)(v & 0xFF);
}

void appendI16BigEndian(uint8_t* buffer, size_t* idx, int16_t value)
{
    const uint16_t v = (uint16_t)value;
    buffer[(*idx)++] = (uint8_t)((v >> 8) & 0xFF);
    buffer[(*idx)++] = (uint8_t)(v & 0xFF);
}

void appendU16BigEndian(uint8_t* buffer, size_t* idx, uint16_t value)
{
    buffer[(*idx)++] = (uint8_t)((value >> 8) & 0xFF);
    buffer[(*idx)++] = (uint8_t)(value & 0xFF);
}

void appendU32BigEndian(uint8_t* buffer, size_t* idx, uint32_t value)
{
    buffer[(*idx)++] = (uint8_t)((value >> 24) & 0xFF);
    buffer[(*idx)++] = (uint8_t)((value >> 16) & 0xFF);
    buffer[(*idx)++] = (uint8_t)((value >> 8) & 0xFF);
    buffer[(*idx)++] = (uint8_t)(value & 0xFF);
}

// 从命令 payload 的小端 IEEE-754 位模式还原 float。
float decodeFloatLittleEndian(const uint8_t* data)
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

// 解析固定数量的小端 float；长度不足时整帧拒绝，避免产生半帧关节目标。
bool parseFloatArrayLittleEndian(const uint8_t* payload, size_t payloadLen, float* outValues, uint8_t count)
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

uint32_t readU32BigEndian(const uint8_t* p)
{
    return ((uint32_t)p[0] << 24) |
           ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) |
           (uint32_t)p[3];
}

int32_t readI32BigEndian(const uint8_t* p)
{
    return (int32_t)readU32BigEndian(p);
}

uint16_t readU16BigEndian(const uint8_t* p)
{
    return ((uint16_t)p[0] << 8) | (uint16_t)p[1];
}

int16_t readI16BigEndian(const uint8_t* p)
{
    return (int16_t)readU16BigEndian(p);
}

float readFloatBigEndian(const uint8_t* p)
{
    const uint32_t bits = readU32BigEndian(p);
    float value = 0.0f;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

// 固定长度表由唯一 schema 生成；codec 不再维护第二份 switch。
size_t getCommandPayloadLength(uint8_t cmd)
{
    return generatedCommandPayloadLength(cmd);
}

// 解析多圈 ABS；绝对位置命令保持既有大端 int32 格式，执行层检查硬件范围。
bool parseInt32ArrayBigEndian(const uint8_t* payload,
                                     size_t payloadLen,
                                     int32_t* outValues,
                                     uint8_t count)
{
    if (!payload || !outValues) {
        return false;
    }
    const size_t required = (size_t)count * sizeof(int32_t);
    if (payloadLen < required) {
        return false;
    }
    for (uint8_t i = 0; i < count; i++) {
        const uint8_t* p = payload + (size_t)i * 4;
        const uint32_t raw =
            ((uint32_t)p[0] << 24) |
            ((uint32_t)p[1] << 16) |
            ((uint32_t)p[2] << 8) |
            (uint32_t)p[3];
        outValues[i] = (int32_t)raw;
    }
    return true;
}

} // namespace p4::host
