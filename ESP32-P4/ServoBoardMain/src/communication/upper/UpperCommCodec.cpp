/**
 * @file UpperCommCodec.cpp
 * @brief 实现上位机协议字段的确定性编解码和 CRC16 校验。
 */
#include "UpperCommCodec.h"

#include <math.h>
#include <string.h>

#include "UpperCommProtocol.h"

namespace mmhand::upper {

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

uint16_t crc16Ccitt(const uint8_t* data, size_t len)
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

// 解析固定数量的大端 int16，并符号扩展为内部 int32。
bool parseInt16ArrayBigEndian(const uint8_t* payload, size_t payloadLen, int32_t* outValues, uint8_t count)
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

// 解析 P4 逻辑多圈 ABS。逻辑坐标可能因单台舵机掉电后的 offset 平移
// 超过 int16 范围，因此绝对位置命令统一使用大端 int32。
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

} // namespace mmhand::upper
