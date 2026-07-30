/**
 * @file UpperCommCodec.h
 * @brief 定义上位机二进制协议的大小端编解码和命令长度查询接口。
 *
 * 本模块不发送串口、不修改系统状态，只负责字节布局。协议字段宽度和顺序
 * 保持原样，便于对其单独审计并防止业务处理代码重复手写移位操作。
 */
#ifndef MMHAND_UPPER_COMM_CODEC_H
#define MMHAND_UPPER_COMM_CODEC_H

#include <stddef.h>
#include <stdint.h>

namespace mmhand::upper {

void appendFloatBigEndian(uint8_t* buffer, size_t* index, float value);
void appendI32BigEndian(uint8_t* buffer, size_t* index, int32_t value);
void appendI16BigEndian(uint8_t* buffer, size_t* index, int16_t value);
void appendU16BigEndian(uint8_t* buffer, size_t* index, uint16_t value);
void appendU32BigEndian(uint8_t* buffer, size_t* index, uint32_t value);
uint32_t readU32BigEndian(const uint8_t* bytes);
int32_t readI32BigEndian(const uint8_t* bytes);
uint16_t readU16BigEndian(const uint8_t* bytes);
float readFloatBigEndian(const uint8_t* bytes);
float decodeFloatLittleEndian(const uint8_t* bytes);

bool parseFloatArrayLittleEndian(const uint8_t* payload,
                                 size_t payloadLen,
                                 float* outValues,
                                 uint8_t count);
bool parseInt16ArrayBigEndian(const uint8_t* payload,
                              size_t payloadLen,
                              int32_t* outValues,
                              uint8_t count);
bool parseInt32ArrayBigEndian(const uint8_t* payload,
                              size_t payloadLen,
                              int32_t* outValues,
                              uint8_t count);

uint16_t crc16Ccitt(const uint8_t* data, size_t length);
size_t getCommandPayloadLength(uint8_t command);

} // namespace mmhand::upper

#endif // MMHAND_UPPER_COMM_CODEC_H
