/**
 * @file HostProtocolCodec.h
 * @brief 定义上位机二进制协议的大小端编解码和命令长度查询接口。
 *
 * 本模块不发送串口、不修改系统状态，只负责字节布局。协议字段宽度和顺序
 * 保持原样，便于对其单独审计并防止业务处理代码重复手写移位操作。
 */
#ifndef MMHAND_UPPER_COMM_CODEC_H
#define MMHAND_UPPER_COMM_CODEC_H

#include <stddef.h>
#include <stdint.h>

#include "../../data/ServoTypes.h"

namespace p4::host {

// append* 把主机标量按大端序追加到 buffer，并同步推进 index。
void appendFloatBigEndian(uint8_t* buffer, size_t* index, float value);
void appendI32BigEndian(uint8_t* buffer, size_t* index, int32_t value);
void appendI16BigEndian(uint8_t* buffer, size_t* index, int16_t value);
void appendU16BigEndian(uint8_t* buffer, size_t* index, uint16_t value);
void appendU32BigEndian(uint8_t* buffer, size_t* index, uint32_t value);
// read* 从固定宽度大端字段解码，不推进外部游标。
uint32_t readU32BigEndian(const uint8_t* bytes);
int32_t readI32BigEndian(const uint8_t* bytes);
uint16_t readU16BigEndian(const uint8_t* bytes);
int16_t readI16BigEndian(const uint8_t* bytes);
float readFloatBigEndian(const uint8_t* bytes);
// 目标角下行沿用 Python struct.pack("<f")，因此单独保留小端浮点解码。
float decodeFloatLittleEndian(const uint8_t* bytes);

// 严格检查 payloadLen 后解析固定长度数组；失败时不接受半包。
bool parseFloatArrayLittleEndian(const uint8_t* payload,
                                 size_t payloadLen,
                                 float* outValues,
                                 uint8_t count);
bool parseInt32ArrayBigEndian(const uint8_t* payload,
                              size_t payloadLen,
                              int32_t* outValues,
                              uint8_t count);

// 返回 command 对应固定 payload 长度；未知命令返回 SIZE_MAX。
size_t getCommandPayloadLength(uint8_t command);

/** 将同一位置快照编码为既有 RAW、ABS 两帧；容量不足返回 0，不写半帧。 */
size_t encodeServoPositionPackets(uint8_t* buffer,
                                  size_t capacity,
                                  const ServoAngleData_t& position);

} // namespace p4::host

#endif // MMHAND_UPPER_COMM_CODEC_H
