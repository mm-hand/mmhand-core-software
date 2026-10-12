/**
 * @file HostProtocol.h
 * @brief 上位机协议生成契约的稳定包含入口。
 *
 * 协议编号、固定 payload 长度和心跳周期不再在本文件手工维护。唯一数据源是
 * desktop/resources/config/upper_protocol_schema.json，修改后运行
 * python tools/generate_upper_protocol.py 重新生成两端常量。
 */
#ifndef UPPER_COMM_PROTOCOL_H
#define UPPER_COMM_PROTOCOL_H

#include "../../generated/UpperProtocol.generated.h"

#endif // UPPER_COMM_PROTOCOL_H
