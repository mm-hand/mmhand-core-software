/**
 * @file ProtocolContractPolicy.h
 * @brief 纯函数验证上位机与 P4 的协议和运动模型契约，不授予运动许可。
 */
#pragma once
#include <stdint.h>

namespace p4::system {
inline bool protocolContractMatches(uint8_t version, uint32_t model, uint32_t topology,
                                    uint8_t expectedVersion, uint32_t expectedModel,
                                    uint32_t expectedTopology)
{
    return version != 0 && version == expectedVersion &&
           model == expectedModel && topology == expectedTopology;
}
}
