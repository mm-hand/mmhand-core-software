/**
 * @file CommunicationSafetyPolicy.h
 * @brief 集中定义 CAN/上位机串口帧的长度、分帧完整性、CRC 和事件限频判断。
 */
#ifndef P4_SAFETY_POLICIES_COMMUNICATION_SAFETY_POLICY_H
#define P4_SAFETY_POLICIES_COMMUNICATION_SAFETY_POLICY_H

#include <stddef.h>
#include <stdint.h>

#include "../../data/SafetyRuleTypes.h"

namespace p4::safety {

/** @brief CAN HAL 初始化失败时返回 COM-6001，成功时返回 None。 */
SafetyRuleId communicationCanInitializationRule(bool initialized);

bool communicationLengthValid(size_t actual, size_t expected);
bool communicationReassemblyComplete(bool active,
                                     uint32_t actualFrameMask,
                                     uint32_t expectedFrameMask);
bool communicationFrameCountComplete(uint32_t actualFrameCount,
                                     uint32_t expectedFrameCount);
bool communicationIndexValid(uint32_t index, uint32_t itemCount);
bool communicationCrcValid(bool metadataValid,
                           uint16_t actualCrc,
                           uint16_t expectedCrc);
bool communicationAssemblyExpired(uint32_t nowMs,
                                  uint32_t firstFrameMs,
                                  uint32_t timeoutMs);
bool communicationStreamStale(bool streamValid,
                              uint32_t nowMs,
                              uint32_t lastUpdateMs,
                              uint32_t staleAfterMs);
bool communicationEventMayReport(uint32_t nowMs,
                                 uint32_t lastReportMs,
                                 uint32_t minimumIntervalMs);

} // namespace p4::safety

#endif // P4_SAFETY_POLICIES_COMMUNICATION_SAFETY_POLICY_H
