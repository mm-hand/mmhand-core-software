/**
 * @file SafetyEventStore.h
 * @brief 声明最近安全事件的定长环形缓存。
 * @details 缓存只保存诊断，不参与控制裁决；写满后覆盖最旧事件，绝不阻塞实时任务。
 */
#ifndef P4_SAFETY_EVENT_STORE_H
#define P4_SAFETY_EVENT_STORE_H

#include "../data/SafetyRuleTypes.h"
#include <stddef.h>

namespace p4::safety {

inline constexpr size_t kSafetyEventCacheCapacity = 16;

/** 记录一条安全事件；sequence 和 timestampMs 由缓存补齐。 */
void recordSafetyEvent(const SafetyEventRecord& event);

/**
 * @brief 由基础设施模块直接记录一条规则事件。
 * @details 统一补齐无关对象的 0xFF 哨兵，避免 CAN/NVS/RTOS 各自拼装不同格式。
 */
void recordSafetyRuleEvent(SafetyRuleId ruleId,
                           uint8_t systemState,
                           uint8_t owner,
                           uint8_t jointIndex = 0xFF,
                           uint8_t motorChannel = 0xFF,
                           int32_t requestedValue = 0,
                           int32_t observedValue = 0);

/**
 * 复制 sequence 严格大于 afterSequence 的事件。
 *
 * 该接口供上位机遥测使用：发送成功后保存最后 sequence，下一周期只取新事件，
 * 从而保证同一条碰撞 Hold 或 LUT clamp 证据不会按控制周期重复上报。
 */
size_t copySafetyEventsAfter(uint32_t afterSequence,
                             SafetyEventRecord* output,
                             size_t capacity);

/** 返回缓存写满后被覆盖的事件总数；该计数只增不减。 */
uint32_t safetyEventOverflowCount();

} // namespace p4::safety

#endif // P4_SAFETY_EVENT_STORE_H
