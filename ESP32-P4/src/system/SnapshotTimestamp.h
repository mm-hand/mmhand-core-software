/** @file SnapshotTimestamp.h
 * @brief 判断周期内读取的异步快照是否新鲜，处理跨任务取样及毫秒回绕。
 */
#pragma once
#include <stdint.h>

namespace p4::system {
// A producer can publish after the consumer sampled its cycle start. Only
// allow that bounded race; old frames and larger future offsets still fail.
inline bool snapshotFreshAtCycle(uint32_t cycleStart, uint32_t sample,
                                 uint32_t timeout, uint32_t cyclePeriod) {
    if (!sample) return false;
    const int32_t age = static_cast<int32_t>(cycleStart - sample);
    return age >= 0 ? static_cast<uint32_t>(age) <= timeout
                    : static_cast<uint32_t>(-static_cast<int64_t>(age)) <= cyclePeriod;
}
}
