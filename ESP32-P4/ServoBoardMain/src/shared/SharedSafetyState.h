/**
 * @file SharedSafetyState.h
 * @brief 提供跨核多字段临界区，并兼容旧的 sharedAtomic* 调用名。
 *
 * 新代码优先直接使用 platform/AtomicValue.h。兼容函数暂时保留，避免在同一
 * 阶段机械改写全部业务文件；待共享状态进一步拆分后再统一删除。
 */
#ifndef SHARED_SAFETY_STATE_H
#define SHARED_SAFETY_STATE_H

#include <Arduino.h>

#include "../platform/AtomicValue.h"

// 串行化跨核的短小多字段安全快照；持锁期间禁止 I/O、队列、delay 和 NVS 操作。
void sharedSafetyLock(void);
void sharedSafetyUnlock(void);

/** RAII 临界区，确保所有提前 return 路径都会释放跨核锁。 */
class SharedSafetyGuard {
public:
    SharedSafetyGuard() { sharedSafetyLock(); }
    ~SharedSafetyGuard() { sharedSafetyUnlock(); }

    SharedSafetyGuard(const SharedSafetyGuard&) = delete;
    SharedSafetyGuard& operator=(const SharedSafetyGuard&) = delete;
};

template <typename T>
static inline T sharedAtomicLoad(const volatile T* value)
{
    return p4::platform::atomicLoad(value);
}

template <typename T>
static inline void sharedAtomicStore(volatile T* value, T next)
{
    p4::platform::atomicStore(value, next);
}

template <typename T>
static inline T sharedAtomicFetchOr(volatile T* value, T bits)
{
    return p4::platform::atomicFetchOr(value, bits);
}

template <typename T>
static inline T sharedAtomicFetchAnd(volatile T* value, T bits)
{
    return p4::platform::atomicFetchAnd(value, bits);
}

template <typename T>
static inline T sharedAtomicAddFetch(volatile T* value, T increment)
{
    return p4::platform::atomicAddFetch(value, increment);
}

#endif // SHARED_SAFETY_STATE_H
