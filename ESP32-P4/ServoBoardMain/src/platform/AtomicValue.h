/**
 * @file AtomicValue.h
 * @brief 封装跨核共享标量的原子读、写和位运算。
 *
 * 这些函数只适用于可由 GCC 原子内建直接处理的标量。多字段一致性快照必须
 * 使用 SharedSafetyGuard，不能用多个独立 atomic 操作拼出“伪快照”。
 */
#ifndef P4_PLATFORM_ATOMIC_VALUE_H
#define P4_PLATFORM_ATOMIC_VALUE_H

namespace p4::platform {

template <typename T>
inline T atomicLoad(const volatile T* value)
{
    return __atomic_load_n(value, __ATOMIC_ACQUIRE);
}

template <typename T>
inline void atomicStore(volatile T* value, T next)
{
    __atomic_store_n(value, next, __ATOMIC_RELEASE);
}

template <typename T>
inline T atomicFetchOr(volatile T* value, T bits)
{
    return __atomic_fetch_or(value, bits, __ATOMIC_ACQ_REL);
}

template <typename T>
inline T atomicFetchAnd(volatile T* value, T bits)
{
    return __atomic_fetch_and(value, bits, __ATOMIC_ACQ_REL);
}

template <typename T>
inline T atomicAddFetch(volatile T* value, T increment)
{
    return __atomic_add_fetch(value, increment, __ATOMIC_ACQ_REL);
}

} // namespace p4::platform

#endif // P4_PLATFORM_ATOMIC_VALUE_H
