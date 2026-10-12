/**
 * @file AtomicValue.h
 * @brief 封装跨核共享标量的原子读、写和位运算。
 *
 * 这些函数只适用于可由 GCC 原子内建直接处理的标量。多字段一致性快照必须
 * 使用 SharedSafetyGuard，不能用多个独立 atomic 操作拼出“伪快照”。
 */
#ifndef P4_PLATFORM_RTOS_ATOMIC_VALUE_H
#define P4_PLATFORM_RTOS_ATOMIC_VALUE_H

namespace p4::platform::rtos {

/** 以 acquire 语义读取跨核标量，保证随后读取不会越过本次 load。 */
template <typename T>
inline T atomicLoad(const volatile T* value)
{
    return __atomic_load_n(value, __ATOMIC_ACQUIRE);
}

/** 以 release 语义写入跨核标量，保证此前写入先于本次 store 可见。 */
template <typename T>
inline void atomicStore(volatile T* value, T next)
{
    __atomic_store_n(value, next, __ATOMIC_RELEASE);
}

/** 原子置位并返回修改前的位图，适用于故障/请求位累加。 */
template <typename T>
inline T atomicFetchOr(volatile T* value, T bits)
{
    return __atomic_fetch_or(value, bits, __ATOMIC_ACQ_REL);
}

/** 原子保留指定位并返回修改前的位图，常用于清除若干 bit。 */
template <typename T>
inline T atomicFetchAnd(volatile T* value, T bits)
{
    return __atomic_fetch_and(value, bits, __ATOMIC_ACQ_REL);
}

/** 原子累加并返回新值，适用于 token 和统计计数器。 */
template <typename T>
inline T atomicAddFetch(volatile T* value, T increment)
{
    return __atomic_add_fetch(value, increment, __ATOMIC_ACQ_REL);
}

} // namespace p4::platform::rtos

#endif // P4_PLATFORM_RTOS_ATOMIC_VALUE_H
