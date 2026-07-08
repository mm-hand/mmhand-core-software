#ifndef SHARED_SAFETY_STATE_H
#define SHARED_SAFETY_STATE_H

#include <Arduino.h>

// Serializes short multi-field safety snapshots across the P4 cores. Never
// perform I/O, queue operations, delays, or NVS access while this lock is held.
void sharedSafetyLock(void);
void sharedSafetyUnlock(void);

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
    return __atomic_load_n(value, __ATOMIC_ACQUIRE);
}

template <typename T>
static inline void sharedAtomicStore(volatile T* value, T next)
{
    __atomic_store_n(value, next, __ATOMIC_RELEASE);
}

template <typename T>
static inline T sharedAtomicFetchOr(volatile T* value, T bits)
{
    return __atomic_fetch_or(value, bits, __ATOMIC_ACQ_REL);
}

template <typename T>
static inline T sharedAtomicFetchAnd(volatile T* value, T bits)
{
    return __atomic_fetch_and(value, bits, __ATOMIC_ACQ_REL);
}

template <typename T>
static inline T sharedAtomicAddFetch(volatile T* value, T increment)
{
    return __atomic_add_fetch(value, increment, __ATOMIC_ACQ_REL);
}

#endif // SHARED_SAFETY_STATE_H
