/**
 * @file SharedSafetyState.cpp
 * @brief 实现跨 ESP32-P4 双核共享状态的短临界区。
 */
#include "SharedSafetyState.h"

/** 仅保护内存快照；持锁期间严禁串口、队列或延时操作。 */
static portMUX_TYPE g_sharedSafetyLock = portMUX_INITIALIZER_UNLOCKED;

void sharedSafetyLock(void)
{
    portENTER_CRITICAL(&g_sharedSafetyLock);
}

void sharedSafetyUnlock(void)
{
    portEXIT_CRITICAL(&g_sharedSafetyLock);
}
