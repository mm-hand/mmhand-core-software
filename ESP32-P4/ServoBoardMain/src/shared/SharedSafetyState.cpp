#include "SharedSafetyState.h"

static portMUX_TYPE g_sharedSafetyLock = portMUX_INITIALIZER_UNLOCKED;

void sharedSafetyLock(void)
{
    portENTER_CRITICAL(&g_sharedSafetyLock);
}

void sharedSafetyUnlock(void)
{
    portEXIT_CRITICAL(&g_sharedSafetyLock);
}
