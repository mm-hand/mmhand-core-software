#include "FirmwareLog.h"

SemaphoreHandle_t fwSerialMutex()
{
    static SemaphoreHandle_t mutex = NULL;
    static portMUX_TYPE initLock = portMUX_INITIALIZER_UNLOCKED;
    if (mutex == NULL) {
        portENTER_CRITICAL(&initLock);
        if (mutex == NULL) {
            mutex = xSemaphoreCreateMutex();
        }
        portEXIT_CRITICAL(&initLock);
    }
    return mutex;
}
