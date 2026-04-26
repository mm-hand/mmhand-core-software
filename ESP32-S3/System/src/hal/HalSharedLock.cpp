#include "HalSharedLock.h"

SemaphoreHandle_t getSharedMuxLock() {
    static SemaphoreHandle_t sMuxLock = nullptr;
    if (sMuxLock == nullptr) {
        sMuxLock = xSemaphoreCreateMutex();
    }
    return sMuxLock;
}

