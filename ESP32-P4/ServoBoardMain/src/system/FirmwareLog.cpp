#include "FirmwareLog.h"

SemaphoreHandle_t fwSerialMutex()
{
    // 首次调用时延迟创建，确保 setup 前后的日志和协议写入使用同一把锁。
    // 双重检查放在临界区内，防止两个任务同时创建不同的互斥量。
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
