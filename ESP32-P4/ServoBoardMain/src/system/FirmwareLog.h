#pragma once

#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#define FW_LOG_LEVEL_ERROR 1
#define FW_LOG_LEVEL_WARN 2
#define FW_LOG_LEVEL_INFO 3
#define FW_LOG_LEVEL_DEBUG 4
#define FW_LOG_LEVEL_TRACE 5

#ifndef FW_LOG_LEVEL
#define FW_LOG_LEVEL FW_LOG_LEVEL_INFO
#endif

// 编译期日志等级。低于当前等级的调用会被内联函数立即过滤。
#define FW_LOG_ENABLED(level) (FW_LOG_LEVEL >= (level))

// 二进制上位机协议和文本日志共用 Serial，所有写操作必须经过同一互斥量。
SemaphoreHandle_t fwSerialMutex();

// 最多等待 50 ms 获取串口；超时后丢弃本条日志，不能反向阻塞实时控制任务。
static inline bool fwSerialTake()
{
    SemaphoreHandle_t mutex = fwSerialMutex();
    if (mutex == NULL) {
        return false;
    }
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        return true;
    }
    return xSemaphoreTake(mutex, pdMS_TO_TICKS(50)) == pdTRUE;
}

static inline void fwSerialGive(bool locked)
{
    if (!locked || xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        return;
    }
    SemaphoreHandle_t mutex = fwSerialMutex();
    if (mutex != NULL) {
        xSemaphoreGive(mutex);
    }
}

static inline size_t fwSerialWrite(const uint8_t* data, size_t len)
{
    if (!data || len == 0) {
        return 0;
    }
    const bool locked = fwSerialTake();
    if (!locked) {
        return 0;
    }
    const size_t written = Serial.write(data, len);
    fwSerialGive(locked);
    return written;
}

static inline void fwLogPrintln(uint8_t level, const char* text)
{
    if (!FW_LOG_ENABLED(level) || !text) {
        return;
    }
    const bool locked = fwSerialTake();
    if (!locked) {
        return;
    }
    Serial.println(text);
    fwSerialGive(locked);
}

// 使用固定栈缓冲格式化日志，避免实时任务中动态分配内存。
static inline void fwLogPrintf(uint8_t level, const char* fmt, ...)
{
    if (!FW_LOG_ENABLED(level) || !fmt) {
        return;
    }
    char buffer[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    const bool locked = fwSerialTake();
    if (!locked) {
        return;
    }
    Serial.print(buffer);
    fwSerialGive(locked);
}

#define FW_SERIAL_WRITE(data, len) fwSerialWrite((data), (len))
#define FW_LOG_PRINTF(level, ...) fwLogPrintf((level), __VA_ARGS__)
#define FW_LOG_PRINTLN(level, text) fwLogPrintln((level), (text))

#define FW_LOG_WARN_PRINTF(...) FW_LOG_PRINTF(FW_LOG_LEVEL_WARN, __VA_ARGS__)
#define FW_LOG_INFO_PRINTF(...) FW_LOG_PRINTF(FW_LOG_LEVEL_INFO, __VA_ARGS__)
#define FW_LOG_DEBUG_PRINTF(...) FW_LOG_PRINTF(FW_LOG_LEVEL_DEBUG, __VA_ARGS__)
#define FW_LOG_TRACE_PRINTF(...) FW_LOG_PRINTF(FW_LOG_LEVEL_TRACE, __VA_ARGS__)

#define FW_LOG_INFO_PRINTLN(text) FW_LOG_PRINTLN(FW_LOG_LEVEL_INFO, text)
