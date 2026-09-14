/**
 * @file FirmwareLog.h
 * @brief 提供与上位机二进制协议共用串口时的线程安全固件日志接口。
 *
 * 日志函数使用固定栈缓冲区并受同一串口互斥量保护。实时任务获取互斥量超时
 * 时会丢弃日志，绝不能让诊断输出反向阻塞控制链路。
 */
#pragma once

#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>

#include "../rtos/FreeRtosKernel.h"

#include "../../hal/serial/HostSerialPort.h"

#define FW_LOG_LEVEL_ERROR 1
#define FW_LOG_LEVEL_WARN 2
#define FW_LOG_LEVEL_INFO 3
#define FW_LOG_LEVEL_DEBUG 4
#define FW_LOG_LEVEL_TRACE 5

#ifndef FW_LOG_LEVEL
#define FW_LOG_LEVEL FW_LOG_LEVEL_INFO
#endif

/** 编译期日志等级；高于当前等级的调用会被内联函数立即过滤。 */
#define FW_LOG_ENABLED(level) (FW_LOG_LEVEL >= (level))

/** 二进制协议和文本日志共用 Serial，所有写操作必须经过该互斥量。 */
SemaphoreHandle_t fwSerialMutex();

/** 按调用方指定的等待时间获取串口；实时文本日志必须使用零等待。 */
static inline bool fwSerialTake(TickType_t waitTicks)
{
    SemaphoreHandle_t mutex = fwSerialMutex();
    if (mutex == NULL) {
        return false;
    }
    if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        return true;
    }
    return xSemaphoreTake(mutex, waitTicks) == pdTRUE;
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
    // 二进制协议关系到命令 ACK 和状态同步，保留原有最多 50 ms 的短等待。
    const bool locked = fwSerialTake(pdMS_TO_TICKS(50));
    if (!locked) {
        return 0;
    }
    const size_t written = HostSerialPort::write(data, len);
    fwSerialGive(locked);
    return written;
}

static inline void fwLogPrintln(uint8_t level, const char* text)
{
    if (!FW_LOG_ENABLED(level) || !text) {
        return;
    }
    // 所有文本日志都只尝试一次；ERROR/WARN 保留输出入口，但同样不能阻塞实时任务。
    const bool locked = fwSerialTake(0);
    if (!locked) {
        return;
    }
    HostSerialPort::println(text);
    fwSerialGive(locked);
}

/** 使用固定栈缓冲区格式化日志，避免实时任务动态分配内存。 */
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
    // 文本诊断与二进制协议争用串口时优先保证协议和实时控制。
    const bool locked = fwSerialTake(0);
    if (!locked) {
        return;
    }
    HostSerialPort::print(buffer);
    fwSerialGive(locked);
}

#define FW_SERIAL_WRITE(data, len) fwSerialWrite((data), (len))
#define FW_LOG_PRINTF(level, ...) fwLogPrintf((level), __VA_ARGS__)
#define FW_LOG_PRINTLN(level, text) fwLogPrintln((level), (text))

#define FW_LOG_WARN_PRINTF(...) FW_LOG_PRINTF(FW_LOG_LEVEL_WARN, __VA_ARGS__)
#define FW_LOG_INFO_PRINTF(...) FW_LOG_PRINTF(FW_LOG_LEVEL_INFO, __VA_ARGS__)
#define FW_LOG_DEBUG_PRINTF(...) FW_LOG_PRINTF(FW_LOG_LEVEL_DEBUG, __VA_ARGS__)
#define FW_LOG_TRACE_PRINTF(...) FW_LOG_PRINTF(FW_LOG_LEVEL_TRACE, __VA_ARGS__)

#define FW_LOG_ERROR_PRINTLN(text) FW_LOG_PRINTLN(FW_LOG_LEVEL_ERROR, text)
#define FW_LOG_INFO_PRINTLN(text) FW_LOG_PRINTLN(FW_LOG_LEVEL_INFO, text)
