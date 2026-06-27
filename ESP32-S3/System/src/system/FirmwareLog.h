#pragma once

#include <Arduino.h>

#define FW_LOG_LEVEL_ERROR 1
#define FW_LOG_LEVEL_WARN 2
#define FW_LOG_LEVEL_INFO 3
#define FW_LOG_LEVEL_DEBUG 4
#define FW_LOG_LEVEL_TRACE 5

#ifndef FW_LOG_LEVEL
#define FW_LOG_LEVEL FW_LOG_LEVEL_INFO
#endif

#define FW_LOG_ENABLED(level) (FW_LOG_LEVEL >= (level))

#define FW_LOG_PRINTF(level, ...)                 \
    do {                                          \
        if (FW_LOG_ENABLED(level)) {              \
            Serial.printf(__VA_ARGS__);           \
        }                                         \
    } while (0)

#define FW_LOG_PRINTLN(level, text)               \
    do {                                          \
        if (FW_LOG_ENABLED(level)) {              \
            Serial.println(text);                 \
        }                                         \
    } while (0)

#define FW_LOG_ERROR_PRINTF(...) FW_LOG_PRINTF(FW_LOG_LEVEL_ERROR, __VA_ARGS__)
#define FW_LOG_WARN_PRINTF(...) FW_LOG_PRINTF(FW_LOG_LEVEL_WARN, __VA_ARGS__)
#define FW_LOG_INFO_PRINTF(...) FW_LOG_PRINTF(FW_LOG_LEVEL_INFO, __VA_ARGS__)
#define FW_LOG_DEBUG_PRINTF(...) FW_LOG_PRINTF(FW_LOG_LEVEL_DEBUG, __VA_ARGS__)
#define FW_LOG_TRACE_PRINTF(...) FW_LOG_PRINTF(FW_LOG_LEVEL_TRACE, __VA_ARGS__)

#define FW_LOG_ERROR_PRINTLN(text) FW_LOG_PRINTLN(FW_LOG_LEVEL_ERROR, text)
#define FW_LOG_WARN_PRINTLN(text) FW_LOG_PRINTLN(FW_LOG_LEVEL_WARN, text)
#define FW_LOG_INFO_PRINTLN(text) FW_LOG_PRINTLN(FW_LOG_LEVEL_INFO, text)
#define FW_LOG_DEBUG_PRINTLN(text) FW_LOG_PRINTLN(FW_LOG_LEVEL_DEBUG, text)
#define FW_LOG_TRACE_PRINTLN(text) FW_LOG_PRINTLN(FW_LOG_LEVEL_TRACE, text)
