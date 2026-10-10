#pragma once

#include <Arduino.h>

// 编译期日志等级：数值越大，保留的日志越详细。
#define FW_LOG_LEVEL_ERROR 1
#define FW_LOG_LEVEL_WARN 2
#define FW_LOG_LEVEL_INFO 3
#define FW_LOG_LEVEL_DEBUG 4
#define FW_LOG_LEVEL_TRACE 5

#ifndef FW_LOG_LEVEL
// 默认只输出INFO及更严重日志；DEBUG/TRACE代码仍会编译，但条件恒为false。
#define FW_LOG_LEVEL FW_LOG_LEVEL_INFO
#endif

#define FW_LOG_ENABLED(level) (FW_LOG_LEVEL >= (level))

// 使用do-while(0)保证宏在if/else中仍表现为单条语句。
#define FW_LOG_PRINTF(level, ...)                 \
    do {                                          \
        if (FW_LOG_ENABLED(level)) {              \
            Serial.printf(__VA_ARGS__);           \
        }                                         \
    } while (0)

#define FW_LOG_DEBUG_PRINTF(...) FW_LOG_PRINTF(FW_LOG_LEVEL_DEBUG, __VA_ARGS__)
