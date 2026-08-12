/**
 * @file Clock.h
 * @brief 提供 P4 业务层统一使用的单调时钟接口。
 *
 * 业务代码不应自行选择 Arduino 或 ESP-IDF 的时钟 API。统一入口既能明确
 * 毫秒单位，也为后续离线测试注入虚拟时间保留边界。
 */
#ifndef P4_PLATFORM_TIME_CLOCK_H
#define P4_PLATFORM_TIME_CLOCK_H

#include <Arduino.h>
#include <stdint.h>

namespace p4::platform::time {

/** 返回自芯片启动以来的毫秒数；允许 uint32_t 自然回绕。 */
inline uint32_t monotonicMillis()
{
    return millis();
}

} // namespace p4::platform::time

#endif // P4_PLATFORM_TIME_CLOCK_H
