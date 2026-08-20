/**
 * @file HostSerialPort.cpp
 * @brief 实现 Arduino USB/UART Serial 的最小硬件适配。
 */
#include "HostSerialPort.h"

#include <Arduino.h>
#include "../../config/BoardConfig.h"

namespace {

// Arduino Serial 的回调运行在其内部事件任务中；这里仅转发唤醒，不读取字节，
// 保证 HostCommTask 仍是协议缓冲区和解析状态的唯一所有者。
HostSerialPort::RxWakeHandler s_rxWakeHandler = nullptr;
void* s_rxWakeContext = nullptr;
volatile uint32_t s_rxErrorFlags = 0;

void notifyHostRx()
{
    if (s_rxWakeHandler) {
        s_rxWakeHandler(s_rxWakeContext);
    }
}

/** Arduino UART 事件线程只锁存错误，具体告警由 HostCommTask 统一上报。 */
void notifyHostRxError(hardwareSerial_error_t error)
{
    const uint8_t bit = static_cast<uint8_t>(error);
    if (bit < 32U) {
        __atomic_fetch_or(&s_rxErrorFlags, 1UL << bit, __ATOMIC_RELAXED);
    }
}

} // namespace

void HostSerialPort::begin(uint32_t baudRate)
{
    // Arduino 要求在 begin() 前设置 RX ring；TX ring 继续保持默认 0 byte。
    Serial.setRxBufferSize(kHostUartRxBufferBytes);
    Serial.begin(baudRate);
}
void HostSerialPort::setRxWakeHandler(RxWakeHandler handler, void* context)
{
    s_rxWakeHandler = handler;
    s_rxWakeContext = context;
    if (handler) {
        // 仅在一帧字节流空闲后回调，避免逐字节唤醒造成额外 CPU 抖动。
        Serial.setRxTimeout(2);
        Serial.onReceive(notifyHostRx, true);
        Serial.onReceiveError(notifyHostRxError);
    }
}
uint32_t HostSerialPort::takeRxErrorFlags()
{
    return __atomic_exchange_n(&s_rxErrorFlags, 0U, __ATOMIC_ACQ_REL);
}
int HostSerialPort::available() { return Serial.available(); }
int HostSerialPort::readByte() { return Serial.read(); }
size_t HostSerialPort::write(const uint8_t* data, size_t length) { return Serial.write(data, length); }
void HostSerialPort::print(const char* text) { Serial.print(text ? text : ""); }
void HostSerialPort::println(const char* text) { Serial.println(text ? text : ""); }
