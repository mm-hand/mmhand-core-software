/**
 * @file HostSerialPort.cpp
 * @brief 实现 Arduino USB/UART Serial 的最小硬件适配。
 */
#include "HostSerialPort.h"

#include <Arduino.h>

namespace {

// Arduino Serial 的回调运行在其内部事件任务中；这里仅转发唤醒，不读取字节，
// 保证 HostCommTask 仍是协议缓冲区和解析状态的唯一所有者。
HostSerialPort::RxWakeHandler s_rxWakeHandler = nullptr;
void* s_rxWakeContext = nullptr;

void notifyHostRx()
{
    if (s_rxWakeHandler) {
        s_rxWakeHandler(s_rxWakeContext);
    }
}

} // namespace

void HostSerialPort::begin(uint32_t baudRate) { Serial.begin(baudRate); }
void HostSerialPort::setRxWakeHandler(RxWakeHandler handler, void* context)
{
    s_rxWakeHandler = handler;
    s_rxWakeContext = context;
    if (handler) {
        // 仅在一帧字节流空闲后回调，避免逐字节唤醒造成额外 CPU 抖动。
        Serial.setRxTimeout(2);
        Serial.onReceive(notifyHostRx, true);
    }
}
int HostSerialPort::available() { return Serial.available(); }
int HostSerialPort::readByte() { return Serial.read(); }
size_t HostSerialPort::write(const uint8_t* data, size_t length) { return Serial.write(data, length); }
void HostSerialPort::print(const char* text) { Serial.print(text ? text : ""); }
void HostSerialPort::println(const char* text) { Serial.println(text ? text : ""); }
