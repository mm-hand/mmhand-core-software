/**
 * @file HostSerialPort.cpp
 * @brief 实现 Arduino USB/UART Serial 的最小硬件适配。
 */
#include "HostSerialPort.h"

#include <Arduino.h>

void HostSerialPort::begin(uint32_t baudRate) { Serial.begin(baudRate); }
int HostSerialPort::available() { return Serial.available(); }
int HostSerialPort::readByte() { return Serial.read(); }
size_t HostSerialPort::write(const uint8_t* data, size_t length) { return Serial.write(data, length); }
void HostSerialPort::print(const char* text) { Serial.print(text ? text : ""); }
void HostSerialPort::println(const char* text) { Serial.println(text ? text : ""); }
