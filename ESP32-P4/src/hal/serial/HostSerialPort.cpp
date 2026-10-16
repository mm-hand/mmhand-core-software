/**
 * @file HostSerialPort.cpp
 * @brief 实现 Arduino USB/UART Serial 的最小硬件适配。
 */
#include "HostSerialPort.h"

#include <Arduino.h>
#include <string.h>
#if !ARDUINO_USB_MODE || !ARDUINO_USB_CDC_ON_BOOT
#error "P4 requires Hardware CDC and JTAG with USB CDC On Boot Enabled"
#endif
#include <HWCDC.h>
#include "../../config/BoardConfig.h"

namespace {

// Arduino Serial 的回调运行在其内部事件任务中；这里仅转发唤醒，不读取字节，
// 保证 HostCommTask 仍是协议缓冲区和解析状态的唯一所有者。
HostSerialPort::RxWakeHandler s_rxWakeHandler = nullptr;
void* s_rxWakeContext = nullptr;
volatile uint32_t s_rxErrorFlags = 0;
HostSerialStatistics s_statistics = {};

void notifyHostRx()
{
    if (s_rxWakeHandler) {
        s_rxWakeHandler(s_rxWakeContext);
    }
}

void notifyHostUsb(void*, esp_event_base_t, int32_t event, void*)
{
    if (event == ARDUINO_HW_CDC_RX_EVENT) notifyHostRx();
    if (event == ARDUINO_HW_CDC_BUS_RESET_EVENT) {
        __atomic_fetch_add(&s_statistics.busResets, 1U, __ATOMIC_RELAXED);
        __atomic_fetch_or(&s_rxErrorFlags, 1UL << 31, __ATOMIC_RELAXED);
    }
}

} // namespace

void HostSerialPort::begin(uint32_t baudRate)
{
    Serial.setRxBufferSize(kHostUartRxBufferBytes);
    Serial.setTxBufferSize(4096);
    Serial.setTxTimeoutMs(5);
    Serial.onEvent(notifyHostUsb);
    Serial.begin(baudRate);
}
void HostSerialPort::setRxWakeHandler(RxWakeHandler handler, void* context)
{
    s_rxWakeHandler = handler;
    s_rxWakeContext = context;
}
uint32_t HostSerialPort::takeRxErrorFlags()
{
    return __atomic_exchange_n(&s_rxErrorFlags, 0U, __ATOMIC_ACQ_REL);
}
int HostSerialPort::available() { return Serial.available(); }
int HostSerialPort::readByte() {
    const int value = Serial.read();
    if (value >= 0) {
        __atomic_fetch_add(&s_statistics.rxBytes, 1U, __ATOMIC_RELAXED);
        __atomic_store_n(&s_statistics.lastRxMs, millis(), __ATOMIC_RELAXED);
    }
    return value;
}
size_t HostSerialPort::write(const uint8_t* data, size_t length) {
    const uint32_t started = micros();
    const size_t sent = Serial.write(data, length);
    const uint32_t elapsed = micros() - started;
    uint32_t previous = __atomic_load_n(&s_statistics.maxWriteUs, __ATOMIC_RELAXED);
    while (elapsed > previous && !__atomic_compare_exchange_n(&s_statistics.maxWriteUs,
           &previous, elapsed, false, __ATOMIC_RELAXED, __ATOMIC_RELAXED)) {}
    __atomic_fetch_add(&s_statistics.txBytes, (uint32_t)sent, __ATOMIC_RELAXED);
    if (sent) __atomic_store_n(&s_statistics.lastTxMs, millis(), __ATOMIC_RELAXED);
    if (sent != length) __atomic_fetch_add(&s_statistics.shortWrites, 1U, __ATOMIC_RELAXED);
    if (!sent && length) __atomic_fetch_add(&s_statistics.zeroWrites, 1U, __ATOMIC_RELAXED);
    return sent;
}
void HostSerialPort::markCommand(uint32_t nowMs) {
    __atomic_fetch_add(&s_statistics.commands, 1U, __ATOMIC_RELAXED);
    __atomic_store_n(&s_statistics.lastCommandMs, nowMs, __ATOMIC_RELAXED);
}
HostSerialStatistics HostSerialPort::statistics() {
    HostSerialStatistics result;
#define COPY_COUNTER(field) result.field = __atomic_load_n(&s_statistics.field, __ATOMIC_RELAXED)
    COPY_COUNTER(rxBytes); COPY_COUNTER(txBytes); COPY_COUNTER(shortWrites); COPY_COUNTER(busResets);
    COPY_COUNTER(lastRxMs); COPY_COUNTER(lastTxMs); COPY_COUNTER(lastCommandMs); COPY_COUNTER(commands);
    COPY_COUNTER(zeroWrites); COPY_COUNTER(maxWriteUs);
#undef COPY_COUNTER
    return result;
}
void HostSerialPort::print(const char* text) {
    if (text) write(reinterpret_cast<const uint8_t*>(text), strlen(text));
}
void HostSerialPort::println(const char* text) {
    print(text);
    write(reinterpret_cast<const uint8_t*>("\r\n"), 2);
}
