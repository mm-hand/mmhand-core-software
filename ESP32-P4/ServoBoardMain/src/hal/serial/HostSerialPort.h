/**
 * @file HostSerialPort.h
 * @brief 隔离 Arduino Serial，供上位机协议和固件日志共用同一物理串口。
 * @details 本接口不解释协议，也不持有业务状态；写互斥仍由 FirmwareLog 管理。
 */
#ifndef P4_HAL_SERIAL_HOST_SERIAL_PORT_H
#define P4_HAL_SERIAL_HOST_SERIAL_PORT_H

#include <stddef.h>
#include <stdint.h>

class HostSerialPort {
public:
    using RxWakeHandler = void (*)(void* context);

    static void begin(uint32_t baudRate);
    /** 注册接收事件回调；回调只允许唤醒任务，禁止在 Serial 事件线程解析协议。 */
    static void setRxWakeHandler(RxWakeHandler handler, void* context);
    /** 原子取走自上次读取以来累计的 UART 接收错误位；0 表示没有硬件层错误。 */
    static uint32_t takeRxErrorFlags();
    static int available();
    static int readByte();
    static size_t write(const uint8_t* data, size_t length);
    static void print(const char* text);
    static void println(const char* text);
};

#endif // P4_HAL_SERIAL_HOST_SERIAL_PORT_H
