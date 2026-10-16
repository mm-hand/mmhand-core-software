// Isolation sketch: SAME native HWCDC adapter as production; no servo, CAN,
// calibration, flash writes or motion. Compile only as an explicit experiment.
#include <Arduino.h>
#include <esp_system.h>
#include "../../../ESP32-P4/src/hal/serial/HostSerialPort.cpp"

uint32_t sequence = 0;
void setup() {
    // Independent observer on the Pico CH343 UART0 (RX38/TX37). Never a
    // second protocol/control owner; only this isolation sketch emits text.
    Serial0.begin(115200, SERIAL_8N1, 38, 37);
    Serial0.printf("USB_PROBE UART boot reset=%u before_hwcdc\n", esp_reset_reason());
    HostSerialPort::begin(kHostUartBaud);
    Serial0.println("USB_PROBE UART after_hwcdc");
    delay(1000);
    Serial.printf("USB_PROBE reset=%u build=%s %s\n", esp_reset_reason(), __DATE__, __TIME__);
}
void loop() {
    while (HostSerialPort::available()) HostSerialPort::readByte();
    char frame[256];
    const auto stats = HostSerialPort::statistics();
    const int length = snprintf(frame, sizeof(frame),
        "USB_PROBE seq=%lu ms=%lu rx=%lu tx=%lu short=%lu reset=%lu heap=%lu\n",
        (unsigned long)sequence++, (unsigned long)millis(), (unsigned long)stats.rxBytes,
        (unsigned long)stats.txBytes, (unsigned long)stats.shortWrites,
        (unsigned long)stats.busResets, (unsigned long)ESP.getFreeHeap());
    HostSerialPort::write(reinterpret_cast<const uint8_t*>(frame), (size_t)length);
    static uint32_t lastUart = 0;
    if (millis() - lastUart >= 1000) {
        lastUart = millis();
        Serial0.print(frame);
    }
    delay(10);
}
