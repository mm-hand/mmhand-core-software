#ifndef HAL_SENSOR_SPI_H
#define HAL_SENSOR_SPI_H

#include <Arduino.h>
#include <driver/spi_master.h>
#include "../config/Config.h"

// ESP-IDF SPI设备句柄：磁编与触觉位于不同SPI Host，互不共享时钟配置。
extern spi_device_handle_t encoderSpiDevice;
extern spi_device_handle_t tactileSpiDevice;

/** @brief 初始化触觉SPI总线和设备，重复调用时直接复用已有句柄。 */
bool beginTactileSensorSpi();
/** @brief 初始化磁编SPI总线和设备，重复调用时直接复用已有句柄。 */
bool beginEncoderSensorSpi();

#endif
