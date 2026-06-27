#ifndef HAL_SENSOR_SPI_H
#define HAL_SENSOR_SPI_H

#include <Arduino.h>
#include <driver/spi_master.h>
#include "../config/Config.h"

extern spi_device_handle_t encoderSpiDevice;
extern spi_device_handle_t tactileSpiDevice;

bool beginTactileSensorSpi();
bool beginEncoderSensorSpi();

#endif
