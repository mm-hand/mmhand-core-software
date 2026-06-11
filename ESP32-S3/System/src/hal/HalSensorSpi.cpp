#include "HalSensorSpi.h"
#include <SPI.h>

spi_device_handle_t encoderSpiDevice = nullptr;
spi_device_handle_t tactileSpiDevice = nullptr;

namespace {

#ifndef ENCODER_SPI_HOST
#define ENCODER_SPI_HOST SPI3_HOST
#endif

#ifndef TACTILE_SPI_HOST
#define TACTILE_SPI_HOST SPI2_HOST
#endif

bool g_encoderStarted = false;
bool g_tactileStarted = false;
constexpr int kTactileMaxTransferBytes =
    5 + 1 + (TACTILE_PAD_TAXELS * TACTILE_AXIS_COUNT) + 1;
constexpr int kEncoderMaxTransferBytes = 2 * 8;

bool beginSpiDevice(
    spi_host_device_t host,
    int sckPin,
    int misoPin,
    int mosiPin,
    int clockHz,
    int spiMode,
    int maxTransferBytes,
    spi_device_handle_t* device,
    bool* started
) {
    if (*started && *device != nullptr) {
        return true;
    }

    spi_bus_config_t busConfig = {};
    busConfig.mosi_io_num = mosiPin;
    busConfig.miso_io_num = misoPin;
    busConfig.sclk_io_num = sckPin;
    busConfig.quadwp_io_num = -1;
    busConfig.quadhd_io_num = -1;
    busConfig.max_transfer_sz = maxTransferBytes;

    const esp_err_t busErr = spi_bus_initialize(host, &busConfig, SPI_DMA_CH_AUTO);
    if (busErr != ESP_OK && busErr != ESP_ERR_INVALID_STATE) {
        return false;
    }

    spi_device_interface_config_t devConfig = {};
    devConfig.clock_speed_hz = clockHz;
    devConfig.mode = spiMode;
    devConfig.spics_io_num = -1;
    devConfig.queue_size = 1;

    const esp_err_t devErr = spi_bus_add_device(host, &devConfig, device);
    if (devErr != ESP_OK) {
        return false;
    }

    *started = true;
    return true;
}

} // namespace

bool beginTactileSensorSpi() {
    return beginSpiDevice(
        TACTILE_SPI_HOST,
        PIN_TAC_SCLK,
        PIN_TAC_MISO,
        PIN_TAC_MOSI,
        SPI_FREQ_TAC,
        SPI_MODE3,
        kTactileMaxTransferBytes,
        &tactileSpiDevice,
        &g_tactileStarted
    );
}

bool beginEncoderSensorSpi() {
    return beginSpiDevice(
        ENCODER_SPI_HOST,
        PIN_ENC_SCLK,
        PIN_ENC_MISO,
        PIN_ENC_MOSI,
        SPI_FREQ_ENC,
        ENC_SPI_MODE,
        kEncoderMaxTransferBytes,
        &encoderSpiDevice,
        &g_encoderStarted
    );
}
