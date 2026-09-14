#include "HalSensorSpi.h"
#include <SPI.h>

// 句柄由beginSpiDevice创建，HAL通过它提交同步SPI事务。
spi_device_handle_t encoderSpiDevice = nullptr;
spi_device_handle_t tactileSpiDevice = nullptr;

namespace {

#ifndef ENCODER_SPI_HOST
// ESP32-S3的SPI3专用于21路AS5047P。
#define ENCODER_SPI_HOST SPI3_HOST
#endif

#ifndef TACTILE_SPI_HOST
// ESP32-S3的SPI2专用于15个触觉模块。
#define TACTILE_SPI_HOST SPI2_HOST
#endif

#ifndef ENC_SPI_MODE
// AS5047P当前板级时序使用SPI Mode 1。
#define ENC_SPI_MODE SPI_MODE1
#endif

// 防止重复向同一Host添加同一设备。
bool g_encoderStarted = false;
bool g_tactileStarted = false;
// 最大事务长度用于ESP-IDF DMA缓冲区规划；触觉按52触元全量帧计算。
constexpr int kTactileMaxTransferBytes =
    5 + 1 + (TACTILE_PAD_TAXELS * TACTILE_AXIS_COUNT) + 1;
// 磁编最大组8颗，每颗16位；当前实际最大组为5颗。
constexpr int kEncoderMaxTransferBytes = 2 * 8;

/**
 * @brief 初始化一个SPI Host并添加不使用硬件CS的同步设备。
 * @param host ESP-IDF SPI控制器编号。
 * @param sckPin/misoPin/mosiPin 三根SPI信号GPIO。
 * @param clockHz 总线时钟频率。
 * @param spiMode CPOL/CPHA模式。
 * @param maxTransferBytes 单次事务最大字节数。
 * @param device 输出设备句柄。
 * @param started 对应设备是否已成功初始化。
 * @note spics_io_num=-1，片选/多路选择由HalEncoders或HalTactile手动控制。
 */
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
    // INVALID_STATE通常表示该Host已由本工程初始化，可继续尝试添加设备。
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
    // 触觉协议使用SPI Mode 3、1MHz。
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
    // 磁编协议使用可配置Mode，当前10MHz。
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
