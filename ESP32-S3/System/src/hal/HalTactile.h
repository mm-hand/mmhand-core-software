#pragma once

#include "../config/Config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

/** @brief 触觉采集诊断快照，记录本轮成功率和最近一次CRC故障现场。 */
struct TactileReadDiag {
    uint32_t frameCount;       ///< update()累计轮数。
    uint16_t enabledMask;      ///< 15个逻辑模块中启用的位图。
    uint16_t okReads;          ///< 本轮摘要和全量都成功的模块数。
    uint16_t failedReads;      ///< 本轮任一读取失败的模块数。
    uint16_t summaryOkMask;    ///< 本轮摘要寄存器读取成功模块位图。
    uint16_t fullOkMask;       ///< 本轮全量触元读取成功模块位图。
    uint16_t okMask;           ///< 本轮读取成功模块位图。
    uint8_t lastStatus;        ///< 最近一次响应状态字节。
    uint8_t lastRecvCrc;       ///< 最近一次设备返回CRC8。
    uint8_t lastCalcCrc;       ///< 最近一次本地计算CRC8。
    uint8_t lastFailedPattern; ///< 最近失败的五位选择器编码。
    uint8_t lastFailStatus;    ///< 最近失败帧状态字节。
    uint8_t lastFailRecvCrc;   ///< 最近失败帧返回CRC8。
    uint8_t lastFailCalcCrc;   ///< 最近失败帧本地CRC8。
};

/**
 * @brief 15个触觉模块的SPI采集HAL。
 *
 * 通过五位板级选择器定位“手指组×指节模块”，每个模块分别读取三轴摘要和
 * 全触元力分布，使用CRC8查表法验证完整响应。
 */
class HalTactile {
public:
    HalTactile();
    /** @brief 在启动延时前将全部选择线置为安全高电平。 */
    void prepareBootSelectPins();
    /** @brief 初始化选择线和触觉SPI设备。 */
    void begin();
    /** @brief 扫描所有启用触觉模块，并原子替换共享快照。 */
    void update();
    /** @brief 返回最近一次完整触觉快照。 */
    TactileData getData();
    /** @brief 返回最近一次采集诊断快照。 */
    TactileReadDiag getDiagnostics();

private:
    bool setupSPI();
    void setAllSelectPinsHigh();
    void setSelectorPattern(uint8_t pattern);
    bool readRegisters(uint8_t selectorPattern, uint16_t startAddr, uint8_t* payload, uint16_t payloadLen);
    bool readSensorSummary(uint8_t groupIndex, uint8_t sensorIndex, uint8_t* globalOut, uint8_t* rawOut);
    bool readSensorForces(uint8_t groupIndex, uint8_t sensorIndex, uint8_t* forcesOut, uint16_t forcesLen);
    static uint8_t crc8(const uint8_t* data, uint32_t len);

    SemaphoreHandle_t _mutex; ///< 保护_data和_diag快照。
    TactileData* _data;       ///< 堆上持有的最新完整触觉数据。
    spi_device_handle_t _spi; ///< ESP-IDF触觉SPI设备句柄。
    bool _spiReady;           ///< SPI初始化是否成功。
    TactileReadDiag _diag;    ///< 最近采集诊断统计。
};

// 全局单例对象，任务层直接调用。
extern HalTactile tactile;
