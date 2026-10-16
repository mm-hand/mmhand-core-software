#pragma once

#include "../config/Config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

constexpr uint8_t TACTILE_MODULE_NUM = TACTILE_GROUP_NUM * TACTILE_SENSOR_PER_GROUP;

enum class TactileReadFailReason : uint8_t {
    None = 0,
    InvalidArgument = 1,
    SpiNotReady = 2,
    CommandTransport = 3,
    ResponseTransport = 4,
    CrcMismatch = 5,
};

/** @brief 触觉采集诊断快照，记录本轮成功率和最近一次CRC故障现场。 */
struct TactileReadDiag {
    uint32_t frameCount;       ///< update()累计轮数。
    uint16_t enabledMask;      ///< 15个逻辑模块中启用的位图。
    uint16_t okReads;          ///< 本轮摘要和全量都成功的模块数。
    uint16_t failedReads;      ///< 本轮任一读取失败的模块数。
    uint16_t summaryOkMask;    ///< 本轮摘要寄存器读取成功模块位图。
    uint16_t fullOkMask;       ///< 本轮全量触元读取成功模块位图。
    uint16_t okMask;           ///< 本轮读取成功模块位图。
    uint16_t commandFailMask;  ///< 本轮命令事务失败模块位图。
    uint16_t responseFailMask; ///< 本轮响应事务失败模块位图。
    uint16_t crcFailMask;      ///< 本轮最终CRC8失败模块位图。
    uint16_t statusNonzeroMask;///< 本轮返回非零状态字节模块位图，仅诊断不判失败。
    uint16_t summaryRetryRecoveredMask; ///< 本轮摘要第二次读取恢复模块位图。
    uint16_t summarySuccessCount[TACTILE_MODULE_NUM]; ///< 分模块摘要成功累计，饱和计数。
    uint16_t summaryFailureCount[TACTILE_MODULE_NUM]; ///< 分模块摘要最终失败累计，饱和计数。
    uint16_t fullSuccessCount[TACTILE_MODULE_NUM];    ///< 分模块全量成功累计，饱和计数。
    uint16_t fullFailureCount[TACTILE_MODULE_NUM];    ///< 分模块全量失败累计，饱和计数。
    uint8_t summaryLastFailReason[TACTILE_MODULE_NUM];///< 分模块最近摘要失败原因。
    uint8_t fullLastFailReason[TACTILE_MODULE_NUM];   ///< 分模块最近全量失败原因。
    uint8_t moduleLastStatus[TACTILE_MODULE_NUM];     ///< 分模块最近响应状态字节。
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
    uint8_t inspectSensor(uint8_t physicalIndex, uint8_t* semanticIndex, uint8_t* model);
    bool writeSensorId(uint8_t physicalIndex, uint8_t destinationId, bool* writeSent);
    bool probePhysical(uint8_t physicalIndex);
    bool verifyPhysicalModel(uint8_t physicalIndex, uint8_t model);
    bool remapSensor(uint8_t semanticIndex, uint8_t newPhysicalIndex);

private:
    struct SpiReadResult {
        TactileReadFailReason reason;
        uint8_t status;
        uint8_t recvCrc;
        uint8_t calcCrc;
    };

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
    SpiReadResult _lastReadResult; ///< 最近一次SPI读取结果，供分模块统计。
    uint8_t _lastReadAttempts;     ///< 最近一次模块读取实际尝试次数。
    uint8_t _physicalRoute[TACTILE_GROUP_NUM][TACTILE_SENSOR_PER_GROUP];
};

// 全局单例对象，任务层直接调用。
extern HalTactile tactile;
