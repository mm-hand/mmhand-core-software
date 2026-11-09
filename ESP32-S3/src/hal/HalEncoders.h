#ifndef HAL_ENCODERS_H
#define HAL_ENCODERS_H

#include <Arduino.h>
#include <SPI.h>
#include "driver/spi_master.h"
#include "../config/Config.h"
#include "EncoderWireClassifier.h"

/** @brief 一次接线自检的结果；由 EncodersTask 产生，CanTask 打印。 */
struct EncoderWireResult {
    uint8_t finger[5];  ///< 每指结论：O=有驱动，N=浮空，S=短地，H=被拉死。
    uint16_t up[5];     ///< 每指首个位置字（内部上拉），仅作串口证据。
    uint16_t down[5];   ///< 每指首个位置字（内部下拉），仅作串口证据。
    uint32_t seq;       ///< 自检序号，0 表示尚未执行。
};

/**
 * @brief 21路AS5047P磁编采集HAL。
 *
 * 五组磁编共用SPI信号，通过138板的CS译码器选择一根手指。
 * 类内部负责16位奇偶校验、一拍命令流水线、错误寄存器读取和物理链序重映射。
 */
class HalEncoders {
public:
    /** @brief 获取进程内唯一实例，避免多个对象同时操作同一SPI总线。 */
    static HalEncoders& getInstance() {
        static HalEncoders instance;
        return instance;
    }

    /** @brief 配置138板CS译码地址脚并初始化磁编SPI。 */
    void begin();
    /** @brief 扫描所有启用组，将结果写入调用者提供的快照。 */
    void getData(EncoderData& outData);
    /** @brief 按值返回一次完整磁编快照的便捷重载。 */
    EncoderData getData();

    /** @brief 节流执行接线自检（开机已测一次，之后每 2 s）。 */
    void serviceWireProbe(uint32_t nowMs);

    // [待确认是否删除] 当前工程没有调用该字符串转换函数，仅适合串口人工诊断。
    static const char* getErrorString(uint16_t errorCode);

private:
    HalEncoders();

    static constexpr uint8_t GROUP_COUNT = 5;    ///< 五根手指对应五个物理组。
    static constexpr uint8_t MAX_GROUP_SIZE = 8;///< SPI临时缓冲上限，当前最大实际组为5。

    // AS5047P寄存器地址：错误、磁场诊断、补偿后角度。
    static constexpr uint16_t AS5047P_REG_ERRFL = 0x0001;
    static constexpr uint16_t AS5047P_REG_DIAAGC = 0x3FFC;
    static constexpr uint16_t AS5047P_REG_ANGLECOM = 0x3FFF;

    // 连续恢复尝试上限；达到后强制回到角度读取状态，避免长期卡在诊断分支。
    static constexpr uint8_t MAX_RECOVERY = 3;

    spi_device_handle_t _spi; ///< ESP-IDF磁编SPI设备句柄。
    const uint8_t group_sizes_[GROUP_COUNT] = {4, 4, 4, 4, 5}; ///< 四指各4颗，拇指5颗。

    EncoderFSMState _fsmStates[ENCODER_TOTAL_NUM]; ///< 每路下一拍要读取的寄存器状态。
    uint16_t _cmdPipeline[ENCODER_TOTAL_NUM];      ///< 上一拍已发命令，用于解释本拍响应。
    uint16_t _latchedErrorCodes[ENCODER_TOTAL_NUM];///< 跨采样保留的详细错误码。
    uint8_t _recoveryAttempts[ENCODER_TOTAL_NUM]; ///< 连续诊断/掉线恢复次数。
    EncoderWireResult _wire{};                    ///< 最近一次接线自检结果。
    uint32_t _wireLastMs = 0;                     ///< 上次自检时间，用于节流。

    /** @brief 获取底层SPI句柄。 */
    void setupSPI();
    /** @brief 设置三位CS译码地址，并等待硬件稳定。 */
    void setDemuxChannel(uint8_t channel);
    /** @brief 在一个组选中期间完成count个16位字的同步收发。 */
    bool transferOnce(int8_t finger, uint8_t count, const uint16_t* txBuf, uint16_t* rxBuf);

    /** @brief 采集138-CS译码拓扑下的一组磁编。 */
    void collectGroup138(uint8_t groupIndex, int groupStart, uint8_t count, EncoderData& outData);
    /** @brief 按上一拍命令解析16位响应并推进单路诊断状态机。 */
    void decodeFrame(EncoderData& outData, int encoderId, uint16_t rawVal, uint16_t issuedCmd);

    /** @brief 将指定通道标为链路丢失并累计恢复次数。 */
    void markLinkLost(EncoderData& outData, int encoderId);
    /** @brief 根据单路FSM选择下一拍寄存器。 */
    uint16_t pickRegisterForState(int encoderId) const;
    /** @brief 将逻辑手指组换算为138板CS译码通道。 */
    uint8_t getGroupDemuxChannel(uint8_t groupIndex) const;
    /** @brief 计算16位值中1的奇偶性。 */
    uint8_t get_parity(uint16_t n);
    /** @brief 组合AS5047P地址、读写位和偶校验位。 */
    uint16_t build_command_frame(uint16_t address, bool is_read);
    /** @brief 将SPI移位顺序换算为组内逻辑关节顺序。 */
    uint8_t remapGroupIndex(uint8_t groupCount, uint8_t transferIndex) const;
    /** @brief 计算当前SPI字最终写入J00~J20中的哪个槽位。 */
    int mapGlobalEncoderIndex(int groupStart, uint8_t groupCount, uint8_t transferIndex) const;

    /** @brief 执行一次逐指 MISO 上/下拉自检。 */
    void probeWires(EncoderWireResult& out);
    /** @brief 选中一根手指并读回整链；pullUp 选择探测用的内部上下拉。 */
    bool probeFinger(uint8_t finger, bool pullUp, uint16_t* rxBuf);
    /** @brief 选手指：-1=全不选；下一代电路在这里同时切 MISO 数据选择器。 */
    void selectFinger(int8_t finger);
    /** @brief 0=无上下拉，1=上拉，2=下拉。 */
    void setMisoPull(uint8_t mode);
};

// 全局引用指向单例，供任务层直接使用。
extern HalEncoders& encoders;

#endif
