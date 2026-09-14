#pragma once

#include "../config/Config.h"

// 21路磁编：每帧4个uint16，随后增加一帧序号、CRC和批次元数据。
#define CAN_ID_ENC_BASE 0x100
#define CAN_ID_ENC_LAST (CAN_ID_ENC_BASE + (ENCODER_TOTAL_NUM + 3) / 4 - 1)
#define CAN_ID_ENC_CRC (CAN_ID_ENC_LAST + 1)
#define CAN_ENC_FRAME_COUNT ((ENCODER_TOTAL_NUM + 3) / 4)
#define CAN_ENC_PAYLOAD_BYTES (ENCODER_TOTAL_NUM * 2)
#define CAN_ENC_CRC_DLC 8

// 21路错误详情：每帧1字节序号+7路压缩错误码，共3帧。
#define CAN_ID_ERROR_DETAIL_BASE 0x1F0
#define CAN_ID_ERROR_DETAIL_LAST (CAN_ID_ERROR_DETAIL_BASE + (ENCODER_TOTAL_NUM + 6) / 7 - 1)

// 触觉摘要：45字节三维力+2字节模块有效位图+2字节CRC16，正好拆成7帧。
#define CAN_ID_TAC_SUMMARY_BASE 0x240
#define CAN_ID_TAC_SUMMARY_LAST 0x246
#define CAN_ID_TAC_RAW_DIAG 0x3F0
#define CAN_TAC_SUMMARY_FRAME_COUNT 7
#define CAN_TAC_SUMMARY_PAYLOAD_PER_FRAME 7
#define CAN_TAC_SUMMARY_DLC 8
#define CAN_TAC_SUMMARY_VALID_MASK_BYTES 2
#define CAN_TAC_SUMMARY_CRC_BYTES 2
#define CAN_TAC_SUMMARY_CRC_INPUT_BYTES (TACTILE_SUMMARY_BYTES + CAN_TAC_SUMMARY_VALID_MASK_BYTES)
#define CAN_TAC_SUMMARY_PACKET_BYTES (CAN_TAC_SUMMARY_CRC_INPUT_BYTES + CAN_TAC_SUMMARY_CRC_BYTES)

// 全量触觉调试帧：每帧1字节序号+7字节数据，默认编译开关关闭发送。
#define CAN_ID_TAC_FULL_DUMP_BASE 0x400
#define CAN_TAC_FULL_PAYLOAD_PER_FRAME 7
#define CAN_TAC_FULL_DLC 8
#define CAN_TAC_FULL_FRAME_COUNT ((TACTILE_FULL_BYTES + CAN_TAC_FULL_PAYLOAD_PER_FRAME - 1) / CAN_TAC_FULL_PAYLOAD_PER_FRAME)
#define CAN_ID_TAC_FULL_DUMP_LAST (CAN_ID_TAC_FULL_DUMP_BASE + CAN_TAC_FULL_FRAME_COUNT - 1)

/** @brief ESP32-S3与P4之间的TWAI/CAN协议封装。 */
class HalTWAI {
public:
    HalTWAI();
    /** @brief 安装并启动1Mbit/s TWAI驱动。 */
    bool begin();

    /** @brief 发送21路磁编数据帧和批次CRC帧，返回成功帧数。 */
    uint8_t sendEncoderData(const EncoderData& encData);
    /** @brief 发送触觉摘要和15位模块有效位图，并返回该批次序号。 */
    uint8_t sendTactileSummary(const TactileData& tacData, uint16_t moduleValidMask);
    /** @brief 发送全量触觉调试数据；默认调用路径关闭。 */
    void sendTactileFullDump(const TactileData& tacData, uint8_t seq);
    /** @brief 仅对显著Z轴数据发送稀疏调试统计。 */
    void sendTactileRawDiag(const TactileData& tacData);
    /** @brief 将21路详细磁编错误码拆成3帧发送。 */
    void sendErrorStatus(const EncoderData& data);
    /** @brief 检查bus-off/停止状态并尝试恢复，返回总线是否RUNNING。 */
    bool maintain();

private:
    /** @brief 发送一帧标准11位ID CAN数据帧。 */
    bool sendFrame(uint32_t id, const uint8_t* data, uint8_t len);
    /** @brief 将S3内部16位错误信息压缩成CAN单字节错误码。 */
    static uint8_t encodeFaultCode(uint16_t rawLatchedError, uint16_t errorFlag);
    bool _is_initialized; ///< TWAI驱动是否已经成功启动。
};

// 全局唯一CAN对象。
extern HalTWAI twaiBus;
