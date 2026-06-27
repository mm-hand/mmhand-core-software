#pragma once

#include "../config/Config.h"

#define CAN_ID_ENC_BASE 0x100
#define CAN_ID_ENC_LAST (CAN_ID_ENC_BASE + (ENCODER_TOTAL_NUM + 3) / 4 - 1)
#define CAN_ID_ENC_CRC (CAN_ID_ENC_LAST + 1)
#define CAN_ENC_FRAME_COUNT ((ENCODER_TOTAL_NUM + 3) / 4)
#define CAN_ENC_PAYLOAD_BYTES (ENCODER_TOTAL_NUM * 2)
#define CAN_ENC_CRC_DLC 8
#define CAN_ID_ERROR_DETAIL_BASE 0x1F0
#define CAN_ID_ERROR_DETAIL_LAST (CAN_ID_ERROR_DETAIL_BASE + (ENCODER_TOTAL_NUM + 6) / 7 - 1)
#define CAN_ID_TAC_SUMMARY_BASE 0x240
#define CAN_ID_TAC_SUMMARY_LAST 0x246
#define CAN_ID_TAC_RAW_DIAG 0x3F0
#define CAN_TAC_SUMMARY_FRAME_COUNT 7
#define CAN_TAC_SUMMARY_PAYLOAD_PER_FRAME 7
#define CAN_TAC_SUMMARY_DLC 8
#define CAN_TAC_SUMMARY_CRC_BYTES 2
#define CAN_TAC_SUMMARY_PACKET_BYTES (TACTILE_SUMMARY_BYTES + CAN_TAC_SUMMARY_CRC_BYTES)
#define CAN_ID_TAC_FULL_DUMP_BASE 0x400
#define CAN_TAC_FULL_PAYLOAD_PER_FRAME 7
#define CAN_TAC_FULL_DLC 8
#define CAN_TAC_FULL_FRAME_COUNT ((TACTILE_FULL_BYTES + CAN_TAC_FULL_PAYLOAD_PER_FRAME - 1) / CAN_TAC_FULL_PAYLOAD_PER_FRAME)
#define CAN_ID_TAC_FULL_DUMP_LAST (CAN_ID_TAC_FULL_DUMP_BASE + CAN_TAC_FULL_FRAME_COUNT - 1)

class HalTWAI {
public:
    HalTWAI();
    bool begin();

    uint8_t sendEncoderData(const EncoderData& encData);
    uint8_t sendTactileSummary(const TactileData& tacData);
    void sendTactileFullDump(const TactileData& tacData, uint8_t seq);
    void sendTactileRawDiag(const TactileData& tacData);
    void sendErrorStatus(const EncoderData& data);
    bool sendCalibrationAck(bool success);

    bool receiveMonitor(RemoteCommand* outCmd);
    bool maintain();

private:
    bool sendFrame(uint32_t id, const uint8_t* data, uint8_t len);
    static uint8_t encodeFaultCode(uint16_t rawLatchedError, uint16_t errorFlag);
    bool _is_initialized;
};

extern HalTWAI twaiBus;
