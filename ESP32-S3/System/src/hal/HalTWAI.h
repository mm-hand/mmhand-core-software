#pragma once

#include "../config/Config.h"

#define CAN_ID_ENC_BASE 0x100
#define CAN_ID_ENC_LAST (CAN_ID_ENC_BASE + (ENCODER_TOTAL_NUM + 3) / 4 - 1)

#define CAN_ID_ERROR_DETAIL_BASE 0x1F0
#define CAN_ID_ERROR_DETAIL_LAST (CAN_ID_ERROR_DETAIL_BASE + (ENCODER_TOTAL_NUM + 6) / 7 - 1)

// 7 frames x 7-byte payload + seq byte per frame => 49 bytes (45 bytes used).
#define CAN_ID_TAC_SUMMARY_BASE 0x240
#define CAN_ID_TAC_SUMMARY_LAST (CAN_ID_TAC_SUMMARY_BASE + 6)
#define CAN_TAC_SUMMARY_FRAME_COUNT (CAN_ID_TAC_SUMMARY_LAST - CAN_ID_TAC_SUMMARY_BASE + 1)
#define CAN_TAC_SUMMARY_PAYLOAD_PER_FRAME 7

class HalTWAI {
public:
    HalTWAI();
    bool begin();

    void sendEncoderData(const EncoderData& encData);
    void sendTactileSummary(const TactileData& tacData);
    void sendTactileFullDump(const TactileData& tacData);
    void sendErrorStatus(const EncoderData& data);
    bool sendCalibrationAck(bool success);
    bool receiveMonitor(RemoteCommand* outCmd);
    bool maintain();

private:
    void sendFrame(uint32_t id, const uint8_t* data, uint8_t len);
    static uint8_t encodeFaultCode(uint16_t rawLatchedError, uint16_t errorFlag);
    bool _is_initialized;
};

extern HalTWAI twaiBus;

