#pragma once
#include "../config/Config.h"

// 【新增】错误状态帧ID
// ==========================================
// 【修改】CAN ID 定义 - 与 HalTWAI 发送端一致
// ==========================================
#define CAN_ID_ENC_BASE   0x100  // 【修改】从 0x200 改为 0x100
#define CAN_ID_ENC_LAST   (CAN_ID_ENC_BASE + (ENCODER_TOTAL_NUM + 3) / 4 - 1) // 0x105
#define CAN_ID_ERROR_DETAIL_BASE 0x1F0
#define CAN_ID_ERROR_DETAIL_LAST (CAN_ID_ERROR_DETAIL_BASE + (ENCODER_TOTAL_NUM + 6) / 7 - 1) // 0x1F2

class HalTWAI {
public:
    HalTWAI();
    bool begin();
    
    // --- 发送函数声明 ---
    // 发送高频编码器数据
    void sendEncoderData(const EncoderData& encData);
    
    // 发送触觉合力概览
    void sendTactileSummary(const TactileData& tacData);
    
    // 发送详细触觉数据 (如果需要)
    void sendTactileFullDump(const TactileData& tacData);
    
    void sendErrorStatus(const EncoderData& data);

    // 【新增 2023-10】发送校准完成信号给 P4
    // 参数 success: true表示成功，false表示失败
    bool sendCalibrationAck(bool success); 
    
    // --- 接收函数声明 ---
    // 注意：统一为带参数版本，与 .cpp 保持一致
    bool receiveMonitor(RemoteCommand* outCmd);

    // 【新增】维护总线状态，返回 true 表示总线正常，false 表示正在恢复中
    bool maintain();

private:
    void sendFrame(uint32_t id, const uint8_t* data, uint8_t len);
    static uint8_t encodeFaultCode(uint16_t rawLatchedError, uint16_t errorFlag);
    bool _is_initialized;
};

extern HalTWAI twaiBus;
