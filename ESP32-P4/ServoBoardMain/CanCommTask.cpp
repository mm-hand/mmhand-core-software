#include "CanCommTask.h"
#include "driver/twai.h"

#include <string.h>

// CanCommTask 模块职责：
// 1) 接收编码器帧并重建完整关节原始值；
// 2) 接收错误详情分片并重组 errorFlags/errorBitmap；
// 3) 透传上层待发送 CAN 指令。

static_assert(
    CAN_ERROR_DETAIL_FRAME_COUNT == (CAN_ID_ERROR_DETAIL_LAST - CAN_ID_ERROR_DETAIL_BASE + 1),
    "Error-detail ID range must match frame count.");
static_assert(
    CAN_ERROR_DETAIL_FRAME_COUNT * CAN_ERROR_DETAIL_CODES_PER_FRAME == ENCODER_TOTAL_NUM,
    "Error-detail payload must cover all encoder channels.");
static_assert(
    CAN_ERROR_DETAIL_DLC == (1 + CAN_ERROR_DETAIL_CODES_PER_FRAME),
    "Error-detail DLC must be seq(1) + channel codes.");
static_assert(ENCODER_TOTAL_NUM <= 32, "errorBitmap only supports up to 32 channels.");

typedef struct {
    uint8_t active;
    uint8_t seq;
    uint8_t frameMask;
    uint8_t codes[ENCODER_TOTAL_NUM];
    uint32_t firstFrameTimeMs;
} ErrorDetailReassemblyState;

// 重置错误详情分片重组状态机。
static inline void resetErrorDetailReassembly(ErrorDetailReassemblyState* state)
{
    if (!state) {
        return;
    }
    state->active = 0;
    state->seq = 0;
    state->frameMask = 0;
    state->firstFrameTimeMs = 0;
    memset(state->codes, 0, sizeof(state->codes));
}

// 根据通道错误码数组构建 32 位错误位图。
static uint32_t buildErrorBitmap(const uint8_t* codes)
{
    if (!codes) {
        return 0;
    }

    uint32_t bitmap = 0;
    for (int i = 0; i < ENCODER_TOTAL_NUM; i++)
    {
        if (codes[i] != 0) {
            bitmap |= (1UL << i);
        }
    }
    return bitmap;
}

// 初始化 TWAI 驱动（幂等调用，仅首次生效）。
static void setupTwai()
{
    static bool installed = false;
    if (installed) {
        return;
    }

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)TWAI_TX_PIN,
        (gpio_num_t)TWAI_RX_PIN,
        TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    g_config.rx_queue_len = 64;

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        twai_start();
        installed = true;
        Serial.println("[CAN] Driver Installed OK");
    }
}

// CAN 通信任务主循环：读总线、重组数据、写共享队列、发送待发命令。
void taskCanComm(void* parameter)
{
    TaskSharedData_t* sharedData = (TaskSharedData_t*)parameter;
    setupTwai();

    RemoteSensorData_t rxBuffer;
    memset(&rxBuffer, 0, sizeof(rxBuffer));

    ErrorDetailReassemblyState errorState;
    resetErrorDetailReassembly(&errorState);

    twai_message_t rxMsg;
    uint32_t lastRxTime = 0;
    uint32_t lastCompleteErrorBatchTime = millis();
    RemoteCommand_t txCmd;

    const uint8_t kAllErrorFramesMask = (uint8_t)((1U << CAN_ERROR_DETAIL_FRAME_COUNT) - 1U);

    for (;;)
    {
        while (twai_receive(&rxMsg, 0) == ESP_OK)
        {
            const uint32_t nowMs = millis();
            lastRxTime = nowMs;

            if (rxMsg.identifier >= CAN_ID_ENC_BASE && rxMsg.identifier <= CAN_ID_ENC_LAST)
            {
                const int frameIdx = (int)(rxMsg.identifier - CAN_ID_ENC_BASE);
                const int baseIdx = frameIdx * 4;
                const int payloadLen = (int)rxMsg.data_length_code;

                for (int i = 0; i < 4; i++)
                {
                    const int realIdx = baseIdx + i;
                    const int dataOffset = i * 2;

                    if (realIdx >= ENCODER_TOTAL_NUM) {
                        continue;
                    }
                    if ((dataOffset + 1) >= payloadLen) {
                        break;
                    }

                    const uint16_t val = ((uint16_t)rxMsg.data[dataOffset] << 8) |
                                         rxMsg.data[dataOffset + 1];
                    rxBuffer.encoderValues[realIdx] = val;
                }

                if (rxMsg.identifier == CAN_ID_ENC_LAST)
                {
                    rxBuffer.timestamp = nowMs;
                    rxBuffer.isValid = true;
                    xQueueOverwrite(sharedData->canRxQueue, &rxBuffer);
                }
            }
            else if (rxMsg.identifier >= CAN_ID_ERROR_DETAIL_BASE &&
                     rxMsg.identifier <= CAN_ID_ERROR_DETAIL_LAST)
            {
                if (rxMsg.data_length_code != CAN_ERROR_DETAIL_DLC) {
                    continue;
                }

                const uint8_t seq = rxMsg.data[0];
                const uint8_t frameIdx = (uint8_t)(rxMsg.identifier - CAN_ID_ERROR_DETAIL_BASE);

                if (errorState.active)
                {
                    const bool seqChanged = (seq != errorState.seq);
                    const bool reassemblyTimedOut =
                        ((uint32_t)(nowMs - errorState.firstFrameTimeMs) >
                         CAN_ERROR_REASSEMBLY_TIMEOUT_MS);
                    if (seqChanged || reassemblyTimedOut) {
                        resetErrorDetailReassembly(&errorState);
                    }
                }

                if (!errorState.active)
                {
                    errorState.active = 1;
                    errorState.seq = seq;
                    errorState.frameMask = 0;
                    errorState.firstFrameTimeMs = nowMs;
                    memset(errorState.codes, 0, sizeof(errorState.codes));
                }

                const int baseIdx = (int)frameIdx * CAN_ERROR_DETAIL_CODES_PER_FRAME;
                for (int i = 0; i < CAN_ERROR_DETAIL_CODES_PER_FRAME; i++)
                {
                    const int channelIdx = baseIdx + i;
                    if (channelIdx < ENCODER_TOTAL_NUM) {
                        errorState.codes[channelIdx] = rxMsg.data[1 + i];
                    }
                }

                errorState.frameMask |= (uint8_t)(1U << frameIdx);
                if (errorState.frameMask == kAllErrorFramesMask)
                {
                    memcpy(rxBuffer.errorFlags, errorState.codes, sizeof(rxBuffer.errorFlags));
                    rxBuffer.errorBitmap = buildErrorBitmap(rxBuffer.errorFlags);
                    lastCompleteErrorBatchTime = nowMs;
                    resetErrorDetailReassembly(&errorState);
                }
            }
        }

        const uint32_t nowMs = millis();
        if (errorState.active &&
            (uint32_t)(nowMs - errorState.firstFrameTimeMs) > CAN_ERROR_REASSEMBLY_TIMEOUT_MS)
        {
            resetErrorDetailReassembly(&errorState);
        }

        if ((uint32_t)(nowMs - lastCompleteErrorBatchTime) > CAN_ERROR_TABLE_CLEAR_TIMEOUT_MS &&
            rxBuffer.errorBitmap != 0)
        {
            memset(rxBuffer.errorFlags, 0, sizeof(rxBuffer.errorFlags));
            rxBuffer.errorBitmap = 0;
        }

        if (millis() - lastRxTime > 500 && rxBuffer.isValid)
        {
            // 保持现有行为：短时总线静默不强制置 isValid=false。
        }

        if (xQueueReceive(sharedData->canTxQueue, &txCmd, 0) == pdTRUE)
        {
            twai_message_t txMsg;
            txMsg.identifier = txCmd.cmdID;
            txMsg.extd = 0;
            txMsg.data_length_code = txCmd.len;
            memcpy(txMsg.data, txCmd.payload, txCmd.len);
            twai_transmit(&txMsg, pdMS_TO_TICKS(10));
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
