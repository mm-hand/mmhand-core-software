#include "CanCommTask.h"
#include "main.h"
#include "task.h"

#include <cstring>

#if CAN_ERROR_DETAIL_FRAME_COUNT != (CAN_ID_ERROR_DETAIL_LAST - CAN_ID_ERROR_DETAIL_BASE + 1)
#error "Error-detail ID range must match frame count."
#endif

#if (CAN_ERROR_DETAIL_FRAME_COUNT * CAN_ERROR_DETAIL_CODES_PER_FRAME) != ENCODER_TOTAL_NUM
#error "Error-detail payload must cover all encoder channels."
#endif

#if CAN_ERROR_DETAIL_DLC != (1 + CAN_ERROR_DETAIL_CODES_PER_FRAME)
#error "Error-detail DLC must be seq(1) + channel codes."
#endif

#if ENCODER_TOTAL_NUM > 32
#error "errorBitmap only supports up to 32 channels."
#endif

#if CAN_TAC_SUMMARY_FRAME_COUNT != (CAN_ID_TAC_SUMMARY_LAST - CAN_ID_TAC_SUMMARY_BASE + 1)
#error "Tactile frame ID range must match frame count."
#endif

#if CAN_TAC_SUMMARY_DLC != (1 + CAN_TAC_SUMMARY_PAYLOAD_PER_FRAME)
#error "Tactile DLC must be seq(1) + payload bytes."
#endif

#if TACTILE_SUMMARY_BYTES > (CAN_TAC_SUMMARY_FRAME_COUNT * CAN_TAC_SUMMARY_PAYLOAD_PER_FRAME)
#error "Tactile summary payload budget too small."
#endif

extern FDCAN_HandleTypeDef hfdcan1;

CanComm can1;

// 保持 freertos.c 里的 C 任务入口不变，把实际 CAN 通信逻辑转交给 can1 对象。
extern "C" void CanCommTask_Run(void *parameter)
{
    can1.run(parameter);
}

// FDCAN Rx FIFO0 中断回调：只做任务唤醒，实际取帧仍放在 canCommunicationTask 中完成。
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((hfdcan != nullptr) && (hfdcan->Instance == FDCAN1) &&
        ((RxFifo0ITs & (FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                        FDCAN_IT_RX_FIFO0_FULL |
                        FDCAN_IT_RX_FIFO0_MESSAGE_LOST)) != 0U)) {
        can1.notifyRxFromIsr();
    }
}

// FDCAN Tx FIFO 空中断回调：唤醒 CAN 任务继续从发送队列补帧。
extern "C" void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan)
{
    if ((hfdcan != nullptr) && (hfdcan->Instance == FDCAN1)) {
        can1.notifyTxFromIsr();
    }
}

// FDCAN Tx Event FIFO 回调：记录硬件发送事件并唤醒 CAN 任务。
extern "C" void HAL_FDCAN_TxEventFifoCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t TxEventFifoITs)
{
    if ((hfdcan != nullptr) && (hfdcan->Instance == FDCAN1) &&
        ((TxEventFifoITs & (FDCAN_IT_TX_EVT_FIFO_NEW_DATA |
                            FDCAN_IT_TX_EVT_FIFO_FULL |
                            FDCAN_IT_TX_EVT_FIFO_ELT_LOST)) != 0U)) {
        can1.notifyTxFromIsr();
    }
}

// FDCAN 错误状态中断回调：记录错误中断并唤醒任务做后续处理。
extern "C" void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    (void)ErrorStatusITs;
    if ((hfdcan != nullptr) && (hfdcan->Instance == FDCAN1)) {
        can1.notifyErrorFromIsr();
    }
}

// FDCAN 通用错误回调：记录错误中断并唤醒任务，避免在 ISR 里做重活。
extern "C" void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    if ((hfdcan != nullptr) && (hfdcan->Instance == FDCAN1)) {
        can1.notifyErrorFromIsr();
    }
}

// 返回当前系统毫秒计时，保持与 P4 侧 millis() 调用语义一致。
uint32_t CanComm::millis(void) const
{
    return HAL_GetTick();
}

// 将 HAL FDCAN 的 DLC 常量转换为 CAN/CAN FD 实际数据长度。
uint8_t CanComm::fdcanDlcToLen(uint32_t dlc) const
{
    if (dlc <= FDCAN_DLC_BYTES_8) {
        return static_cast<uint8_t>(dlc);
    }

    switch (dlc) {
    case FDCAN_DLC_BYTES_12:
        return 12;
    case FDCAN_DLC_BYTES_16:
        return 16;
    case FDCAN_DLC_BYTES_20:
        return 20;
    case FDCAN_DLC_BYTES_24:
        return 24;
    case FDCAN_DLC_BYTES_32:
        return 32;
    case FDCAN_DLC_BYTES_48:
        return 48;
    case FDCAN_DLC_BYTES_64:
        return 64;
    default:
        return 0;
    }
}

// 将 0..64 字节长度转换为 HAL FDCAN 发送头需要的 DLC 常量。
uint32_t CanComm::lenToFdcanDlc(uint8_t len) const
{
    if (len == 0U) {
        return FDCAN_DLC_BYTES_0;
    } else if (len == 1U) {
        return FDCAN_DLC_BYTES_1;
    } else if (len == 2U) {
        return FDCAN_DLC_BYTES_2;
    } else if (len == 3U) {
        return FDCAN_DLC_BYTES_3;
    } else if (len == 4U) {
        return FDCAN_DLC_BYTES_4;
    } else if (len == 5U) {
        return FDCAN_DLC_BYTES_5;
    } else if (len == 6U) {
        return FDCAN_DLC_BYTES_6;
    } else if (len == 7U) {
        return FDCAN_DLC_BYTES_7;
    } else if (len <= 8U) {
        return FDCAN_DLC_BYTES_8;
    } else if (len <= 12U) {
        return FDCAN_DLC_BYTES_12;
    } else if (len <= 16U) {
        return FDCAN_DLC_BYTES_16;
    } else if (len <= 20U) {
        return FDCAN_DLC_BYTES_20;
    } else if (len <= 24U) {
        return FDCAN_DLC_BYTES_24;
    } else if (len <= 32U) {
        return FDCAN_DLC_BYTES_32;
    } else if (len <= 48U) {
        return FDCAN_DLC_BYTES_48;
    }

    return FDCAN_DLC_BYTES_64;
}

// 清空错误详情分片重组状态，丢弃未完成的一批错误码帧。
void CanComm::resetErrorDetailReassembly(void)
{
    errorState.active = 0;
    errorState.seq = 0;
    errorState.frameMask = 0;
    errorState.firstFrameTimeMs = 0;
    std::memset(errorState.codes, 0, sizeof(errorState.codes));
}

// 清空触觉汇总分片重组状态，丢弃未完成的一批触觉数据帧。
void CanComm::resetTactileReassembly(void)
{
    tactileState.active = 0;
    tactileState.seq = 0;
    tactileState.frameMask = 0;
    tactileState.firstFrameTimeMs = 0;
    std::memset(tactileState.bytes, 0, sizeof(tactileState.bytes));
}

// 将每个编码器通道的错误码压缩为 bitmap，便于上层快速判断故障通道。
uint32_t CanComm::buildErrorBitmap(const uint8_t *codes) const
{
    if (codes == nullptr) {
        return 0;
    }

    uint32_t bitmap = 0;
    for (int i = 0; i < ENCODER_TOTAL_NUM; i++) {
        if (codes[i] != 0) {
            bitmap |= (1UL << i);
        }
    }

    return bitmap;
}

// 把最近一次收到的 CAN 原始帧保存到 can1.RX* 字段，方便调试器直接展开查看。
void CanComm::rememberRxFrame(const FDCAN_RxHeaderTypeDef &header,
                              const uint8_t *data,
                              uint8_t payloadLen,
                              uint32_t nowMs)
{
    RXheader = header;
    RXid = header.Identifier;
    RXlen = payloadLen;
    RXtimestamp = nowMs;
    RXframeCount++;

    std::memset(RXdata, 0, sizeof(RXdata));
    const uint8_t copyLen = (payloadLen <= sizeof(RXdata)) ?
                            payloadLen :
                            static_cast<uint8_t>(sizeof(RXdata));
    if ((data != nullptr) && (copyLen > 0U)) {
        std::memcpy(RXdata, data, copyLen);
    }
}

// 把最近一次准备发送的 CAN 帧保存到 can1.TX* 字段，方便调试器直接展开查看。
void CanComm::rememberTxFrame(const FDCAN_TxHeaderTypeDef &header,
                              const uint8_t *data,
                              uint8_t payloadLen,
                              uint32_t nowMs)
{
    TXheader = header;
    TXid = header.Identifier;
    TXlen = payloadLen;
    TXtimestamp = nowMs;

    std::memset(TXdata, 0, sizeof(TXdata));
    const uint8_t copyLen = (payloadLen <= sizeof(TXdata)) ?
                            payloadLen :
                            static_cast<uint8_t>(sizeof(TXdata));
    if ((data != nullptr) && (copyLen > 0U)) {
        std::memcpy(TXdata, data, copyLen);
    }
}

// 在 ISR 中递增调试计数并唤醒 CAN 任务，避免中断里直接处理 FIFO。
void CanComm::notifyTaskFromIsr(volatile uint32_t *counter)
{
    if (counter != nullptr) {
        (*counter)++;
    }

    if (taskHandle != nullptr) {
        BaseType_t higherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(taskHandle, &higherPriorityTaskWoken);
        portYIELD_FROM_ISR(higherPriorityTaskWoken);
    }
}

// 记录 FDCAN 接收中断次数并唤醒 CAN 任务。
void CanComm::notifyRxFromIsr(void)
{
    notifyTaskFromIsr(&RXinterruptCount);
}

// 记录 FDCAN 发送相关中断次数并唤醒 CAN 任务。
void CanComm::notifyTxFromIsr(void)
{
    notifyTaskFromIsr(&TXinterruptCount);
}

// 记录 FDCAN 错误中断次数并唤醒 CAN 任务。
void CanComm::notifyErrorFromIsr(void)
{
    notifyTaskFromIsr(&fdcanErrorInterruptCount);
}

// 配置 FDCAN 标准帧接收过滤器并启动 FDCAN1，只会成功启动一次。
bool CanComm::setupFdcan(void)
{
    if (fdcanStarted != 0U) {
        return true;
    }

    FDCAN_FilterTypeDef filter = {};
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0;
    filter.FilterID2 = 0;

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) {
        fdcanStartFailCount++;
        return false;
    }

    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK) {
        fdcanStartFailCount++;
        return false;
    }

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        fdcanStartFailCount++;
        return false;
    }

    if (HAL_FDCAN_ActivateNotification(&hfdcan1,
                                       FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                                       FDCAN_IT_RX_FIFO0_FULL |
                                       FDCAN_IT_RX_FIFO0_MESSAGE_LOST |
                                       FDCAN_IT_TX_FIFO_EMPTY |
                                       FDCAN_IT_TX_EVT_FIFO_NEW_DATA |
                                       FDCAN_IT_TX_EVT_FIFO_FULL |
                                       FDCAN_IT_TX_EVT_FIFO_ELT_LOST |
                                       FDCAN_IT_ERROR_WARNING |
                                       FDCAN_IT_ERROR_PASSIVE |
                                       FDCAN_IT_BUS_OFF,
                                       0U) != HAL_OK) {
        fdcanStartFailCount++;
        return false;
    }

    fdcanStarted = 1;
    return true;
}

// 解析编码器 CAN 帧，每帧最多 4 个通道，收到最后一帧后发布传感器快照。
void CanComm::handleEncoderFrame(uint32_t identifier,
                                 const uint8_t *data,
                                 uint8_t payloadLen,
                                 uint32_t nowMs)
{
    const int frameIdx = static_cast<int>(identifier - CAN_ID_ENC_BASE);
    const int baseIdx = frameIdx * 4;

    for (int i = 0; i < 4; i++) {
        const int realIdx = baseIdx + i;
        const int dataOffset = i * 2;

        if (realIdx >= ENCODER_TOTAL_NUM) {
            continue;
        }

        if ((dataOffset + 1) >= payloadLen) {
            break;
        }

        sensor.encoderValues[realIdx] =
            (static_cast<uint16_t>(data[dataOffset]) << 8) | data[dataOffset + 1];
    }

    if (identifier == CAN_ID_ENC_LAST) {
        sensor.timestamp = nowMs;
        sensor.isValid = true;

        if ((shared != nullptr) && (shared->canRxQueue != nullptr)) {
            (void)xQueueOverwrite(shared->canRxQueue, &sensor);
        }
    }
}

// 解析并重组 0x1F0..0x1F2 错误详情帧，完整后更新 can1.sensor 错误数组和错误位图。
void CanComm::handleErrorDetailFrame(uint32_t identifier,
                                     const uint8_t *data,
                                     uint8_t payloadLen,
                                     uint32_t nowMs)
{
    if (payloadLen != CAN_ERROR_DETAIL_DLC) {
        return;
    }

    const uint8_t seq = data[0];
    const uint8_t frameIdx = static_cast<uint8_t>(identifier - CAN_ID_ERROR_DETAIL_BASE);

    if (errorState.active != 0U) {
        const bool seqChanged = (seq != errorState.seq);
        const bool timedOut =
            ((uint32_t)(nowMs - errorState.firstFrameTimeMs) > CAN_ERROR_REASSEMBLY_TIMEOUT_MS);

        if (seqChanged || timedOut) {
            resetErrorDetailReassembly();
        }
    }

    if (errorState.active == 0U) {
        errorState.active = 1;
        errorState.seq = seq;
        errorState.frameMask = 0;
        errorState.firstFrameTimeMs = nowMs;
        std::memset(errorState.codes, 0, sizeof(errorState.codes));
    }

    const int baseIdx = static_cast<int>(frameIdx) * CAN_ERROR_DETAIL_CODES_PER_FRAME;
    for (int i = 0; i < CAN_ERROR_DETAIL_CODES_PER_FRAME; i++) {
        const int channelIdx = baseIdx + i;

        if (channelIdx < ENCODER_TOTAL_NUM) {
            errorState.codes[channelIdx] = data[1 + i];
        }
    }

    const uint8_t allErrorFramesMask =
        static_cast<uint8_t>((1U << CAN_ERROR_DETAIL_FRAME_COUNT) - 1U);
    errorState.frameMask |= static_cast<uint8_t>(1U << frameIdx);

    if (errorState.frameMask == allErrorFramesMask) {
        std::memcpy(sensor.errorFlags, errorState.codes, sizeof(sensor.errorFlags));
        sensor.errorBitmap = buildErrorBitmap(sensor.errorFlags);
        lastCompleteErrorBatchTime = nowMs;
        resetErrorDetailReassembly();
    }
}

// 解析并重组 0x240..0x246 触觉汇总帧，完整后更新 can1.tactile 并覆盖 tactileQueue。
void CanComm::handleTactileFrame(uint32_t identifier,
                                 const uint8_t *data,
                                 uint8_t payloadLen,
                                 uint32_t nowMs)
{
    if (payloadLen != CAN_TAC_SUMMARY_DLC) {
        return;
    }

    const uint8_t seq = data[0];
    const uint8_t frameIdx = static_cast<uint8_t>(identifier - CAN_ID_TAC_SUMMARY_BASE);

    if (tactileState.active != 0U) {
        const bool seqChanged = (seq != tactileState.seq);
        const bool timedOut =
            ((uint32_t)(nowMs - tactileState.firstFrameTimeMs) > CAN_TAC_REASSEMBLY_TIMEOUT_MS);

        if (seqChanged || timedOut) {
            resetTactileReassembly();
        }
    }

    if (tactileState.active == 0U) {
        tactileState.active = 1;
        tactileState.seq = seq;
        tactileState.frameMask = 0;
        tactileState.firstFrameTimeMs = nowMs;
        std::memset(tactileState.bytes, 0, sizeof(tactileState.bytes));
    }

    const int baseIdx = static_cast<int>(frameIdx) * CAN_TAC_SUMMARY_PAYLOAD_PER_FRAME;
    for (int i = 0; i < CAN_TAC_SUMMARY_PAYLOAD_PER_FRAME; i++) {
        const int tactileIdx = baseIdx + i;

        if (tactileIdx < TACTILE_SUMMARY_BYTES) {
            tactileState.bytes[tactileIdx] = data[1 + i];
        }
    }

    const uint8_t allTactileFramesMask =
        static_cast<uint8_t>((1U << CAN_TAC_SUMMARY_FRAME_COUNT) - 1U);
    tactileState.frameMask |= static_cast<uint8_t>(1U << frameIdx);

    if (tactileState.frameMask == allTactileFramesMask) {
        std::memset(&tactile, 0, sizeof(tactile));
        std::memcpy(tactile.values, tactileState.bytes, sizeof(tactile.values));
        tactile.seq = tactileState.seq;
        tactile.timestamp = nowMs;
        tactile.isValid = true;

        if ((shared != nullptr) && (shared->tactileQueue != nullptr)) {
            (void)xQueueOverwrite(shared->tactileQueue, &tactile);
        }

        resetTactileReassembly();
    }
}

// 按 CAN ID 分发收到的数据帧到编码器、错误详情或触觉汇总处理逻辑。
void CanComm::handleRxFrame(uint32_t identifier,
                            const uint8_t *data,
                            uint8_t payloadLen,
                            uint32_t nowMs)
{
    if ((identifier >= CAN_ID_ENC_BASE) && (identifier <= CAN_ID_ENC_LAST)) {
        handleEncoderFrame(identifier, data, payloadLen, nowMs);
    } else if ((identifier >= CAN_ID_ERROR_DETAIL_BASE) && (identifier <= CAN_ID_ERROR_DETAIL_LAST)) {
        handleErrorDetailFrame(identifier, data, payloadLen, nowMs);
    } else if ((identifier >= CAN_ID_TAC_SUMMARY_BASE) && (identifier <= CAN_ID_TAC_SUMMARY_LAST)) {
        handleTactileFrame(identifier, data, payloadLen, nowMs);
    } else {
        RXignoredCount++;
    }
}

// 清空 FDCAN Rx FIFO0，将有效的标准数据帧交给协议层解析。
void CanComm::drainRxFifo(void)
{
    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[CAN_COMM_MAX_DATA_SIZE];

    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0U) {
        std::memset(rxData, 0, sizeof(rxData));

        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK) {
            break;
        }

        const uint8_t payloadLen = fdcanDlcToLen(rxHeader.DataLength);
        const uint32_t nowMs = millis();
        rememberRxFrame(rxHeader, rxData, payloadLen, nowMs);

        if ((rxHeader.IdType != FDCAN_STANDARD_ID) ||
            (rxHeader.RxFrameType != FDCAN_DATA_FRAME)) {
            RXignoredCount++;
            continue;
        }

        RXacceptedCount++;
        handleRxFrame(rxHeader.Identifier, rxData, payloadLen, nowMs);
    }
}

// 从 canTxQueue 取上层待发送命令，封装为标准 CAN FD 帧并放入 FDCAN Tx FIFO。
void CanComm::processTxQueue(void)
{
    if ((shared == nullptr) || (shared->canTxQueue == nullptr)) {
        return;
    }

    RemoteCommand_t txCmd;
    if (xQueueReceive(shared->canTxQueue, &txCmd, 0) != pdTRUE) {
        return;
    }

    if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U) {
        TXretryCount++;
        (void)xQueueSendToFront(shared->canTxQueue, &txCmd, 0);
        return;
    }

    TXcommand = txCmd;
    const uint8_t payloadLen = (txCmd.len <= sizeof(txCmd.payload)) ?
                               txCmd.len :
                               static_cast<uint8_t>(sizeof(txCmd.payload));

    FDCAN_TxHeaderTypeDef txHeader = {};
    txHeader.Identifier = txCmd.cmdID;
    txHeader.IdType = FDCAN_STANDARD_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = lenToFdcanDlc(payloadLen);
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_FD_CAN;
    txHeader.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
    txHeader.MessageMarker = 0;

    uint8_t txPayload[CAN_COMM_MAX_DATA_SIZE] = {};
    std::memcpy(txPayload, txCmd.payload, payloadLen);
    rememberTxFrame(txHeader, txPayload, payloadLen, millis());

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, txPayload) != HAL_OK) {
        TXfailCount++;
        (void)xQueueSendToFront(shared->canTxQueue, &txCmd, 0);
        return;
    }

    TXframeCount++;
}

// 处理错误详情和触觉分片超时，同时清理长时间未刷新的历史错误状态。
void CanComm::handleReassemblyTimeouts(uint32_t nowMs)
{
    if ((errorState.active != 0U) &&
        ((uint32_t)(nowMs - errorState.firstFrameTimeMs) > CAN_ERROR_REASSEMBLY_TIMEOUT_MS)) {
        resetErrorDetailReassembly();
    }

    if ((tactileState.active != 0U) &&
        ((uint32_t)(nowMs - tactileState.firstFrameTimeMs) > CAN_TAC_REASSEMBLY_TIMEOUT_MS)) {
        resetTactileReassembly();
    }

    if (((uint32_t)(nowMs - lastCompleteErrorBatchTime) > CAN_ERROR_TABLE_CLEAR_TIMEOUT_MS) &&
        (sensor.errorBitmap != 0U)) {
        std::memset(sensor.errorFlags, 0, sizeof(sensor.errorFlags));
        sensor.errorBitmap = 0;
    }
}

// CAN 通信任务主体：初始化 FDCAN，循环处理接收重组、超时清理和发送队列。
void CanComm::run(void *parameter)
{
    shared = static_cast<TaskSharedData_t *>(parameter);

    if (shared == nullptr) {
        vTaskDelete(nullptr);
        return;
    }

    taskHandle = xTaskGetCurrentTaskHandle();
    taskRunning = 1;
    fdcanStarted = 0;
    fdcanStartFailCount = 0;
    RXinterruptCount = 0;
    TXinterruptCount = 0;
    fdcanErrorInterruptCount = 0;
    RXid = 0;
    RXlen = 0;
    RXtimestamp = 0;
    RXframeCount = 0;
    RXacceptedCount = 0;
    RXignoredCount = 0;
    TXid = 0;
    TXlen = 0;
    TXtimestamp = 0;
    TXframeCount = 0;
    TXretryCount = 0;
    TXfailCount = 0;
    std::memset(&RXheader, 0, sizeof(RXheader));
    std::memset(RXdata, 0, sizeof(RXdata));
    std::memset(&TXheader, 0, sizeof(TXheader));
    std::memset(&TXcommand, 0, sizeof(TXcommand));
    std::memset(TXdata, 0, sizeof(TXdata));
    std::memset(&sensor, 0, sizeof(sensor));
    std::memset(&tactile, 0, sizeof(tactile));
    resetErrorDetailReassembly();
    resetTactileReassembly();
    lastCompleteErrorBatchTime = millis();

    while (!setupFdcan()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    for (;;) {
        drainRxFifo();
        handleReassemblyTimeouts(millis());
        processTxQueue();
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(5));
    }
}
