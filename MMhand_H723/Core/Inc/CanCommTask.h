#ifndef CAN_COMM_TASK_H
#define CAN_COMM_TASK_H

#include "TaskSharedData.h"

#define CAN_ID_ENC_BASE 0x100
#define CAN_ID_ENC_LAST (CAN_ID_ENC_BASE + (ENCODER_TOTAL_NUM + 3) / 4 - 1)

#define CAN_ID_ERROR_DETAIL_BASE 0x1F0
#define CAN_ID_ERROR_DETAIL_LAST 0x1F2
#define CAN_ERROR_DETAIL_FRAME_COUNT 3
#define CAN_ERROR_DETAIL_CODES_PER_FRAME 7
#define CAN_ERROR_DETAIL_DLC 8
#define CAN_ERROR_REASSEMBLY_TIMEOUT_MS 100
#define CAN_ERROR_TABLE_CLEAR_TIMEOUT_MS 300

#define CAN_ID_TAC_SUMMARY_BASE 0x240
#define CAN_ID_TAC_SUMMARY_LAST 0x246
#define CAN_TAC_SUMMARY_FRAME_COUNT 7
#define CAN_TAC_SUMMARY_PAYLOAD_PER_FRAME 7
#define CAN_TAC_SUMMARY_DLC 8
#define CAN_TAC_REASSEMBLY_TIMEOUT_MS 80

#ifdef __cplusplus
extern "C" {
#endif

void canCommunicationTask(void *parameter);
void CanCommTask_Run(void *parameter);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include "main.h"
#include "task.h"

typedef struct {
    uint8_t active;
    uint8_t seq;
    uint8_t frameMask;
    uint8_t codes[ENCODER_TOTAL_NUM];
    uint32_t firstFrameTimeMs;
} CanErrorDetailReassemblyState_t;

typedef struct {
    uint8_t active;
    uint8_t seq;
    uint8_t frameMask;
    uint8_t bytes[TACTILE_SUMMARY_BYTES];
    uint32_t firstFrameTimeMs;
} CanTactileReassemblyState_t;

class CanComm {
public:
    TaskSharedData_t *shared;
    TaskHandle_t taskHandle;
    uint8_t taskRunning;

    uint8_t fdcanStarted;
    uint32_t fdcanStartFailCount;
    volatile uint32_t RXinterruptCount;
    volatile uint32_t TXinterruptCount;
    volatile uint32_t fdcanErrorInterruptCount;

    FDCAN_RxHeaderTypeDef RXheader;
    uint32_t RXid;
    uint8_t RXlen;
    uint8_t RXdata[CAN_COMM_MAX_DATA_SIZE];
    uint32_t RXtimestamp;
    uint32_t RXframeCount;
    uint32_t RXacceptedCount;
    uint32_t RXignoredCount;

    FDCAN_TxHeaderTypeDef TXheader;
    RemoteCommand_t TXcommand;
    uint32_t TXid;
    uint8_t TXlen;
    uint8_t TXdata[CAN_COMM_MAX_DATA_SIZE];
    uint32_t TXtimestamp;
    uint32_t TXframeCount;
    uint32_t TXretryCount;
    uint32_t TXfailCount;

    RemoteSensorData_t sensor;
    RemoteTactileData_t tactile;
    CanErrorDetailReassemblyState_t errorState;
    CanTactileReassemblyState_t tactileState;
    uint32_t lastCompleteErrorBatchTime;

    void run(void *parameter);
    bool setupFdcan(void);
    void drainRxFifo(void);
    void processTxQueue(void);
    void handleReassemblyTimeouts(uint32_t nowMs);
    void notifyRxFromIsr(void);
    void notifyTxFromIsr(void);
    void notifyErrorFromIsr(void);

private:
    uint32_t millis(void) const;
    uint8_t fdcanDlcToLen(uint32_t dlc) const;
    uint32_t lenToFdcanDlc(uint8_t len) const;
    void resetErrorDetailReassembly(void);
    void resetTactileReassembly(void);
    uint32_t buildErrorBitmap(const uint8_t *codes) const;
    void rememberRxFrame(const FDCAN_RxHeaderTypeDef &header,
                         const uint8_t *data,
                         uint8_t payloadLen,
                         uint32_t nowMs);
    void rememberTxFrame(const FDCAN_TxHeaderTypeDef &header,
                         const uint8_t *data,
                         uint8_t payloadLen,
                         uint32_t nowMs);
    void handleRxFrame(uint32_t identifier,
                       const uint8_t *data,
                       uint8_t payloadLen,
                       uint32_t nowMs);
    void handleEncoderFrame(uint32_t identifier,
                            const uint8_t *data,
                            uint8_t payloadLen,
                            uint32_t nowMs);
    void handleErrorDetailFrame(uint32_t identifier,
                                const uint8_t *data,
                                uint8_t payloadLen,
                                uint32_t nowMs);
    void handleTactileFrame(uint32_t identifier,
                            const uint8_t *data,
                            uint8_t payloadLen,
                            uint32_t nowMs);
    void notifyTaskFromIsr(volatile uint32_t *counter);
};

extern CanComm can1;

#endif

#endif /* CAN_COMM_TASK_H */
