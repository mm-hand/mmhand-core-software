#ifndef USB_COMM_TASK_H
#define USB_COMM_TASK_H

#include "TaskSharedData.h"
#include "task.h"

#define USB_COMM_STREAM_CHUNK_SIZE 2048U
#define USB_COMM_TX_BENCHMARK_ACK_DELAY_MS 100U

#ifdef __cplusplus
extern "C" {
#endif

typedef uint16_t (*UsbCommStreamProvider_t)(uint8_t *buffer, uint16_t maxLen);

void usbCommunicationTask(void *parameter);
void UsbCommTask_Run(void *parameter);
void UsbComm_OnCdcInit(void);
void UsbComm_OnCdcDeInit(void);
void UsbComm_OnCdcControl(uint8_t cmd, uint8_t *pbuf, uint16_t length);
void UsbComm_FillLineCoding(uint8_t *pbuf, uint16_t length);
void UsbComm_OnCdcRx(uint8_t *data, uint32_t length);
void UsbComm_OnCdcTxComplete(uint8_t *data, uint32_t *length, uint8_t epnum);
uint8_t UsbComm_Send(const uint8_t *data, uint16_t length);
uint8_t UsbComm_EnqueueTx(const uint8_t *data, uint16_t length);
void UsbComm_SetStreamProvider(UsbCommStreamProvider_t provider);
uint8_t UsbComm_StartTxBenchmark(uint32_t durationMs);
uint8_t UsbComm_IsTxBenchmarkRunning(void);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class UsbComm {
public:
    TaskSharedData_t *shared;
    TaskHandle_t taskHandle;
    uint8_t taskRunning;

    uint8_t cdcInitialized;
    uint8_t hostConnected;
    uint8_t echoEnabled;
    volatile uint8_t txBusy;
    volatile uint8_t echoPending;

    uint8_t RXdata[USB_COMM_MAX_DATA_SIZE];
    uint16_t RXlen;
    uint32_t RXtimestamp;
    uint32_t RXpacketCount;
    uint32_t RXbyteCount;
    uint32_t RXtruncatedCount;
    uint32_t RXqueueDropCount;

    uint8_t TXdata[USB_COMM_MAX_DATA_SIZE];
    uint16_t TXlen;
    uint32_t TXtimestamp;
    uint32_t TXpacketCount;
    uint32_t TXbyteCount;
    uint32_t TXcompleteCount;
    uint32_t TXbusyCount;
    uint32_t TXfailCount;
    uint32_t TXqueueFullCount;
    uint32_t TXqueueDropCount;

    uint8_t lastControlCmd;
    uint16_t lastControlLength;
    uint32_t controlCount;
    uint32_t lineDTERate;
    uint8_t lineCharFormat;
    uint8_t lineParityType;
    uint8_t lineDataBits;
    uint16_t controlLineState;

    uint32_t initCount;
    uint32_t deinitCount;

    volatile uint8_t benchmarkPending;
    volatile uint8_t benchmarkActive;
    uint32_t benchmarkStartAtMs;
    uint32_t benchmarkStartMs;
    uint32_t benchmarkStopMs;
    uint32_t benchmarkDurationMs;
    uint32_t benchmarkPayloadBytes;
    uint32_t benchmarkPacketCount;
    uint32_t benchmarkSeq;

    void run(void *parameter);
    void onInit(void);
    void onDeInit(void);
    void onControl(uint8_t cmd, uint8_t *pbuf, uint16_t length);
    void fillLineCoding(uint8_t *pbuf, uint16_t length) const;
    void onRx(uint8_t *data, uint32_t length);
    void onTxComplete(uint8_t *data, uint32_t *length, uint8_t epnum);
    uint8_t send(const uint8_t *data, uint16_t length);
    uint8_t enqueueTx(const uint8_t *data, uint16_t length);
    void setStreamProvider(UsbCommStreamProvider_t provider);
    uint8_t startTxBenchmark(uint32_t durationMs);
    uint8_t isTxBenchmarkRunning(void) const;
    void processEcho(void);
    void processTxQueue(void);
    void processStreamProvider(void);

private:
    uint8_t echoData[USB_COMM_MAX_DATA_SIZE];
    uint16_t echoLen;
    uint8_t streamData[USB_COMM_STREAM_CHUNK_SIZE];
    UsbCommStreamProvider_t streamProvider;

    uint32_t millis(void) const;
    uint16_t clampLen(uint32_t length) const;
    uint8_t isInInterrupt(void) const;
    void notifyTask(void);
    void updateBenchmarkState(void);
    uint8_t sendDirect(uint8_t *data, uint16_t length);
    uint16_t fillBenchmarkData(uint8_t *buffer, uint16_t maxLen);
};

extern UsbComm usb1;

#endif

#endif /* USB_COMM_TASK_H */
