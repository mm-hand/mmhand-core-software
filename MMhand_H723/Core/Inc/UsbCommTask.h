#ifndef USB_COMM_TASK_H
#define USB_COMM_TASK_H

#include "TaskSharedData.h"

#ifdef __cplusplus
extern "C" {
#endif

void usbCommunicationTask(void *parameter);
void UsbCommTask_Run(void *parameter);
void UsbComm_OnCdcInit(void);
void UsbComm_OnCdcDeInit(void);
void UsbComm_OnCdcControl(uint8_t cmd, uint8_t *pbuf, uint16_t length);
void UsbComm_FillLineCoding(uint8_t *pbuf, uint16_t length);
void UsbComm_OnCdcRx(uint8_t *data, uint32_t length);
void UsbComm_OnCdcTxComplete(uint8_t *data, uint32_t *length, uint8_t epnum);
uint8_t UsbComm_Send(const uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class UsbComm {
public:
    TaskSharedData_t *shared;
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

    void run(void *parameter);
    void onInit(void);
    void onDeInit(void);
    void onControl(uint8_t cmd, uint8_t *pbuf, uint16_t length);
    void fillLineCoding(uint8_t *pbuf, uint16_t length) const;
    void onRx(uint8_t *data, uint32_t length);
    void onTxComplete(uint8_t *data, uint32_t *length, uint8_t epnum);
    uint8_t send(const uint8_t *data, uint16_t length);
    void processEcho(void);
    void processTxQueue(void);

private:
    uint8_t echoData[USB_COMM_MAX_DATA_SIZE];
    uint16_t echoLen;

    uint32_t millis(void) const;
    uint16_t clampLen(uint32_t length) const;
    uint8_t isInInterrupt(void) const;
};

extern UsbComm usb1;

#endif

#endif /* USB_COMM_TASK_H */
