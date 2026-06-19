#ifndef UPPER_COMM_TASK_H
#define UPPER_COMM_TASK_H

#include "TaskSharedData.h"
#include "UpperCommProtocol.h"

#ifdef __cplusplus
extern "C" {
#endif

void upperCommunicationTask(void *parameter);
void UpperCommTask_Run(void *parameter);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

class UpperComm {
public:
    TaskSharedData_t *shared;
    uint8_t taskRunning;
    uint8_t sensorStreamMode;

    uint8_t RXbyte;
    uint8_t RXbuffer[UPPER_COMM_RX_BUFFER_SIZE];
    uint16_t RXlen;
    uint32_t RXusbPacketCount;
    uint32_t RXbyteCount;
    uint32_t RXframeCount;
    uint32_t RXlegacyCommandCount;
    uint32_t RXinvalidFrameCount;
    uint32_t RXunsupportedCommandCount;
    uint32_t RXoverflowCount;
    uint8_t lastCommand;
    uint16_t lastCommandPayloadLen;
    uint32_t lastCommandTimestamp;

    uint8_t TXdata[USB_COMM_MAX_DATA_SIZE];
    uint16_t TXlen;
    uint8_t TXpacketType;
    uint32_t TXpacketCount;
    uint32_t TXbyteCount;
    uint32_t TXdropCount;
    uint32_t TXqueueFullCount;
    uint32_t lastTxTimestamp;

    RemoteSensorData_t lastSensorRaw;
    RemoteTactileData_t lastTactile;
    MappedAngleData_t lastMappedAngle;
    ServoAngleData_t lastServoAngle;
    ServoAngleData_t lastServoRaw;
    ServoTelemetryData_t lastTelemetry;
    JointDebugData_t lastJointDebug;

    void run(void *parameter);
    void resetDebugState(void);
    void handleUsbPacket(const UsbPacket_t &packet);
    void parseRxBuffer(void);
    void handleParsedCommand(const uint8_t *frame, uint16_t frameLen);
    void handleLegacySingleByteCommand(uint8_t command);
    void publishPeriodicData(void);
    bool sendPacket(const uint8_t *data, uint16_t len, uint8_t packetType);
    uint32_t millis(void) const;

private:
    uint32_t lastSensorTimestampSent;
    uint32_t lastEncoderRawTimestampSent;
    uint32_t lastFaultStatusSent;
    uint32_t lastFaultStatusSentMs;
    uint8_t faultStatusSentInitialized;
    uint32_t lastReleaseFaultSent;
    uint32_t lastReleaseFaultSentMs;
    uint8_t releaseFaultSentInitialized;
    uint32_t lastSystemStatusTransitionSent;
    uint32_t lastSystemStatusSentMs;
    uint8_t systemStatusSentInitialized;
    uint32_t lastMotorAbsLimitSentMs;
    uint32_t lastMotorAbsLimitValidSent;
    uint32_t lastMotorAbsLimitLockedSent;
};

extern UpperComm upper1;
extern volatile uint8_t g_calibrationUIStatus;

#endif

#endif /* UPPER_COMM_TASK_H */
