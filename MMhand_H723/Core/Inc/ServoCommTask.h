#ifndef SERVO_COMM_TASK_H
#define SERVO_COMM_TASK_H

#include "TaskSharedData.h"

#ifdef __cplusplus

#include "main.h"

#define SERVO_DEBUG_BUS_COUNT NUM_BUSES
#define SERVO_DEBUG_ID_COUNT 9U

struct ServoDebugId {
    volatile uint8_t initialized;
    volatile uint8_t busIndex;
    volatile uint8_t servoId;
    volatile uint8_t online;
    volatile uint8_t lastStatus;
    volatile uint8_t lastError;
    volatile uint8_t send;
    int32_t absolutePosition;
    int16_t rawPosition;
    int16_t speed;
    int16_t load;
    uint8_t voltage;
    uint8_t temperature;
    uint32_t lastUpdate;
    volatile uint32_t readOkCount;
    volatile uint32_t timeoutCount;
    volatile uint32_t writeOkCount;
    volatile int16_t targetPosition;
    volatile uint16_t targetSpeed;
    volatile uint8_t targetAcc;
};

struct ServoDebugBus {
    volatile uint8_t initialized;
    volatile uint8_t busIndex;
    volatile uint8_t onlineCount;
    ServoDebugId ID[SERVO_DEBUG_ID_COUNT + 1U];
};

class ServoComm {
public:
    ServoComm();

    TaskSharedData_t *shared;
    volatile uint8_t taskRunning;
    volatile uint8_t selectedBus;
    volatile uint8_t selectedId;
    volatile uint8_t online;
    volatile uint8_t lastStatus;
    volatile uint8_t lastError;
    volatile uint32_t loopCount;
    volatile uint32_t txFrameCount;
    volatile uint32_t rxFrameCount;
    volatile uint32_t readOkCount;
    volatile uint32_t timeoutCount;
    volatile uint32_t checksumErrorCount;
    volatile uint32_t invalidFrameCount;
    volatile uint32_t writeBatchCount;
    volatile uint32_t emergencyHoldCount;
    volatile uint32_t restoreRequestCount;
    volatile uint32_t lastPollDurationMs;
    int32_t absolutePosition;
    int16_t rawPosition;
    int16_t speed;
    int16_t load;
    uint8_t voltage;
    uint8_t temperature;
    uint8_t lastTxFrame[96];
    uint8_t lastTxLen;
    uint8_t lastRxFrame[128];
    uint8_t lastRxLen;
    uint8_t busOnlineCount[NUM_BUSES];

    void run(void *parameter);

    void recordTx(const uint8_t *data, uint16_t len);
    void recordRx(const uint8_t *data, uint16_t len);
    void recordFeedback(uint8_t busIndex,
                        uint8_t id,
                        int16_t raw,
                        int32_t absolute,
                        int16_t currentSpeed,
                        int16_t currentLoad,
                        uint8_t currentVoltage,
                        uint8_t currentTemperature);
};

extern ServoDebugBus servo1[SERVO_DEBUG_BUS_COUNT + 1U];
extern ServoComm servoComm;

extern "C" {
#endif

void ServoCommTask_Run(void *parameter);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_COMM_TASK_H */
