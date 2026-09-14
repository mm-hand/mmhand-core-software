/**
 * @file CalibrationFeedbackReader.cpp
 * @brief 实现自动标定使用的磁编码器、舵机位置和负载快照读取。
 */
#include "CalibrationFeedbackReader.h"
#include "CalibrationRuntimePort.h"

#include <Arduino.h>

#include "../../modules/calibration/CalibrationMath.h"
#include "../../config/CalibrationConfig.h"
#include "../../config/topology/HardwareMap.h"
#include "../../hal/servo/ServoValue.h"
#include "../../platform/rtos/SharedSafetyState.h"
#include "../../platform/log/FirmwareLog.h"

namespace mmhand::calibration {
namespace {

/** 同时检查磁编在线位和故障位，避免只凭 CAN 队列里残留数据做标定。 */
bool encoderOnlineAndHealthy(SystemRuntimeContext* sharedData, uint32_t bit)
{
    if (!sharedData) {
        return false;
    }
    SharedSafetyGuard guard;
    const uint32_t onlineMask =
        sharedAtomicLoad(&sharedData->encoder_online_bitmap);
    const uint32_t faultMask =
        sharedAtomicLoad(&sharedData->encoder_fault_bitmap);
    return (onlineMask & bit) != 0 && (faultMask & bit) == 0;
}

} // namespace

bool readLatestEncoderSample(SystemRuntimeContext* sharedData,
                             uint8_t jointIndex,
                             uint16_t* outRaw,
                             int32_t* outOriented)
{
    if (!sharedData || !outRaw || !outOriented ||
        jointIndex >= ENCODER_TOTAL_NUM) {
        return false;
    }

    RemoteSensorData_t sensorData;
    const uint32_t nowMs = calibrationNowMs();
    const uint32_t bit = 1UL << jointIndex;
    if (!sharedData->canRxQueue.peek(&sensorData) ||
        !sensorData.isValid ||
        (uint32_t)(nowMs - sensorData.timestamp) >
            mmhand::config::calibration::kEncoderTrustedSampleMaxAgeMs ||
        !encoderOnlineAndHealthy(sharedData, bit)) {
        return false;
    }

    *outRaw = sensorData.encoderValues[jointIndex];
    *outOriented = calibrationOrientEncoderRaw(
        sensorData.encoderValues[jointIndex], g_encoderDirection[jointIndex]);
    return true;
}

bool readLatestEncoderOriented(SystemRuntimeContext* sharedData,
                               uint8_t jointIndex,
                               int32_t* outEncoder)
{
    uint16_t raw = 0;
    return readLatestEncoderSample(
        sharedData, jointIndex, &raw, outEncoder);
}

bool readLatestMotorServoAbs(SystemRuntimeContext* sharedData,
                             uint8_t motorChannel,
                             int32_t* outAbs)
{
    if (!sharedData || !outAbs || !isMotorChannelValid(motorChannel)) {
        return false;
    }

    ServoAngleData_t servoData;
    if (!sharedData->servoFeedbackQueue.peek(&servoData) ||
        servoData.onlineStatus[motorChannel] == 0) {
        return false;
    }
    *outAbs = servoData.servoAngles[motorChannel];
    return true;
}

bool readLatestMotorServoLoad(SystemRuntimeContext* sharedData,
                              uint8_t motorChannel,
                              int16_t* outLoad)
{
    if (!sharedData || !outLoad || !isMotorChannelValid(motorChannel)) {
        return false;
    }

    ServoTelemetryData_t telemetry;
    const bool telemetryOk =
        sharedData->servoTelemetryQueue.peek(&telemetry);
    if (!telemetryOk ||
        telemetry.onlineStatus[motorChannel] == 0) {
        return false;
    }
    *outLoad = telemetry.load[motorChannel];
    return true;
}

bool readLatestMotorServoRaw(SystemRuntimeContext* sharedData,
                             uint8_t motorChannel,
                             int16_t* outRaw)
{
    if (!sharedData || !outRaw || !isMotorChannelValid(motorChannel)) {
        return false;
    }
    ServoAngleData_t servoData;
    if (!sharedData->servoFeedbackQueue.peek(&servoData) ||
        servoData.onlineStatus[motorChannel] == 0) {
        return false;
    }
    *outRaw = mmhand::servo::normalizeRawSingleTurn(
        servoData.servoRawPositions[motorChannel]);
    return true;
}

void logInitialFeedbackDiagnostic(SystemRuntimeContext* sharedData,
                                  uint8_t jointIndex,
                                  uint8_t motorChannel,
                                  bool encoderOk,
                                  bool absOk,
                                  bool rawOk)
{
    const int channel = isMotorChannelValid(motorChannel)
        ? motorChannel
        : -1;
    const uint32_t nowMs = calibrationNowMs();
    bool sensorQueueOk = false;
    bool sensorValid = false;
    uint8_t sensorError = 0xFF;
    uint32_t sensorAgeMs = 0xFFFFFFFFUL;
    RemoteSensorData_t sensorData;
    if (sharedData && sharedData->canRxQueue.peek(&sensorData)) {
        sensorQueueOk = true;
        sensorValid = sensorData.isValid;
        if (jointIndex < ENCODER_TOTAL_NUM) {
            sensorError = sensorData.errorFlags[jointIndex];
        }
        sensorAgeMs = nowMs - sensorData.timestamp;
    }

    bool servoQueueOk = false;
    uint8_t servoOnline = 0;
    uint32_t servoAgeMs = 0xFFFFFFFFUL;
    int32_t servoAbs = 0;
    int16_t servoRaw = 0;
    ServoAngleData_t servoData;
    if (sharedData && sharedData->servoFeedbackQueue.peek(&servoData)) {
        servoQueueOk = true;
        servoAgeMs = nowMs - servoData.timestamp;
        if (channel >= 0 && channel < SERVO_TOTAL_NUM) {
            servoOnline = servoData.onlineStatus[channel];
            servoAbs = servoData.servoAngles[channel];
            servoRaw = servoData.servoRawPositions[channel];
        }
    }

    FW_LOG_INFO_PRINTF(
        "[AUTO CALIB] INIT_FEEDBACK J%02u M%02u "
        "encoderOk=%u absOk=%u rawOk=%u sensorQ=%u valid=%u "
        "err=0x%02X age=%lu servoQ=%u online=%u age=%lu abs=%ld raw=%d\r\n",
        jointIndex, motorChannel,
        encoderOk ? 1 : 0, absOk ? 1 : 0, rawOk ? 1 : 0,
        sensorQueueOk ? 1 : 0, sensorValid ? 1 : 0, sensorError,
        (unsigned long)sensorAgeMs, servoQueueOk ? 1 : 0, servoOnline,
        (unsigned long)servoAgeMs, (long)servoAbs, (int)servoRaw);
}

} // namespace mmhand::calibration
