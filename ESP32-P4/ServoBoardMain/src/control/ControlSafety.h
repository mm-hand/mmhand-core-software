#ifndef CONTROL_SAFETY_H
#define CONTROL_SAFETY_H

#include <Arduino.h>

#include "../shared/TaskSharedData.h"

// Feedback validity required before closed-loop output is allowed.
bool shouldEmergencyStop(bool canBusOnline,
                         const RemoteSensorData_t& sensorData,
                         const MappedAngleData_t& mappedData,
                         uint8_t jointIndex,
                         bool encoderFeedbackFault);

#endif // CONTROL_SAFETY_H
