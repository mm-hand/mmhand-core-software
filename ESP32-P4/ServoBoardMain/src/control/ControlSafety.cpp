#include "ControlSafety.h"

bool shouldEmergencyStop(bool canBusOnline,
                         const RemoteSensorData_t& sensorData,
                         const MappedAngleData_t& mappedData,
                         uint8_t jointIndex,
                         bool encoderFeedbackFault)
{
    if (!canBusOnline || !sensorData.isValid) {
        return true;
    }
    if (jointIndex >= ENCODER_TOTAL_NUM) {
        return true;
    }
    if (encoderFeedbackFault) {
        return true;
    }
    return mappedData.validFlags[jointIndex] == 0;
}
