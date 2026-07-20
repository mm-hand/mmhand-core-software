#include "ControlSafety.h"

bool shouldEmergencyStop(bool canBusOnline,
                         const RemoteSensorData_t& sensorData,
                         const MappedAngleData_t& mappedData,
                         uint8_t jointIndex,
                         bool encoderFeedbackFault)
{
    // 只有 CAN 反馈新鲜且目标关节编码器已成功映射时，才允许闭环关节控制。
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
