/**
 * @file ControlInputSafetyPolicy.cpp
 * @brief 实现 CTL-2001~CTL-2004 的反馈安全裁决。
 */
#include "ControlInputSafetyPolicy.h"

namespace p4::safety {

namespace {

ControlSafetyDecision allowControl()
{
    return {true, SafetyRuleId::None, 1, 1};
}

ControlSafetyDecision rejectControl(SafetyRuleId ruleId,
                                    int32_t requestedValue,
                                    int32_t observedValue)
{
    return {false, ruleId, requestedValue, observedValue};
}

} // namespace

ControlSafetyDecision evaluateControlSafety(
    bool canBusOnline,
    const RemoteSensorData_t& sensorData,
    const MappedAngleData_t& mappedData,
    uint8_t jointIndex,
    bool encoderFeedbackFault)
{
    // CTL-2001：总线离线和整批磁编快照无效具有相同后果，均不得把旧角度送入 PID。
    if (!canBusOnline || !sensorData.isValid) {
        const int32_t observed =
            (canBusOnline ? 0x01 : 0x00) |
            (sensorData.isValid ? 0x02 : 0x00);
        return rejectControl(SafetyRuleId::ControlCanFeedbackInvalid,
                             0x03,
                             observed);
    }

    // CTL-2002 必须先于任何数组访问执行，防止错误拓扑把 jointIndex 变成越界读写。
    if (jointIndex >= ENCODER_TOTAL_NUM) {
        return rejectControl(SafetyRuleId::ControlJointIndexInvalid,
                             ENCODER_TOTAL_NUM - 1,
                             jointIndex);
    }

    // CTL-2003：单路磁编已经被健康监视器判为故障时，仅保持该关节，不拖停其他关节。
    if (encoderFeedbackFault) {
        return rejectControl(SafetyRuleId::ControlEncoderFault, 0, 1);
    }

    // CTL-2004：Manual/Auto 映射失败或输入超出可解释范围时，不允许使用哨兵角继续闭环。
    if (mappedData.validFlags[jointIndex] == 0) {
        return rejectControl(SafetyRuleId::ControlMappedAngleInvalid,
                             1,
                             mappedData.validFlags[jointIndex]);
    }

    return allowControl();
}

} // namespace p4::safety
