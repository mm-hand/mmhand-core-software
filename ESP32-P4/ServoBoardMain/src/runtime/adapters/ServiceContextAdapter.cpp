/**
 * @file ServiceContextAdapter.cpp
 * @brief 实现共享上下文到服务快照/输出端口的系统边界适配。
 */
#include "ServiceContextAdapter.h"

#include "../SystemRuntimeContext.h"
#include "../ports/SystemEventPort.h"
#include "../../platform/rtos/AtomicValue.h"

namespace {

bool postEventAdapter(void* context,
                      uint8_t event,
                      uint32_t token,
                      bool tokenValid)
{
    SystemRuntimeContext* sharedData = static_cast<SystemRuntimeContext*>(context);
    return tokenValid ? postSystemEventForToken(sharedData, event, token)
                      : postSystemEvent(sharedData, event);
}

} // namespace

ControlServiceInputSnapshot captureControlServiceInput(
    SystemRuntimeContext* sharedData, uint32_t nowMs)
{
    ControlServiceInputSnapshot out = {};
    out.nowMs = nowMs;
    if (!sharedData) return out;
    out.activeMotorMask =
        p4::platform::rtos::atomicLoad(&sharedData->active_motor_mask);
    out.activeJointMask =
        p4::platform::rtos::atomicLoad(&sharedData->active_joint_mask);
    out.servoOnlineMask =
        p4::platform::rtos::atomicLoad(&sharedData->servo_online_bitmap);
    out.encoderOnlineMask =
        p4::platform::rtos::atomicLoad(&sharedData->encoder_online_bitmap);
    out.encoderFaultMask =
        p4::platform::rtos::atomicLoad(&sharedData->encoder_fault_bitmap);
    out.multiTurnUnknownMask =
        p4::platform::rtos::atomicLoad(&sharedData->multi_turn_unknown_bitmap);
    out.systemState =
        p4::platform::rtos::atomicLoad(&sharedData->system_state);
    out.owner =
        p4::platform::rtos::atomicLoad(&sharedData->servo_target_owner);
    out.controlMode =
        p4::platform::rtos::atomicLoad(&sharedData->control_mode);
    out.controlEnabled =
        p4::platform::rtos::atomicLoad(&sharedData->control_enabled);
    return out;
}

ServoServiceInputSnapshot captureServoServiceInput(
    SystemRuntimeContext* sharedData, uint32_t nowMs)
{
    ServoServiceInputSnapshot out = {};
    out.nowMs = nowMs;
    if (!sharedData) return out;
    out.activeMotorMask =
        p4::platform::rtos::atomicLoad(&sharedData->active_motor_mask);
    out.calibrationAuxMotorMask =
        p4::platform::rtos::atomicLoad(
            &sharedData->calibration_aux_motor_mask);
    out.multiTurnUnknownMask =
        p4::platform::rtos::atomicLoad(
            &sharedData->multi_turn_unknown_bitmap);
    out.emergencyStopToken =
        p4::platform::rtos::atomicLoad(
            &sharedData->servo_emergency_stop_token);
    out.systemState =
        p4::platform::rtos::atomicLoad(&sharedData->system_state);
    out.owner =
        p4::platform::rtos::atomicLoad(&sharedData->servo_target_owner);
    out.controlEnabled =
        p4::platform::rtos::atomicLoad(&sharedData->control_enabled);
    out.emergencyStopLatched =
        p4::platform::rtos::atomicLoad(
            &sharedData->emergency_stop_latched);
    return out;
}

CalibrationServiceInputSnapshot captureCalibrationServiceInput(
    SystemRuntimeContext* sharedData, uint32_t nowMs)
{
    CalibrationServiceInputSnapshot out = {};
    out.nowMs = nowMs;
    if (!sharedData) return out;
    out.activeMotorMask =
        p4::platform::rtos::atomicLoad(&sharedData->active_motor_mask);
    out.activeJointMask =
        p4::platform::rtos::atomicLoad(&sharedData->active_joint_mask);
    out.servoOnlineMask =
        p4::platform::rtos::atomicLoad(&sharedData->servo_online_bitmap);
    out.encoderOnlineMask =
        p4::platform::rtos::atomicLoad(&sharedData->encoder_online_bitmap);
    out.multiTurnUnknownMask =
        p4::platform::rtos::atomicLoad(
            &sharedData->multi_turn_unknown_bitmap);
    return out;
}

StateServiceInputSnapshot captureStateServiceInput(
    SystemRuntimeContext* sharedData, uint32_t nowMs)
{
    StateServiceInputSnapshot out = {};
    out.nowMs = nowMs;
    if (!sharedData) return out;
    out.emergencyStopToken =
        p4::platform::rtos::atomicLoad(
            &sharedData->emergency_stop_request_token);
    out.activeMotorMask =
        p4::platform::rtos::atomicLoad(&sharedData->active_motor_mask);
    out.activeJointMask =
        p4::platform::rtos::atomicLoad(&sharedData->active_joint_mask);
    out.servoOnlineMask =
        p4::platform::rtos::atomicLoad(&sharedData->servo_online_bitmap);
    out.encoderOnlineMask =
        p4::platform::rtos::atomicLoad(&sharedData->encoder_online_bitmap);
    out.multiTurnUnknownMask =
        p4::platform::rtos::atomicLoad(
            &sharedData->multi_turn_unknown_bitmap);
    out.systemState =
        p4::platform::rtos::atomicLoad(&sharedData->system_state);
    out.owner =
        p4::platform::rtos::atomicLoad(&sharedData->servo_target_owner);
    return out;
}

ServiceOutputPort makeSystemServiceOutputPort(SystemRuntimeContext* sharedData)
{
    ServiceOutputPort port = {};
    port.context = sharedData;
    port.postEvent = postEventAdapter;
    return port;
}
