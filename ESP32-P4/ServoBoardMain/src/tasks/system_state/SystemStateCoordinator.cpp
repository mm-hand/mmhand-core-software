/**
 * @file SystemStateCoordinator.cpp
 * @brief 运行 P4 总状态编排器，处理事件、READY 门禁、故障和运动 owner 切换。
 */
#include "SystemStateCoordinator.h"
#include "../../runtime/ports/SystemEventPort.h"
#include "DirectCommandCoordinator.h"
#include "FeedbackReadyDiagnostics.h"
#include "HostLeaseSupervisor.h"
#include "../../safety/policies/SystemReadinessPolicy.h"
#include "../../safety/policies/SystemFaultPolicy.h"
#include "RopeInitController.h"
#include "../../modules/runtime/ServiceCyclePolicy.h"
#include "../../safety/policies/HostLeaseSafetyPolicy.h"
#include "../../safety/policies/StateAdmissionPolicy.h"
#include "../../safety/SafetyRuleRegistry.h"

#include <math.h>
#include <string.h>

#include "../../config/SafetyConfig.h"
#include "../../generated/UrdfJointLimits.generated.h"
#include "../../config/topology/HardwareMap.h"
#include "../../platform/rtos/AtomicValue.h"
#include "../../platform/rtos/SharedSafetyState.h"
#include "../../platform/log/FirmwareLog.h"
#include "../../runtime/metrics/TaskRuntimeMetrics.h"

#include "SystemEventDispatcher.h"


bool initSystemStateCoordinator(SystemStateCoordinatorContext* context,
                                SystemRuntimeContext* sharedData)
{
    if (!context || !sharedData) {
        return false;
    }
    memset(context, 0, sizeof(*context));
    setSystemState(sharedData, SYSTEM_STATE_UNCONTROLLABLE);
    context->initialized = true;
    return true;
}

/**
 * 推进一个顶层状态周期。事件顺序保持 E-STOP、普通事件、Rope Init、租约、
 * 故障汇总和 READY 重评估，避免重构改变原有状态迁移优先级。
 */
bool stepSystemStateCoordinator(SystemStateCoordinatorContext* context,
                                SystemRuntimeContext* sharedData,
                                const StateServiceInputSnapshot& input,
                                const ServiceOutputPort& output)
{
    (void)output;
    if (!context || !context->initialized || !sharedData) {
        return false;
    }
    if (!stateSnapshotIsConsistent(input)) {
        return false;
    }
    const uint32_t nowMs = input.nowMs;

        // E-STOP token 比普通队列优先，且只处理每个新 token 一次。
        const uint32_t emergencyStopToken =
            input.emergencyStopToken;
        if (emergencyStopToken != context->handledEmergencyStopToken) {
            context->handledEmergencyStopToken = emergencyStopToken;
            SystemEvent_t stopEvent{};
            stopEvent.event = SYSTEM_EVENT_STOP;
            stopEvent.timestamp = nowMs;
            stopEvent.token = emergencyStopToken;
            handleSystemEvent(sharedData, stopEvent);
        }

        SystemEvent_t event;
        // 单周期清空当前事件队列，保持状态切换延迟低于一个 10 ms 周期。
        while (sharedData->stateEventQueue.receive(&event)) {
            handleSystemEvent(sharedData, event);
        }

        // Rope Init 不再占用独立任务；状态机每 10 ms 调用一次，控制器内部
        // 保持原 50 ms 运动节拍，并通过统一 actuator mailbox 发送目标。
        p4::recovery::stepRopeInitController(sharedData, nowMs);

        // 所有舵机运动态都必须由桌面端公共心跳续租；ROS2 不另设消息超时。
        // 租约服务只推进 Hold 事务；状态迁移和持久化仍由总状态机端口统一执行。
        const p4::state::HostLeaseSupervisorPort hostLeasePort = {
            idleOrReadyState,
            setSystemState,
        };
        p4::state::updateHostControlLease(sharedData, hostLeasePort);

        const uint32_t faults = p4::safety::collectSystemFaults(sharedData);
        sharedAtomicStore(&sharedData->system_fault_bitmap, faults);

        const uint32_t blockingFaults =
            p4::safety::blockingFaultsForState(sharedData,
                                              sharedData->system_state,
                                              faults);
        if (stateOwnsMotionResources(sharedData) && blockingFaults != 0) {
            enterFaultHold(sharedData, blockingFaults);
        } else if (!stateOwnsMotionResources(sharedData)) {
            // 空闲态不锁死 READY；新反馈、active scope 或标定变化会自动更新状态。
            updateReadyStateIfIdle(sharedData);
        }

    return true;
}
