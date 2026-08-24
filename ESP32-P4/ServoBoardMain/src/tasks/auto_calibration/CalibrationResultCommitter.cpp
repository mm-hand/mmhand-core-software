/**
 * @file CalibrationResultCommitter.cpp
 * @brief 实现标定结果、告警和关节目标的一致性提交。
 */
#include "CalibrationResultCommitter.h"

#include "../../config/topology/HardwareMap.h"
#include "../../platform/rtos/AtomicValue.h"
#include "../../platform/time/Clock.h"
#include "../../runtime/SystemRuntimeContext.h"

namespace {

/** 在命令互斥锁内同步一个关节目标，并推进 token 通知控制任务读取新快照。 */
bool syncJointTarget(SystemRuntimeContext* sharedData,
                     uint8_t jointIndex,
                     float targetDeg)
{
    if (!sharedData || jointIndex >= ENCODER_TOTAL_NUM) {
        return false;
    }
    SemaphoreHandle_t lock = sharedData->commandStateMutex;
    if (lock && xSemaphoreTake(lock, pdMS_TO_TICKS(10)) != pdTRUE) {
        return false;
    }
    sharedData->targetAngles[jointIndex] = targetDeg;
    sharedData->joint_command_accepted_ms =
        p4::platform::time::monotonicMillis();
    p4::platform::rtos::atomicAddFetch(&sharedData->joint_command_token,
                                       (uint32_t)1);
    if (lock) {
        xSemaphoreGive(lock);
    }
    return true;
}

} // namespace

bool commitCalibrationResult(SystemRuntimeContext* sharedData,
                             uint8_t jointIndex,
                             const JointCalibrationResult& result,
                             float finalTargetDeg)
{
    if (!sharedData || jointIndex >= ENCODER_TOTAL_NUM || !result.success) {
        return false;
    }

    // 先在统一临界区发布完整结果，再生成告警和推进目标 token；跨核读者不会看到半份 LUT。
    if (!publishJointCalibrationResult(jointIndex, result)) {
        return false;
    }
    updateCalibrationAssemblyWarning(sharedData, jointIndex, result);
    const bool targetSynced = syncJointTarget(
        sharedData, jointIndex, finalTargetDeg);
    if (targetSynced) {
        // 结果一旦完整提交就立即登记完成位。HostComm 依靠该位发送单关节快照，
        // 不能等整个选中序列退出后才登记，否则 UI 会长期保留灰色旧角度。
        p4::platform::rtos::atomicFetchOr(
            &sharedData->calibration_completed_joint_bitmap,
            (uint32_t)(1UL << jointIndex));
    }
    // 标定完成属于新反馈映射事件，即使最终目标未变化也立即刷新 mappedAngleQueue。
    sharedData->motionControlWake.notify(p4::runtime::wake::kMotionTarget);
    return targetSynced;
}
