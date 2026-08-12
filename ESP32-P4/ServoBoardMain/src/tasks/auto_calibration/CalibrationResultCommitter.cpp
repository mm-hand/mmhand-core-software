/**
 * @file CalibrationResultCommitter.cpp
 * @brief 实现标定结果、告警和关节目标的一致性提交。
 */
#include "CalibrationResultCommitter.h"

#include "../../config/topology/HardwareMap.h"
#include "../../platform/rtos/AtomicValue.h"
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

    // 先写完整 POD 结果，再生成告警，最后推进目标 token；读者不会看到未完成的 LUT。
    g_jointCalibResult[jointIndex] = result;
    updateCalibrationAssemblyWarning(sharedData, jointIndex, result);
    return syncJointTarget(sharedData, jointIndex, finalTargetDeg);
}
