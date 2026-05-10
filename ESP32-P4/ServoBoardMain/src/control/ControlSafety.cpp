#include "ControlSafety.h"

#include <math.h>
#include <string.h>

// 反绕保护参数：
// 目标向释放方向变化时进入观察窗口；如果舵机明显运动但编码器没有按预期回退，
// 或出现反向异常运动，则锁存释放故障并让 ControlTask 对该关节保持当前位置。
static const float kReleaseWindowEnterTargetDeltaDeg = -0.3f;
static const float kReleaseWindowExitTargetDeltaDeg = 0.3f;
static const float kReleaseFilterAlpha = 0.25f;
static const float kReleaseActualDeadbandDeg = 0.2f;
static const int32_t kReleaseAccumPrecheckCounts = 256;
static const int32_t kReleaseAccumEarlyFaultCounts = 512;
static const int32_t kReleaseAccumHalfTurnCounts = 2048;
static const int32_t kReleaseAccumFullTurnCounts = 4096;
static const float kReleaseForwardDeltaDeg = 2.0f;
static const float kReleaseMinReturnAccumDeg = 1.5f;
static const uint8_t kReleaseForwardAnomalyCycles = 3;
static const float kReleaseRecoverPullCmdDeg = 2.5f;
static const float kReleaseRecoverTrackErrDeg = 1.5f;
static const uint8_t kReleaseRecoverStableCycles = 8;
static const int32_t kTendonGuardReleaseMarginCounts = 0;
static const float kTendonGuardReleaseAngleEpsDeg = 0.0f;

bool shouldEmergencyStop(bool canBusOnline,
                         const RemoteSensorData_t& sensorData,
                         const MappedAngleData_t& mappedData,
                         uint8_t jointIndex)
{
    // 闭环控制必须同时具备 CAN 在线、通道无错误、映射值有效三类条件。
    if (!canBusOnline || !sensorData.isValid) {
        return true;
    }
    if (jointIndex >= ENCODER_TOTAL_NUM) {
        return true;
    }
    if (sensorData.errorFlags[jointIndex] != 0) {
        return true;
    }
    if (mappedData.validFlags[jointIndex] == 0) {
        return true;
    }
    return false;
}

static void clearReleaseWindowTracking(ReleaseGuardState* guard)
{
    // 只清除窗口内累计量，不清除已锁存 faultActive。
    if (!guard) {
        return;
    }
    guard->releaseWindowActive = 0;
    guard->releaseServoAccumCounts = 0;
    guard->actualReturnAccumDeg = 0.0f;
    guard->forwardAnomalyCycles = 0;
}

static void resetReleaseGuardState(ReleaseGuardState* guard)
{
    if (guard) {
        memset(guard, 0, sizeof(*guard));
    }
}

void resetAllReleaseGuardStates(ReleaseGuardState guards[ENCODER_TOTAL_NUM])
{
    for (uint8_t i = 0; i < ENCODER_TOTAL_NUM; i++) {
        resetReleaseGuardState(&guards[i]);
    }
}

bool updateReleaseGuardState(ReleaseGuardState* guard,
                             float targetDeg,
                             float actualDeg,
                             int32_t servoPos,
                             bool inputsValid)
{
    if (!guard) {
        return false;
    }
    if (!isfinite(targetDeg)) {
        targetDeg = 0.0f;
    }
    if (!isfinite(actualDeg)) {
        actualDeg = 0.0f;
    }

    if (!guard->initialized) {
        // 首次进入只建立参考点，不立即判定故障。
        guard->initialized = 1;
        guard->filteredActualDeg = actualDeg;
        guard->prevActualDeg = actualDeg;
        guard->prevTargetDeg = targetDeg;
        guard->prevServoPos = servoPos;
        return guard->faultActive != 0;
    }

    const float targetDelta = targetDeg - guard->prevTargetDeg;
    guard->filteredActualDeg =
        kReleaseFilterAlpha * actualDeg + (1.0f - kReleaseFilterAlpha) * guard->filteredActualDeg;
    float actualDelta = guard->filteredActualDeg - guard->prevActualDeg;
    if (fabsf(actualDelta) < kReleaseActualDeadbandDeg) {
        actualDelta = 0.0f;
    }

    int32_t servoDelta = servoPos - guard->prevServoPos;
    if (servoDelta < 0) {
        servoDelta = -servoDelta;
    }

    if (!inputsValid) {
        // 反馈不可用时保持已有故障，不用无效数据更新窗口累计。
        clearReleaseWindowTracking(guard);
        guard->recoverStableCycles = 0;
        guard->initialized = 0;
        guard->prevTargetDeg = targetDeg;
        guard->prevActualDeg = guard->filteredActualDeg;
        guard->prevServoPos = servoPos;
        return guard->faultActive != 0;
    }

    if (targetDelta <= kReleaseWindowEnterTargetDeltaDeg) {
        guard->releaseWindowActive = 1;
    }
    if (targetDelta >= kReleaseWindowExitTargetDeltaDeg) {
        clearReleaseWindowTracking(guard);
    }

    if (guard->releaseWindowActive) {
        // 舵机运动量与编码器实际回退量的关系，是反绕/空转检测的核心。
        if (servoDelta > 0) {
            const int32_t remaining = 0x7FFFFFFF - guard->releaseServoAccumCounts;
            guard->releaseServoAccumCounts += (servoDelta > remaining) ? remaining : servoDelta;
        }
        if (actualDelta < 0.0f) {
            guard->actualReturnAccumDeg += -actualDelta;
        }

        if ((guard->releaseServoAccumCounts >= kReleaseAccumPrecheckCounts) &&
            (actualDelta >= kReleaseForwardDeltaDeg)) {
            if (guard->forwardAnomalyCycles < 255) {
                guard->forwardAnomalyCycles++;
            }
        } else {
            guard->forwardAnomalyCycles = 0;
        }

        if (guard->releaseServoAccumCounts >= kReleaseAccumFullTurnCounts) {
            guard->faultActive = 1;
            guard->catastrophicFault = 1;
        } else if (guard->releaseServoAccumCounts >= kReleaseAccumHalfTurnCounts) {
            guard->faultActive = 1;
        } else if ((guard->releaseServoAccumCounts >= kReleaseAccumEarlyFaultCounts) &&
                   (guard->actualReturnAccumDeg <= kReleaseMinReturnAccumDeg)) {
            guard->faultActive = 1;
        } else if ((guard->releaseServoAccumCounts >= kReleaseAccumPrecheckCounts) &&
                   (guard->forwardAnomalyCycles >= kReleaseForwardAnomalyCycles)) {
            guard->faultActive = 1;
        }
    } else {
        guard->releaseServoAccumCounts = 0;
        guard->actualReturnAccumDeg = 0.0f;
        guard->forwardAnomalyCycles = 0;
    }

    if (guard->faultActive && !guard->catastrophicFault) {
        // 普通释放故障允许通过“向收紧方向拉回”或“稳定跟踪”自动恢复。
        bool clearFault = false;
        if (targetDelta >= kReleaseRecoverPullCmdDeg) {
            clearFault = true;
        } else {
            const float trackErr = fabsf(targetDeg - guard->filteredActualDeg);
            if (trackErr <= kReleaseRecoverTrackErrDeg) {
                if (guard->recoverStableCycles < 255) {
                    guard->recoverStableCycles++;
                }
            } else {
                guard->recoverStableCycles = 0;
            }
            if (guard->recoverStableCycles >= kReleaseRecoverStableCycles) {
                clearFault = true;
            }
        }

        if (clearFault) {
            guard->faultActive = 0;
            guard->releaseWindowActive = 0;
            guard->releaseServoAccumCounts = 0;
            guard->actualReturnAccumDeg = 0.0f;
            guard->forwardAnomalyCycles = 0;
            guard->recoverStableCycles = 0;
        }
    }

    guard->prevTargetDeg = targetDeg;
    guard->prevActualDeg = guard->filteredActualDeg;
    guard->prevServoPos = servoPos;
    return guard->faultActive != 0;
}

uint32_t buildReleaseFaultBitmap(const ReleaseGuardState guards[ENCODER_TOTAL_NUM])
{
    uint32_t bitmap = 0;
    for (uint8_t i = 0; i < ENCODER_TOTAL_NUM; i++) {
        if (i == kJoint16Index) {
            continue;
        }
        if (guards[i].faultActive) {
            bitmap |= (1UL << i);
        }
    }
    return bitmap;
}

bool shouldBlockReleaseByTendonGuard(const ControlCommandSnapshot_t* commandSnapshot,
                                     uint8_t jointIndex,
                                     float targetDeg,
                                     float actualDeg,
                                     int32_t absNow,
                                     bool inputsValid)
{
    // Tendon guard 是上位机配置的单关节释放边界：只在释放命令方向上生效。
    if (!commandSnapshot || jointIndex >= ENCODER_TOTAL_NUM || !inputsValid) {
        return false;
    }
    if (!isfinite(targetDeg) || !isfinite(actualDeg)) {
        return false;
    }
    if (commandSnapshot->tendonGuardEnabled[jointIndex] == 0) {
        return false;
    }

    const bool isReleaseCmd = (targetDeg < (actualDeg - kTendonGuardReleaseAngleEpsDeg));
    if (!isReleaseCmd) {
        return false;
    }

    const int8_t sign = (commandSnapshot->tendonGuardSign[jointIndex] < 0) ? -1 : 1;
    const int32_t x1Abs = (int32_t)commandSnapshot->tendonGuardX1Abs[jointIndex];
    if (sign > 0) {
        return absNow <= (x1Abs + kTendonGuardReleaseMarginCounts);
    }
    return absNow >= (x1Abs - kTendonGuardReleaseMarginCounts);
}
