#include "ControlSafety.h"

#include <math.h>
#include <string.h>

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
