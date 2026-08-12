/**
 * @file ControlBoundarySafetyPolicy.cpp
 * @brief 实现控制域边界类安全规则的纯判断函数。
 */
#include "ControlBoundarySafetyPolicy.h"

#include <math.h>

namespace p4::safety {

BoundaryClampDecision clampFloatToBoundary(float requested,
                                           float minimum,
                                           float maximum)
{
    BoundaryClampDecision decision = {false, false, requested, 0.0f};
    if (!isfinite(requested) || !isfinite(minimum) || !isfinite(maximum) ||
        minimum >= maximum) {
        return decision;
    }

    decision.valid = true;
    decision.applied = requested;
    if (requested < minimum) {
        decision.applied = minimum;
        decision.clamped = true;
    } else if (requested > maximum) {
        decision.applied = maximum;
        decision.clamped = true;
    }
    return decision;
}

int32_t clampAbsToBoundary(int32_t requested,
                           int32_t minimum,
                           int32_t maximum,
                           bool* valid,
                           bool* clamped)
{
    if (valid) *valid = minimum <= maximum;
    if (clamped) *clamped = false;
    if (minimum > maximum) return requested;

    if (requested < minimum) {
        if (clamped) *clamped = true;
        return minimum;
    }
    if (requested > maximum) {
        if (clamped) *clamped = true;
        return maximum;
    }
    return requested;
}

bool actualAbsReachedDirectionalBoundary(int32_t actualAbs,
                                         int32_t pushDeltaAbs,
                                         int32_t minimum,
                                         int32_t maximum,
                                         int32_t toleranceCounts)
{
    if (minimum > maximum || toleranceCounts < 0) return true;
    if (pushDeltaAbs > 0) {
        return actualAbs >= maximum - toleranceCounts;
    }
    if (pushDeltaAbs < 0) {
        return actualAbs <= minimum + toleranceCounts;
    }
    return false;
}

WorkspaceWarningDecision evaluateUpperWorkspaceWarning(
    int32_t outsideCounts,
    int32_t warningThresholdCounts,
    uint8_t stableFrames,
    uint8_t previousStableCount,
    bool previouslyActive)
{
    WorkspaceWarningDecision decision = {
        previousStableCount, previouslyActive, false, false};
    if (stableFrames == 0) stableFrames = 1;

    if (outsideCounts > warningThresholdCounts) {
        if (decision.nextStableCount < UINT8_MAX) {
            ++decision.nextStableCount;
        }
        decision.active = decision.nextStableCount >= stableFrames;
        decision.entered = decision.active && !previouslyActive;
        return decision;
    }

    decision.nextStableCount = 0;
    decision.active = false;
    decision.recovered = previouslyActive;
    return decision;
}

bool shouldEnterHotplugHold(bool wasOnline, bool isOnline)
{
    return !wasOnline && isOnline;
}

bool shouldReportHotplugHold(bool baselineValid,
                             bool wasOnline,
                             bool isOnline)
{
    return baselineValid && shouldEnterHotplugHold(wasOnline, isOnline);
}

} // namespace p4::safety
