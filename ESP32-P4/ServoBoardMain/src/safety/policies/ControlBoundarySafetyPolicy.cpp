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

AbsBoundaryClampDecision clampAbsToBoundary(int32_t requested,
                                            int32_t minimum,
                                            int32_t maximum)
{
    AbsBoundaryClampDecision decision = {
        minimum <= maximum, false, requested, requested};
    if (!decision.valid) {
        return decision;
    }
    if (requested < minimum) {
        decision.applied = minimum;
        decision.clamped = true;
    } else if (requested > maximum) {
        decision.applied = maximum;
        decision.clamped = true;
    }
    return decision;
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
