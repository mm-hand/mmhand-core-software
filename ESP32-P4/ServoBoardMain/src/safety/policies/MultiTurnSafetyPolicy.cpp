/**
 * @file MultiTurnSafetyPolicy.cpp
 * @brief 实现无硬件依赖的多圈恢复与快照策略。
 */
#include "MultiTurnSafetyPolicy.h"

#include "../../config/ServoConfig.h"
#include "../../data/RecoveryTypes.h"
#include "../../data/SystemTypes.h"

namespace p4::recovery {

bool isMultiTurnMotionState(uint8_t systemState)
{
    return systemState == SYSTEM_STATE_RUN_FEEDBACK ||
           systemState == SYSTEM_STATE_RUN_DIRECT ||
           systemState == SYSTEM_STATE_CALIBRATION_RUNNING;
}

uint8_t completedMotionSaveReason(uint8_t systemState)
{
    return systemState == SYSTEM_STATE_CALIBRATION_RUNNING
               ? MULTITURN_NVS_SAVE_REASON_CALIB_DONE
               : MULTITURN_NVS_SAVE_REASON_RUN_DONE;
}

bool isRawNearTurnEdge(int16_t raw)
{
    int32_t normalized = static_cast<int32_t>(raw) % 4096;
    if (normalized < 0) {
        normalized += 4096;
    }
    return normalized <= kMultiTurnEdgeRawMarginCounts ||
           normalized >=
               (4096 - kMultiTurnEdgeRawMarginCounts);
}

} // namespace p4::recovery
