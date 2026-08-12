/**
 * @file PersistenceSafetyPolicy.cpp
 * @brief 实现 NVS 操作结果与 PST 规则 ID 的确定性映射。
 */
#include "PersistenceSafetyPolicy.h"

namespace p4::safety {

SafetyRuleId persistenceMultiTurnWriteFailureRule(bool success,
                                                  bool resetZeroOperation)
{
    if (success) return SafetyRuleId::None;
    return resetZeroOperation
        ? SafetyRuleId::PersistenceResetZeroSaveFailed
        : SafetyRuleId::PersistenceMultiTurnSaveFailed;
}

SafetyRuleId persistenceManualWriteFailureRule(bool success)
{
    return success
        ? SafetyRuleId::None
        : SafetyRuleId::PersistenceManualSaveFailed;
}

SafetyRuleId persistenceControlFailureRule(bool success)
{
    return success
        ? SafetyRuleId::None
        : SafetyRuleId::PersistenceMultiTurnSaveFailed;
}

SafetyRuleId persistenceSnapshotValidityRule(bool persistenceEnabled,
                                             bool snapshotValid)
{
    return persistenceEnabled && !snapshotValid
        ? SafetyRuleId::PersistenceSnapshotInvalid
        : SafetyRuleId::None;
}

} // namespace p4::safety
