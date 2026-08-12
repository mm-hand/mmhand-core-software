/**
 * @file PersistenceSafetyPolicy.h
 * @brief 把 NVS 写入结果转换为统一 PST 规则，业务层只负责执行和发布结果。
 */
#ifndef P4_SAFETY_POLICIES_PERSISTENCE_SAFETY_POLICY_H
#define P4_SAFETY_POLICIES_PERSISTENCE_SAFETY_POLICY_H

#include "../../data/SafetyRuleTypes.h"

namespace p4::safety {

SafetyRuleId persistenceMultiTurnWriteFailureRule(bool success,
                                                  bool resetZeroOperation);
SafetyRuleId persistenceManualWriteFailureRule(bool success);

/** @brief 多圈 NVS 开关/清空事务失败仍归入多圈持久化失败。 */
SafetyRuleId persistenceControlFailureRule(bool success);

/** @brief 仅在持久化启用时，把启动快照无效映射为诊断规则。 */
SafetyRuleId persistenceSnapshotValidityRule(bool persistenceEnabled,
                                             bool snapshotValid);

} // namespace p4::safety

#endif // P4_SAFETY_POLICIES_PERSISTENCE_SAFETY_POLICY_H
