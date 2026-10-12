/**
 * @file ServiceCyclePolicy.h
 * @brief 声明只依赖纯值快照的周期准入策略，供固件与主机单测复用。
 */
#ifndef P4_MODULES_RUNTIME_SERVICE_CYCLE_POLICY_H
#define P4_MODULES_RUNTIME_SERVICE_CYCLE_POLICY_H

#include "../data/ServiceSnapshots.h"

bool controlSnapshotHasActiveScope(const ControlServiceInputSnapshot& input);
bool servoSnapshotIsConsistent(const ServoServiceInputSnapshot& input);
bool stateSnapshotIsConsistent(const StateServiceInputSnapshot& input);

#endif // P4_MODULES_RUNTIME_SERVICE_CYCLE_POLICY_H
