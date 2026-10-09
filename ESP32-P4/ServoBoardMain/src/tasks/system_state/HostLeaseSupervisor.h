/**
 * @file HostLeaseSupervisor.h
 * @brief 声明上位机控制租约、失联 ABS Hold 和安全收尾服务。
 * @details
 * - 输入：上位机心跳、当前状态、目标 owner 和舵机反馈快照。
 * - 输出：Hold 批次、租约诊断，以及通过端口请求状态迁移。
 * - 状态所有权：本模块拥有 host_control_hold_* 事务字段，不拥有总状态机。
 * - 禁止依赖：不得解析串口协议、创建 RTOS 任务或直接访问持久化介质。
 */
#ifndef P4_MODULES_STATE_HOST_LEASE_SUPERVISOR_H
#define P4_MODULES_STATE_HOST_LEASE_SUPERVISOR_H

#include <stdint.h>

struct SystemRuntimeContext;

namespace p4::state {

/**
 * 租约服务调用总状态机能力的最小端口。
 * 函数指针明确了依赖方向，避免租约模块包含整个 Coordinator 实现。
 */
struct HostLeaseSupervisorPort {
    uint8_t (*resolveIdleOrReady)(SystemRuntimeContext* sharedData);
    void (*transitionTo)(SystemRuntimeContext* sharedData, uint8_t nextState);
};

/** 清理上一次 Hold 的状态；请求 token 保持单调递增。 */
void resetHostControlLeaseRuntime(SystemRuntimeContext* sharedData);

/** 新控制命令进入前检查心跳和未完成 Hold，拒绝复用失联前旧目标。 */
bool prepareHostControlCommandAdmission(SystemRuntimeContext* sharedData);

/** 每个状态机周期推进租约超时、ABS Hold 确认和 owner 释放。 */
void updateHostControlLease(SystemRuntimeContext* sharedData,
                            const HostLeaseSupervisorPort& port);

} // namespace p4::state

#endif // P4_MODULES_STATE_HOST_LEASE_SUPERVISOR_H
