/**
 * @file SystemEventDispatcher.h
 * @brief 声明系统事件解释、状态迁移和运动资源归属判断入口。
 * @details 本模块不创建任务、不读取事件队列；StateCoordinator 负责调度，本模块只执行一条事件。
 */
#ifndef P4_TASKS_SYSTEM_STATE_SYSTEM_EVENT_DISPATCHER_H
#define P4_TASKS_SYSTEM_STATE_SYSTEM_EVENT_DISPATCHER_H

#include "../../data/SystemTypes.h"
#include "../../runtime/SystemRuntimeContext.h"

/** 解释一条系统事件并原子更新状态、owner、控制门禁和诊断结果。 */
void handleSystemEvent(SystemRuntimeContext* sharedData, const SystemEvent_t& event);

/** 写入系统状态并同步状态变更日志。仅供状态协调循环与事件分发器使用。 */
void setSystemState(SystemRuntimeContext* sharedData, uint8_t state);

/** 写入持续标定状态；上位机通过 SYSTEM_STATUS 读取，不再插队发送瞬时 ACK。 */
void setCalibrationStatus(uint8_t status);

/** 撤销运动 owner、保持安全输出并切入不可控状态。 */
void enterFaultHold(SystemRuntimeContext* sharedData, uint32_t faultBits);

/** 根据当前在线、标定和工作空间条件计算空闲侧状态。 */
uint8_t idleOrReadyState(SystemRuntimeContext* sharedData);

/** 空闲侧根据最新反馈重新评估 UNCONTROLLABLE/IDLE/READY。 */
void updateReadyStateIfIdle(SystemRuntimeContext* sharedData);

/** 判断当前状态是否应继续持有舵机运动资源。 */
bool stateOwnsMotionResources(SystemRuntimeContext* sharedData);

#endif // P4_TASKS_SYSTEM_STATE_SYSTEM_EVENT_DISPATCHER_H
