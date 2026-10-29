/**
 * @file PullJointIntoWorkspaceController.h
 * @brief 声明由 SystemStateTask 周期驱动的 Pull Joint into Workspace 非阻塞控制器。
 * @details
 * - 线程：start/step/cancel 只能由 SystemStateTask 调用。
 * - 输入输出：读取反馈快照，通过 ActuatorCommandGateway 发布逻辑 ABS 目标。
 * - 所有权：运行态保存在 SystemRuntimeContext；本模块不创建任务、不等待、不保存静态会话。
 */
#ifndef P4_MODULES_RECOVERY_PULL_JOINT_INTO_WORKSPACE_CONTROLLER_H
#define P4_MODULES_RECOVERY_PULL_JOINT_INTO_WORKSPACE_CONTROLLER_H

#include "../../system/SystemRuntimeContext.h"

namespace p4::recovery {

/** 初始化本轮 Pull Joint into Workspace；调用方应先授予 owner 并切换系统状态。 */
bool startPullJointIntoWorkspaceController(SystemRuntimeContext* sharedData, uint32_t nowMs);

/** 推进至多一个 50 ms 控制步；未到执行时刻时立即返回。 */
void stepPullJointIntoWorkspaceController(SystemRuntimeContext* sharedData, uint32_t nowMs);

/** 外部状态切换时终止运行态；不会重复发送完成事件。 */
void cancelPullJointIntoWorkspaceController(SystemRuntimeContext* sharedData);

uint8_t pullJointIntoWorkspaceActiveScopeRejectReason(SystemRuntimeContext* sharedData);

} // namespace p4::recovery

#endif // P4_MODULES_RECOVERY_PULL_JOINT_INTO_WORKSPACE_CONTROLLER_H
