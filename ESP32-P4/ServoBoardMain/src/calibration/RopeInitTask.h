#ifndef ROPE_INIT_TASK_H
#define ROPE_INIT_TASK_H

#include <Arduino.h>

#include "../shared/TaskSharedData.h"

// 创建 Rope Init 任务。任务只负责把低于动态 Current Min 的活动关节收回
// READY 工作区，不重新计算或覆盖任何标定结果。
bool startRopeInitTask(TaskSharedData_t* sharedData);
// 返回当前活动关节是否具备执行 Rope Init 的 Manual/Auto 数据。
bool ropeInitActiveScopeReady(TaskSharedData_t* sharedData);
// 返回 ROPE_INIT_REASON_*，供状态机和上位机显示具体拒绝原因。
uint8_t ropeInitActiveScopeRejectReason(TaskSharedData_t* sharedData);

#endif // ROPE_INIT_TASK_H
