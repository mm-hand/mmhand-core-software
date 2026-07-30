/**
 * @file DirectCommandCoordinator.h
 * @brief 管理直控命令的暂存、校验代际、提升和拒绝。
 *
 * 上位机写入 pending 命令后，状态机先完成在线、多圈和状态门禁，再通过
 * 本模块原子地提升为正式直控命令，防止旧事件覆盖新命令。
 */
#ifndef MMHAND_DIRECT_COMMAND_COORDINATOR_H
#define MMHAND_DIRECT_COMMAND_COORDINATOR_H

#include <stdint.h>

#include "../shared/TaskSharedData.h"

namespace mmhand::system {

void resetCommandState(TaskSharedData_t* sharedData);
bool setCommandModeJoint(TaskSharedData_t* sharedData);
void setCommandModeDirect(TaskSharedData_t* sharedData);
void clearPendingDirectCommand(TaskSharedData_t* sharedData,
                               uint8_t rejectReason);
bool pendingDirectCommandMatches(TaskSharedData_t* sharedData,
                                 uint32_t generation);
uint32_t pendingDirectMotorMask(TaskSharedData_t* sharedData,
                                uint32_t generation);
void rejectPendingDirectCommand(TaskSharedData_t* sharedData,
                                uint32_t generation,
                                uint8_t rejectReason);
bool promotePendingDirectCommand(TaskSharedData_t* sharedData,
                                 uint32_t generation);

} // namespace mmhand::system

#endif // MMHAND_DIRECT_COMMAND_COORDINATOR_H
