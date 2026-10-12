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

#include "../../system/SystemRuntimeContext.h"

namespace mmhand::system {

/** 清空关节/直控目标并推进全部 token，使复位前遗留事件失效。 */
void resetCommandState(SystemRuntimeContext* sharedData);
/** 切换到关节角控制模式；互斥锁超时时返回 false。 */
bool setCommandModeJoint(SystemRuntimeContext* sharedData);
/** 切换到舵机直控模式，不改变已经应用的目标数组。 */
void setCommandModeDirect(SystemRuntimeContext* sharedData);
/** 清除任意代际的 pending 直控命令，并保存拒绝原因。 */
void clearPendingDirectCommand(SystemRuntimeContext* sharedData,
                               uint8_t rejectReason);
/** 检查指定 generation 是否仍是当前待审查命令。 */
bool pendingDirectCommandMatches(SystemRuntimeContext* sharedData,
                                 uint32_t generation);
/** 读取指定 generation 的舵机范围；代际已过期时返回 0。 */
uint32_t pendingDirectMotorMask(SystemRuntimeContext* sharedData,
                                uint32_t generation);
/** 仅在 generation 匹配时拒绝 pending，防止旧事件误伤新命令。 */
void rejectPendingDirectCommand(SystemRuntimeContext* sharedData,
                                uint32_t generation,
                                uint8_t rejectReason);
/** 当前代际且无更新 pending 命令时，原子释放普通直控输出权。 */
bool retireDirectCommand(SystemRuntimeContext* sharedData, uint32_t generation);
/** 门禁通过后原子应用 pending 目标、实际输出范围和控制模式。 */
bool promotePendingDirectCommand(SystemRuntimeContext* sharedData,
                                 uint32_t generation);

} // namespace mmhand::system

#endif // MMHAND_DIRECT_COMMAND_COORDINATOR_H
