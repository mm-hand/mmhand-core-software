/**
 * @file StateAdmissionPolicy.cpp
 * @brief 实现顶层状态机的纯状态准入表。
 */
#include "StateAdmissionPolicy.h"

#include "../../data/SystemTypes.h"

namespace p4::system {

bool canAcceptStart(uint8_t state)
{
    return state == SYSTEM_STATE_UNCONTROLLABLE ||
           state == SYSTEM_STATE_IDLE ||
           state == SYSTEM_STATE_READY ||
           state == SYSTEM_STATE_JOINT_BEYOND_WORKSPACE;
}

bool canAcceptRunDirect(uint8_t state)
{
    return state == SYSTEM_STATE_IDLE ||
           state == SYSTEM_STATE_READY ||
           state == SYSTEM_STATE_JOINT_BEYOND_WORKSPACE ||
           state == SYSTEM_STATE_RUN_DIRECT;
}

bool canAcceptRunFeedback(uint8_t state)
{
    // 普通反馈只能从 READY 进入；JOINT_BEYOND_WORKSPACE 必须先走 Pull Joint into Workspace。
    return state == SYSTEM_STATE_READY;
}

} // namespace p4::system
