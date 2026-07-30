/**
 * @file SystemStatePolicy.cpp
 * @brief 实现顶层状态机的纯状态准入表。
 */
#include "SystemStatePolicy.h"

#include "../domain/SystemTypes.h"

namespace p4::system {

bool canAcceptStart(uint8_t state)
{
    return state == SYSTEM_STATE_UNCONTROLLABLE ||
           state == SYSTEM_STATE_IDLE ||
           state == SYSTEM_STATE_READY ||
           state == SYSTEM_STATE_RECOVERY_REQUIRED ||
           state == SYSTEM_STATE_PAUSED;
}

bool canAcceptRunDirect(uint8_t state)
{
    return state == SYSTEM_STATE_IDLE ||
           state == SYSTEM_STATE_READY ||
           state == SYSTEM_STATE_RECOVERY_REQUIRED ||
           state == SYSTEM_STATE_RUN_DIRECT;
}

bool canAcceptRunFeedback(uint8_t state)
{
    // 普通反馈只能从 READY 进入；RECOVERY_REQUIRED 必须先走 Rope Init。
    return state == SYSTEM_STATE_READY;
}

bool canAcceptCalibration(uint8_t state)
{
    // 首次标定尚无 Auto 数据时可能处于 UNCONTROLLABLE，因此允许从该状态启动。
    return state == SYSTEM_STATE_UNCONTROLLABLE ||
           state == SYSTEM_STATE_IDLE ||
           state == SYSTEM_STATE_READY ||
           state == SYSTEM_STATE_RECOVERY_REQUIRED;
}

} // namespace p4::system
