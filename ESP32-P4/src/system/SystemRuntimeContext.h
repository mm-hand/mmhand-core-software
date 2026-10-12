/**
 * @file SystemRuntimeContext.h
 * @brief 组合 P4 任务共享的运行时资源与各领域状态。
 * @details
 * - 本文件不再声明业务字段，只组合 runtime/state 与任务间通道聚合结构。
 * - 各子结构保持无构造、无虚函数，保证全局静态实例仍可安全零初始化。
 * - 各状态聚合以零额外存储的平坦基类组合，便于实时路径直接访问字段；这里不表达面向对象继承关系。
 */
#ifndef P4_RUNTIME_SYSTEM_RUNTIME_CONTEXT_H
#define P4_RUNTIME_SYSTEM_RUNTIME_CONTEXT_H

#include "../data/CalibrationRuntimeState.h"
#include "../data/CommandRuntimeState.h"
#include "../data/ControlRuntimeState.h"
#include "channels/InterTaskChannels.h"
#include "../data/ServoRuntimeState.h"

struct SystemRuntimeContext :
    InterTaskChannels,
    CommandRuntimeState,
    ControlRuntimeState,
    ServoRuntimeState,
    CalibrationRuntimeState {
};

#endif // P4_RUNTIME_SYSTEM_RUNTIME_CONTEXT_H
