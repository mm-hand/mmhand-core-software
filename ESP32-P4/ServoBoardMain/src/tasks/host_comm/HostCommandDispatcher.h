/**
 * @file HostCommandDispatcher.h
 * @brief 声明已完成结构校验的上位机命令分发入口。
 * @details FrameReader 负责帧边界，本模块负责命令长度、参数、状态门禁和 ACK。
 */
#ifndef P4_TASKS_HOST_COMM_HOST_COMMAND_DISPATCHER_H
#define P4_TASKS_HOST_COMM_HOST_COMMAND_DISPATCHER_H

#include <stddef.h>
#include <stdint.h>

struct SystemRuntimeContext;

namespace p4::host {

/** 分发一帧 CMD+payload；非法命令只返回结构化 ACK，不直接阻塞串口循环。 */
void dispatchHostCommand(SystemRuntimeContext* sharedData,
                              const uint8_t* frame,
                              size_t frameLength);

} // namespace p4::host

#endif // P4_TASKS_HOST_COMM_HOST_COMMAND_DISPATCHER_H
