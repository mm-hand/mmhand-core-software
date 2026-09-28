/**
 * @file SystemEventPort.h
 * @brief 声明跨层系统事件投递接口，隔离业务模块与状态机任务实现。
 *
 * 线程：任意业务任务可调用，状态机任务是事件的唯一消费者。
 * 输入：共享运行上下文、系统事件及可选命令 token。
 * 输出：事件是否成功入队；急停使用独立锁存 token，不受普通队列满影响。
 * 状态所有权：本接口不直接改变 system_state，最终状态仍由 SystemStateTask 裁决。
 * 禁止依赖：调用方不得因此包含 tasks/SystemStateTask.h。
 */
#ifndef P4_RUNTIME_PORTS_SYSTEM_EVENT_PORT_H
#define P4_RUNTIME_PORTS_SYSTEM_EVENT_PORT_H

#include <Arduino.h>

#include "../SystemRuntimeContext.h"

/** 投递无 token 的普通事件；队列满或参数无效时返回 false。 */
bool postSystemEvent(SystemRuntimeContext* sharedData, uint8_t event);

/** 投递带命令代次的事件，防止旧 DONE/TIMEOUT 释放新命令。 */
bool postSystemEventForToken(SystemRuntimeContext* sharedData,
                             uint8_t event,
                             uint32_t token);

/** 锁存 E-STOP 并通知舵机任务；急停不依赖普通事件队列。 */
bool requestEmergencyStop(SystemRuntimeContext* sharedData);

#endif // P4_RUNTIME_PORTS_SYSTEM_EVENT_PORT_H
