#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include <Arduino.h>
#include "../shared/TaskSharedData.h"

// ControlTask 是实时控制层入口：
// - 周期读取 ControlCommandSnapshot_t 和底层反馈；
// - 在 joint 模式下执行 PID，在 direct motor 模式下透传/展开舵机目标；
// - 只向 servoTargetQueue 写 batch，不直接访问舵机 UART。
void controlTask(void* parameter);

// 标定流程只会在反馈控制停止时调用此函数。下一次反馈控制周期会基于
// 新预加载的姿态重建释放锚点。motorMask 使用物理电机通道 bit，
// 不是关节索引 bit。
void invalidateDualTendonReleaseAnchors(uint32_t motorMask);

#endif // CONTROL_TASK_H
