#ifndef CONTROL_TASK_H
#define CONTROL_TASK_H

#include <Arduino.h>
#include "../shared/TaskSharedData.h"

// ControlTask 是实时控制层入口：
// - 周期读取 ControlCommandSnapshot_t 和底层反馈；
// - 在 joint 模式下执行 PID，在 direct motor 模式下透传/展开舵机目标；
// - 只向 servoTargetQueue 写 batch，不直接访问舵机 UART。
void controlTask(void* parameter);

// Calibration calls this only while feedback control is stopped. The next
// feedback cycle rebuilds release anchors from the newly preloaded pose.
void invalidateDualTendonReleaseAnchors(uint32_t motorMask);

#endif // CONTROL_TASK_H
