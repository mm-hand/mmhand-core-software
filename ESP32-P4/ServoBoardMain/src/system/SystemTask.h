#ifndef SYSTEM_TASK_H
#define SYSTEM_TASK_H

#include <stdint.h>

// 标定状态分成两条上行通道：
// 1) g_calibrationStatus 是持续状态，随 SYSTEM_STATUS 周期上报；
// 2) g_calibrationAckStatus 是短时事件，由 UpperCommTask 重发若干次，
//    避免上位机漏掉“标定成功/失败/警告”的瞬时变化。
extern volatile uint8_t g_calibrationStatus;
extern volatile uint8_t g_calibrationAckStatus;
extern volatile uint8_t g_calibrationAckRepeatRemaining;

// Arduino setup() 的系统入口：创建共享资源、恢复 NVS，并启动全部任务。
void System_Init();

// Arduino loop() 的系统入口：这里只让出 CPU，业务逻辑均由 FreeRTOS 任务执行。
void System_Loop();

#endif // SYSTEM_TASK_H
