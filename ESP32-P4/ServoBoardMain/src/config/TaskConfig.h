/**
 * @file TaskConfig.h
 * @brief 集中定义 FreeRTOS 任务优先级、栈大小和调度资源。
 * @details 调整任务参数前必须结合 TaskRuntimeMonitor 的实测数据。
 */
#ifndef MMHAND_TASK_CONFIG_H
#define MMHAND_TASK_CONFIG_H

#include <stdint.h>

inline constexpr uint8_t TASK_UPPER_COMM_PRIORITY = 2;
inline constexpr uint8_t TASK_STATE_MACHINE_PRIORITY = 5;
inline constexpr uint8_t TASK_CAN_COMM_PRIORITY = 3;
inline constexpr uint8_t TASK_SERVO_COMM_PRIORITY = 3;
inline constexpr uint8_t TASK_CONTROL_PRIORITY = 4;
inline constexpr uint8_t TASK_AUTO_CALIB_PRIORITY = 2;
inline constexpr uint8_t TASK_MONITOR_PRIORITY = 1;
inline constexpr uint8_t TASK_NVS_PERSIST_PRIORITY = 1;

inline constexpr uint32_t UPPER_COMM_TASK_STACK_SIZE = 8192;
inline constexpr uint32_t CAN_COMM_TASK_STACK_SIZE = 4096;
inline constexpr uint32_t STATE_MACHINE_TASK_STACK_SIZE = 4096;
inline constexpr uint32_t SERVO_COMM_TASK_STACK_SIZE = 6144;
inline constexpr uint32_t CONTROL_TASK_STACK_SIZE = 8192;
inline constexpr uint32_t AUTO_CALIB_TASK_STACK_SIZE = 8192;
inline constexpr uint32_t TASK_MONITOR_STACK_SIZE = 4096;
inline constexpr uint32_t NVS_PERSIST_TASK_STACK_SIZE = 4096;

#endif // MMHAND_TASK_CONFIG_H
