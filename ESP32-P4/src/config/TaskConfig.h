/**
 * @file TaskConfig.h
 * @brief 集中定义 FreeRTOS 任务优先级、栈大小和调度资源。
 * @details 调整任务参数前必须结合 TaskRuntimeMonitor 的实测数据。
 */
#ifndef MMHAND_TASK_CONFIG_H
#define MMHAND_TASK_CONFIG_H

#include <stdint.h>

inline constexpr uint8_t TASK_HOST_COMM_PRIORITY = 2;    ///< 上位机协议收发优先级。
inline constexpr uint8_t TASK_SYSTEM_STATE_PRIORITY = 5; ///< 顶层安全状态机最高业务优先级。
inline constexpr uint8_t TASK_S3_CAN_RX_PRIORITY = 3;      ///< 磁编/触觉 CAN 接收优先级。
inline constexpr uint8_t TASK_SERVO_BUS_PRIORITY = 3;    ///< 舵机总线反馈与发送优先级。
inline constexpr uint8_t TASK_MOTION_CONTROL_PRIORITY = 4;       ///< 10 ms 闭环控制优先级。
inline constexpr uint8_t TASK_MONITOR_PRIORITY = 1;       ///< 运行时统计任务优先级。

inline constexpr uint32_t HOST_COMM_SERVICE_PERIOD_MS = 5; ///< 上位机收发周期兜底；RX 事件可提前唤醒。
inline constexpr uint32_t HOST_TELEMETRY_PERIOD_MS = 10; ///< 常规遥测最多 100 Hz，样本时间戳仍来自采集层。

inline constexpr uint32_t HOST_COMM_TASK_STACK_SIZE = 8192;    ///< FreeRTOS 栈深度，单位 word。
inline constexpr uint32_t S3_CAN_RX_TASK_STACK_SIZE = 4096;      ///< FreeRTOS 栈深度，单位 word。
inline constexpr uint32_t SYSTEM_STATE_TASK_STACK_SIZE = 4096; ///< FreeRTOS 栈深度，单位 word。
inline constexpr uint32_t SERVO_BUS_TASK_STACK_SIZE = 6144;    ///< FreeRTOS 栈深度，单位 word。
inline constexpr uint32_t MOTION_CONTROL_TASK_STACK_SIZE = 8192;       ///< FreeRTOS 栈深度，单位 word。
inline constexpr uint32_t TASK_MONITOR_STACK_SIZE = 4096;       ///< FreeRTOS 栈深度，单位 word。

#endif // MMHAND_TASK_CONFIG_H
