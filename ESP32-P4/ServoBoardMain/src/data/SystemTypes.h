/**
 * @file SystemTypes.h
 * @brief 定义顶层状态机事件、系统状态及运行模式的稳定内部 ABI。
 * @details
 * - 数值必须保持不变，避免状态包和任务间事件语义发生漂移。
 * - 本文件不包含状态转移逻辑，转移规则由 system 层实现。
 */
#ifndef MMHAND_SYSTEM_TYPES_H
#define MMHAND_SYSTEM_TYPES_H

#include "RopeInitTypes.h"
#include <stdint.h>

inline constexpr uint8_t CONTROL_MODE_JOINT = 0;        ///< 磁编关节角反馈控制。
inline constexpr uint8_t CONTROL_MODE_DIRECT_MOTOR = 1; ///< 舵机 logical ABS 直控。

inline constexpr uint8_t MOTOR_DIRECT_SOURCE_NONE = 0;     ///< 没有待执行直控来源。
inline constexpr uint8_t MOTOR_DIRECT_SOURCE_ABSOLUTE = 1; ///< 已解析的 logical ABS 目标。

/** 顶层状态机事件编号是任务间 ABI，缺失编号为已删除但不能复用的历史事件。 */
inline constexpr uint8_t SYSTEM_EVENT_START = 1;
inline constexpr uint8_t SYSTEM_EVENT_STOP = 2;
inline constexpr uint8_t SYSTEM_EVENT_RESET = 3;
inline constexpr uint8_t SYSTEM_EVENT_CALIBRATE = 4;
inline constexpr uint8_t SYSTEM_EVENT_CALIBRATION_DONE = 5;
inline constexpr uint8_t SYSTEM_EVENT_CALIBRATION_FAILED = 6;
inline constexpr uint8_t SYSTEM_EVENT_RUN_DIRECT = 7;
inline constexpr uint8_t SYSTEM_EVENT_RUN_DIRECT_DONE = 8;
inline constexpr uint8_t SYSTEM_EVENT_RUN_FEEDBACK = 9;
inline constexpr uint8_t SYSTEM_EVENT_RUN_FEEDBACK_DONE = 10;
inline constexpr uint8_t SYSTEM_EVENT_PAUSE = 11;
inline constexpr uint8_t SYSTEM_EVENT_REEVALUATE_READY = 14;
inline constexpr uint8_t SYSTEM_EVENT_RUN_FEEDBACK_TIMEOUT = 15;
inline constexpr uint8_t SYSTEM_EVENT_ROPE_INIT = 16;
inline constexpr uint8_t SYSTEM_EVENT_ROPE_INIT_DONE = 17;
inline constexpr uint8_t SYSTEM_EVENT_ROPE_INIT_FAILED = 18;

/** 顶层状态编号同步上报上位机，缺失编号不得被新状态占用。 */
inline constexpr uint8_t SYSTEM_STATE_UNCONTROLLABLE = 0;
inline constexpr uint8_t SYSTEM_STATE_IDLE = 1;
inline constexpr uint8_t SYSTEM_STATE_RUN_FEEDBACK = 2;
inline constexpr uint8_t SYSTEM_STATE_CALIBRATION_RUNNING = 4;
inline constexpr uint8_t SYSTEM_STATE_READY = 5;
inline constexpr uint8_t SYSTEM_STATE_RUN_DIRECT = 6;
inline constexpr uint8_t SYSTEM_STATE_PAUSED = 7;
inline constexpr uint8_t SYSTEM_STATE_ROPE_INIT_RUNNING = 8;
inline constexpr uint8_t SYSTEM_STATE_RECOVERY_REQUIRED = 9;

/** Rope Init 子任务状态及失败原因，用于状态栏和一次性弹窗。 */
/** 直控命令在状态机准入阶段的拒绝原因。 */
inline constexpr uint8_t DIRECT_REJECT_NONE = 0;
inline constexpr uint8_t DIRECT_REJECT_STATE = 1;
inline constexpr uint8_t DIRECT_REJECT_HARDWARE = 2;
inline constexpr uint8_t DIRECT_REJECT_MULTITURN = 3;
inline constexpr uint8_t DIRECT_REJECT_EVENT_QUEUE = 4;
inline constexpr uint8_t DIRECT_REJECT_STALE = 5;
inline constexpr uint8_t DIRECT_REJECT_FAULT = 6;

/** 反馈命令的异步准入状态和拒绝原因。 */
inline constexpr uint8_t FEEDBACK_ADMISSION_IDLE = 0;
inline constexpr uint8_t FEEDBACK_ADMISSION_PENDING = 1;
inline constexpr uint8_t FEEDBACK_ADMISSION_ACCEPTED = 2;
inline constexpr uint8_t FEEDBACK_ADMISSION_REJECTED = 3;

inline constexpr uint8_t FEEDBACK_REJECT_NONE = 0;
inline constexpr uint8_t FEEDBACK_REJECT_MANUAL_REFERENCE = 1;
inline constexpr uint8_t FEEDBACK_REJECT_STATE = 2;
inline constexpr uint8_t FEEDBACK_REJECT_WORKSPACE = 3;
inline constexpr uint8_t FEEDBACK_REJECT_MULTITURN = 4;
inline constexpr uint8_t FEEDBACK_REJECT_ABS_LIMIT = 5;
inline constexpr uint8_t FEEDBACK_REJECT_MODE_SWITCH = 6;
inline constexpr uint8_t FEEDBACK_REJECT_EVENT_QUEUE = 7;

struct SystemEvent_t {
    uint8_t event;       ///< SYSTEM_EVENT_*。
    uint32_t timestamp;  ///< 事件生成的单调毫秒时间。
    // 完成事件携带命令代次，防止旧任务完成事件释放较新的控制命令。
    uint32_t token;
};

#endif // MMHAND_SYSTEM_TYPES_H
