/**
 * @file RecoveryTypes.h
 * @brief 定义多圈 NVS、舵机掉电恢复和 Reset Zero 事务数据。
 */
#ifndef MMHAND_RECOVERY_TYPES_H
#define MMHAND_RECOVERY_TYPES_H

#include "../config/DimensionsConfig.h"
#include <stdint.h>

inline constexpr uint8_t MULTITURN_NVS_SAVE_REASON_RUN_START = 1; ///< 运动前保护性快照。
inline constexpr uint8_t MULTITURN_NVS_SAVE_REASON_RUN_DONE = 2; ///< 直控/反馈动作完成。
inline constexpr uint8_t MULTITURN_NVS_SAVE_REASON_HOST_OFFLINE = 3; ///< 上位机心跳超时。
inline constexpr uint8_t MULTITURN_NVS_SAVE_REASON_HOST_RESTORE = 4; ///< 上位机重新上线。
inline constexpr uint8_t MULTITURN_NVS_SAVE_REASON_CALIB_DONE = 5; ///< 自动标定动作完成。
inline constexpr uint8_t MULTITURN_NVS_SAVE_REASON_OFFLINE_MOVE = 6; ///< 无控制期间检测到位置变化。
inline constexpr uint8_t MULTITURN_NVS_SAVE_REASON_OFFLINE_TURN_EDGE = 7; ///< raw 接近跨圈边界。
inline constexpr uint8_t MULTITURN_NVS_SAVE_REASON_SERVO_POWER_LOSS = 8; ///< 单舵机离线前最后快照。
inline constexpr uint8_t MULTITURN_NVS_SAVE_REASON_SERVO_RESET_ZERO = 9; ///< 舵机内部圈数归零。

inline constexpr uint8_t MULTITURN_NVS_OPERATION_SAVE = 0; ///< 普通 A/B 快照写入。
inline constexpr uint8_t MULTITURN_NVS_OPERATION_RESET_ZERO = 1; ///< 清除指定舵机持久化圈数。

inline constexpr uint8_t SERVO_RESET_ZERO_STATUS_IDLE = 0;    ///< 没有 Reset Zero 事务。
inline constexpr uint8_t SERVO_RESET_ZERO_STATUS_RUNNING = 1; ///< 舵机与 NVS 正在归零。
inline constexpr uint8_t SERVO_RESET_ZERO_STATUS_SUCCESS = 2; ///< 最近一次事务成功。
inline constexpr uint8_t SERVO_RESET_ZERO_STATUS_FAILED = 3;  ///< 最近一次事务失败。

struct MultiTurnNvsSaveJob_t {
    uint32_t requestToken; ///< 单调事务号，用于匹配异步完成结果。
    uint32_t validMask;    ///< 本事务中具有有效快照的 Mxx 位图。
    int32_t motorAbs[SERVO_TOTAL_NUM]; ///< P4 logical ABS 快照。
    int16_t motorRaw[SERVO_TOTAL_NUM]; ///< hardware raw 快照。
    int16_t motorTurn[SERVO_TOTAL_NUM]; ///< logical ABS 拆出的圈数诊断值。
    uint32_t queuedAtMs;   ///< 进入后台队列的单调毫秒时间。
    uint8_t reason;        ///< MULTITURN_NVS_SAVE_REASON_*。
    uint8_t turnChanged;   ///< 相比上次落盘是否发生整圈变化。
    uint8_t edgeChanged;   ///< 是否因跨圈边界触发。
    // SAVE 与 RESET_ZERO 共用后台队列，以 operation 区分事务语义。
    uint8_t operation;
};

struct MultiTurnNvsSaveResult_t {
    uint32_t requestToken; ///< 原样回传请求 token。
    uint32_t validMask;    ///< 实际成功写入的舵机位图。
    uint32_t sequence;     ///< NVS A/B 快照递增序号。
    uint32_t queuedAtMs;   ///< 请求入队时间。
    int32_t motorAbs[SERVO_TOTAL_NUM]; ///< 已写入的 logical ABS。
    int16_t motorRaw[SERVO_TOTAL_NUM]; ///< 已写入的 hardware raw。
    int16_t motorTurn[SERVO_TOTAL_NUM]; ///< 已写入的圈数诊断值。
    uint32_t completedAtMs; ///< Flash 操作完成时间。
    uint8_t reason;         ///< 保存触发原因。
    uint8_t success;        ///< 1 表示整个事务成功。
    uint8_t turnChanged;    ///< 原样回传请求诊断。
    uint8_t edgeChanged;    ///< 原样回传请求诊断。
    uint8_t operation;      ///< MULTITURN_NVS_OPERATION_*。
};

#endif // MMHAND_RECOVERY_TYPES_H
