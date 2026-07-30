/**
 * @file RecoveryTypes.h
 * @brief 定义多圈 NVS、舵机掉电恢复和 Reset Zero 事务数据。
 */
#ifndef MMHAND_RECOVERY_TYPES_H
#define MMHAND_RECOVERY_TYPES_H

#include "../config/DimensionsConfig.h"
#include <stdint.h>

inline constexpr uint8_t MULTITURN_NVS_SAVE_REASON_RUN_START = 1;
inline constexpr uint8_t MULTITURN_NVS_SAVE_REASON_RUN_DONE = 2;
inline constexpr uint8_t MULTITURN_NVS_SAVE_REASON_HOST_OFFLINE = 3;
inline constexpr uint8_t MULTITURN_NVS_SAVE_REASON_HOST_RESTORE = 4;
inline constexpr uint8_t MULTITURN_NVS_SAVE_REASON_CALIB_DONE = 5;
inline constexpr uint8_t MULTITURN_NVS_SAVE_REASON_OFFLINE_MOVE = 6;
inline constexpr uint8_t MULTITURN_NVS_SAVE_REASON_OFFLINE_TURN_EDGE = 7;
inline constexpr uint8_t MULTITURN_NVS_SAVE_REASON_SERVO_POWER_LOSS = 8;
inline constexpr uint8_t MULTITURN_NVS_SAVE_REASON_SERVO_RESET_ZERO = 9;

inline constexpr uint8_t MULTITURN_NVS_OPERATION_SAVE = 0;
inline constexpr uint8_t MULTITURN_NVS_OPERATION_RESET_ZERO = 1;

inline constexpr uint8_t SERVO_RESET_ZERO_STATUS_IDLE = 0;
inline constexpr uint8_t SERVO_RESET_ZERO_STATUS_RUNNING = 1;
inline constexpr uint8_t SERVO_RESET_ZERO_STATUS_SUCCESS = 2;
inline constexpr uint8_t SERVO_RESET_ZERO_STATUS_FAILED = 3;

struct MultiTurnNvsSaveJob_t {
    uint32_t requestToken;
    uint32_t validMask;
    int32_t motorAbs[SERVO_TOTAL_NUM];
    int16_t motorRaw[SERVO_TOTAL_NUM];
    int16_t motorTurn[SERVO_TOTAL_NUM];
    uint32_t queuedAtMs;
    uint8_t reason;
    uint8_t turnChanged;
    uint8_t edgeChanged;
    // SAVE 与 RESET_ZERO 共用后台队列，以 operation 区分事务语义。
    uint8_t operation;
};

struct MultiTurnNvsSaveResult_t {
    uint32_t requestToken;
    uint32_t validMask;
    uint32_t sequence;
    uint32_t queuedAtMs;
    int32_t motorAbs[SERVO_TOTAL_NUM];
    int16_t motorRaw[SERVO_TOTAL_NUM];
    int16_t motorTurn[SERVO_TOTAL_NUM];
    uint32_t completedAtMs;
    uint8_t reason;
    uint8_t success;
    uint8_t turnChanged;
    uint8_t edgeChanged;
    uint8_t operation;
};

#endif // MMHAND_RECOVERY_TYPES_H
