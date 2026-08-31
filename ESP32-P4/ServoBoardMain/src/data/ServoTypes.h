/**
 * @file ServoTypes.h
 * @brief 定义舵机位置、在线状态及遥测反馈快照。
 * @details 区分 logical ABS 与单圈 raw；跨重启恢复由桌面 JSON 完成。
 */
#ifndef MMHAND_SERVO_TYPES_H
#define MMHAND_SERVO_TYPES_H

#include "../config/DimensionsConfig.h"
#include <stdint.h>

/** 四条 BUS 同一轮同步读的时间和完整性元数据。 */
struct ServoFeedbackCycleMeta_t {
    uint32_t sequence; ///< 每完成一轮四 BUS 事务递增一次。
    uint64_t cycleStartedUs; ///< 第一条 BUS 请求开始时间。
    uint64_t cycleCompletedUs; ///< 全部 BUS 进入终态的时间。
    uint64_t busSampleTimestampUs[NUM_BUSES]; ///< 各BUS完成或超时的收集时刻。
    uint8_t requestedBusMask; ///< 本轮实际发送同步读请求的 BUS。
    uint8_t freshBusMask; ///< 请求内全部舵机均成功返回的 BUS。
    uint8_t timeoutBusMask; ///< 截止时仍有缺失反馈的 BUS。
};

struct ServoAngleData_t {
    // P4 逻辑多圈位置，控制与标定统一使用该坐标。
    int32_t servoAngles[SERVO_TOTAL_NUM];
    // 舵机当前 0..4095 单圈位置。
    int16_t servoRawPositions[SERVO_TOTAL_NUM];
    uint8_t onlineStatus[SERVO_TOTAL_NUM]; ///< 逻辑 Mxx 在线标志。
    uint32_t timestamp; ///< 完整位置快照的发布时间。
    ServoFeedbackCycleMeta_t cycle; ///< 与 raw/遥测共享的同轮采样标识。
};

struct ServoTelemetryData_t {
    int16_t speed[SERVO_TOTAL_NUM]; ///< 飞特 Present Speed 原始有符号值。
    int16_t load[SERVO_TOTAL_NUM];  ///< 飞特 Present Load 原始有符号值。
    // ST3215 Present Current 原始有符号读数，物理单位由上位机解释。
    int16_t current[SERVO_TOTAL_NUM];
    uint8_t voltage[SERVO_TOTAL_NUM];     ///< Present Voltage 原始值。
    uint8_t temperature[SERVO_TOTAL_NUM]; ///< Present Temperature，单位摄氏度。
    uint8_t onlineStatus[SERVO_TOTAL_NUM]; ///< 遥测字段对应舵机在线标志。
    uint32_t timestamp;                    ///< 完整遥测快照发布时间。
    ServoFeedbackCycleMeta_t cycle; ///< 与位置/raw共享的同轮采样标识。
};

#endif // MMHAND_SERVO_TYPES_H
