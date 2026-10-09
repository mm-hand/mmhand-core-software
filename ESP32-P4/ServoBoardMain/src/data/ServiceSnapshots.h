/**
 * @file ServiceSnapshots.h
 * @brief 定义 Control、Servo、State 每周期使用的纯值输入快照。
 * @details
 * - 快照不含 Queue、Mutex、TaskHandle 或 TaskSharedData 指针，可在主机单测中直接构造。
 * - 任务层在周期开始时采集一次，业务层同一周期不得重新读取同名门禁字段。
 * - 大数组反馈仍通过专用消息通道传递，避免每 10 ms 复制整个系统上下文。
 */
#ifndef MMHAND_SERVICE_SNAPSHOTS_H
#define MMHAND_SERVICE_SNAPSHOTS_H

#include <stdint.h>

struct ControlServiceInputSnapshot {
    uint32_t nowMs;
    uint32_t activeMotorMask;
    uint32_t activeJointMask;
    uint32_t servoOnlineMask;
    uint32_t encoderOnlineMask;
    uint32_t encoderFaultMask;
    uint32_t multiTurnUnknownMask;
    uint8_t systemState;
    uint8_t owner;
    uint8_t controlMode;
    uint8_t controlEnabled;
};

struct ServoServiceInputSnapshot {
    uint32_t nowMs;
    uint32_t activeMotorMask;
    uint32_t multiTurnUnknownMask;
    uint32_t emergencyStopToken;
    uint8_t systemState;
    uint8_t owner;
    uint8_t controlEnabled;
    uint8_t emergencyStopLatched;
};

struct StateServiceInputSnapshot {
    uint32_t nowMs;
    uint32_t emergencyStopToken;
    uint32_t activeMotorMask;
    uint32_t activeJointMask;
    uint32_t servoOnlineMask;
    uint32_t encoderOnlineMask;
    uint32_t multiTurnUnknownMask;
    uint8_t systemState;
    uint8_t owner;
};

#endif // MMHAND_SERVICE_SNAPSHOTS_H
