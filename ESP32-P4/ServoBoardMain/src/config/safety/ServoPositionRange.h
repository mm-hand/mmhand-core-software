/**
 * @file ServoPositionRange.h
 * @brief 定义飞特舵机硬件多圈命令范围及本次上电会话的 logical ABS 换算规则。
 * @details
 * - 线程：纯函数，可由控制、安全策略、反馈发布和 HAL 驱动并发调用。
 * - 输入输出：输入 logicalOffset/logicalAbs，输出会话可达范围或 hardware ABS。
 * - 所有权：属于 config/safety，数值唯一来源为 SafetyConfig.h。
 * - 禁止依赖：不得包含任务、日志、状态机、通信或飞特 SDK。
 */
#ifndef P4_CONFIG_SAFETY_SERVO_POSITION_RANGE_H
#define P4_CONFIG_SAFETY_SERVO_POSITION_RANGE_H

#include <limits.h>
#include <stdint.h>

#include "../SafetyConfig.h"

using mmhand::config::safety::kServoHardwareCommandMin;
using mmhand::config::safety::kServoHardwareCommandMax;
using mmhand::config::safety::kServoResetZeroTriggerMin;
using mmhand::config::safety::kServoResetZeroTriggerMax;

/** 单台舵机在当前 logicalOffset 下可安全到达的逻辑多圈范围。 */
struct ServoLogicalRange {
    int32_t minAbs;
    int32_t maxAbs;
};

/** 根据 logicalOffset 计算当前上电会话可达到的 logical ABS 范围。 */
static inline ServoLogicalRange servoLogicalRangeFromOffset(int32_t logicalOffset)
{
    ServoLogicalRange range;
    const int64_t minValue =
        (int64_t)logicalOffset + (int64_t)kServoHardwareCommandMin;
    const int64_t maxValue =
        (int64_t)logicalOffset + (int64_t)kServoHardwareCommandMax;
    range.minAbs = (minValue < INT32_MIN) ? INT32_MIN : (int32_t)minValue;
    range.maxAbs = (maxValue > INT32_MAX) ? INT32_MAX : (int32_t)maxValue;
    return range;
}

/**
 * 把业务 logical ABS 转换为 ST3215 本次上电会话接受的 hardware ABS。
 * 结果超出硬件命令范围时返回 false，禁止依赖 int16_t 截断产生反向运动。
 */
static inline bool servoLogicalToHardwareAbs(int32_t logicalAbs,
                                             int32_t logicalOffset,
                                             int16_t* outHardwareAbs)
{
    if (!outHardwareAbs) {
        return false;
    }
    const int64_t hardwareAbs =
        (int64_t)logicalAbs - (int64_t)logicalOffset;
    if (hardwareAbs < kServoHardwareCommandMin ||
        hardwareAbs > kServoHardwareCommandMax) {
        return false;
    }
    *outHardwareAbs = (int16_t)hardwareAbs;
    return true;
}

#endif // P4_CONFIG_SAFETY_SERVO_POSITION_RANGE_H
