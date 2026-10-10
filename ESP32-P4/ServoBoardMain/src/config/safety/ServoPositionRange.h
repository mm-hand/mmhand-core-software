/**
 * @file ServoPositionRange.h
 * @brief 定义飞特舵机统一 ABS 命令范围及受检的硬件目标转换。
 * @details
 * - 线程：纯函数，可由控制、安全策略、反馈发布和 HAL 驱动并发调用。
 * - 输入输出：输入 ABS，输出固定可达范围或硬件目标。
 * - 所有权：属于 config/safety，数值唯一来源为 SafetyConfig.h。
 * - 禁止依赖：不得包含任务、日志、状态机、通信或飞特 SDK。
 */
#ifndef P4_CONFIG_SAFETY_SERVO_POSITION_RANGE_H
#define P4_CONFIG_SAFETY_SERVO_POSITION_RANGE_H

#include <stdint.h>

#include "../SafetyConfig.h"

using mmhand::config::safety::kServoHardwareCommandMin;
using mmhand::config::safety::kServoHardwareCommandMax;
using mmhand::config::safety::kServoResetZeroTriggerMin;
using mmhand::config::safety::kServoResetZeroTriggerMax;

/** 业务与舵机使用同一 ABS 坐标时的固定可达范围。 */
struct ServoLogicalRange {
    int32_t minAbs;
    int32_t maxAbs;
};

/** 返回舵机 ABS 的硬件命令范围。 */
static inline ServoLogicalRange servoPositionRange()
{
    ServoLogicalRange range = {
        kServoHardwareCommandMin, kServoHardwareCommandMax};
    return range;
}

/**
 * 检查统一 ABS 范围后转换为 ST3215 的 int16_t 命令。
 * 结果超出硬件命令范围时返回 false，禁止依赖 int16_t 截断产生反向运动。
 */
static inline bool servoHardwareCommandFromAbs(int32_t absolutePosition,
                                               int16_t* outHardwareAbs)
{
    if (!outHardwareAbs) {
        return false;
    }
    if (absolutePosition < kServoHardwareCommandMin ||
        absolutePosition > kServoHardwareCommandMax) {
        return false;
    }
    *outHardwareAbs = (int16_t)absolutePosition;
    return true;
}

#endif // P4_CONFIG_SAFETY_SERVO_POSITION_RANGE_H
