#ifndef SERVO_POSITION_RANGE_H
#define SERVO_POSITION_RANGE_H

#include <Arduino.h>
#include <limits.h>

// ST3215 多圈模式在单次上电会话内使用有符号位置命令。这里的端点只约束
// 舵机硬件坐标，不再约束 P4 的逻辑多圈坐标。
static const int32_t kServoHardwareCommandMin = -30719;
static const int32_t kServoHardwareCommandMax = 30719;

// 部分 ST3215 在接近硬件端点时会停在约 ±30717，无法继续越过 ±30719。
// 因此 Reset Zero 提前 4 counts 触发，给检测和复位流程保留可达余量。
static const int32_t kServoResetZeroTriggerMin = -30715;
static const int32_t kServoResetZeroTriggerMax = 30715;
static_assert(kServoResetZeroTriggerMin > kServoHardwareCommandMin &&
                  kServoResetZeroTriggerMax < kServoHardwareCommandMax,
              "Reset Zero trigger must stay inside the hardware command range");

struct ServoLogicalRange {
    int32_t minAbs;
    int32_t maxAbs;
};

// 根据单台舵机自己的 logicalOffset 计算本次上电会话可达的逻辑范围。
// 每台舵机独立计算，因此一台舵机掉电不会改变其他电机的行程。
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

// 将 P4 逻辑 ABS 转换为舵机本次上电会话的硬件 ABS。只有转换结果落在
// ST3215 实际命令范围内才允许下发，禁止依赖 int16 强制转换产生反向运动。
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

static inline int32_t servoClampLogicalToSessionRange(int32_t logicalAbs,
                                                       int32_t logicalOffset)
{
    const ServoLogicalRange range =
        servoLogicalRangeFromOffset(logicalOffset);
    if (logicalAbs < range.minAbs) return range.minAbs;
    if (logicalAbs > range.maxAbs) return range.maxAbs;
    return logicalAbs;
}

#endif // SERVO_POSITION_RANGE_H
