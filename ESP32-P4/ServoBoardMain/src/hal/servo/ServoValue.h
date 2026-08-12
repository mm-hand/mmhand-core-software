/**
 * @file ServoValue.h
 * @brief 定义飞特舵机反馈值的无状态转换函数。
 *
 * 本文件只处理协议数值语义，不访问串口、不修改共享状态。控制、标定、
 * 热插拔恢复必须复用这里的实现，避免各模块分别解释单圈位置和负载位。
 */
#ifndef MMHAND_SERVO_VALUE_H
#define MMHAND_SERVO_VALUE_H

#include <stdint.h>
#include <stdlib.h>

namespace mmhand::servo {

inline constexpr int32_t kSingleTurnCounts = 4096;

/** 把任意多圈位置折回飞特舵机的 0..4095 单圈坐标。 */
inline int16_t normalizeRawSingleTurn(int32_t value)
{
    int32_t raw = value % kSingleTurnCounts;
    if (raw < 0) {
        raw += kSingleTurnCounts;
    }
    return static_cast<int16_t>(raw);
}

/** 计算两个单圈位置之间的最短圆周距离，返回 0..2048 counts。 */
inline int32_t circularRawDistance(int32_t a, int32_t b)
{
    int32_t diff =
        abs(static_cast<int32_t>(normalizeRawSingleTurn(a)) -
            static_cast<int32_t>(normalizeRawSingleTurn(b))) %
        kSingleTurnCounts;
    if (diff > kSingleTurnCounts / 2) {
        diff = kSingleTurnCounts - diff;
    }
    return diff;
}

/**
 * 提取飞特负载反馈中的 10 bit 幅值。
 *
 * 飞特协议用符号/方向编码表示负载方向，安全阈值只比较绝对幅值，
 * 因此统一屏蔽到 0..1023。
 */
inline int16_t loadMagnitude(int16_t rawLoad)
{
    int16_t magnitude = rawLoad;
    if (magnitude < 0) {
        magnitude = static_cast<int16_t>(-magnitude);
    }
    return static_cast<int16_t>(magnitude & 0x03FF);
}

} // namespace mmhand::servo

#endif // MMHAND_SERVO_VALUE_H
