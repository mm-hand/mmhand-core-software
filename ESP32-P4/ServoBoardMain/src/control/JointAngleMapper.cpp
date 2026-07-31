/**
 * @file JointAngleMapper.cpp
 * @brief 实现 14 bit 环形磁编码器与标定后关节角之间的连续映射。
 */
#include "JointAngleMapper.h"

#include <math.h>

#include "../config/ControlConfig.h"
#include "../generated/UrdfJointLimits.generated.h"

namespace p4::control {

using namespace mmhand::config::control;

int32_t orientEncoderRaw(uint16_t rawValue, int8_t direction)
{
    // 先折回 14 bit，再把反向安装统一成“机械正方向 raw 增加”。
    int32_t oriented = static_cast<int32_t>(rawValue) & (kEncoderModulo - 1);
    if (direction < 0) {
        oriented = (kEncoderModulo - oriented) & (kEncoderModulo - 1);
    }
    return oriented;
}

int32_t wrapEncoderCount(int32_t value)
{
    value %= kEncoderModulo;
    if (value < 0) {
        value += kEncoderModulo;
    }
    return value;
}

int32_t forwardEncoderDelta(int32_t from, int32_t to)
{
    // 结果始终取正向环形距离，跨 16383 -> 0 时仍保持连续。
    return wrapEncoderCount(to - from);
}

float encoderCountToDeg(int32_t encoderCount)
{
    return static_cast<float>(encoderCount) * 360.0f /
           static_cast<float>(kEncoderModulo);
}

int32_t degToEncoderCount(float deg)
{
    const float scaled =
        deg * static_cast<float>(kEncoderModulo) / 360.0f;
    return static_cast<int32_t>(
        scaled + (scaled >= 0.0f ? 0.5f : -0.5f));
}

float mapCalibratedEncoderToDeg(const JointCalibrationResult& calibration,
                                int32_t orientedRaw)
{
    // scaled 模型在 Auto 区间内相信实测跨度；超过末端后按一圈比例连续外推。
    if (calibration.scaled &&
        calibration.encoderSpan > 0 &&
        calibration.totalDeg > 0.0f) {
        int32_t delta =
            forwardEncoderDelta(calibration.startEncoder, orientedRaw);
        if (delta > calibration.encoderSpan + kEncoderMarginCounts) {
            // 正向距离远超标定末端，实际位置位于起点反方向，改成负值表示。
            delta -= kEncoderModulo;
        }
        return (static_cast<float>(delta) /
                static_cast<float>(calibration.encoderSpan)) *
                   calibration.totalDeg +
               calibration.startDeg;
    }

    // 非 scaled 路径只服务仍在运行的旧映射版本，不参与当前 V3 Auto 结果生成。
    const int32_t offset = calibration.offset;
    int32_t delta = forwardEncoderDelta(offset, orientedRaw);
    int32_t hardMaxFromOffset =
        forwardEncoderDelta(offset, calibration.encoderMax);
    if (hardMaxFromOffset <= 0) {
        hardMaxFromOffset = degToEncoderCount(calibration.angleMax);
    }
    if (delta > hardMaxFromOffset + kEncoderMarginCounts) {
        delta -= kEncoderModulo;
    }
    return encoderCountToDeg(delta);
}

float clampJointTargetDeg(float targetDeg, uint8_t jointIndex)
{
    if (!isfinite(targetDeg) || jointIndex >= ENCODER_TOTAL_NUM) {
        return 0.0f;
    }

    float minDeg = 0.0f;
    float maxDeg = 0.0f;
    // 有效 Auto 使用动态 Current Min/URDF Max；未标定仅给 UI/诊断提供静态 URDF 范围。
    const JointCalibrationResult& calibration =
        g_jointCalibResult[jointIndex];
    if (calibration.success) {
        minDeg = calibration.mappingVersion >= 2
                     ? calibration.angleMin
                     : 0.0f;
        maxDeg = calibration.angleMax;
    } else {
        minDeg = kUrdfJointLimits[jointIndex].minDeg;
        maxDeg = kUrdfJointLimits[jointIndex].maxDeg;
    }

    if (!isfinite(minDeg) || !isfinite(maxDeg) || minDeg >= maxDeg) {
        return 0.0f;
    }
    if (targetDeg < minDeg) {
        return minDeg;
    }
    if (targetDeg > maxDeg) {
        return maxDeg;
    }
    return targetDeg;
}

} // namespace p4::control
