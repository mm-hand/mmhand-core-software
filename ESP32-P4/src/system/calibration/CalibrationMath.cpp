/**
 * @file CalibrationMath.cpp
 * @brief 实现磁编跨圈、Manual counts 窗口和 V5 k/b 角度映射数学。
 */
#include "CalibrationMath.h"

#include <math.h>
#include <string.h>

#include "../../generated/UrdfJointLimits.generated.h"
#include "../../system/log/FirmwareLog.h"

static_assert(kManualScaledUrdfFeedbackMappingVersion == 5,
              "Manual-scaled calibration contract must remain v5");

// 环形取模，兼容负数输入。
int32_t calibrationWrapEncoderCount(int32_t value)
{
    value %= kCalibrationEncoderModulo;
    if (value < 0) {
        value += kCalibrationEncoderModulo;
    }
    return value;
}

// 计算环形正向距离。
int32_t calibrationForwardEncoderDelta(int32_t from, int32_t to)
{
    return calibrationWrapEncoderCount(to - from);
}

// 计算两个环形坐标之间的最短有符号差。
int32_t calibrationWrapEncoderDelta(int32_t target, int32_t actual)
{
    int32_t delta = target - actual;
    while (delta > kCalibrationEncoderHalfTurn) {
        delta -= kCalibrationEncoderModulo;
    }
    while (delta < -kCalibrationEncoderHalfTurn) {
        delta += kCalibrationEncoderModulo;
    }
    return delta;
}

// 将诊断角度换算为磁编 counts，并四舍五入到整数。
int32_t calibrationDegToEncoderCount(float deg)
{
    const float scaled = deg * (float)kCalibrationEncoderModulo / 360.0f;
    return (int32_t)(scaled + (scaled >= 0.0f ? 0.5f : -0.5f));
}

// V5：Manual 机械行程仍是 counts 唯一的物理窗口；两端的角度由 angleK/angleB 决定。
// joint calibration 给 k0/b0；contact calibration 通过同一条 CALIB_LOAD_JOINT 覆盖 k/b。
bool applyManualScaledUrdfMappingToCalibration(
    uint8_t jointIndex,
    const ManualEncoderReference& manual,
    uint16_t autoMinRaw,
    uint16_t autoMaxRaw,
    uint32_t manualEntryCrc,
    JointCalibrationResult* result)
{
    if (!result || jointIndex >= ENCODER_TOTAL_NUM || !manual.valid ||
        manual.minRaw >= kCalibrationEncoderModulo ||
        manual.maxRaw >= kCalibrationEncoderModulo ||
        autoMinRaw >= kCalibrationEncoderModulo ||
        autoMaxRaw >= kCalibrationEncoderModulo) {
        return false;
    }

    const int32_t manualSpan = calibrationForwardEncoderDelta(
        manual.minRaw, manual.maxRaw);
    const int32_t autoSpan = calibrationForwardEncoderDelta(
        autoMinRaw, autoMaxRaw);
    if (manualSpan <= 0 || manualSpan >= kCalibrationEncoderModulo ||
        autoSpan <= 0 || autoSpan >= kCalibrationEncoderModulo) {
        return false;
    }

    const UrdfJointLimitDeg& limit = kUrdfJointLimits[jointIndex];
    const float urdfSpanDeg = limit.maxDeg - limit.minDeg;
    // joint calibration 的初值；contact calibration 的 incoming k/b 覆盖它。
    const float k0 = urdfSpanDeg / (float)manualSpan;
    const float b0 = limit.minDeg;
    if (!isfinite(limit.minDeg) || !isfinite(limit.maxDeg) ||
        !isfinite(urdfSpanDeg) || urdfSpanDeg <= 0.0f ||
        !isfinite(k0) || k0 <= 0.0f) {
        return false;
    }
    const bool hasIncoming = isfinite(result->angleK) && result->angleK > 0.0f &&
                             isfinite(result->angleB);
    const float k = hasIncoming ? result->angleK : k0;
    const float b = hasIncoming ? result->angleB : b0;
    // 拟合结果的拒绝带；counts 安全域不受影响。
    if (!isfinite(k) || !isfinite(b) || k <= 0.0f ||
        fabsf(k / k0 - 1.0f) > 0.10f || fabsf(b - b0) > 5.0f) {
        return false;
    }

    result->mappingVersion = kManualScaledUrdfFeedbackMappingVersion;
    result->autoMinRaw = autoMinRaw;
    result->autoMaxRaw = autoMaxRaw;
    result->manualSpanCounts = manualSpan;
    result->autoSpanCounts = autoSpan;
    result->manualReferenceCrc = manualEntryCrc;
    result->angleK = k;
    result->angleB = b;
    result->angleMin = b;
    result->angleMax = k * (float)manualSpan + b;
    result->urdfConfigCrc = kManualScaledUrdfConfigCrc;
    const float autoManualRatio = (float)autoSpan / (float)manualSpan;
    FW_LOG_INFO_PRINTF(
        "[CALIB MAP V5] J%02u manual=%ld(%u->%u) "
        "auto=%ld(%u->%u) ratio=%.6f "
        "k=%.6f b=%.3f range=[%.3f,%.3f] crc=0x%08lX\r\n",
        (unsigned)jointIndex,
        (long)manualSpan,
        (unsigned)manual.minRaw,
        (unsigned)manual.maxRaw,
        (long)autoSpan,
        (unsigned)autoMinRaw,
        (unsigned)autoMaxRaw,
        autoManualRatio,
        k,
        b,
        result->angleMin,
        result->angleMax,
        (unsigned long)manualEntryCrc);
    if (autoManualRatio > 1.0f) {
        FW_LOG_WARN_PRINTF(
            "[CALIB MAP V5] WARN J%02u auto/manual ratio %.6f > 1; "
            "result kept for diagnostics and control\r\n",
            (unsigned)jointIndex,
            autoManualRatio);
    }
    return true;
}

// 把环形 raw 转成以窗口起点为 0 的有符号坐标，用于正确处理跨零和窗口外反馈。
static int32_t signedDirectedCoordinateFromWindowStart(
    uint16_t windowStartRaw,
    uint16_t currentRaw,
    int32_t windowSpan)
{
    const int32_t forward = calibrationForwardEncoderDelta(windowStartRaw, currentRaw);
    if (forward <= windowSpan) {
        return forward;
    }

    const int32_t aboveWindow = forward - windowSpan;
    const int32_t belowWindow = kCalibrationEncoderModulo - forward;
    return belowWindow < aboveWindow ? -belowWindow : forward;
}

// V5 实时映射：theta = angleK * countCoordinate + angleB。
// counts 窗口/READY/outside 不变；k/b 只解释角度，不改安全域。
bool mapManualScaledEncoderToUrdf(uint8_t jointIndex,
                                  const JointCalibrationResult& calibration,
                                  const ManualEncoderReference& manual,
                                  uint16_t currentRaw,
                                  EncoderUrdfMapResult* out)
{
    if (!out) {
        return false;
    }
    memset(out, 0, sizeof(*out));
    if (jointIndex >= ENCODER_TOTAL_NUM ||
        calibration.mappingVersion != kManualScaledUrdfFeedbackMappingVersion ||
        calibration.urdfConfigCrc != kManualScaledUrdfConfigCrc ||
        !manual.valid ||
        manual.minRaw >= kCalibrationEncoderModulo ||
        manual.maxRaw >= kCalibrationEncoderModulo ||
        currentRaw >= kCalibrationEncoderModulo ||
        calibration.manualSpanCounts <= 0 ||
        calibration.manualSpanCounts >= kCalibrationEncoderModulo ||
        !isfinite(calibration.angleK) || calibration.angleK <= 0.0f ||
        !isfinite(calibration.angleB) ||
        !isfinite(calibration.angleMin) || !isfinite(calibration.angleMax) ||
        calibration.angleMax <= calibration.angleMin) {
        return false;
    }

    // 运行时重新根据 P4 当前 Manual 端点计算跨度，禁止旧快照中的
    // manualSpanCounts 与当前 Manual 数据不一致时继续参与角度换算。
    const int32_t currentManualSpan = calibrationForwardEncoderDelta(
        manual.minRaw, manual.maxRaw);
    if (currentManualSpan <= 0 ||
        currentManualSpan >= kCalibrationEncoderModulo ||
        currentManualSpan != calibration.manualSpanCounts) {
        return false;
    }
    // 以 Manual Min 为零点建立有符号坐标，跨 0/16383 时仍能区分上下边界。
    const int32_t manualCoordinate =
        signedDirectedCoordinateFromWindowStart(
            manual.minRaw, currentRaw, currentManualSpan);
    const int32_t belowMinCounts = manualCoordinate < 0
        ? -manualCoordinate : 0;
    const int32_t aboveMaxCounts =
        manualCoordinate > currentManualSpan
        ? manualCoordinate - currentManualSpan : 0;

    const float mappedDeg = calibration.angleK *
        (float)manualCoordinate + calibration.angleB;
    if (!isfinite(mappedDeg)) {
        return false;
    }

    out->valid = true;
    out->feedbackReady = belowMinCounts == 0;
    out->outsideCounts = belowMinCounts > 0
        ? -belowMinCounts : aboveMaxCounts;
    out->belowMinCounts = belowMinCounts;
    out->aboveMaxCounts = aboveMaxCounts;
    out->urdfDeg = mappedDeg;
    return true;
}
