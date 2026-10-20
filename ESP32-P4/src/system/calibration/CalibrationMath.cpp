/**
 * @file CalibrationMath.cpp
 * @brief 实现磁编跨圈、Manual 全行程比例和 URDF 静态区间映射数学。
 */
#include "CalibrationMath.h"

#include <math.h>
#include <string.h>

#include "../../generated/UrdfJointLimits.generated.h"
#include "../../system/log/FirmwareLog.h"

static_assert(kManualScaledUrdfFeedbackMappingVersion == 4,
              "Manual-scaled calibration contract must remain v4");

// 按固定安装方向归一化原始磁编值，使“关节正方向”始终对应坐标增加。
int32_t calibrationOrientEncoderRaw(uint16_t rawValue, int8_t direction)
{
    int32_t oriented = (int32_t)rawValue & (kCalibrationEncoderModulo - 1);
    if (direction < 0) {
        oriented = (kCalibrationEncoderModulo - oriented) & (kCalibrationEncoderModulo - 1);
    }
    return oriented;
}

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

// 直接在原始磁编值上按安装方向计算正向距离。
int32_t calibrationDirectedEncoderDelta(uint16_t fromRaw,
                                        uint16_t toRaw,
                                        int8_t direction)
{
    if (direction != 1 && direction != -1) {
        return -1;
    }
    const int32_t from = (int32_t)fromRaw & (kCalibrationEncoderModulo - 1);
    const int32_t to = (int32_t)toRaw & (kCalibrationEncoderModulo - 1);
    return direction > 0
        ? calibrationWrapEncoderCount(to - from)
        : calibrationWrapEncoderCount(from - to);
}

// 把正向距离折算为半圈内的最短有符号位移。
int32_t calibrationSignedDirectedEncoderDelta(uint16_t fromRaw,
                                              uint16_t toRaw,
                                              int8_t direction)
{
    const int32_t forward = calibrationDirectedEncoderDelta(
        fromRaw, toRaw, direction);
    if (forward < 0) {
        return 0;
    }
    return forward > kCalibrationEncoderHalfTurn
        ? forward - kCalibrationEncoderModulo
        : forward;
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

// 仅按编码器物理分辨率换算角度，不代表标定区间内的真实机械比例。
float calibrationEncoderCountToDeg(int32_t count)
{
    return (float)count * 360.0f / (float)kCalibrationEncoderModulo;
}

// 将诊断角度换算为磁编 counts，并四舍五入到整数。
int32_t calibrationDegToEncoderCount(float deg)
{
    const float scaled = deg * (float)kCalibrationEncoderModulo / 360.0f;
    return (int32_t)(scaled + (scaled >= 0.0f ? 0.5f : -0.5f));
}

// V4 将完整 Manual 机械行程固定映射到完整 URDF 行程。
// Auto 端点只描述本次自动标定覆盖范围，不再改变工作区或实时角度比例。
bool applyManualScaledUrdfMappingToCalibration(
    uint8_t jointIndex,
    const ManualEncoderReference& manual,
    uint16_t autoMinRaw,
    uint16_t autoMaxRaw,
    int8_t direction,
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

    const int32_t manualSpan = calibrationDirectedEncoderDelta(
        manual.minRaw, manual.maxRaw, direction);
    const int32_t autoSpan = calibrationDirectedEncoderDelta(
        autoMinRaw, autoMaxRaw, direction);
    if (manualSpan <= 0 || manualSpan >= kCalibrationEncoderModulo ||
        autoSpan <= 0 || autoSpan >= kCalibrationEncoderModulo) {
        return false;
    }

    const UrdfJointLimitDeg& limit = kUrdfJointLimits[jointIndex];
    const float urdfSpanDeg = limit.maxDeg - limit.minDeg;
    const float degPerCount = urdfSpanDeg / (float)manualSpan;
    // V4 工作区唯一来源是生成的 URDF 静态 Min/Max，不再保存 current 范围副本。
    if (!isfinite(limit.minDeg) || !isfinite(limit.maxDeg) ||
        !isfinite(urdfSpanDeg) || urdfSpanDeg <= 0.0f ||
        !isfinite(degPerCount) || degPerCount <= 0.0f) {
        return false;
    }

    result->mappingVersion = kManualScaledUrdfFeedbackMappingVersion;
    result->autoMinRaw = autoMinRaw;
    result->autoMaxRaw = autoMaxRaw;
    result->manualSpanCounts = manualSpan;
    result->autoSpanCounts = autoSpan;
    result->manualReferenceCrc = manualEntryCrc;
    result->angleMin = limit.minDeg;
    result->angleMax = limit.maxDeg;
    result->urdfConfigCrc = kManualScaledUrdfConfigCrc;
    const float autoManualRatio = (float)autoSpan / (float)manualSpan;
    FW_LOG_INFO_PRINTF(
        "[CALIB MAP V4] J%02u dir=%d manual=%ld(%u->%u) "
        "auto=%ld(%u->%u) ratio=%.6f "
        "urdf=[%.3f,%.3f] crc=0x%08lX\r\n",
        (unsigned)jointIndex,
        (int)direction,
        (long)manualSpan,
        (unsigned)manual.minRaw,
        (unsigned)manual.maxRaw,
        (long)autoSpan,
        (unsigned)autoMinRaw,
        (unsigned)autoMaxRaw,
        autoManualRatio,
        limit.minDeg,
        limit.maxDeg,
        (unsigned long)manualEntryCrc);
    if (autoManualRatio > 1.0f) {
        FW_LOG_WARN_PRINTF(
            "[CALIB MAP V4] WARN J%02u auto/manual ratio %.6f > 1; "
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
    int8_t direction,
    int32_t windowSpan)
{
    const int32_t forward = calibrationDirectedEncoderDelta(
        windowStartRaw, currentRaw, direction);
    if (forward <= windowSpan) {
        return forward;
    }

    const int32_t aboveWindow = forward - windowSpan;
    const int32_t belowWindow = kCalibrationEncoderModulo - forward;
    return belowWindow < aboveWindow ? -belowWindow : forward;
}

// V4 实时映射：Manual 区间严格对应 URDF Min/Max；Manual 区间外才按
// 16384 counts/360° 物理比例外推。Auto 区间不参与角度换算。
bool mapManualScaledEncoderToUrdf(uint8_t jointIndex,
                                  const JointCalibrationResult& calibration,
                                  const ManualEncoderReference& manual,
                                  uint16_t currentRaw,
                                  int8_t direction,
                                  EncoderUrdfMapResult* out)
{
    if (!out) {
        return false;
    }
    memset(out, 0, sizeof(*out));
    const UrdfJointLimitDeg* limit = jointIndex < ENCODER_TOTAL_NUM
        ? &kUrdfJointLimits[jointIndex] : nullptr;
    if (jointIndex >= ENCODER_TOTAL_NUM ||
        !limit ||
        calibration.mappingVersion != kManualScaledUrdfFeedbackMappingVersion ||
        calibration.urdfConfigCrc != kManualScaledUrdfConfigCrc ||
        !manual.valid ||
        manual.minRaw >= kCalibrationEncoderModulo ||
        manual.maxRaw >= kCalibrationEncoderModulo ||
        currentRaw >= kCalibrationEncoderModulo ||
        calibration.manualSpanCounts <= 0 ||
        calibration.manualSpanCounts >= kCalibrationEncoderModulo ||
        !isfinite(calibration.angleMin) ||
        !isfinite(calibration.angleMax) ||
        calibration.angleMax <= calibration.angleMin ||
        fabsf(calibration.angleMin - limit->minDeg) > 0.01f ||
        fabsf(calibration.angleMax - limit->maxDeg) > 0.01f) {
        return false;
    }

    const float urdfSpanDeg = limit->maxDeg - limit->minDeg;
    // 运行时重新根据 P4 当前 Manual 端点计算跨度，禁止旧快照中的
    // manualSpanCounts 与当前 Manual 数据不一致时继续参与角度换算。
    const int32_t currentManualSpan = calibrationDirectedEncoderDelta(
        manual.minRaw, manual.maxRaw, direction);
    if (currentManualSpan <= 0 ||
        currentManualSpan >= kCalibrationEncoderModulo ||
        currentManualSpan != calibration.manualSpanCounts) {
        return false;
    }
    const float manualDegPerCount =
        urdfSpanDeg / (float)currentManualSpan;
    // 以 Manual Min 为零点建立有符号坐标，跨 0/16383 时仍能区分上下边界。
    const int32_t manualCoordinate =
        signedDirectedCoordinateFromWindowStart(
            manual.minRaw, currentRaw, direction,
            currentManualSpan);
    const int32_t belowMinCounts = manualCoordinate < 0
        ? -manualCoordinate : 0;
    const int32_t aboveMaxCounts =
        manualCoordinate > currentManualSpan
        ? manualCoordinate - currentManualSpan : 0;

    float mappedDeg = 0.0f;
    if (manualCoordinate < 0) {
        mappedDeg = limit->minDeg +
            calibrationEncoderCountToDeg(manualCoordinate);
    } else if (manualCoordinate <= currentManualSpan) {
        mappedDeg = limit->minDeg +
            (float)manualCoordinate * manualDegPerCount;
    } else {
        mappedDeg = limit->maxDeg +
            calibrationEncoderCountToDeg(
            manualCoordinate - currentManualSpan);
    }
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
