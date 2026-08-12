/**
 * @file CalibrationLutBuilder.cpp
 * @brief 实现 Tendon LUT 重采样及 Manual/Auto 标定结果的数据建模。
 */
#include "CalibrationLutBuilder.h"

#include <math.h>
#include <string.h>

#include "../../modules/calibration/CalibrationMath.h"
#include "../../config/CalibrationConfig.h"
#include "../../config/topology/HardwareMap.h"
#include "../../modules/persistence/ManualCalibrationNvsStore.h"

namespace mmhand::calibration {

using namespace mmhand::config::calibration;

// float 四舍五入为 int32，正负数保持对称。
int32_t roundToInt32(float value)
{
    return (int32_t)(value + (value >= 0.0f ? 0.5f : -0.5f));
}

// 计算两个方向坐标间的最短有符号磁编差。
int32_t signedEncoderDelta(int32_t fromEncoder, int32_t toEncoder)
{
    return calibrationWrapEncoderDelta(toEncoder, fromEncoder);
}

// 当实际机械正方向与原始方向坐标相反时镜像坐标，统一 LUT 正行程方向。
int32_t encoderForPositiveTravel(int32_t encoder, int8_t travelSign)
{
    if (travelSign >= 0) {
        return calibrationWrapEncoderCount(encoder);
    }
    return calibrationWrapEncoderCount(kCalibrationEncoderModulo - encoder);
}

// 清空一次 LUT 原始采样缓冲。
void resetTendonRawFitBuffer(TendonRawFitBuffer* buffer)
{
    if (!buffer) {
        return;
    }
    memset(buffer, 0, sizeof(*buffer));
}

// 追加一个去重后的磁编/舵机 ABS 样本；相同三元组不会重复占用容量。
bool appendTendonRawFitSample(TendonRawFitBuffer* buffer,
                                         int32_t encoder,
                                         int32_t primaryAbs,
                                         int32_t secondaryAbs)
{
    if (!buffer || buffer->count >= kTendonRawSampleCapacity) {
        return false;
    }
    if (buffer->count > 0) {
        TendonRawFitSample& prev = buffer->samples[buffer->count - 1];
        if (prev.encoder == encoder &&
            prev.primaryAbs == primaryAbs &&
            prev.secondaryAbs == secondaryAbs) {
            return true;
        }
    }
    TendonRawFitSample& sample = buffer->samples[buffer->count++];
    sample.encoder = calibrationWrapEncoderCount(encoder);
    sample.primaryAbs = primaryAbs;
    sample.secondaryAbs = secondaryAbs;
    return true;
}

// 当磁编行程跨过下一个 5° 采样点时，记录当前单/双电机 ABS。
void maybeAppendTendonTravelFitSample(TendonRawFitBuffer* buffer,
                                                 float travelDeg,
                                                 int32_t encoder,
                                                 int32_t primaryAbs,
                                                 int32_t secondaryAbs)
{
    if (!buffer || !isfinite(travelDeg)) {
        return;
    }
    if (travelDeg + 0.1f < buffer->nextTravelDeg) {
        return;
    }
    while (travelDeg + 0.1f >= buffer->nextTravelDeg) {
        buffer->nextTravelDeg += kTendonFitSampleStepDeg;
    }
    appendTendonRawFitSample(buffer, encoder, primaryAbs, secondaryAbs);
}

// 将原始样本的磁编位置转换为 LUT 内部轨迹角。
static float tendonFitSampleAngleDeg(const JointCalibrationResult& result,
                                   int32_t encoder)
{
    if (result.encoderSpan <= 0 || result.totalDeg <= 0.0f) {
        return NAN;
    }
    const int32_t delta =
        calibrationForwardEncoderDelta(result.startEncoder,
                                       calibrationWrapEncoderCount(encoder));
    return result.startDeg +
        ((float)delta / (float)result.encoderSpan) * result.totalDeg;
}

// 在原始样本中按角度线性插值，得到指定执行器的理论 ABS。
static bool interpolateTendonFitAbs(float targetDeg,
                                  const float* sampleDeg,
                                  const int32_t* sampleAbs,
                                  uint8_t sampleCount,
                                  int32_t* outAbs)
{
    if (!sampleDeg || !sampleAbs || !outAbs || sampleCount == 0) {
        return false;
    }
    if (targetDeg <= sampleDeg[0]) {
        *outAbs = sampleAbs[0];
        return true;
    }
    for (uint8_t i = 1; i < sampleCount; i++) {
        if (targetDeg <= sampleDeg[i]) {
            const float span = sampleDeg[i] - sampleDeg[i - 1];
            if (span <= 0.0f) {
                return false;
            }
            const float t = (targetDeg - sampleDeg[i - 1]) / span;
            const float value =
                (float)sampleAbs[i - 1] +
                ((float)sampleAbs[i] - (float)sampleAbs[i - 1]) * t;
            *outAbs = roundToInt32(value);
            return true;
        }
    }
    *outAbs = sampleAbs[sampleCount - 1];
    return true;
}

// 单、双腱共用同一套固定两行存储：单电机只写第 0 行，第 1 行保持
// motor=0xFF；双电机同时写两行。LUT 保存的是舵机 raw ABS counts。
// 将不等间隔原始样本重采样为固定 5° 步长 Tendon LUT。
bool buildTendonLutFromRawSamples(JointCalibrationResult* result,
                                         uint8_t primaryMotor,
                                         uint8_t secondaryMotor,
                                         uint8_t actuatorCount,
                                         const TendonRawFitBuffer& raw)
{
    if (!result || actuatorCount == 0 ||
        actuatorCount > TENDON_LUT_ACTUATOR_COUNT) {
        return false;
    }
    result->tendonLutValid = 0;
    result->tendonLutCount = 0;
    memset(result->tendonLutMotor, 0xFF, sizeof(result->tendonLutMotor));
    memset(result->tendonLutMotorAbs, 0, sizeof(result->tendonLutMotorAbs));
    result->tendonLutAngleStartDeg = result->startDeg;
    result->tendonLutAngleStepDeg = kTendonFitSampleStepDeg;

    if (!result->success || !result->scaled ||
        result->encoderSpan <= 0 ||
        result->totalDeg <= 0.0f ||
        raw.count < 3) {
        return false;
    }

    float sampleDeg[kTendonRawSampleCapacity];
    int32_t primaryAbs[kTendonRawSampleCapacity];
    int32_t secondaryAbs[kTendonRawSampleCapacity];
    uint8_t sampleCount = 0;

    for (uint8_t i = 0; i < raw.count; i++) {
        const float angleDeg =
            tendonFitSampleAngleDeg(*result, raw.samples[i].encoder);
        if (!isfinite(angleDeg) ||
            angleDeg < result->startDeg - kTendonFitSampleStepDeg ||
            angleDeg > result->tightDeg + kTendonFitSampleStepDeg) {
            continue;
        }
        if (sampleCount >= kTendonRawSampleCapacity) {
            continue;
        }

        uint8_t insertAt = sampleCount;
        while (insertAt > 0 && sampleDeg[insertAt - 1] > angleDeg) {
            sampleDeg[insertAt] = sampleDeg[insertAt - 1];
            primaryAbs[insertAt] = primaryAbs[insertAt - 1];
            secondaryAbs[insertAt] = secondaryAbs[insertAt - 1];
            insertAt--;
        }
        if (insertAt > 0 && fabsf(sampleDeg[insertAt - 1] - angleDeg) < 0.2f) {
            sampleDeg[insertAt - 1] = angleDeg;
            primaryAbs[insertAt - 1] = raw.samples[i].primaryAbs;
            secondaryAbs[insertAt - 1] = raw.samples[i].secondaryAbs;
            continue;
        }
        sampleDeg[insertAt] = angleDeg;
        primaryAbs[insertAt] = raw.samples[i].primaryAbs;
        secondaryAbs[insertAt] = raw.samples[i].secondaryAbs;
        sampleCount++;
    }

    if (sampleCount < 3 ||
        sampleDeg[sampleCount - 1] - sampleDeg[0] < kTendonFitSampleStepDeg * 2.0f ||
        sampleDeg[0] > result->startDeg + kTendonFitSampleStepDeg ||
        sampleDeg[sampleCount - 1] < result->tightDeg - kTendonFitSampleStepDeg) {
        return false;
    }

    uint8_t lutCount =
        (uint8_t)ceilf(result->totalDeg / kTendonFitSampleStepDeg) + 1;
    if (lutCount > TENDON_LUT_MAX_SAMPLES) {
        lutCount = TENDON_LUT_MAX_SAMPLES;
    }
    if (lutCount < 3) {
        return false;
    }

    result->tendonLutMotor[0] = primaryMotor;
    if (actuatorCount > 1) {
        result->tendonLutMotor[1] = secondaryMotor;
    }
    result->tendonLutAngleStartDeg = result->startDeg;
    // 固定容量下仍强制包含两个机械端点。行程过长时步长会略大于
    // 5 度，但不会因简单截断而丢失紧端样本。
    result->tendonLutAngleStepDeg =
        result->totalDeg / (float)(lutCount - 1);
    for (uint8_t i = 0; i < lutCount; i++) {
        const float targetDeg =
            result->tendonLutAngleStartDeg +
            result->tendonLutAngleStepDeg * (float)i;
        if (!interpolateTendonFitAbs(targetDeg,
                                     sampleDeg,
                                     primaryAbs,
                                     sampleCount,
                                     &result->tendonLutMotorAbs[0][i]) ||
            (actuatorCount > 1 &&
             !interpolateTendonFitAbs(targetDeg,
                                      sampleDeg,
                                      secondaryAbs,
                                      sampleCount,
                                      &result->tendonLutMotorAbs[1][i]))) {
            result->tendonLutValid = 0;
            result->tendonLutCount = 0;
            return false;
        }
    }
    result->tendonLutCount = lutCount;
    result->tendonLutValid = 1;
    return true;
}

// 由松端/紧端、舵机 ABS 和 Manual 参考构造完整 V3 标定结果。
static bool buildAutoZeroTightCalibrationResultWithRange(uint8_t jointIndex,
                                                         int32_t tautEncoder,
                                                         int32_t zeroEncoder,
                                                         int32_t tightEncoder,
                                                         int32_t tautServoAbs,
                                                         int32_t zeroServoAbs,
                                                         int32_t tightServoAbs,
                                                         float startDeg,
                                                         JointCalibrationResult* out)
{
    (void)zeroEncoder;
    if (!out) {
        return false;
    }

    memset(out, 0, sizeof(*out));

    const int32_t startToTightSpan =
        calibrationForwardEncoderDelta(tautEncoder, tightEncoder);
    const float totalDeg = calibrationEncoderCountToDeg(startToTightSpan);
    if (startToTightSpan <= 0 ||
        !isfinite(totalDeg) ||
        totalDeg < kMinTravelDeg ||
        totalDeg >= kMaxTravelDeg) {
        return false;
    }
    const float tightDeg = startDeg + totalDeg;

    const float zeroFraction =
        (kLutZeroDeg - startDeg) / totalDeg;
    if (zeroFraction <= 0.0f || zeroFraction >= 1.0f) {
        return false;
    }

    const int32_t zeroOffsetCount =
        roundToInt32((float)startToTightSpan * zeroFraction);
    const int32_t computedZeroEncoder =
        calibrationWrapEncoderCount(tautEncoder + zeroOffsetCount);

    out->success = true;
    out->angleMax = tightDeg;
    out->startEncoder = tautEncoder;
    out->tightEncoder = tightEncoder;
    out->zeroEncoder = computedZeroEncoder;
    out->encoderSpan = startToTightSpan;
    out->startServoAbs = tautServoAbs;
    out->tightServoAbs = tightServoAbs;
    out->zeroServoAbs = zeroServoAbs;
    out->startDeg = startDeg;
    out->tightDeg = tightDeg;
    out->totalDeg = totalDeg;
    out->scaled = true;

    // 上面的历史标定字段使用按关节方向统一后的磁编坐标；V3 的
    // autoMinRaw/autoMaxRaw 属于通信与 NVS 契约，必须保存物理 14-bit raw。
    // calibrationOrientEncoderRaw() 在 direction=-1 时是自逆变换，因此这里
    // 再调用一次即可还原物理 raw，同时不会改变 direction=+1 的关节。
    const int8_t encoderDirection = g_encoderDirection[jointIndex];
    const uint16_t autoMinRaw = (uint16_t)calibrationOrientEncoderRaw(
        (uint16_t)calibrationWrapEncoderCount(tautEncoder), encoderDirection);
    const uint16_t autoMaxRaw = (uint16_t)calibrationOrientEncoderRaw(
        (uint16_t)calibrationWrapEncoderCount(tightEncoder), encoderDirection);
    if (!applyManualScaledUrdfMappingToCalibration(
            jointIndex,
            g_manualEncoderReference[jointIndex],
            autoMinRaw,
            autoMaxRaw,
            encoderDirection,
            manualEncoderReferenceEntryCrc(
                jointIndex, g_manualEncoderReference[jointIndex]),
            out)) {
        memset(out, 0, sizeof(*out));
        return false;
    }
    return true;
}

// 单电机关节结果构造包装：使用当前 URDF 上限和 Manual 条目。
bool buildAutoZeroTightCalibrationResult(uint8_t jointIndex,
                                                int32_t tautEncoder,
                                                int32_t zeroEncoder,
                                                int32_t tightEncoder,
                                                int32_t tautServoAbs,
                                                int32_t zeroServoAbs,
                                                int32_t tightServoAbs,
                                                JointCalibrationResult* out)
{
    return buildAutoZeroTightCalibrationResultWithRange(jointIndex,
                                                        tautEncoder,
                                                        zeroEncoder,
                                                        tightEncoder,
                                                        tautServoAbs,
                                                        zeroServoAbs,
                                                        tightServoAbs,
                                                        kLutStartDeg,
                                                        out);
}

// 双腱关节结果构造包装：两路舵机共享同一磁编 Auto Min/Max。
bool buildDualZeroTightCalibrationResult(uint8_t jointIndex,
                                                int32_t startEncoder,
                                                int32_t zeroEncoder,
                                                int32_t tightEncoder,
                                                int32_t startServoAbs,
                                                int32_t zeroServoAbs,
                                                int32_t tightServoAbs,
                                                JointCalibrationResult* out)
{
    return buildAutoZeroTightCalibrationResultWithRange(jointIndex,
                                                        startEncoder,
                                                        zeroEncoder,
                                                        tightEncoder,
                                                        startServoAbs,
                                                        zeroServoAbs,
                                                        tightServoAbs,
                                                        kLutStartDeg,
                                                        out);
}


} // namespace mmhand::calibration
