/**
 * @file CalibrationRuntimeStore.h
 * @brief 定义标定结果、Manual 参考、LUT 和诊断协议结构。
 */
#ifndef CALIBRATION_TASK_H
#define CALIBRATION_TASK_H

#include <Arduino.h>
#include "../../data/CalibrationTypes.h"
#include "../../runtime/SystemRuntimeContext.h"

// 标定公共数据与接口。
// 标定生命周期由 SystemStateTask 统一管理；本模块只保存标定结果，并提供
// 编码器方向、装配告警和运行期映射所需的公共算法。
// 三项装配告警阈值，单位均为磁编码器 counts。
// 告警不影响标定结果可用性，只用于提示机械装配或人工端点录入偏差。
#define TENDON_LUT_ACTUATOR_COUNT 2  // 每个关节最多两路腱绳执行器。
#define TENDON_LUT_MAX_SAMPLES 24    // 每路执行器最多保存 24 个角度采样点。

// 操作员在 Device Check 页面录入的磁编码器机械端点。
// minRaw 为人工松端，maxRaw 为人工紧端；两者都是 0..16383 原始值，允许跨零。
// 正方向由 g_encoderDirection 单独定义，避免在 NVS 中重复保存方向。
struct ManualEncoderReference {
    uint8_t valid;   // 1 表示该关节的人工端点已写入并通过 NVS 校验。
    uint16_t minRaw; // 人工松端原始磁编值。
    uint16_t maxRaw; // 人工紧端原始磁编值。
};

// 一次自动标定结束后计算出的装配诊断量。
struct CalibrationAssemblyWarningInfo {
    uint8_t reasonMask;              // CALIB_WARN_* 的组合位图。
    int16_t autoMinOffsetCounts;     // Auto Min 相对 Manual Min 的正向偏移。
    int16_t autoMaxShortfallCounts;  // Manual Max 相对 Auto Max 的正向余量。
    uint16_t autoSpanCounts;         // 自动标定实际工作区跨度。
    uint16_t manualSpanCounts;       // 人工录入机械端点跨度。
    uint16_t urdfSpanCounts;         // 按 16384 counts/360° 换算的 URDF 理论跨度。
    uint16_t spanDifferenceCounts;   // Manual Span 与 URDF Span 的绝对差。
};

// 每个关节的自动标定结果。当前结构只表达 Manual 全行程映射 V4。
struct JointCalibrationResult {
    bool success;       // true 表示该结构中的 Auto 结果已完整生成。
    float angleMax;     // 控制工作区最大角，V4 中固定等于 URDF Max。

    // 映射版本与模型契约。mappingVersion 只允许当前 V4。
    uint8_t mappingVersion;
    float angleMin;           // 控制工作区最小角，V4 中固定等于 URDF Min。
    uint32_t urdfConfigCrc;   // 标定结果绑定的 URDF/映射契约 CRC。

    // Manual 全行程映射 V4。修改某一关节 Manual 后，只使该关节 Auto 失效。
    uint16_t autoMinRaw;       // 自动标定松端原始磁编值。
    uint16_t autoMaxRaw;       // 自动标定紧端原始磁编值。
    int32_t manualSpanCounts;  // 沿安装正方向计算的 Manual Span。
    int32_t autoSpanCounts;    // 沿安装正方向计算的 Auto Span。
    uint32_t manualReferenceCrc; // 该关节 Manual Min/Max 的条目 CRC。
    float currentMinDeg;       // Manual Min 对应的 URDF Min。
    float currentMaxDeg;       // Manual Max 对应的 URDF Max。

    // 标定内部轨迹坐标仅用于诊断和原始采样定位；V4 Tendon LUT 横轴使用 URDF 角。
    int32_t startEncoder;  // 搜索起点磁编方向坐标，对应 startDeg。
    int32_t tightEncoder;  // 紧端磁编方向坐标，对应 tightDeg。
    int32_t zeroEncoder;   // 内部 0° 参考磁编方向坐标。
    int32_t encoderSpan;   // startEncoder 到 tightEncoder 的正向跨度。
    int32_t startServoAbs; // 搜索起点舵机多圈 ABS。
    int32_t tightServoAbs; // 紧端舵机多圈 ABS。
    int32_t zeroServoAbs;  // 内部 0° 参考舵机多圈 ABS。
    float startDeg;        // LUT 内部轨迹起始角。
    float tightDeg;        // LUT 内部轨迹终止角。
    float totalDeg;        // LUT 内部轨迹总角度。
    bool scaled;           // true 表示内部轨迹可按角度插值。

    // 通用腱绳传动 LUT。单电机关节只使用第 0 行并将第 1 路电机记为
    // 0xFF；双电机关节使用两行。字段顺序和尺寸保持不变，避免破坏
    // 当前 V4 标定快照的二进制布局。
    uint8_t tendonLutValid; // 1 表示 LUT 样本与电机索引有效。
    uint8_t tendonLutCount; // 每路有效样本数。
    uint8_t tendonLutMotor[TENDON_LUT_ACTUATOR_COUNT]; // LUT 对应 Mxx。
    float tendonLutAngleStartDeg; // 第一个 LUT 样本的 URDF 角。
    float tendonLutAngleStepDeg;  // 标称步长；最后一段可小于该值并终止于 currentMaxDeg。
    int32_t tendonLutMotorAbs[TENDON_LUT_ACTUATOR_COUNT][TENDON_LUT_MAX_SAMPLES];
};

// 21 路运行期 Auto 结果和 Manual 端点表。
extern JointCalibrationResult g_jointCalibResult[ENCODER_TOTAL_NUM];
extern ManualEncoderReference g_manualEncoderReference[ENCODER_TOTAL_NUM];

// 固定安装方向表与运行期方向表。自动标定开始时会恢复对应关节的固定方向。
extern const int8_t kDefaultEncoderDirection[ENCODER_TOTAL_NUM];
extern int8_t g_encoderDirection[ENCODER_TOTAL_NUM];
bool setRuntimeEncoderDirection(uint8_t jointIndex, int8_t direction);

// 清空 RAM 中的 Auto/Manual 运行期结果；NVS 恢复由持久化模块随后执行。
void initializeCalibrationRuntimeState(void);

bool evaluateCalibrationAssemblyWarning(
    uint8_t jointIndex,
    const JointCalibrationResult& result,
    CalibrationAssemblyWarningInfo* out);
bool updateCalibrationAssemblyWarning(
    SystemRuntimeContext* sharedData,
    uint8_t jointIndex,
    const JointCalibrationResult& result);
void clearCalibrationAssemblyWarnings(SystemRuntimeContext* sharedData,
                                      uint32_t jointMask);

#endif // CALIBRATION_TASK_H
