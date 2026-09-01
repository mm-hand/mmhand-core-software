/**
 * @file ControlConfig.h
 * @brief 集中定义反馈控制、碰撞检测和控制完成判定使用的固定参数。
 *
 * 本文件只保存不可变配置，不保存 PID 历史状态。修改这些数值会改变机械行为，
 * 必须经过日志对比和实机回归；控制算法不得在实现文件中复制同名“临时阈值”。
 */
#ifndef MMHAND_CONTROL_CONFIG_H
#define MMHAND_CONTROL_CONFIG_H

#include <stdint.h>

namespace mmhand::config::control {

/** P4 关节反馈控制任务的固定周期，单位秒。 */
inline constexpr float kPeriodSec = 0.010f;

/** 14 bit 磁编码器一圈的原始计数。 */
inline constexpr int32_t kEncoderModulo = 16384;
inline constexpr uint16_t kEncoderDisconnectRaw = 0xFFFF;

/** 单腱 PID 调参日志开关；双腱 PID 日志已移除。 */
inline constexpr bool kPidDiagLogEnabled = true;
inline constexpr uint32_t kPidDiagLogPeriodMs = 50;

inline constexpr uint32_t kCanBusOfflineTimeoutMs = 1000;
inline constexpr uint32_t kServoTelemetryTimeoutMs = 500;

/** 直控与反馈模式的硬过载阈值；碰撞检测不使用该负载值。 */
inline constexpr int16_t kDirectOverloadLoadThreshold = 800;
inline constexpr int16_t kFeedbackOverloadLoadThreshold = 2000;
inline constexpr uint8_t kDirectOverloadLatchCycles = 20;
inline constexpr uint8_t kFeedbackOverloadLatchCycles = 20;

/** 速度 0 表示使用舵机固件默认速度，适合连续反馈控制。 */
inline constexpr uint16_t kFeedbackServoSpeed = 0;

inline constexpr int32_t kDirectDoneAbsToleranceCounts = 40;
inline constexpr uint8_t kDirectDoneStableCycles = 5;
inline constexpr float kFeedbackDoneToleranceDeg = 0.8f;
inline constexpr uint32_t kFeedbackDoneStableMs = 500;
inline constexpr uint32_t kFeedbackRunTimeoutMs = 8000;

inline constexpr int32_t kUpperWorkspaceWarningCounts = 48;
inline constexpr uint8_t kUpperWorkspaceWarningStableFrames = 3;

/**
 * 单腱关节角度 PID。2026-07-19 实测弹簧紧绳方向约有 1.5 度静差；
 * D 项对低通后的磁编反馈速度提供阻尼，避免遥操目标阶跃产生微分冲击。
 */
inline constexpr float kSingleTendonKp = 5.7f;
inline constexpr float kSingleTendonKi = 0.0f;
inline constexpr float kSingleTendonKd = 0.10f;
inline constexpr float kSingleTendonDeadbandDeg = 0.0f;
inline constexpr float kSingleTendonIntegralLimit = 100.0f;
/** 有效 LUT 单腱在反馈完成阈值内仍保持的固定比例增益，用于连续跟随。 */
inline constexpr float kSingleTendonNearTargetKp = 20.0f;
/** D 项使用低通后的磁编反馈速度，避免遥操目标阶跃产生微分冲击。 */
inline constexpr float kSingleTendonVelocityFilterTauSec = 0.050f;
inline constexpr float kSingleTendonDTermLimitCounts = 20.0f;

/**
 * 单腱双向增益调度：紧绳近端使用 1.2 倍局部斜率，松绳近端保持
 * 0.7 倍局部斜率；大误差统一按 0.7 倍平均斜率推进。
 */
inline constexpr float kSingleTendonDynamicKpErrorMaxDeg = 3.0f;
inline constexpr float kSingleTendonDynamicKpBlendStartDeg = 2.5f;
inline constexpr float kSingleTendonDynamicLutGain = 1.0f;
inline constexpr float kSingleTendonLargeErrorLutGain = 0.75f;
/** 单腱目标速度前馈：预测未来目标并通过本关节 LUT 转换为腱绳 counts。 */
inline constexpr bool kSingleTendonVelocityFeedforwardEnabled = true;
inline constexpr float kSingleTendonVelocityLeadTimeSec = 0.070f;
inline constexpr float kSingleTendonTargetVelocityFilterTauSec = 0.040f;
inline constexpr float kSingleTendonVelocityLeadMaxDeg = 3.0f;
/**
 * 单腱换向连续补绳：目标沿反方向累计确认后立即取绳，并以真实控制 dt 持续推进。
 * 只有关节沿新方向累计响应到退出距离才撤销补偿；最大 counts 仅防异常无限收绳。
 */
inline constexpr bool kSingleTendonDirectionalTakeupEnabled = true;
inline constexpr float kSingleTendonDirectionalTakeupConfirmDeg = 1.0f;
inline constexpr int32_t kSingleTendonDirectionalTakeupInitialCounts = 60;
inline constexpr float kSingleTendonDirectionalTakeupRateCountsPerSec = 5000.0f;
inline constexpr float kSingleTendonDirectionalTakeupExitJointTravelDeg = 0.25f;
inline constexpr int32_t kSingleTendonDirectionalTakeupMaxCounts = 140;
static_assert(kSingleTendonLargeErrorLutGain > 0.0f &&
                  kSingleTendonLargeErrorLutGain <= 1.0f,
              "single tendon large-error LUT gain must be in (0, 1]");
static_assert(kFeedbackDoneToleranceDeg <
                  kSingleTendonDynamicKpBlendStartDeg &&
                  kSingleTendonDynamicKpBlendStartDeg <
                  kSingleTendonDynamicKpErrorMaxDeg,
              "single tendon dynamic Kp error ranges must be ordered");
static_assert(kSingleTendonDirectionalTakeupConfirmDeg > 0.0f &&
                  kSingleTendonDirectionalTakeupInitialCounts >= 0 &&
                  kSingleTendonDirectionalTakeupRateCountsPerSec > 0.0f &&
                  kSingleTendonDirectionalTakeupExitJointTravelDeg > 0.0f &&
                  kSingleTendonDirectionalTakeupMaxCounts >=
                      kSingleTendonDirectionalTakeupInitialCounts,
              "single tendon directional takeup parameters must be valid");
static_assert(kSingleTendonVelocityLeadTimeSec >= 0.0f &&
                  kSingleTendonTargetVelocityFilterTauSec > 0.0f &&
                  kSingleTendonVelocityLeadMaxDeg >= 0.0f,
              "single tendon velocity feedforward parameters must be valid");

/** 双腱位置环参数。误差和输出均在标定 LUT 的舵机 counts 坐标中计算。 */
inline constexpr float kDualTendonKp = 0.60f;
inline constexpr float kDualTendonKi = 0.0f;
inline constexpr float kDualTendonKd = 0.0f;
inline constexpr float kDualTendonIntegralLimit = 0.0f;
inline constexpr float kDualTendonDTermLimitCounts = 100.0f;
inline constexpr float kDualTendonVelocityFilterTauSec = 0.050f;
/**
 * 单腱和双腱闭环允许越过 LUT 端点的运行裕量，单位为 logical ABS counts。
 * 裕量按腱绳方向而非 ABS 数值正负计算，兼容不同电机方向映射。
 */
inline constexpr int32_t kTendonLutReleaseMarginCounts = 500;
inline constexpr int32_t kTendonLutTightMarginCounts = 1000;
static_assert(kTendonLutReleaseMarginCounts >= 0 &&
                  kTendonLutTightMarginCounts >= 0,
              "tendon LUT margins must be non-negative");

/** 碰撞检测状态机参数。 */
inline constexpr float kCollisionMinErrorDeg = 4.0f;
inline constexpr float kCollisionTargetResetDeg = 1.0f;
inline constexpr uint32_t kCollisionVelocityWindowMs = 200;
inline constexpr float kCollisionArmVelocityDegPerSec = 8.0f;
// 所有舵机 ABS 增加均代表收绳。放绳侧保留较快的停滞判定，
// 收绳侧给弹簧、摩擦与抓取形变更长建立时间，降低正常收紧时的误判。
inline constexpr uint32_t kCollisionReleaseProgressWindowMs = 600;
inline constexpr float kCollisionReleaseMinProgressDeg = 0.4f;
inline constexpr uint32_t kCollisionTightenProgressWindowMs = 750;
inline constexpr float kCollisionTightenMinProgressDeg = 0.5f;
inline constexpr float kCollisionTargetRetreatReleaseDeg = 1.0f;

} // namespace mmhand::config::control

#endif // MMHAND_CONTROL_CONFIG_H
