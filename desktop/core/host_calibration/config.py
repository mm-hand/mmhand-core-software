"""config.py - MMHand Desktop 模块契约。

职责：保留原 P4 标定配置，提供上位机自动校准的原速参数。
线程：纯常量与无状态查询，不拥有线程。
输入：关节或电机索引。
输出：安装映射、标定阈值及参数。
状态所有权：流程实例拥有参数，硬件映射复用 core 唯一映射。
禁止依赖：禁止依赖 NiceGUI/UI、串口、ROS2 或 Replay。
"""

from dataclasses import dataclass

from ..motor_mapping import (
    MOTOR_TO_JOINT_INDEX,
    JOINT_TO_PRIMARY_MOTOR_INDEX,
    motor_tendon_shorten_abs_sign,
)
from ..protocol import ENCODER_COUNT, MOTOR_COUNT, TENDON_LUT_MAX_SAMPLES

# 保留固件名字，便于与 src/config/CalibrationConfig.h 逐项核对。
kFeedbackReadyServoTimeoutMs = 500
kEncoderTrustedSampleMaxAgeMs = 300
kInitialFeedbackWaitMs = 1200
kEncoderRecoveryTimeoutMs = 2000
kEncoderRecoveryStableSamples = 3
kServoRecoveryTimeoutMs = 5000
kServoRecoveryStableSamples = 3
kMinTravelDeg = 10.0
# 四指 AA 的行程会被没让位（或电机编号接错）的邻指挡住。合格件历史实测是
# URDF 量程的 56%~109%，2026-10-24 被挡住那次只有 26%。低于这个比例直接
# 判失败，不把截断的 LUT 写回。
kMinAaTravelFractionOfRange = 0.45
# AA 中点回位速度；原 100 被 min(100, servoSpeed) 压住，约 1°/s，太慢。
kAaMidpointSpeedCountsPerSec = 400
# AA 中点只要求大致到位：允许偏离中点 ±20% 关节量程。腱的跟随差 + 到达判定
# 抖动会让 ±2°/3° 级别的门限压线误杀（2026-10-25 J00 偏 3.05° 被中止）。
# 到达窗和保持门限共用这个比例，只放宽一处会在窗口边缘立刻触发另一处。
kAaMidpointToleranceFractionOfRange = 0.20
# 收紧目标接近舵机多圈上限时自动重新设零并重跑该关节。P4 越圈保护线是
# ±30715，这里留 715 counts 余量，避免目标先进保护带触发整手 STOP。
kAbsRebaseTriggerCounts = 30000
# 每次重跑重新获得一个 ±30715 counts 的窗口（约 7.5 圈）。
kAbsRebaseRetryCounts = 3
kMaxTravelDeg = 359.0
kRejectNearFullWrapSpanCounts = 15360
kLutStartDeg = -5.0
kLutZeroDeg = 0.0
kServoZeroPullCounts = 500
kDualInitialSlackCounts = 3200
kMotorAbsWindowCounts = 4096 * 5
kMotorAbsReleaseMarginCounts = 500
kMotorAbsTightMarginCounts = 900
kOverloadLoadThreshold = 800
kDualPrimaryTightLoadThreshold = 190
kDualSecondaryPreloadLoadThreshold = 190
kDualSecondaryFinalLoadThreshold = 250
kWorkspaceTightLoadThreshold = 190
kWorkspaceReleaseCounts = 3200
kWorkspaceTargetToleranceCounts = 60
kReverseMinFollowDeg = 12.0
# 上位机串口事务预算；不替代 P4 机械状态机内部的 100 ms 执行确认配置。
kHostTargetResultTimeoutMs = 1000
kHostRebaseFeedbackTimeoutMs = 3000
kTendonFitSampleStepDeg = 5.0
kSingleTendonMaxReleaseCounts = 4096 * 2
kAngleStallStableSamples = 2
kPreReleaseSyncTargetToleranceCounts = 40
kPreReleaseSyncSettleDeltaCounts = 20
kPreReleaseSyncStableSamples = 2
kTendonRawSampleCapacity = TENDON_LUT_MAX_SAMPLES * 3
kCalibrationEncoderModulo = 16384
kCalibrationEncoderHalfTurn = 8192
kFeedbackReadyEnterOutsideToleranceCounts = 24


@dataclass(frozen=True)
class AutoCalibrationParams:
    """原 defaultAutoCalibrationParams()；字段保留 P4 拼写。"""

    tightenSpeedCountsPerSec: int = 400
    servoSpeed: int = 400
    servoAcc: int = 20
    loopMs: int = 20
    windowMs: int = 200
    traceMs: int = 200
    windowServoDeltaCounts: int = 50
    zeroReachAbsTolerance: int = 200
    stallActualDeg: float = 0.75
    tightLoadHigh: int = 250


def default_auto_calibration_params(joint: int | None = None) -> AutoCalibrationParams:
    # 侧摆双舵机恢复原速：400 counts/s × 200 ms 仍约 80 counts，
    # 保留原 50-count 运动门禁；joint 参数保留调用接口。
    return AutoCalibrationParams()


def joint_motors(joint: int) -> tuple[int, ...]:
    return tuple(m for m, mapped in enumerate(MOTOR_TO_JOINT_INDEX) if mapped == joint)


def finger_aa_joint(joint: int) -> int | None:
    """Five fingers' FE/PIP/DIP depend on their AA; thumb CMC is separate."""
    return joint - joint % 4 if 0 <= joint < 20 and joint % 4 else None


def primary_motor(joint: int) -> int:
    return JOINT_TO_PRIMARY_MOTOR_INDEX[joint] if 0 <= joint < ENCODER_COUNT else -1


def shorten_sign(motor: int) -> int:
    return motor_tendon_shorten_abs_sign(motor) if 0 <= motor < MOTOR_COUNT else 0


def load_magnitude(raw: int) -> int:
    """ServoValue.h：负载方向位不参与机械阈值比较。"""
    return abs(int(raw)) & 0x03FF


def motor_apply_tendon_delta(motor: int, raw_abs: int, tendon_delta: int) -> int:
    # P4 clampCalibrationServoAbs 是恒等函数；最终硬件范围由 IO 检查。
    return int(raw_abs) + shorten_sign(motor) * int(tendon_delta)


def single_tendon_release_counts(motor: int, tight_abs: int, current_abs: int) -> int:
    return max(0, (int(tight_abs) - int(current_abs)) * shorten_sign(motor))
