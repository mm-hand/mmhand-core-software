"""config.py - MMHand Desktop 模块契约。

职责：标定阈值、安装映射与运动参数。
线程：纯常量与无状态查询，不拥有线程。
输入：关节或电机索引。
输出：安装映射、标定阈值及参数。
状态所有权：流程实例拥有参数，硬件映射复用 core 唯一映射。
禁止依赖：禁止依赖 NiceGUI/UI、串口、ROS2 或 Replay。

The calibration judges every mechanical endpoint by the absolute joint angle
(encoder raw mapped through the Manual endpoints). Load is only a "the tendon
is taut" signal. There is deliberately no travel threshold and no fraction of
the URDF range: both were proxies that could not tell "tight" from "pulling
slack", which is exactly how J00 was mis-calibrated on 2026-11-15.
"""

from dataclasses import dataclass

from ..motor_mapping import (
    MOTOR_TO_JOINT_INDEX,
    JOINT_TO_PRIMARY_MOTOR_INDEX,
    motor_tendon_shorten_abs_sign,
)
from ..protocol import ENCODER_COUNT, MOTOR_COUNT, TENDON_LUT_MAX_SAMPLES

from ..generated_control_parameters import DEFAULT_OUTPUT_MARGINS

# 反馈与恢复。
kFeedbackReadyServoTimeoutMs = 500
kEncoderTrustedSampleMaxAgeMs = 300
kEncoderRecoveryTimeoutMs = 2000
kEncoderRecoveryStableSamples = 3
kServoRecoveryTimeoutMs = 5000
kServoRecoveryStableSamples = 3

# 端点搜索。唯一判据是"角度冻结 + 离目标端点足够近"。
kTautLoadCounts = 190            # 腱绷紧；只用来决定何时放对侧腱
kEndStillDeg = 0.3               # 每 windowMs 的角度冻结阈值（编码器抖动约 ±0.11°）
kEndStillWindows = 3             # 连续冻结窗口数 → 到达端点
kStuckWindows = 5                # 连续冻结窗口数 → 卡住（绷紧却不在端点）
kEndToleranceDeg = 8.0           # 距目标端点容差（约 350 counts，与装配告警同量级）
kSlackRunFactor = 1.5            # 未绷紧却卷动 > 1.5×Manual 行程 → 空行程异常

# 结果与 LUT。
kMaxTravelDeg = 359.0
kRejectNearFullWrapSpanCounts = 15360
kCalibrationEncoderHalfTurn = 8192
kCalibrationEncoderModulo = 16384
kLutStartDeg = -5.0
kLutZeroDeg = 0.0
kServoZeroPullCounts = 500
kTendonFitSampleStepDeg = 5.0
kTendonRawSampleCapacity = TENDON_LUT_MAX_SAMPLES * 3
# 相邻两点的斜率超过全表中位斜率的倍数，判为"关节已到底、腱还在走"的坏段。
# LUT 上传用 int16 紧凑格式，一个坏点会把整表顶出范围。
kMaxSegmentSlopeRatio = 8.0

# ABS 安全与单腱释放上限。
# 收紧目标接近舵机多圈上限时自动重新设零并重跑该关节。P4 越圈保护线是
# ±30715，这里留 715 counts 余量，避免目标先进保护带触发整手 STOP。
kAbsRebaseTriggerCounts = 30000
kAbsRebaseRetryCounts = 3
kSingleTendonMaxReleaseCounts = 4096 * 2

# AA 依赖：摆到中点时只求大致到位。
kAaMidpointSpeedCountsPerSec = 400
kAaMidpointToleranceDeg = 15.0

# 上位机串口事务预算；不替代 P4 机械状态机内部的 100 ms 执行确认配置。
kHostTargetResultTimeoutMs = 1000
kHostRebaseFeedbackTimeoutMs = 3000


def calibration_margins(io):
    """Production IO freezes applied parameters; pure algorithm fixtures use generated defaults."""
    getter = getattr(io, 'lut_margins', None)
    return getter() if callable(getter) else DEFAULT_OUTPUT_MARGINS


@dataclass(frozen=True)
class AutoCalibrationParams:
    """端点搜索的运动参数。"""

    tightenSpeedCountsPerSec: int = 400
    servoSpeed: int = 400
    servoAcc: int = 20
    loopMs: int = 20
    windowMs: int = 200


def default_auto_calibration_params(joint: int | None = None) -> AutoCalibrationParams:
    # 侧摆双舵机恢复原速：400 counts/s × 200 ms 仍约 80 counts。
    # joint 参数保留调用接口。
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
