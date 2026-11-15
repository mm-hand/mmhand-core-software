"""sweep.py - 端点搜索:把关节拉到一端,用绝对角度判断到没到。

职责：唯一的关节运动原语。双腱、单腱和邻指让位都走这一条路径。
线程：由标定工作线程串行调用；不创建线程。
输入：校准 IO、关节、目标端点、拉/放电机。
输出：端点采样（角度/编码器/两个电机 ABS）或结构化诊断。
状态所有权：只拥有本次轨迹的局部命令余量。
禁止依赖：禁止依赖 NiceGUI/UI、串口、ROS2 或 Replay。

判据只有两条,缺一不可:
  * 关节角度冻结（连续 kEndStillWindows 个窗口内变化 ≤ kEndStillDeg）
  * 冻结位置离命令端点 ≤ kEndToleranceDeg
负载只用于"腱已经绷紧,可以把对侧腱放出来"这一个用途,从不判断端点。
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

from .. import protocol
from . import config
from .math import forward_encoder_delta

END_MIN, END_MAX = 0, 1


@dataclass(frozen=True)
class EndPoint:
    angle: float
    encoder: int
    primary_abs: int | None
    secondary_abs: int | None
    load: int


@dataclass(frozen=True)
class SweepOutcome:
    point: EndPoint | None = None
    reason: int = protocol.CALIB_DIAG_REASON_NONE


def _motor_abs(io, motor):
    return None if motor is None else io.servo_abs(motor)


def _sample_abs(io, motor):
    # 单腱关节没有副腱列；LUT 采样用 0 占位（None 会让斜率过滤崩掉）。
    return 0 if motor is None else (io.servo_abs(motor) or 0)


def _fail(io, reason, phase, joint, motor, a=0, b=0, c=0, bitmap=0):
    emit = getattr(io, 'diag_if_empty', None) or io.diag
    emit(reason, phase, joint, motor, a, b, c, bitmap)
    return SweepOutcome(None, reason)


def sweep_to_end(io, joint, end, *, motors, pull, release,
                 fit=None, hold=None, params=None):
    """Drive `joint` to one URDF endpoint and return where it stopped.

    motors:  (primary, secondary) for a dual joint, (motor,) for a single tendon.
    pull:    motor tightened toward the endpoint, or None for a pure release
             (single tendon returning on its spring).
    release: antagonist paid out once `pull` is taut, or None.
    fit:     optional TendonRawFitBuffer; sampled every kTendonFitSampleStepDeg.
    hold:    optional callable returning extra targets for other joints.
    """
    p = params or config.default_auto_calibration_params()
    at_max = int(end) == END_MAX
    phase = protocol.CALIB_DIAG_PHASE_FIND_MAX if at_max else protocol.CALIB_DIAG_PHASE_FIND_MIN
    target = float(io.urdf_limits(joint)[1 if at_max else 0])
    primary = int(motors[0])
    secondary = int(motors[1]) if len(motors) > 1 else None
    bitmap = sum(1 << m for m in (pull, release) if m is not None)
    motor = pull if pull is not None else (release if release is not None else 255)

    manual = io.manual_reference(joint)
    span = forward_encoder_delta(int(manual['minRaw']), int(manual['maxRaw']))
    max_wind = int(config.kSlackRunFactor * span) if span > 0 else 0

    speed = max(1, int(p.tightenSpeedCountsPerSec))
    start_angle = io.angle(joint)
    p_cmd = io.servo_abs(pull) if pull is not None else None
    r_cmd = io.servo_abs(release) if release is not None else None
    start_r_abs = r_cmd
    missing = (start_angle is None
               or (pull is not None and p_cmd is None)
               or (release is not None and r_cmd is None))
    if missing:
        return _fail(io, protocol.CALIB_DIAG_REASON_FEEDBACK_LOST,
                     phase, joint, motor, bitmap=bitmap)

    still = 0
    wound = 0
    taut = pull is None
    recent = deque(maxlen=config.kEndStillWindows)
    remainders = 0
    last_command = io.now_ms()
    window_ms = io.now_ms()
    io.trace("END_BEGIN", joint, motor, end="MAX" if at_max else "MIN",
             target_deg=target, start_deg=round(start_angle, 3))

    while io.owns_control():
        # 步长按真实经过时间折算：publish 的执行确认会拉长循环，
        # 固定步长会把 400 counts/s 拖成 ~80 counts/s。
        now = io.now_ms()
        counts = 0
        if now - last_command >= p.loopMs:
            counts, remainders = divmod(
                (now - last_command) * speed + remainders, 1000)
            last_command = now
        targets = {}
        if pull is not None:
            if counts:
                p_cmd = config.motor_apply_tendon_delta(pull, p_cmd, counts)
                wound += counts
            targets[pull] = p_cmd
        if release is not None:
            if taut and counts:
                r_cmd = config.motor_apply_tendon_delta(release, r_cmd, -counts)
                released = config.single_tendon_release_counts(release, start_r_abs, r_cmd)
                if released > config.kSingleTendonMaxReleaseCounts:
                    # 纯释放时"放了两圈还没回来"是弹簧回位不足，不是腱断了。
                    reason = (protocol.CALIB_DIAG_REASON_WEAK_RETURN if pull is None
                              else protocol.CALIB_DIAG_REASON_RELEASE_OVER_TWO_TURNS)
                    return _fail(io, reason, phase, joint, release, released,
                                 config.kSingleTendonMaxReleaseCounts, released, bitmap)
            targets[release] = r_cmd
        if hold is not None:
            extra = hold()
            if extra is None:
                return _fail(io, protocol.CALIB_DIAG_REASON_HOLD_TARGET_FAILED,
                             phase, joint, motor, bitmap=bitmap)
            targets.update(extra)
        if not io.publish(targets, p.servoSpeed, p.servoAcc):
            return _fail(io, protocol.CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED,
                         phase, joint, motor, bitmap=bitmap)

        angle = io.angle(joint)
        encoder = io.encoder_raw(joint)
        load_raw = io.servo_load(pull) if pull is not None else None
        if angle is None or encoder is None or (pull is not None and load_raw is None):
            return _fail(io, protocol.CALIB_DIAG_REASON_FEEDBACK_LOST,
                         phase, joint, motor, bitmap=bitmap)
        load = config.load_magnitude(load_raw) if load_raw is not None else 0
        engaged = load >= config.kTautLoadCounts or abs(angle - start_angle) > config.kEndToleranceDeg
        if pull is not None and not taut and engaged:
            # 腱已绷紧(或关节已经被带着走了):开始放对侧腱。
            # 重新给冻结计数,给关节留出 1 秒破静摩擦的时间。
            taut = True
            still = 0
            recent.clear()
            window_ms = now

        if now - window_ms >= p.windowMs:
            recent.append(angle)
            # 单窗口的角度变化会被编码器抖动(±0.11°)盖住；看最近几个窗口的总漂移，
            # 慢速爬升的关节不会被误判成"停住"。
            moved = (max(recent) - min(recent)) if len(recent) > 1 else 0.0
            if moved > config.kEndStillDeg:
                still = 0
                wound = 0
                if fit is not None:
                    fit.maybe_append(abs(angle - start_angle), encoder,
                                     _sample_abs(io, primary), _sample_abs(io, secondary))
            else:
                still += 1
            window_ms = now
            error = abs(angle - target)
            near = error <= config.kEndToleranceDeg
            # 纯释放(单腱弹簧回位)没有"拉到某个角度"的能力：腱完全松开后
            # 关节停在哪，哪就是 Min。只要求它确实被弹回来过，避免起步就误判。
            returned = abs(angle - start_angle) >= config.kEndToleranceDeg
            io.trace("END_CHECK", joint, motor, angle=round(angle, 3), target_deg=target,
                     error=round(error, 3), moved=round(moved, 3),
                     still=still, near=near, load=load, taut=taut)
            point = EndPoint(angle, int(encoder), _motor_abs(io, primary),
                             _motor_abs(io, secondary), load)
            if still >= config.kEndStillWindows and (near if pull is not None else returned):
                io.trace("END_REACHED", joint, motor, angle=round(angle, 3),
                         encoder=int(encoder), error=round(error, 3))
                return SweepOutcome(point)
            # 纯释放（单腱弹簧回位）没有"卡住"判据：弹簧慢一些不该误报，
            # 由两圈上限兜底。只有被主动拉向端点时才判卡住。
            if pull is not None and still >= config.kStuckWindows and taut and not near:
                io.trace("END_STUCK", joint, motor, angle=round(angle, 3),
                         target_deg=target, error=round(error, 3))
                return _fail(io, protocol.CALIB_DIAG_REASON_END_NOT_REACHED,
                             phase, joint, motor, int(round(angle * 1000)),
                             int(round(error * 1000)), int(encoder), bitmap)
            if pull is not None and not taut and max_wind and wound > max_wind:
                io.trace("END_SLACK", joint, motor, wound=wound, manual_span=span)
                return _fail(io, protocol.CALIB_DIAG_REASON_SLACK_TOO_LARGE,
                             phase, joint, motor, wound, span, int(encoder), bitmap)
        io.sleep_ms(p.loopMs)

    return _fail(io, protocol.CALIB_DIAG_REASON_CONTROL_LOST, phase, joint, motor,
                 bitmap=bitmap)
