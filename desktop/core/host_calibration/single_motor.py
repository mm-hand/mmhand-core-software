"""single_motor.py - MMHand Desktop 模块契约。

职责：逐分支移植 P4 SingleMotorCalibrationMachine.cpp 的单腱显式状态机。
线程：由校准流程的工作线程串行运行；不创建线程。
输入：校准 IO 的当前反馈、原 AutoCalibrationParams 和选中关节序列。
输出：完成/失败/中止结果、原 V4 LUT 和本地 ABS 限位。
状态所有权：本模块独占单关节状态；结果由总流程按旧协议提交一次。
禁止依赖：禁止依赖 Qt/UI、串口实现、ROS2 或 Replay。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum

from .. import protocol as p
from .config import (
    AutoCalibrationParams, ENCODER_COUNT, MOTOR_COUNT,
    primary_motor, shorten_sign, default_encoder_direction, load_magnitude,
    motor_apply_tendon_delta, single_tendon_release_counts,
    kInitialFeedbackWaitMs, kSingleTendonMaxReleaseCounts,
    kReverseMinFollowDeg, kAngleStallStableSamples,
    kPreReleaseSyncTargetToleranceCounts, kPreReleaseSyncSettleDeltaCounts,
    kPreReleaseSyncStableSamples, kTendonFitSampleStepDeg,
    kLutStartDeg, kLutZeroDeg, kServoZeroPullCounts,
    kRejectNearFullWrapSpanCounts, kMotorAbsReleaseMarginCounts,
    kMotorAbsTightMarginCounts,
)
from .math import (
    TendonRawFitBuffer, encoder_count_to_deg, wrap_encoder_delta,
    forward_encoder_delta, wrap_encoder_count, encoder_for_positive_travel,
    signed_encoder_delta, round_to_int32, f32,
    build_calibration_result, build_tendon_lut, tendon_motor_abs_range,
    tendon_motor_abs_limits,
)
from .time_alignment import TimeAlignmentError


class AutoCalibState(IntEnum):
    IDLE = 0
    INIT = 1
    TIGHTEN = 2
    PRE_RELEASE_SYNC = 3
    DONE = 4
    FAILED = 5


def _elapsed(now: int, before: int) -> int:
    return (int(now) - int(before)) & 0xFFFFFFFF


@dataclass
class JointCalibContext:
    joint: int
    state: AutoCalibState = AutoCalibState.INIT
    motor: int = 255
    direction_sign: int = 0
    encoder_travel_sign: int = 0
    command_speed: int = 0
    command_acc: int = 0
    state_begin_ms: int = 0
    last_command_ms: int = 0
    last_trace_ms: int = 0
    command_abs: int = 0
    command_remainder: int = 0
    target_abs: int = 0
    baseline_load: int = 0
    start_found: bool = False
    start_search_raw: int = 0
    start_search_encoder: int = 0
    start_search_abs: int = 0
    start_encoder: int = 0
    start_servo_abs: int = 0
    zero_encoder: int = 0
    zero_servo_abs: int = 0
    window_start_encoder: int = 0
    window_start_abs: int = 0
    window_start_ms: int = 0
    init_started: bool = False
    release_command_started: bool = False
    pre_release_sync_target_abs: int = 0
    pre_release_sync_window_start_abs: int = 0
    pre_release_sync_window_start_ms: int = 0
    pre_release_sync_stable_samples: int = 0
    release_angle_stall_stable: int = 0
    fit_samples: TendonRawFitBuffer = field(default_factory=TendonRawFitBuffer)
    lut_sampling_window: object | None = None
    result: dict | None = None
    limits: dict[int, tuple[int, int]] = field(default_factory=dict)
    reason: str = ""


def _diag_if_empty(io, *values):
    getattr(io, "diag_if_empty", io.diag)(*values)


def _fail(ctx, io, text, reason, phase, motor=None, a=0, b=0, c=0, bitmap=0, if_empty=False):
    if ctx.state == AutoCalibState.FAILED:
        return
    values = (reason, phase, ctx.joint, ctx.motor if motor is None else motor, a, b, c, bitmap)
    if if_empty:
        _diag_if_empty(io, *values)
    else:
        io.diag(*values)
    ctx.reason = text
    ctx.state = AutoCalibState.FAILED
    io.trace("FAIL", ctx.joint, ctx.motor, reason=text, diag_reason=reason, phase=phase)


def _feedback_lost(ctx, io, text, phase):
    joint_bit = 1 << ctx.joint if 0 <= ctx.joint < ENCODER_COUNT else 0
    motor_bit = 1 << ctx.motor if 0 <= ctx.motor < MOTOR_COUNT else 0
    encoder_ok = io.encoder(ctx.joint) is not None and io.encoder_raw(ctx.joint) is not None
    _fail(ctx, io, text, p.CALIB_DIAG_REASON_FEEDBACK_LOST, phase,
          a=joint_bit if encoder_ok else 0,
          b=motor_bit if io.servo_abs(ctx.motor) is not None else 0,
          c=motor_bit if io.servo_raw(ctx.motor) is not None else 0,
          bitmap=motor_bit)


def _append(ctx, io, targets, value, phase, text):
    if not 0 <= ctx.motor < MOTOR_COUNT:
        _fail(ctx, io, text, p.CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED, phase,
              a=value, c=-1, bitmap=1 << ctx.motor if ctx.motor < MOTOR_COUNT else 0)
        return False
    targets[ctx.motor] = int(value)
    return True


def _initialize_tighten(ctx, io, params):
    raw, encoder = io.encoder_raw(ctx.joint), io.encoder(ctx.joint)
    actual = io.servo_abs(ctx.motor)
    if raw is None or encoder is None or actual is None:
        return False
    load = io.servo_load(ctx.motor)
    ctx.baseline_load = load_magnitude(load) if load is not None else 0
    ctx.start_search_raw, ctx.start_search_encoder, ctx.start_search_abs = raw, encoder, actual
    ctx.start_found = False
    ctx.start_encoder = ctx.start_servo_abs = ctx.zero_encoder = ctx.zero_servo_abs = 0
    ctx.encoder_travel_sign = 1
    ctx.window_start_encoder, ctx.window_start_abs = encoder, actual
    ctx.window_start_ms = io.now_ms()
    ctx.command_abs, ctx.command_remainder = actual, 0
    ctx.last_command_ms, ctx.state_begin_ms, ctx.last_trace_ms = io.now_ms(), io.now_ms(), 0
    ctx.state = AutoCalibState.TIGHTEN
    io.trace("AUTO_SAMPLE_RANGE_BEGIN", ctx.joint, ctx.motor, start_deg=kLutStartDeg)
    io.trace("START", ctx.joint, ctx.motor, abs=actual, raw=raw, oriented=encoder,
             direction=io.direction(ctx.joint), baseline_load=ctx.baseline_load,
             speed_cps=params.tightenSpeedCountsPerSec)
    return True


def _step_counts(ctx, now, params):
    numerator = _elapsed(now, ctx.last_command_ms) * params.tightenSpeedCountsPerSec + ctx.command_remainder
    # 原 C++ numerator 在正常周期/配置为非负；int32 范围由固定窗口约束。
    counts = numerator // 1000
    ctx.command_remainder = numerator % 1000
    return counts


def _inside_limit(ctx, io, targets, phase, text):
    if io.inside_limit(ctx.motor, ctx.command_abs):
        return True
    limits = io.logical_range(ctx.motor)
    lo, hi = limits if limits is not None else (0, 0)
    targets[ctx.motor] = ctx.command_abs
    _fail(ctx, io, text, p.CALIB_DIAG_REASON_ABS_LIMIT_INVALID, phase,
          a=ctx.command_abs, b=lo, c=hi)
    return False


def _single_lut_limit(result, motor):
    # Workspace comes from the fitted LUT line plus the configurable extension.
    # A measured "tight" position is deliberately not used: tube bending changes
    # the tendon path, so that measurement is only valid at the calibration pose.
    return tendon_motor_abs_limits(
        result, 0, motor,
        kMotorAbsReleaseMarginCounts, kMotorAbsTightMarginCounts,
    )


def _finish_release(ctx, io, params, targets, current_raw, encoder, actual,
                    load_mag, release_deg, window_deg, now):
    ctx.zero_encoder, ctx.zero_servo_abs = encoder, actual
    signed_tight_to_max = signed_encoder_delta(ctx.zero_encoder, ctx.start_encoder)
    ctx.encoder_travel_sign = 1
    max_encoder, max_abs = ctx.start_encoder, ctx.start_servo_abs
    release_encoder, release_abs = ctx.zero_encoder, ctx.zero_servo_abs
    if max_abs == release_abs:
        _fail(ctx, io, "zero find has no servo travel", p.CALIB_DIAG_REASON_RESULT_SPAN_INVALID,
              p.CALIB_DIAG_PHASE_RELEASE, a=0, b=release_abs, c=max_abs,
              bitmap=1 << ctx.motor, if_empty=True)
        return
    taut_encoder = encoder_for_positive_travel(release_encoder, ctx.encoder_travel_sign)
    tight_encoder = encoder_for_positive_travel(max_encoder, ctx.encoder_travel_sign)
    span = forward_encoder_delta(taut_encoder, tight_encoder)
    total_deg = encoder_count_to_deg(span)
    zero_fraction = f32(f32(kLutZeroDeg - kLutStartDeg) / total_deg) if total_deg > 0 else 0.0
    zero_encoder = wrap_encoder_count(taut_encoder + round_to_int32(f32(f32(span) * zero_fraction)))
    zero_abs = motor_apply_tendon_delta(ctx.motor, release_abs, kServoZeroPullCounts)
    if ctx.lut_sampling_window is not None:
        try:
            # Replace the whole buffer: unsynchronised mechanical endpoints
            # must never be appended to the common-time interpolation results.
            ctx.fit_samples = io.finish_lut_sampling(ctx.lut_sampling_window)
        except TimeAlignmentError as exc:
            targets[ctx.motor] = actual
            _fail(ctx, io, str(exc), p.CALIB_DIAG_REASON_RESULT_BUILD_FAILED,
                  p.CALIB_DIAG_PHASE_RELEASE, bitmap=1 << ctx.motor)
            return
    else:
        # Algorithm-only fake ports can still exercise the original machine.
        tight_stored = ctx.fit_samples.append(max_encoder, max_abs, 0)
        release_stored = ctx.fit_samples.append(release_encoder, release_abs, 0)
        if not tight_stored or not release_stored:
            _fail(ctx, io, "single tendon LUT endpoint buffer full", p.CALIB_DIAG_REASON_RESULT_BUILD_FAILED,
                  p.CALIB_DIAG_PHASE_RELEASE, a=ctx.fit_samples.count, b=max_abs, c=release_abs)
            return
    result = build_calibration_result(io, ctx.joint, taut_encoder, zero_encoder, tight_encoder,
                                      release_abs, zero_abs, max_abs)
    if result is None:
        _fail(ctx, io, "invalid max/release result", p.CALIB_DIAG_REASON_ZERO_CROSS_INVALID,
              p.CALIB_DIAG_PHASE_RELEASE, a=taut_encoder, b=zero_encoder, c=tight_encoder)
        return
    if not build_tendon_lut(io, ctx.joint, result, ctx.motor, 255, 1, ctx.fit_samples):
        _fail(ctx, io, "single tendon LUT build failed", p.CALIB_DIAG_REASON_RESULT_BUILD_FAILED,
              p.CALIB_DIAG_PHASE_RELEASE, a=ctx.fit_samples.count,
              b=result["encoderSpan"], c=result.get("tendonLutCount", 0))
        return
    envelope = tendon_motor_abs_range(result, 0)
    if result["tendonLutMotor"][0] != ctx.motor or envelope is None:
        _fail(ctx, io, "single tendon LUT range invalid", p.CALIB_DIAG_REASON_RESULT_BUILD_FAILED,
              p.CALIB_DIAG_PHASE_RELEASE, a=result["tendonLutCount"],
              b=result["tendonLutMotor"][0], c=0, bitmap=0)
        return
    io.trace("SINGLE_LUT_ENDPOINT", ctx.joint, ctx.motor, release_abs=release_abs, tight_abs=max_abs,
             lut=(result["tendonLutMotorAbs"][0][0],
                  result["tendonLutMotorAbs"][0][result["tendonLutCount"] - 1]), samples=ctx.fit_samples.count)
    if result["encoderSpan"] >= kRejectNearFullWrapSpanCounts:
        _fail(ctx, io, "rejected wrap-like span", p.CALIB_DIAG_REASON_WRAP_TOO_LARGE,
              p.CALIB_DIAG_PHASE_RELEASE, a=result["encoderSpan"], b=release_encoder, c=max_encoder)
        return
    io.trace("ENCODER_RESULT", ctx.joint, ctx.motor,
             default_direction=default_encoder_direction(ctx.joint), runtime_direction=io.direction(ctx.joint),
             tight_to_max=signed_tight_to_max, result_taut=taut_encoder,
             result_zero=zero_encoder, result_tight=tight_encoder)
    io.trace("SINGLE_LUT", ctx.joint, ctx.motor, raw_samples=ctx.fit_samples.count,
             lut_samples=result["tendonLutCount"], start_deg=result["tendonLutAngleStartDeg"],
             step_deg=result["tendonLutAngleStepDeg"])
    # 固件的 commitCalibrationResult / commitJointServoMultiTurnTrust 由总流程
    # 原协议写回完成；此处只保留机械结果，F0 已在搜索前重建多圈可信。
    ctx.result, ctx.target_abs = result, actual
    limit = _single_lut_limit(result, ctx.motor)
    if limit is not None:
        ctx.limits[ctx.motor] = limit
        io.set_limit(ctx.motor, *limit)
    ctx.state, ctx.state_begin_ms, ctx.last_trace_ms = AutoCalibState.DONE, now, now
    io.trace("TIGHT_FOUND", ctx.joint, ctx.motor, start_deg=result["startDeg"], raw=current_raw,
             encoder=release_encoder, abs=release_abs, load=load_mag,
             release_deg=release_deg, window_deg=window_deg)
    io.trace("AUTO_SAMPLE_RANGE", ctx.joint, ctx.motor, start_deg=result["startDeg"],
             zero_deg=0.0, tight_deg=result["tightDeg"], total_deg=result["totalDeg"],
             urdf_max=result["angleMax"], span=result["encoderSpan"])
    io.trace("ZERO_FOUND", ctx.joint, ctx.motor, encoder=result["zeroEncoder"],
             servo_zero=result["zeroServoAbs"], pull_counts=kServoZeroPullCounts,
             shorten_sign=ctx.direction_sign)
    io.trace("RESULT", ctx.joint, ctx.motor, result=dict(result))
    io.trace("SINGLE_DONE_AT_AUTO_MIN", ctx.joint, ctx.motor, hold_abs=ctx.target_abs,
             urdf_min_deg=result.get("angleMin"))
    _append(ctx, io, targets, ctx.target_abs, p.CALIB_DIAG_PHASE_RELEASE, "auto-min hold target append failed")


def step_single(ctx: JointCalibContext, io, params: AutoCalibrationParams,
                targets: dict[int, int]) -> None:
    """原 stepAutoCalibrationJoint；仅推进一帧，不等待、不发布总批次。"""
    if ctx.state in (AutoCalibState.DONE, AutoCalibState.FAILED):
        return
    now = io.now_ms()
    if not io.owns_control():
        _fail(ctx, io, "control lost", p.CALIB_DIAG_REASON_CONTROL_LOST,
              p.CALIB_DIAG_PHASE_SEQUENCE, if_empty=True)
        return
    if ctx.state == AutoCalibState.INIT:
        if not 0 <= ctx.joint < ENCODER_COUNT:
            _fail(ctx, io, "invalid init or owner", p.CALIB_DIAG_REASON_CONTROL_LOST,
                  p.CALIB_DIAG_PHASE_INIT, motor=255, if_empty=True)
            return
        ctx.command_speed, ctx.command_acc = params.servoSpeed, params.servoAcc
        motor = primary_motor(ctx.joint)
        if not 0 <= motor < MOTOR_COUNT:
            _fail(ctx, io, "primary motor channel not mapped", p.CALIB_DIAG_REASON_MOTOR_MAPPING_INVALID,
                  p.CALIB_DIAG_PHASE_INIT, motor=255, a=ctx.joint, b=motor, c=-1, if_empty=True)
            return
        ctx.motor, ctx.direction_sign = motor, shorten_sign(motor)
        if not ctx.init_started:
            ctx.init_started = True
            ctx.state_begin_ms, ctx.last_command_ms, ctx.last_trace_ms = now, 0, 0
            if hasattr(io, "set_direction"):
                io.set_direction(ctx.joint, default_encoder_direction(ctx.joint))
            io.clear_limits(ctx.joint)
        unknown_mask = int(getattr(io, "multi_turn_unknown_bitmap", 0))
        if unknown_mask & (1 << ctx.motor):
            io.trace("MULTITURN_UNKNOWN_REBUILD", ctx.joint, ctx.motor, unknown_bitmap=unknown_mask)
        raw, encoder = io.encoder_raw(ctx.joint), io.encoder(ctx.joint)
        actual, servo_raw = io.servo_abs(ctx.motor), io.servo_raw(ctx.motor)
        encoder_ok, abs_ok, raw_ok = raw is not None and encoder is not None, actual is not None, servo_raw is not None
        if not encoder_ok or not abs_ok or not raw_ok:
            if _elapsed(now, ctx.last_trace_ms) >= params.traceMs:
                ctx.last_trace_ms = now
                io.trace("INITIAL_FEEDBACK", ctx.joint, ctx.motor, encoder_ok=encoder_ok, abs_ok=abs_ok, raw_ok=raw_ok)
            if _elapsed(now, ctx.state_begin_ms) < kInitialFeedbackWaitMs:
                return
            io.trace("INITIAL_FEEDBACK", ctx.joint, ctx.motor, encoder_ok=encoder_ok, abs_ok=abs_ok, raw_ok=raw_ok)
            _fail(ctx, io, "missing initial encoder/servo feedback after wait",
                  p.CALIB_DIAG_REASON_ENCODER_ERROR if not encoder_ok else p.CALIB_DIAG_REASON_SERVO_OFFLINE,
                  p.CALIB_DIAG_PHASE_INIT, a=int(encoder_ok), b=int(abs_ok), c=int(raw_ok),
                  bitmap=int(getattr(io, "multi_turn_unknown_bitmap", 0)))
            return
        turn = (actual - actual % 4096) // 4096
        if turn:
            io.trace("SKIP_NORMALIZE_TURN", ctx.joint, ctx.motor, abs=actual, raw=servo_raw, turn=turn)
        if not _initialize_tighten(ctx, io, params):
            _feedback_lost(ctx, io, "missing feedback after init", p.CALIB_DIAG_PHASE_INIT)
        return
    if ctx.state == AutoCalibState.PRE_RELEASE_SYNC:
        encoder, actual = io.encoder(ctx.joint), io.servo_abs(ctx.motor)
        if encoder is None or actual is None:
            _feedback_lost(ctx, io, "pre-release sync lost servo feedback", p.CALIB_DIAG_PHASE_RELEASE)
            return
        if abs(actual - ctx.pre_release_sync_target_abs) > kPreReleaseSyncTargetToleranceCounts:
            ctx.pre_release_sync_target_abs = actual
            ctx.pre_release_sync_window_start_abs = actual
            ctx.pre_release_sync_window_start_ms = now
            ctx.pre_release_sync_stable_samples = 0
        if not _append(ctx, io, targets, ctx.pre_release_sync_target_abs,
                       p.CALIB_DIAG_PHASE_RELEASE, "pre-release sync target append failed"):
            return
        if _elapsed(now, ctx.pre_release_sync_window_start_ms) < params.windowMs:
            return
        window_abs = abs(actual - ctx.pre_release_sync_window_start_abs)
        reached = abs(actual - ctx.pre_release_sync_target_abs) <= kPreReleaseSyncTargetToleranceCounts
        settled = window_abs <= kPreReleaseSyncSettleDeltaCounts
        if reached and settled:
            ctx.pre_release_sync_stable_samples = min(255, ctx.pre_release_sync_stable_samples + 1)
        else:
            ctx.pre_release_sync_stable_samples = 0
        io.trace("PRE_RELEASE_SYNC", ctx.joint, ctx.motor, target_abs=ctx.pre_release_sync_target_abs,
                 current_abs=actual, error=abs(actual - ctx.pre_release_sync_target_abs),
                 target_tolerance=kPreReleaseSyncTargetToleranceCounts,
                 window_abs=window_abs, settle_delta=kPreReleaseSyncSettleDeltaCounts,
                 stable=ctx.pre_release_sync_stable_samples, stable_required=kPreReleaseSyncStableSamples)
        if ctx.pre_release_sync_stable_samples < kPreReleaseSyncStableSamples:
            ctx.pre_release_sync_window_start_abs, ctx.pre_release_sync_window_start_ms = actual, now
            return
        ctx.command_abs, ctx.command_remainder = actual, 0
        ctx.window_start_abs, ctx.window_start_ms, ctx.last_command_ms = actual, now, now
        ctx.release_angle_stall_stable = 0
        ctx.start_encoder, ctx.start_servo_abs = encoder, actual
        ctx.fit_samples.reset()
        begin_sampling = getattr(io, "begin_lut_sampling", None)
        if callable(begin_sampling):
            try:
                ctx.lut_sampling_window = begin_sampling(ctx.joint, (ctx.motor,))
                if ctx.lut_sampling_window is None:
                    raise TimeAlignmentError("time-aligned LUT sampling window is missing")
            except TimeAlignmentError as exc:
                _fail(ctx, io, str(exc), p.CALIB_DIAG_REASON_RESULT_BUILD_FAILED,
                      p.CALIB_DIAG_PHASE_RELEASE, bitmap=1 << ctx.motor)
                return
        else:
            ctx.fit_samples.append(encoder, actual, 0)
            ctx.fit_samples.next_travel_deg = kTendonFitSampleStepDeg
        ctx.state = AutoCalibState.TIGHTEN
        io.trace("PRE_RELEASE_SYNC_READY", ctx.joint, ctx.motor, release_start_abs=actual)
        return
    if ctx.state != AutoCalibState.TIGHTEN:
        return
    raw, encoder = io.encoder_raw(ctx.joint), io.encoder(ctx.joint)
    actual = io.servo_abs(ctx.motor)
    phase = p.CALIB_DIAG_PHASE_RELEASE if ctx.start_found else p.CALIB_DIAG_PHASE_FIND_MAX
    if raw is None or encoder is None or actual is None:
        _feedback_lost(ctx, io, "lost encoder/servo feedback", phase)
        return
    load = io.servo_load(ctx.motor)
    load_mag = load_magnitude(load) if load is not None else 0
    total_abs = abs(actual - ctx.start_search_abs)
    total_encoder = wrap_encoder_delta(encoder, ctx.start_search_encoder)
    total_deg = abs(encoder_count_to_deg(total_encoder))
    load_high = load is not None and load_mag >= params.tightLoadHigh
    if ctx.start_found:
        released = single_tendon_release_counts(ctx.motor, ctx.start_servo_abs, actual)
        if released > kSingleTendonMaxReleaseCounts:
            ctx.command_abs, targets[ctx.motor] = actual, actual
            _fail(ctx, io, "single tendon release exceeded two turns", p.CALIB_DIAG_REASON_RELEASE_OVER_TWO_TURNS,
                  p.CALIB_DIAG_PHASE_RELEASE, a=released, b=kSingleTendonMaxReleaseCounts,
                  c=released, bitmap=1 << ctx.motor)
            return
        release_delta = wrap_encoder_delta(encoder, ctx.start_encoder)
        if ctx.lut_sampling_window is None:
            ctx.fit_samples.maybe_append(abs(encoder_count_to_deg(release_delta)), encoder, actual, 0)
    if _elapsed(now, ctx.last_trace_ms) >= params.traceMs:
        ctx.last_trace_ms = now
        io.trace("TRACE", ctx.joint, ctx.motor, phase="RELEASE_TO_TIGHT" if ctx.start_found else "FIND_MAX",
                 cmd=ctx.command_abs, abs=actual, raw=raw, oriented=encoder, delta_count=total_encoder,
                 total_abs=total_abs, total_deg=total_deg, load=load_mag,
                 start_found=ctx.start_found, elapsed=_elapsed(now, ctx.state_begin_ms))
    if not ctx.start_found:
        found = False
        if _elapsed(now, ctx.window_start_ms) >= params.windowMs:
            window_abs = abs(actual - ctx.window_start_abs)
            window_encoder = wrap_encoder_delta(encoder, ctx.window_start_encoder)
            window_deg = abs(encoder_count_to_deg(window_encoder))
            moving = window_abs >= params.windowServoDeltaCounts
            stalled = window_deg <= f32(params.stallActualDeg)
            io.trace("CHECK_MAX", ctx.joint, ctx.motor, abs=actual, raw=raw, oriented=encoder,
                     delta_count=total_encoder, window_count=window_encoder, window_abs=window_abs,
                     window_abs_required=params.windowServoDeltaCounts, window_deg=window_deg,
                     stall_deg=params.stallActualDeg, total_deg=total_deg, load=load_mag,
                     load_threshold=params.tightLoadHigh, servo=moving, angle=stalled, load_high=load_high)
            found = load_high and moving and stalled
            ctx.window_start_abs, ctx.window_start_encoder, ctx.window_start_ms = actual, encoder, now
        if found:
            ctx.encoder_travel_sign, ctx.start_found = 1, True
            ctx.start_encoder, ctx.start_servo_abs = encoder, actual
            ctx.window_start_encoder, ctx.window_start_abs, ctx.window_start_ms = encoder, actual, now
            ctx.last_command_ms, ctx.command_abs, ctx.command_remainder = now, actual, 0
            ctx.release_command_started = False
            ctx.pre_release_sync_target_abs, ctx.pre_release_sync_window_start_abs = actual, actual
            ctx.pre_release_sync_window_start_ms, ctx.pre_release_sync_stable_samples = now, 0
            ctx.release_angle_stall_stable = 0
            ctx.state = AutoCalibState.PRE_RELEASE_SYNC
            if not _append(ctx, io, targets, actual, p.CALIB_DIAG_PHASE_RELEASE, "pre-release sync target append failed"):
                return
            io.trace("START_FOUND", ctx.joint, ctx.motor, raw=raw, oriented=encoder, abs=actual,
                     load=load_mag, by_load=load_high, by_stall=found, total_deg=total_deg,
                     load_threshold=params.tightLoadHigh, encoder_travel_sign=ctx.encoder_travel_sign)
            io.trace("PRE_RELEASE_SYNC_BEGIN", ctx.joint, ctx.motor, target_abs=actual,
                     tolerance=kPreReleaseSyncTargetToleranceCounts,
                     settle_delta=kPreReleaseSyncSettleDeltaCounts, stable=kPreReleaseSyncStableSamples)
            return
        if _elapsed(now, ctx.last_command_ms) >= params.loopMs:
            counts = _step_counts(ctx, now, params)
            if counts > 0:
                ctx.command_abs = motor_apply_tendon_delta(ctx.motor, ctx.command_abs, counts)
                if not _inside_limit(ctx, io, targets, p.CALIB_DIAG_PHASE_FIND_MAX,
                                     "find max command outside session range"):
                    return
                if not _append(ctx, io, targets, ctx.command_abs, p.CALIB_DIAG_PHASE_FIND_MAX,
                               "find max target append failed"):
                    return
            ctx.last_command_ms = now
        return
    if _elapsed(now, ctx.last_command_ms) >= params.loopMs:
        counts = _step_counts(ctx, now, params)
        if counts > 0:
            next_command = motor_apply_tendon_delta(ctx.motor, ctx.command_abs, -counts)
            actual_release = single_tendon_release_counts(ctx.motor, ctx.start_servo_abs, actual)
            command_release = single_tendon_release_counts(ctx.motor, ctx.start_servo_abs, next_command)
            if command_release > kSingleTendonMaxReleaseCounts:
                ctx.command_abs, targets[ctx.motor] = actual, actual
                _fail(ctx, io, "single tendon release command would exceed two turns",
                      p.CALIB_DIAG_REASON_RELEASE_OVER_TWO_TURNS, p.CALIB_DIAG_PHASE_RELEASE,
                      a=actual_release, b=kSingleTendonMaxReleaseCounts,
                      c=command_release, bitmap=1 << ctx.motor)
                return
            ctx.command_abs = next_command
            if not _inside_limit(ctx, io, targets, p.CALIB_DIAG_PHASE_RELEASE,
                                 "release command outside session range"):
                return
            if not _append(ctx, io, targets, ctx.command_abs, p.CALIB_DIAG_PHASE_RELEASE,
                           "release target append failed"):
                return
            if not ctx.release_command_started:
                ctx.release_command_started = True
                io.trace("BEGIN_RELEASE", ctx.joint, ctx.motor, from_max_abs=ctx.start_servo_abs,
                         first_target_abs=ctx.command_abs, step=counts,
                         direction_sign=ctx.direction_sign, target_deg=kLutStartDeg)
        ctx.last_command_ms = now
    if _elapsed(now, ctx.window_start_ms) >= params.windowMs:
        window_abs = abs(actual - ctx.window_start_abs)
        window_encoder = wrap_encoder_delta(encoder, ctx.window_start_encoder)
        window_deg = abs(encoder_count_to_deg(window_encoder))
        moving = window_abs >= params.windowServoDeltaCounts
        stalled = window_deg <= f32(params.stallActualDeg)
        ctx.release_angle_stall_stable = min(255, ctx.release_angle_stall_stable + 1) if stalled else 0
        release_count = wrap_encoder_delta(encoder, ctx.start_encoder)
        release_deg = abs(encoder_count_to_deg(release_count))
        traveled = release_deg >= kReverseMinFollowDeg
        io.trace("CHECK_RELEASE", ctx.joint, ctx.motor, abs=actual, raw=raw, oriented=encoder,
                 release_count=release_count, release_deg=release_deg, follow_deg=kReverseMinFollowDeg,
                 window_count=window_encoder, window_abs=window_abs,
                 window_abs_required=params.windowServoDeltaCounts, window_deg=window_deg,
                 stall_deg=params.stallActualDeg, load=load_mag, servo=moving, angle=stalled,
                 angle_stable=ctx.release_angle_stall_stable,
                 stable_required=kAngleStallStableSamples, travel=traveled)
        if moving and ctx.release_angle_stall_stable >= kAngleStallStableSamples and traveled:
            _finish_release(ctx, io, params, targets, raw, encoder, actual, load_mag,
                            release_deg, window_deg, now)
            return
        ctx.window_start_abs, ctx.window_start_encoder, ctx.window_start_ms = actual, encoder, now


def run_single(io, joint: int, params: AutoCalibrationParams,
               ordered_joints, failed_joint_mask: int, sequence_index: int,
               selected_joint_mask: int) -> dict:
    """原 runSingleMotorCalibrationMachine 的串行循环与中止语义。"""
    prepared, message = io.prepare_finger_aa(joint)
    if not prepared:
        return {"success": False, "aborted": True, "reason": message,
                "result": None, "limits": {}}
    ctx = JointCalibContext(joint)
    io.trace("SEQUENCE_SERIAL_BEGIN", joint, 255, sequence_index=sequence_index,
             joint_count=len(ordered_joints))
    while ctx.state not in (AutoCalibState.DONE, AutoCalibState.FAILED):
        if not io.owns_control():
            io.diag(p.CALIB_DIAG_REASON_CONTROL_LOST, p.CALIB_DIAG_PHASE_SEQUENCE, joint, 255,
                    -1, -1, -1, int(getattr(io, "active_joint_mask", 0)))
            return {"success": False, "aborted": True, "reason": "control lost", "result": None, "limits": {}}
        targets = io.hold_targets_except(ordered_joints, ctx.joint, failed_joint_mask, params)
        if targets is None:
            io.diag(p.CALIB_DIAG_REASON_HOLD_TARGET_FAILED, p.CALIB_DIAG_PHASE_SEQUENCE,
                    joint, 255, sequence_index, len(ordered_joints), 0, selected_joint_mask)
            return {"success": False, "aborted": True,
                    "reason": getattr(io, 'motion_publish_error', '') or "hold target failed",
                    "result": None, "limits": {}}
        step_single(ctx, io, params, targets)
        if not io.publish(targets, params.servoSpeed, params.servoAcc):
            mask = sum(1 << motor for motor in targets)
            items = list(targets.items())
            apply_status = int(getattr(io, "target_apply_status", 0))
            _diag_if_empty(io,
                           p.CALIB_DIAG_REASON_TARGET_APPLY_TIMEOUT if apply_status == 1
                           else p.CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED,
                           p.CALIB_DIAG_PHASE_SEQUENCE, joint, items[0][0] if items else 255,
                           items[0][1] if items else 0,
                           items[1][1] if len(items) > 1 else 0, apply_status, mask)
            return {"success": False, "aborted": True, "reason": "target publish failed",
                    "result": ctx.result, "limits": ctx.limits}
        io.sleep_ms(params.loopMs)
    if ctx.state == AutoCalibState.FAILED:
        # IO 保留首个具体诊断，避免总调度兜底覆盖它。
        _diag_if_empty(io, p.CALIB_DIAG_REASON_JOINT_STEP_FAILED,
                       p.CALIB_DIAG_PHASE_SEQUENCE, joint, ctx.motor, 0, 0, 0, 0)
        return {"success": False, "aborted": False, "reason": ctx.reason,
                "result": ctx.result, "limits": ctx.limits}
    io.trace("SEQUENCE_SERIAL_DONE", joint, ctx.motor)
    return {"success": True, "aborted": False, "reason": "", "result": ctx.result, "limits": ctx.limits}
