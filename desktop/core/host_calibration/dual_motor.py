"""dual_motor.py - 双腱关节:两次端点搜索 + 结果构建。

职责：邻指让位、找 Max、找 Min（同时采 LUT）、构建结果与 ABS 限位。
线程：由上位机校准流程线程串行调用，不创建线程。
输入：IO 的反馈、控制权、逻辑范围和运动参数。
输出：每关节的标定结果、ABS 限位和结构化诊断。
状态所有权：只拥有本关节的轨迹；端点判定由 sweep.py 完成。
禁止依赖：禁止依赖 NiceGUI/UI、串口、ROS2 或 Replay；不直接提交协议数据。

The firmware machine is gone: the host no longer searches endpoints by load.
Every endpoint is the absolute joint angle freezing near the commanded URDF
end (see sweep.py). The four AA joints still yield to each other because that
is a mechanical prerequisite, not because of a threshold.
"""

from .. import protocol
from . import config
from .math import (
    apply_contact_refinement,
    build_calibration_result,
    build_tendon_lut,
    extend_motor_abs_limits,
    forward_encoder_delta,
    tendon_motor_abs_limits,
    TendonRawFitBuffer,
)
from .sweep import END_MAX, END_MIN, sweep_to_end
from .time_alignment import TimeAlignmentError

WORKSPACE_DUAL_JOINTS = (0, 4, 8, 12)
POSE_NONE, POSE_A, POSE_B = 0, 1, 2
STAGE_PREFLIGHT, STAGE_EXECUTE, STAGE_POST_CALIBRATION = 101, 102, 103
PAIR_READY = "ready"
PAIR_ALL_OFFLINE = "all-offline"
PAIR_PARTIAL_ONLINE = "partial-online"
PAIR_TURN_UNKNOWN = "turn-unknown"
PAIR_FEEDBACK_STALE = "feedback-stale"
PAIR_MAP_INVALID = "map-invalid"
PAIR_ROUTE_CONFLICT = "route-conflict"


def _get(snapshot, name, default=None):
    return snapshot.get(name, default) if isinstance(snapshot, dict) else getattr(snapshot, name, default)


def _dual_motors(joint):
    # Dual joints: motors[0] (primary) tightens toward URDF Max;
    # motors[1] (secondary) tightens toward URDF Min.
    motors = config.joint_motors(joint)
    return tuple(motors) if len(motors) == 2 else None


def classify_dual_workspace_pair(io, joint):
    """Original order: mapping, snapshot age, online pair, unknown turns."""
    motors = _dual_motors(joint)
    if motors is None:
        return PAIR_MAP_INVALID
    snapshot = io.snapshot()
    if int(_get(snapshot, 'servo_route_conflict_bitmap', 0)) & sum(1 << motor for motor in motors):
        return PAIR_ROUTE_CONFLICT
    age = _get(snapshot, "servo_feedback_age_ms")
    if age is None:
        timestamp = _get(snapshot, "servo_feedback_timestamp_ms")
        if timestamp is None:
            return PAIR_FEEDBACK_STALE
        age = (io.now_ms() - timestamp) & 0xFFFFFFFF
    if age > config.kFeedbackReadyServoTimeoutMs:
        return PAIR_FEEDBACK_STALE
    online = int(_get(snapshot, "servo_online_bitmap", 0))
    p_online, s_online = (bool(online & (1 << motor)) for motor in motors)
    if not p_online and not s_online:
        return PAIR_ALL_OFFLINE
    if p_online != s_online:
        return PAIR_PARTIAL_ONLINE
    if int(_get(snapshot, "multi_turn_unknown_bitmap", 0)) & sum(1 << motor for motor in motors):
        return PAIR_TURN_UNKNOWN
    return PAIR_READY


def build_dynamic_workspace_motor_mask(io, active_joints, *, allow_unknown=False):
    """Exclude fully offline helpers; reject partial/unknown/stale helpers.

    AA 邻指让位是 AA 行程搜索的机械前提，与本次标定选了哪些关节无关：
    未选中的邻指仍要纳入让位范围（原 P4 固件总是准备全部四指），
    只有完全离线才跳过。
    """
    mask = 0
    for target in active_joints:
        if target not in WORKSPACE_DUAL_JOINTS:
            continue
        for candidate in WORKSPACE_DUAL_JOINTS:
            motors = _dual_motors(candidate)
            pair_mask = sum(1 << motor for motor in motors) if motors else 0
            availability = classify_dual_workspace_pair(io, candidate)
            if availability == PAIR_READY:
                mask |= pair_mask
                continue
            # Discovery only: readiness/configuration is checked for the full scope
            # before coordinate preparation can write or enable anything.
            if allow_unknown and availability == PAIR_TURN_UNKNOWN:
                mask |= pair_mask
                io.trace("WORKSPACE_MASK_PREPARE_COORDINATES", target, mask=pair_mask)
                continue
            if candidate != target and availability == PAIR_ALL_OFFLINE:
                io.trace("WORKSPACE_MASK_SKIP", target, helper=candidate, reason=availability)
                continue
            io.diag(protocol.CALIB_DIAG_REASON_WORKSPACE_PREP_FAILED,
                    protocol.CALIB_DIAG_PHASE_DUAL_PRELOAD, target,
                    motors[0] if motors else 255,
                    candidate, POSE_NONE, STAGE_PREFLIGHT, pair_mask)
            return None
    return mask & ((1 << 27) - 1)


def build_dual_workspace_plan(io, target):
    if target not in WORKSPACE_DUAL_JOINTS:
        return []
    candidates = [(joint, POSE_A) for joint in WORKSPACE_DUAL_JOINTS if joint < target]
    candidates += [(joint, POSE_B) for joint in reversed(WORKSPACE_DUAL_JOINTS) if joint > target]
    steps = []
    for joint, pose in candidates:
        availability = classify_dual_workspace_pair(io, joint)
        if availability == PAIR_ALL_OFFLINE:
            io.trace("WORKSPACE_PLAN_SKIP", target, helper=joint, reason=availability)
            continue
        if availability != PAIR_READY or len(steps) >= len(WORKSPACE_DUAL_JOINTS) - 1:
            return None
        steps.append((joint, pose))
    return steps


def _pose_end(pose):
    """POSE_A = antagonist tight = URDF Min; POSE_B = primary tight = URDF Max."""
    return END_MAX if int(pose) == POSE_B else END_MIN


def _end_motors(motors, end):
    primary, secondary = motors
    return (primary, secondary) if int(end) == END_MAX else (secondary, primary)


class _DualMachine:
    """One joint: neighbours yield, then two endpoint sweeps."""

    def __init__(self, io, joint, params):
        self.io, self.joint, self.params = io, joint, params
        # 辅助手指让位沿用独立原速参数，不跟随当前校准的自定义参数。
        self.workspace_params = config.default_auto_calibration_params()
        self.reason = protocol.CALIB_DIAG_REASON_NONE

    def _diag(self, reason, phase, joint=None, motor=255, a=0, b=0, c=0, bitmap=0):
        # Same first-root-cause semantics as setCalibrationDiagIfEmpty.
        if self.reason != protocol.CALIB_DIAG_REASON_NONE:
            return
        self.reason = reason
        emit = getattr(self.io, 'diag_if_empty', None) or self.io.diag
        emit(reason, phase, self.joint if joint is None else joint, motor, a, b, c, bitmap)

    def _sweep(self, end, fit=None, **limits):
        motors = _dual_motors(self.joint)
        pull, release = _end_motors(motors, end)
        outcome = sweep_to_end(self.io, self.joint, end, motors=motors, pull=pull,
                               release=release, fit=fit, params=self.params, **limits)
        if outcome.point is None:
            phase = (protocol.CALIB_DIAG_PHASE_FIND_MAX if int(end) == END_MAX
                     else protocol.CALIB_DIAG_PHASE_FIND_MIN)
            self._diag(outcome.reason, phase, motor=pull)
        return outcome.point

    def _preload_secondary(self, tight):
        """主腱保持不动，副腱收到绷紧，两根绳都带张力后再开始交换。"""
        primary, secondary = _dual_motors(self.joint)
        hold = int(tight.primary_abs)
        cmd = self.io.servo_abs(secondary)
        if cmd is None:
            self._diag(protocol.CALIB_DIAG_REASON_FEEDBACK_LOST,
                       protocol.CALIB_DIAG_PHASE_DUAL_PRELOAD, motor=secondary)
            return False
        p = self.params
        speed = max(1, int(p.tightenSpeedCountsPerSec))
        deadline = self.io.now_ms() + config.kPreloadTimeoutMs
        last = self.io.now_ms()
        remainders = 0
        bitmap = (1 << primary) | (1 << secondary)
        self.io.trace("DUAL_PRELOAD_BEGIN", self.joint, secondary, hold_abs=hold)
        while self.io.owns_control() and self.io.now_ms() < deadline:
            now = self.io.now_ms()
            if now - last >= p.loopMs:
                counts, remainders = divmod((now - last) * speed + remainders, 1000)
                last = now
                cmd = config.motor_apply_tendon_delta(secondary, cmd, counts)
            if not self.io.publish({primary: hold, secondary: cmd}, p.servoSpeed, p.servoAcc):
                self._diag(protocol.CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED,
                           protocol.CALIB_DIAG_PHASE_DUAL_PRELOAD, motor=secondary)
                return False
            load = config.load_magnitude(self.io.servo_load(secondary) or 0)
            hold_load = config.load_magnitude(self.io.servo_load(primary) or 0)
            if max(load, hold_load) >= config.kOverloadLoadCounts:
                self._diag(protocol.CALIB_DIAG_REASON_OVERLOAD,
                           protocol.CALIB_DIAG_PHASE_DUAL_PRELOAD, motor=secondary,
                           a=load, b=config.kOverloadLoadCounts, c=0, bitmap=bitmap)
                return False
            if load >= config.kTautLoadCounts:
                self.io.trace("DUAL_PRELOAD_DONE", self.joint, secondary, load=load)
                return True
            self.io.sleep_ms(p.loopMs)
        self._diag(protocol.CALIB_DIAG_REASON_END_TIMEOUT,
                   protocol.CALIB_DIAG_PHASE_DUAL_PRELOAD, motor=secondary,
                   a=config.kPreloadTimeoutMs, b=0, c=0, bitmap=bitmap)
        return False

    def _pose_helper(self, helper, pose):
        motors = _dual_motors(helper)
        end = _pose_end(pose)
        target = float(self.io.urdf_limits(helper)[1 if end == END_MAX else 0])
        angle = self.io.angle(helper)
        if angle is None:
            # 让位现在按绝对角度走：辅助关节必须先有可用的 Manual 端点与编码器。
            self.io.trace("WORKSPACE_NO_ANGLE", helper, pose=pose)
            self._diag(protocol.CALIB_DIAG_REASON_WORKSPACE_PREP_FAILED,
                       protocol.CALIB_DIAG_PHASE_DUAL_PRELOAD, helper, motors[0],
                       helper, pose, STAGE_PREFLIGHT,
                       sum(1 << motor for motor in motors))
            return False
        if abs(angle - target) <= config.kEndToleranceDeg:
            self.io.trace("WORKSPACE_SKIP", helper, pose=pose, angle=round(angle, 3))
            return True
        pull, release = _end_motors(motors, end)
        outcome = sweep_to_end(self.io, helper, end, motors=motors, pull=pull,
                               release=release, params=self.workspace_params,
                               endpoint_load=config.kTautLoadCounts,
                               timeout_ms=config.kMaxSweepTimeoutMs)
        if outcome.point is None:
            # 具体原因（停在几度/腱太松）比笼统的"让位失败"更有用。
            self._diag(outcome.reason, protocol.CALIB_DIAG_PHASE_DUAL_PRELOAD, helper,
                       motors[0], helper, pose, STAGE_EXECUTE,
                       sum(1 << motor for motor in motors))
            return False
        self.io.trace("WORKSPACE_ESTABLISHED", helper, pose=pose,
                      angle=round(outcome.point.angle, 3), encoder=outcome.point.encoder)
        note_helper = getattr(self.io, 'note_workspace_helper', None)
        if callable(note_helper):
            note_helper(helper)
        return True

    def prepare_workspace(self):
        if self.joint not in WORKSPACE_DUAL_JOINTS:
            return True
        steps = build_dual_workspace_plan(self.io, self.joint)
        if steps is None:
            self._diag(protocol.CALIB_DIAG_REASON_WORKSPACE_PREP_FAILED,
                       protocol.CALIB_DIAG_PHASE_DUAL_PRELOAD, self.joint,
                       _dual_motors(self.joint)[0], self.joint, POSE_NONE, STAGE_PREFLIGHT,
                       sum(1 << m for m in config.joint_motors(self.joint)))
            return False
        self.io.trace("WORKSPACE_PLAN", self.joint, helpers=steps)
        return all(self._pose_helper(helper, pose) for helper, pose in steps)

    def calibrate(self):
        motors = _dual_motors(self.joint)
        if motors is None:
            self._diag(protocol.CALIB_DIAG_REASON_MOTOR_MAPPING_INVALID,
                       protocol.CALIB_DIAG_PHASE_INIT, motor=255,
                       a=self.joint, b=-1, c=-1)
            return None
        self.io.clear_limits(self.joint)
        tight = self._sweep(END_MAX, endpoint_load=config.kTautLoadCounts,
                            timeout_ms=config.kMaxSweepTimeoutMs)
        if tight is None or tight.primary_abs is None or tight.secondary_abs is None:
            return None
        if not self._preload_secondary(tight):
            return None
        fit = TendonRawFitBuffer()
        window = None
        begin_sampling = getattr(self.io, "begin_lut_sampling", None)
        if callable(begin_sampling):
            try:
                window = begin_sampling(self.joint, motors)
                if window is None:
                    raise TimeAlignmentError("time-aligned LUT sampling window is missing")
            except TimeAlignmentError as exc:
                self.io.trace("LUT_TIME_FAILED", self.joint, motors[0], reason=str(exc))
                self._diag(protocol.CALIB_DIAG_REASON_RESULT_BUILD_FAILED,
                           protocol.CALIB_DIAG_PHASE_FIND_MIN, motor=motors[0])
                return None
        else:
            fit.append(tight.encoder, tight.primary_abs, tight.secondary_abs)
            fit.next_travel_deg = config.kTendonFitSampleStepDeg
        limit = self._sweep(END_MIN, fit=None if window is not None else fit,
                            endpoint_load=config.kDualLimitLoadCounts,
                            timeout_ms=config.kMinSweepTimeoutMs)
        if limit is None or limit.primary_abs is None or limit.secondary_abs is None:
            return None
        if window is not None:
            try:
                fit = self.io.finish_lut_sampling(window)
            except TimeAlignmentError as exc:
                self.io.trace("LUT_TIME_FAILED", self.joint, motors[0], reason=str(exc))
                self._diag(protocol.CALIB_DIAG_REASON_RESULT_BUILD_FAILED,
                           protocol.CALIB_DIAG_PHASE_FIND_MIN, motor=motors[0])
                return None
        else:
            fit.append(limit.encoder, limit.primary_abs, limit.secondary_abs)

        span = forward_encoder_delta(limit.encoder, tight.encoder)
        self.io.trace("DUAL_SPAN_CHECK", self.joint, motors[0],
                      tight_encoder=tight.encoder, limit_encoder=limit.encoder,
                      span=span, tight_deg=round(tight.angle, 3),
                      limit_deg=round(limit.angle, 3))
        result = build_calibration_result(
            self.io, self.joint, limit.encoder, 0, tight.encoder,
            limit.primary_abs, tight.primary_abs, tight.primary_abs)
        if result is None:
            self._diag(protocol.CALIB_DIAG_REASON_RESULT_BUILD_FAILED,
                       protocol.CALIB_DIAG_PHASE_FIND_MIN, motor=motors[0],
                       a=fit.count, b=span, c=0,
                       bitmap=sum(1 << m for m in motors))
            return None
        lut_built = build_tendon_lut(self.io, self.joint, result, motors[0], motors[1], 2, fit)
        if lut_built:
            apply_contact_refinement(self.io, self.joint, result)
        measured = ((limit.primary_abs, tight.primary_abs),
                    (tight.secondary_abs, limit.secondary_abs))
        limits = {}
        for actuator, motor in enumerate(motors):
            bounds = tendon_motor_abs_limits(
                result, actuator, motor, *config.calibration_margins(self.io))
            if bounds is None:
                lo, hi = sorted(measured[actuator])
                bounds = extend_motor_abs_limits(
                    motor, lo, hi, *config.calibration_margins(self.io))
            if bounds is None:
                self._diag(protocol.CALIB_DIAG_REASON_RESULT_BUILD_FAILED,
                           protocol.CALIB_DIAG_PHASE_FIND_MIN, motor=motor, a=0, b=0, c=0,
                           bitmap=sum(1 << m for m in motors))
                return None
            limits[motor] = bounds
            self.io.set_limit(motor, *bounds)
        self.io.trace("DUAL_RESULT", self.joint, motors[0],
                      span=span, tight_encoder=tight.encoder, limit_encoder=limit.encoder,
                      tight_deg=round(tight.angle, 3), limit_deg=round(limit.angle, 3),
                      tight_load=tight.load, limit_load=limit.load,
                      lut_built=lut_built, raw_samples=fit.count)
        return {"result": result, "limits": limits}


def run_dual(io, joint, sequence_index, joint_count, selected_joint_mask, params):
    """Serial dual calibration; caller handles per-joint Manual/LUT commit."""
    setter = getattr(io, 'set_active_calibration_motors', None)
    if callable(setter):
        setter(sum(1 << motor for motor in config.joint_motors(joint)))
    machine = _DualMachine(io, joint, params)
    outcome = {"success": False, "aborted": False, "preflight_failed": False,
               "reason": protocol.CALIB_DIAG_REASON_NONE, "result": None, "limits": {}}
    motors = _dual_motors(joint)
    if motors is None:
        machine._diag(protocol.CALIB_DIAG_REASON_MOTOR_MAPPING_INVALID,
                      protocol.CALIB_DIAG_PHASE_INIT, motor=255,
                      a=joint, b=-1, c=-1)
    elif not machine.prepare_workspace():
        outcome["preflight_failed"] = True
    else:
        io.trace("SEQUENCE_SERIAL_BEGIN", joint, motors[0],
                 index=sequence_index + 1, count=joint_count, secondary=motors[1])
        result = machine.calibrate()
        if result is not None:
            outcome.update(result)
            outcome["success"] = True
            io.trace("SEQUENCE_SERIAL_DONE", joint)
        else:
            machine._diag(protocol.CALIB_DIAG_REASON_JOINT_STEP_FAILED,
                          protocol.CALIB_DIAG_PHASE_SEQUENCE, joint, motors[0], 0, 0, 0, 0)
    outcome["reason"] = machine.reason
    outcome["aborted"] = not outcome["success"] and not io.owns_control()
    return outcome
