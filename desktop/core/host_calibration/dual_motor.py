"""dual_motor.py - MMHand Desktop 模块契约。

职责：逐分支移植 P4 双腱机械端搜索和四指工作空间准备。
线程：由上位机校准流程线程串行调用，不创建线程。
输入：IO 的反馈、控制权、逻辑范围和原 AutoCalibrationParams。
输出：每关节的标定结果、ABS 限位和结构化诊断。
状态所有权：工作空间缓存只由本次序列 IO 实例拥有。
禁止依赖：禁止依赖 Qt/UI、串口、ROS2 或 Replay；不直接提交协议数据。

The motor trajectories, sample windows and stop predicates follow the firmware
branches. Transport/ownership live in the IO port; committing the resulting
Manual and LUT belongs to the sequence coordinator, once per completed joint.
The firmware machine deliberately has no general movement timeout. Its waits
end on feedback/command failures or loss of ownership; only helper feedback
preflight has the original 1200 ms timeout.
"""

from dataclasses import dataclass

from .. import protocol
from . import config
from .math import (
    TendonRawFitBuffer,
    build_calibration_result,
    build_tendon_lut,
    encoder_count_to_deg,
    encoder_for_positive_travel,
    forward_encoder_delta,
    f32,
    map_manual_scaled_encoder_to_urdf,
    round_to_int32,
    signed_encoder_delta,
    wrap_encoder_count,
    wrap_encoder_delta,
)


WORKSPACE_DUAL_JOINTS = (0, 4, 8, 12)
POSE_NONE, POSE_A, POSE_B = 0, 1, 2
STAGE_PREFLIGHT, STAGE_EXECUTE, STAGE_POST_CALIBRATION = 101, 102, 103
PAIR_READY = "ready"
PAIR_ALL_OFFLINE = "all-offline"
PAIR_PARTIAL_ONLINE = "partial-online"
PAIR_TURN_UNKNOWN = "turn-unknown"
PAIR_FEEDBACK_STALE = "feedback-stale"
PAIR_MAP_INVALID = "map-invalid"


@dataclass
class WorkspaceCacheEntry:
    valid: bool = False
    current_pose: int = POSE_NONE
    primary_motor: int = 255
    secondary_motor: int = 255
    primary_tight_abs: int = 0
    primary_release_abs: int = 0
    secondary_tight_abs: int = 0
    secondary_release_abs: int = 0


def _get(snapshot, name, default=None):
    return snapshot.get(name, default) if isinstance(snapshot, dict) else getattr(snapshot, name, default)


def _dual_motors(joint):
    motors = config.joint_motors(joint)
    return tuple(motors) if len(motors) == 2 else None


def _cache(io):
    entries = getattr(io, "_dual_workspace_cache", None)
    if entries is None:
        entries = {joint: WorkspaceCacheEntry() for joint in WORKSPACE_DUAL_JOINTS}
        io._dual_workspace_cache = entries
    return entries


def reset_dual_workspace_cache(io):
    """RAM posture measurements are trusted for this sequence only."""
    io._dual_workspace_cache = {joint: WorkspaceCacheEntry() for joint in WORKSPACE_DUAL_JOINTS}


def invalidate_dual_workspace_cache_joint(io, joint):
    """A changed logical ABS origin invalidates this joint's measured posture."""
    if joint in WORKSPACE_DUAL_JOINTS:
        _cache(io)[joint] = WorkspaceCacheEntry()


def classify_dual_workspace_pair(io, joint):
    """Original order: mapping, snapshot age, online pair, unknown turns."""
    motors = _dual_motors(joint)
    if motors is None:
        return PAIR_MAP_INVALID
    snapshot = io.snapshot()
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


def build_dynamic_workspace_motor_mask(io, active_joints):
    """Exclude fully offline helpers; reject partial/unknown/stale helpers."""
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
            snapshot = io.snapshot()
            configured = int(_get(snapshot, "servo_multiturn_configured_bitmap", 0))
            pending = int(_get(snapshot, "servo_recovery_pending_bitmap", 0))
            failed = int(_get(snapshot, "servo_recovery_failed_bitmap", 0))
            target_can_rebuild = (
                candidate == target and availability == PAIR_TURN_UNKNOWN
                and configured & pair_mask == pair_mask
                and pending & pair_mask == 0 and failed & pair_mask == 0
            )
            if target_can_rebuild:
                mask |= pair_mask
                io.trace("WORKSPACE_MASK_TARGET_REBUILD", target, mask=pair_mask)
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


class _DualMachine:
    """One joint's machine, retaining the original helper branch structure."""

    def __init__(self, io, joint, params):
        self.io, self.joint, self.params = io, joint, params
        # 辅助手指让位沿用原速度，不跟随当前侧摆校准的低速参数。
        self.workspace_params = config.default_auto_calibration_params()
        self.reason = protocol.CALIB_DIAG_REASON_NONE

    def _diag(self, reason, phase, joint, motor=255, *values):
        # Same first-root-cause semantics as setCalibrationDiagIfEmpty.
        if self.reason == protocol.CALIB_DIAG_REASON_NONE:
            self.reason = reason
            if hasattr(self.io, 'diag_if_empty'):
                self.io.diag_if_empty(reason, phase, joint, motor, *values)
            else:
                self.io.diag(reason, phase, joint, motor, *values)

    def _control_lost(self, phase, joint, motor):
        snapshot = self.io.snapshot()
        self._diag(protocol.CALIB_DIAG_REASON_CONTROL_LOST, phase, joint, motor,
                   _get(snapshot, "system_state", -1),
                   _get(snapshot, "servo_target_owner", -1),
                   int(_get(snapshot, "control_enabled", -1)),
                   _get(snapshot, "active_joint_mask", 0))

    def _feedback_lost(self, phase, joint, motor, motor_mask):
        encoder_mask = (1 << joint) if self.io.encoder(joint) is not None else 0
        abs_mask = raw_mask = 0
        for channel in range(27):
            bit = 1 << channel
            if motor_mask & bit:
                if self.io.servo_abs(channel) is not None:
                    abs_mask |= bit
                if self.io.servo_raw(channel) is not None:
                    raw_mask |= bit
        self._diag(protocol.CALIB_DIAG_REASON_FEEDBACK_LOST, phase, joint, motor,
                   encoder_mask, abs_mask, raw_mask, motor_mask)

    def _workspace_failed(self, target, helper, pose, stage):
        motors = _dual_motors(helper)
        self._diag(protocol.CALIB_DIAG_REASON_WORKSPACE_PREP_FAILED,
                   protocol.CALIB_DIAG_PHASE_DUAL_PRELOAD, target,
                   motors[0] if motors else 255, helper, pose, stage,
                   sum(1 << motor for motor in motors) if motors else 0)

    def _publish(self, targets, phase, joint, motor, params=None):
        # P4 publishDualMotorCalibrationTargets/publishCalibrationBatch gates.
        if not self.io.owns_control():
            self._control_lost(phase, joint, motor)
            return False
        for channel, target in targets.items():
            if not self.io.inside_limit(channel, target):
                values = list(targets.values()) + [0]
                self._diag(protocol.CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED,
                           phase, joint, motor, values[0], values[1], -1,
                           sum(1 << c for c in targets))
                return False
        for channel in targets:
            load = self.io.servo_load(channel)
            if load is not None and config.load_magnitude(load) >= config.kOverloadLoadThreshold:
                self._diag(protocol.CALIB_DIAG_REASON_OVERLOAD,
                           protocol.CALIB_DIAG_PHASE_SEQUENCE, 255, channel,
                           config.load_magnitude(load), config.kOverloadLoadThreshold, load, 1 << channel)
                return False
        params = self.params if params is None else params
        if self.io.publish(targets, params.servoSpeed, params.servoAcc):
            return True
        values = list(targets.values()) + [0]
        self._diag(protocol.CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED,
                   phase, joint, motor, values[0], values[1],
                   _get(self.io.snapshot(), "calibration_target_apply_status", -1),
                   sum(1 << c for c in targets))
        return False

    @staticmethod
    def _delta(motor, value, counts):
        return value + config.shorten_sign(motor) * counts

    def _wait_motor_target(self, motor, target):
        phase = protocol.CALIB_DIAG_PHASE_DUAL_PRELOAD
        while self.io.owns_control():
            if not self._publish({motor: target}, phase, self.joint, motor):
                return False
            current = self.io.servo_abs(motor)
            if current is not None and abs(current - target) <= self.params.zeroReachAbsTolerance:
                self.io.trace("DUAL_SLACK_SECONDARY_REACHED", self.joint, motor,
                              target=target, current=current)
                return True
            self.io.sleep_ms(self.params.loopMs)
        self._control_lost(phase, self.joint, motor)
        return False

    def _wait_initial_feedback(self, primary, secondary):
        while self.io.owns_control():
            p_abs, s_abs = self.io.servo_abs(primary), self.io.servo_abs(secondary)
            encoder, raw = self.io.encoder(self.joint), self.io.encoder_raw(self.joint)
            if p_abs is not None and s_abs is not None and encoder is not None and raw is not None:
                return p_abs, s_abs, raw, encoder
            self.io.sleep_ms(20)
        self._control_lost(protocol.CALIB_DIAG_PHASE_INIT, self.joint, primary)
        return None

    def _find_tight(self, motor, start_abs, start_encoder, threshold, hold_motor=None, hold_abs=0):
        p, phase = self.params, protocol.CALIB_DIAG_PHASE_FIND_MAX
        command, remainder = start_abs, 0
        last_command = window_start = self.io.now_ms()
        window_abs, window_encoder, stable = start_abs, start_encoder, 0
        while self.io.owns_control():
            now = self.io.now_ms()
            raw, encoder, current = self.io.encoder_raw(self.joint), self.io.encoder(self.joint), self.io.servo_abs(motor)
            if raw is None or encoder is None or current is None:
                self._feedback_lost(phase, self.joint, motor, 1 << motor)
                return None
            load_raw = self.io.servo_load(motor)
            load = config.load_magnitude(load_raw) if load_raw is not None else 0
            load_high = load_raw is not None and load >= threshold
            found = False
            if now - window_start >= p.windowMs:
                abs_delta = abs(current - window_abs)
                angle_delta = abs(encoder_count_to_deg(wrap_encoder_delta(encoder, window_encoder)))
                stable = min(stable + 1, 255) if angle_delta <= p.stallActualDeg else 0
                found = load_high and abs_delta >= p.windowServoDeltaCounts and stable >= config.kAngleStallStableSamples
                self.io.trace("CHECK_POS80", self.joint, motor, window_abs=abs_delta,
                              window_deg=angle_delta, angle_stable=stable, load=load,
                              load_high=load_high, tight_found=found)
                window_abs, window_encoder, window_start = current, encoder, now
            targets = {motor: current}
            if hold_motor is not None:
                targets[hold_motor] = hold_abs
            if found:
                if not self._publish(targets, phase, self.joint, motor):
                    return None
                return current, raw, encoder, load
            dt = now - last_command
            if dt >= p.loopMs:
                step, remainder = divmod(dt * p.tightenSpeedCountsPerSec + remainder, 1000)
                if step > 0:
                    command = self._delta(motor, command, step)
                    targets[motor] = command
                    if not self._publish(targets, phase, self.joint, motor):
                        return None
                last_command = now
            self.io.sleep_ms(p.loopMs)
        self._control_lost(phase, self.joint, motor)
        return None

    def _wait_workspace_feedback(self, joint, primary, secondary):
        mask, begin = (1 << primary) | (1 << secondary), self.io.now_ms()
        while self.io.owns_control():
            unknown = int(_get(self.io.snapshot(), "multi_turn_unknown_bitmap", 0)) & mask
            if unknown:
                self._diag(protocol.CALIB_DIAG_REASON_WORKSPACE_PREP_FAILED,
                           protocol.CALIB_DIAG_PHASE_DUAL_PRELOAD, joint,
                           primary if unknown & (1 << primary) else secondary,
                           joint, POSE_NONE, STAGE_PREFLIGHT, mask)
                return None
            p_abs, s_abs = self.io.servo_abs(primary), self.io.servo_abs(secondary)
            p_load, s_load = self.io.servo_load(primary), self.io.servo_load(secondary)
            if p_abs is not None and s_abs is not None and p_load is not None and s_load is not None:
                return p_abs, s_abs
            if self.io.now_ms() - begin >= config.kInitialFeedbackWaitMs:
                self._diag(protocol.CALIB_DIAG_REASON_WORKSPACE_PREP_FAILED,
                           protocol.CALIB_DIAG_PHASE_DUAL_PRELOAD, joint,
                           secondary if p_abs is not None and p_load is not None else primary,
                           joint, POSE_NONE, STAGE_PREFLIGHT, mask)
                return None
            self.io.sleep_ms(20)
        self._control_lost(protocol.CALIB_DIAG_PHASE_DUAL_PRELOAD, joint, primary)
        return None

    def _wait_workspace_targets(self, joint, primary, p_target, secondary, s_target, state):
        phase = protocol.CALIB_DIAG_PHASE_DUAL_PRELOAD
        p = self.workspace_params
        while self.io.owns_control():
            p_abs, s_abs = self.io.servo_abs(primary), self.io.servo_abs(secondary)
            if p_abs is None or s_abs is None:
                self._workspace_failed(joint, joint, POSE_NONE, STAGE_EXECUTE)
                return False
            if not self._publish({primary: p_target, secondary: s_target}, phase, joint, primary, p):
                return False
            if abs(p_abs - p_target) <= config.kWorkspaceTargetToleranceCounts and abs(s_abs - s_target) <= config.kWorkspaceTargetToleranceCounts:
                self.io.trace("WORKSPACE_" + state + "_REACHED", joint, primary,
                              primary_abs=p_abs, secondary_abs=s_abs)
                return True
            self.io.sleep_ms(p.loopMs)
        self._control_lost(phase, joint, primary)
        return False

    def _tighten_workspace_motor(self, joint, motor, hold_motor, hold_abs):
        phase, p = protocol.CALIB_DIAG_PHASE_DUAL_PRELOAD, self.workspace_params
        start = self.io.servo_abs(motor)
        if start is None:
            return None
        command, remainder, last_command = start, 0, self.io.now_ms()
        while self.io.owns_control():
            current, load_raw = self.io.servo_abs(motor), self.io.servo_load(motor)
            if current is None or load_raw is None:
                return None
            if config.load_magnitude(load_raw) >= config.kWorkspaceTightLoadThreshold:
                if not self._publish({motor: current, hold_motor: hold_abs}, phase, joint, motor, p):
                    return None
                return current
            now = self.io.now_ms()
            dt = now - last_command
            if dt >= p.loopMs:
                step, remainder = divmod(dt * p.tightenSpeedCountsPerSec + remainder, 1000)
                if step > 0:
                    command = self._delta(motor, command, step)
                    if not self._publish({motor: command, hold_motor: hold_abs}, phase, joint, motor, p):
                        return None
                last_command = now
            self.io.sleep_ms(p.loopMs)
        self._control_lost(phase, joint, motor)
        return None

    def _ensure_workspace_pose(self, joint, desired):
        if joint not in WORKSPACE_DUAL_JOINTS or desired == POSE_NONE:
            return False
        motors = _dual_motors(joint)
        if motors is None:
            return False
        primary, secondary = motors
        values = self._wait_workspace_feedback(joint, primary, secondary)
        if values is None:
            return False
        p_abs, s_abs = values
        entry = _cache(self.io)[joint]
        if entry.valid and (entry.primary_motor, entry.secondary_motor) != motors:
            entry = WorkspaceCacheEntry()
            _cache(self.io)[joint] = entry
        entry.primary_motor, entry.secondary_motor = motors
        if entry.valid and entry.current_pose == desired:
            p_target = entry.primary_release_abs if desired == POSE_A else entry.primary_tight_abs
            s_target = entry.secondary_tight_abs if desired == POSE_A else entry.secondary_release_abs
            if abs(p_abs - p_target) <= config.kWorkspaceTargetToleranceCounts and abs(s_abs - s_target) <= config.kWorkspaceTargetToleranceCounts:
                self.io.trace("WORKSPACE_REUSE_NO_COMMAND", joint, pose=desired)
                return True
            self.io.trace("WORKSPACE_REUSE_INVALIDATE", joint, pose=desired)
            entry = WorkspaceCacheEntry(primary_motor=primary, secondary_motor=secondary)
            _cache(self.io)[joint] = entry
        if entry.valid and entry.current_pose == POSE_A and desired == POSE_B:
            entry.secondary_release_abs = self._delta(secondary, entry.secondary_tight_abs, -config.kWorkspaceReleaseCounts)
            if not self._wait_workspace_targets(joint, primary, entry.primary_release_abs,
                                                secondary, entry.secondary_release_abs, "A_TO_B_RELEASE"):
                return False
            tight = self._tighten_workspace_motor(joint, primary, secondary, entry.secondary_release_abs)
            if tight is None:
                return False
            entry.primary_tight_abs = tight
            entry.current_pose, entry.valid = POSE_B, True
            return True
        if entry.valid and entry.current_pose == POSE_B and desired == POSE_A:
            entry.primary_release_abs = self._delta(primary, entry.primary_tight_abs, -config.kWorkspaceReleaseCounts)
            if not self._wait_workspace_targets(joint, primary, entry.primary_release_abs,
                                                secondary, entry.secondary_release_abs, "B_TO_A_RELEASE"):
                return False
            tight = self._tighten_workspace_motor(joint, secondary, primary, entry.primary_release_abs)
            if tight is None:
                return False
            entry.secondary_tight_abs = tight
            entry.current_pose, entry.valid = POSE_A, True
            return True
        release_motor, tighten_motor = (primary, secondary) if desired == POSE_A else (secondary, primary)
        release_start, hold_abs = (p_abs, s_abs) if desired == POSE_A else (s_abs, p_abs)
        release_target = self._delta(release_motor, release_start, -config.kWorkspaceReleaseCounts)
        if not self.io.inside_limit(release_motor, release_target):
            limits = self.io.logical_range(release_motor) or (0, 0)
            self._diag(protocol.CALIB_DIAG_REASON_ABS_LIMIT_INVALID,
                       protocol.CALIB_DIAG_PHASE_DUAL_PRELOAD, joint, release_motor,
                       release_target, *limits, 1 << release_motor)
            return False
        p_target, s_target = (release_target, hold_abs) if desired == POSE_A else (hold_abs, release_target)
        if not self._wait_workspace_targets(joint, primary, p_target, secondary, s_target, "DIRECT_RELEASE"):
            return False
        tight = self._tighten_workspace_motor(joint, tighten_motor, release_motor, release_target)
        if tight is None:
            return False
        if desired == POSE_A:
            entry.primary_release_abs, entry.secondary_tight_abs = release_target, tight
        else:
            entry.primary_tight_abs, entry.secondary_release_abs = tight, release_target
        entry.current_pose, entry.valid = desired, True
        self.io.trace("WORKSPACE_ESTABLISHED", joint, pose=desired,
                      release_motor=release_motor, release_abs=release_target,
                      tighten_motor=tighten_motor, tighten_abs=tight)
        return True

    def prepare_workspace(self):
        if self.joint not in WORKSPACE_DUAL_JOINTS:
            return True
        motors = _dual_motors(self.joint)
        if motors is None or self._wait_workspace_feedback(self.joint, *motors) is None:
            self._workspace_failed(self.joint, self.joint, POSE_NONE, STAGE_PREFLIGHT)
            return False
        steps = build_dual_workspace_plan(self.io, self.joint)
        if steps is None:
            self._workspace_failed(self.joint, self.joint, POSE_B, STAGE_PREFLIGHT)
            return False
        self.io.trace("WORKSPACE_PLAN", self.joint, helpers=steps)
        # Preflight every helper before executing the first movement.
        for helper, pose in steps:
            motors = _dual_motors(helper)
            if motors is None or self._wait_workspace_feedback(helper, *motors) is None:
                self._workspace_failed(self.joint, helper, pose, STAGE_PREFLIGHT)
                return False
        for helper, pose in steps:
            if not self._ensure_workspace_pose(helper, pose):
                self._workspace_failed(self.joint, helper, pose, STAGE_EXECUTE)
                return False
        return True

    def _find_secondary_preload(self, secondary, start_abs, primary, hold_abs):
        p, phase = self.params, protocol.CALIB_DIAG_PHASE_DUAL_PRELOAD
        command, remainder, last_command = start_abs, 0, self.io.now_ms()
        threshold = config.kDualSecondaryPreloadLoadThreshold
        while self.io.owns_control():
            now = self.io.now_ms()
            raw, encoder, current = self.io.encoder_raw(self.joint), self.io.encoder(self.joint), self.io.servo_abs(secondary)
            if raw is None or encoder is None or current is None:
                self._feedback_lost(phase, self.joint, secondary, (1 << primary) | (1 << secondary))
                return None
            load_raw = self.io.servo_load(secondary)
            load = config.load_magnitude(load_raw) if load_raw is not None else 0
            if load_raw is not None and load >= threshold:
                if not self._publish({secondary: current, primary: hold_abs}, phase, self.joint, secondary):
                    return None
                return current, raw, encoder, load
            dt = now - last_command
            if dt >= p.loopMs:
                step, remainder = divmod(dt * p.tightenSpeedCountsPerSec + remainder, 1000)
                if step > 0:
                    command = self._delta(secondary, command, step)
                    if not self._publish({secondary: command, primary: hold_abs}, phase, self.joint, secondary):
                        return None
                last_command = now
            self.io.sleep_ms(p.loopMs)
        self._control_lost(phase, self.joint, secondary)
        return None

    def _find_exchange(self, secondary, start_s, primary, start_p, start_encoder, fit):
        p, phase = self.params, protocol.CALIB_DIAG_PHASE_DUAL_EXCHANGE
        s_command, p_command, remainder = start_s, start_p, 0
        last_command = window_start = self.io.now_ms()
        window_encoder, window_s, window_p, stable = start_encoder, start_s, start_p, 0
        threshold = config.kDualSecondaryFinalLoadThreshold
        motor_mask = (1 << primary) | (1 << secondary)
        while self.io.owns_control():
            now = self.io.now_ms()
            raw, encoder = self.io.encoder_raw(self.joint), self.io.encoder(self.joint)
            s_abs, p_abs = self.io.servo_abs(secondary), self.io.servo_abs(primary)
            if raw is None or encoder is None or s_abs is None or p_abs is None:
                self._feedback_lost(phase, self.joint, secondary, motor_mask)
                return None
            load_raw = self.io.servo_load(secondary)
            load = config.load_magnitude(load_raw) if load_raw is not None else 0
            load_high = load_raw is not None and load >= threshold
            total_delta = wrap_encoder_delta(encoder, start_encoder)
            total_deg = abs(encoder_count_to_deg(total_delta))
            fit.maybe_append(total_deg, encoder, p_abs, s_abs)
            found = False
            if now - window_start >= p.windowMs:
                angle_delta = abs(encoder_count_to_deg(wrap_encoder_delta(encoder, window_encoder)))
                s_delta, p_delta = abs(s_abs - window_s), abs(p_abs - window_p)
                enough_travel = total_deg >= config.kReverseMinFollowDeg
                angle_stalled = angle_delta <= p.stallActualDeg
                stable = min(stable + 1, 255) if load_high and not enough_travel and angle_stalled else 0
                self.io.trace("CHECK_DUAL_EXCHANGE_NEG3", self.joint, secondary,
                              total_deg=total_deg, window_deg=angle_delta,
                              secondary_delta=s_delta, primary_delta=p_delta,
                              angle_stable=stable, load=load, enough_travel=enough_travel)
                if stable >= config.kAngleStallStableSamples:
                    self._diag(protocol.CALIB_DIAG_REASON_DUAL_EXCHANGE_STALLED,
                               phase, self.joint, secondary,
                               load, threshold, total_delta, motor_mask)
                    return None
                # Original endpoint does NOT require angle_stalled.
                found = load_high and enough_travel and s_delta >= p.windowServoDeltaCounts and p_delta >= p.windowServoDeltaCounts
                window_encoder, window_s, window_p, window_start = encoder, s_abs, p_abs, now
            if found:
                if not self._publish({secondary: s_abs, primary: p_abs}, phase, self.joint, secondary):
                    return None
                return s_abs, p_abs, raw, encoder, load
            dt = now - last_command
            if dt >= p.loopMs:
                step, remainder = divmod(dt * p.tightenSpeedCountsPerSec + remainder, 1000)
                if step > 0:
                    s_command = self._delta(secondary, s_command, step)
                    p_command = self._delta(primary, p_command, -step)
                    if not self._publish({secondary: s_command, primary: p_command}, phase, self.joint, secondary):
                        return None
                last_command = now
            self.io.sleep_ms(p.loopMs)
        self._control_lost(phase, self.joint, secondary)
        return None

    def calibrate(self):
        motors = _dual_motors(self.joint)
        if motors is None:
            self._diag(protocol.CALIB_DIAG_REASON_MOTOR_MAPPING_INVALID,
                       protocol.CALIB_DIAG_PHASE_INIT, self.joint, 255,
                       self.joint, -1, -1, 0)
            return None
        primary, secondary = motors
        self.io.set_direction(self.joint, config.default_encoder_direction(self.joint))
        self.io.clear_limits(self.joint)
        initial = self._wait_initial_feedback(primary, secondary)
        if initial is None:
            self._feedback_lost(protocol.CALIB_DIAG_PHASE_INIT, self.joint,
                                primary, (1 << primary) | (1 << secondary))
            return None
        p_abs, s_abs, initial_raw, initial_encoder = initial
        workspace = _cache(self.io).get(self.joint)
        reuse = False
        if workspace and workspace.valid and workspace.current_pose == POSE_B and (workspace.primary_motor, workspace.secondary_motor) == motors:
            reuse = abs(p_abs - workspace.primary_tight_abs) <= config.kWorkspaceTargetToleranceCounts and abs(s_abs - workspace.secondary_release_abs) <= config.kWorkspaceTargetToleranceCounts
            if not reuse:
                workspace.valid = False
        if reuse:
            load = self.io.servo_load(primary)
            tight = p_abs, initial_raw, initial_encoder, config.load_magnitude(load) if load is not None else 0
            self.io.trace("DUAL_WORKSPACE_B_REUSE", self.joint, primary, abs=p_abs)
        else:
            slack_target = self._delta(secondary, s_abs, -config.kDualInitialSlackCounts)
            if not self._wait_motor_target(secondary, slack_target):
                return None
            initial_encoder, p_abs = self.io.encoder(self.joint), self.io.servo_abs(primary)
            if initial_encoder is None or p_abs is None:
                self._feedback_lost(protocol.CALIB_DIAG_PHASE_FIND_MAX, self.joint, primary, 1 << primary)
                return None
            tight = self._find_tight(primary, p_abs, initial_encoder, config.kDualPrimaryTightLoadThreshold)
            if tight is None:
                return None
        tight_abs, tight_raw, tight_encoder, tight_load = tight
        s_start, s_encoder = self.io.servo_abs(secondary), self.io.encoder(self.joint)
        if s_start is None or s_encoder is None:
            self._feedback_lost(protocol.CALIB_DIAG_PHASE_DUAL_PRELOAD, self.joint, secondary, 1 << secondary)
            return None
        preload = self._find_secondary_preload(secondary, s_start, primary, tight_abs)
        if preload is None:
            return None
        preload_abs, preload_raw, preload_encoder, preload_load = preload
        exchange_encoder = self.io.encoder(self.joint)
        if exchange_encoder is None:
            self._feedback_lost(protocol.CALIB_DIAG_PHASE_DUAL_EXCHANGE,
                                self.joint, secondary, (1 << primary) | (1 << secondary))
            return None
        fit = TendonRawFitBuffer()
        fit.append(exchange_encoder, tight_abs, preload_abs)
        fit.next_travel_deg = config.kTendonFitSampleStepDeg
        endpoint = self._find_exchange(secondary, preload_abs, primary, tight_abs, exchange_encoder, fit)
        if endpoint is None:
            return None
        s_limit, p_final, ready_raw, limit_encoder, ready_load = endpoint
        fit.append(limit_encoder, p_final, s_limit)
        start_encoder = encoder_for_positive_travel(limit_encoder, 1)
        result_tight = encoder_for_positive_travel(tight_encoder, 1)
        span = forward_encoder_delta(start_encoder, result_tight)
        if span <= 0 or span >= config.kRejectNearFullWrapSpanCounts:
            self._diag(protocol.CALIB_DIAG_REASON_RESULT_SPAN_INVALID,
                       protocol.CALIB_DIAG_PHASE_DUAL_EXCHANGE, self.joint, primary,
                       span, limit_encoder, tight_encoder, (1 << primary) | (1 << secondary))
            return None
        total_deg = encoder_count_to_deg(span)
        fraction = f32(f32(config.kLutZeroDeg - config.kLutStartDeg) / total_deg) if total_deg > 0 else 0
        zero_encoder = wrap_encoder_count(start_encoder + round_to_int32(f32(f32(span) * fraction)))
        result = build_calibration_result(self.io, self.joint, start_encoder, zero_encoder,
                                          result_tight, s_limit, tight_abs, tight_abs)
        if result is None:
            self._diag(protocol.CALIB_DIAG_REASON_RESULT_BUILD_FAILED,
                       protocol.CALIB_DIAG_PHASE_DUAL_EXCHANGE, self.joint, primary,
                       len(fit.samples), span, 0, (1 << primary) | (1 << secondary))
            return None
        # Match firmware: lutBuilt is logged; valid mapping/result governs commit.
        lut_built = build_tendon_lut(self.io, self.joint, result, primary, secondary, 2, fit)
        ready_map = map_manual_scaled_encoder_to_urdf(self.io, self.joint, result,
                                                     ready_raw, self.io.direction(self.joint))
        if not ready_map or not ready_map["valid"]:
            self._workspace_failed(self.joint, self.joint, POSE_NONE, STAGE_POST_CALIBRATION)
            return None
        inside_ready = ready_map["outside_counts"] >= -config.kFeedbackReadyEnterOutsideToleranceCounts
        limits = {}
        for motor, tight_value in ((primary, tight_abs), (secondary, s_limit)):
            slack = self._delta(motor, tight_value, -config.kMotorAbsWindowCounts)
            margin = self._delta(motor, tight_value, config.kMotorAbsTightMarginCounts)
            limits[motor] = min(slack, margin), max(slack, margin)
            self.io.set_limit(motor, *limits[motor])
        if workspace:
            if inside_ready:
                workspace.primary_motor, workspace.secondary_motor = motors
                workspace.primary_release_abs, workspace.secondary_tight_abs = p_final, s_limit
                workspace.current_pose, workspace.valid = POSE_A, True
            else:
                workspace.valid = False
        self.io.trace("DUAL_RESULT", self.joint, primary,
                      span=span, signed_span=signed_encoder_delta(limit_encoder, tight_encoder),
                      tight_raw=tight_raw, tight_load=tight_load,
                      preload_raw=preload_raw, preload_encoder=preload_encoder,
                      preload_abs=preload_abs, preload_load=preload_load,
                      ready_raw=ready_raw, ready_load=ready_load,
                      ready_deg=ready_map["urdf_deg"], inside_ready=inside_ready,
                      lut_built=lut_built, raw_samples=len(fit.samples))
        return {"result": result, "limits": limits,
                "final_target_deg": ready_map["urdf_deg"],
                "requires_rope_init": not inside_ready}


def run_dual(io, joint, sequence_index, joint_count, selected_joint_mask, params):
    """Serial dual calibration; caller handles per-joint Manual/LUT commit."""
    machine = _DualMachine(io, joint, params)
    outcome = {"success": False, "aborted": False, "preflight_failed": False,
               "reason": protocol.CALIB_DIAG_REASON_NONE, "result": None, "limits": {}}
    motors = _dual_motors(joint)
    if motors is None:
        machine._diag(protocol.CALIB_DIAG_REASON_MOTOR_MAPPING_INVALID,
                      protocol.CALIB_DIAG_PHASE_INIT, joint, 255, joint, -1, -1, 0)
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
                          protocol.CALIB_DIAG_PHASE_DUAL_EXCHANGE, joint, motors[0], 0, 0, 0, 0)
    outcome["reason"] = machine.reason
    outcome["aborted"] = not outcome["success"] and not io.owns_control()
    return outcome
