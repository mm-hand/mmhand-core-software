"""runtime.py - 上位机校准的设备与持久化适配。

职责：提供可信反馈、原子目标执行确认、每关节 Manual/LUT 写回和多圈锚点。
线程：校准工作线程调用；通过控制器同步边界读取通信线程状态。
输入：HandController、会话取消条件和活动范围。
输出：原协议命令、宿主校准状态及逐关节快照。
状态所有权：本轮方向、工作限位和宿主状态；硬件 owner 仍由 P4 仲裁。
禁止依赖：禁止依赖 NiceGUI/UI、页面控件或替代 P4 的急停和执行安全校验。
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import threading
import time

from desktop.paths import CALIB_ACTIVE_SNAPSHOT_PATH

from ..angle_mapping import get_spec, urdf_manual_scaled_mapping_metadata
from ..calibration_snapshot import (
    CalibrationSnapshotStore, update_snapshot_multiturn_anchors,
)
from ..data_models import HandModel, resolve_joint_workspace_range
from ..communication.feedback_history import (
    FeedbackCursor, FeedbackHistoryError, TimestampedFeedbackHistory,
)
from ..motor_mapping import MOTOR_TO_JOINT_INDEX
from ..protocol import (
    CALIB_DIAG_PHASE_NONE, CALIB_DIAG_PHASE_SEQUENCE,
    CALIB_DIAG_REASON_NONE, CALIB_DIAG_REASON_ENCODER_ERROR,
    CALIB_DIAG_REASON_OVERLOAD, CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED,
    CALIB_DIAG_REASON_SERVO_RECOVERY_TIMEOUT, CALIB_DIAG_REASON_TARGET_APPLY_TIMEOUT,
    CMD_CALIB_LOAD_JOINT, CMD_CALIB_LOAD_LIMITS, CMD_MANUAL_ENCODER_SINGLE,
    CMD_MOTOR_DIRECT_RELEASE, CMD_MOTOR_POS_ABS,
    CMD_RESET, CMD_ROPE_INIT, CMD_START, ControlMode, ENCODER_COUNT, HAND_PROFILE,
    MOTOR_COUNT, PROTO_ACK_STATUS_OK, PROTO_ACK_STATUS_QUEUED,
    STATE_SYNC_ALL, SYSTEM_STATE_IDLE, SYSTEM_STATE_READY,
    SYSTEM_STATE_RECOVERY_REQUIRED, SYSTEM_STATE_RUN_DIRECT,
    SYSTEM_STATE_UNCONTROLLABLE, SYSTEM_FAULT_MULTITURN_UNKNOWN,
    SYSTEM_FAULT_SERVO_OFFLINE, SYSTEM_FAULT_CALIBRATION,
    STATE_SYNC_MULTITURN_STATE, VALID_MOTOR_MASK, build_calib_load_joint_cmd,
)
from ..session_log import log_session
from .config import (
    default_auto_calibration_params,
    default_encoder_direction, joint_motors, finger_aa_joint, shorten_sign,
    motor_apply_tendon_delta, kEncoderRecoveryStableSamples,
    kEncoderRecoveryTimeoutMs, kEncoderTrustedSampleMaxAgeMs,
    kFeedbackReadyServoTimeoutMs, kServoRecoveryStableSamples,
    kServoRecoveryTimeoutMs, kHostTargetResultTimeoutMs, kHostRebaseFeedbackTimeoutMs,
    kOverloadLoadThreshold, load_magnitude,
    kTendonFitSampleStepDeg,
)
from .math import (TendonRawFitBuffer, encoder_count_to_deg, tendon_lut_sample_angle,
                   wrap_encoder_delta, map_manual_scaled_encoder_to_urdf)
from .dependencies import usable_snapshot_aa_result
from .finish import dual_lut_targets
from .time_alignment import TimeAlignmentError, align_feedback


@dataclass(frozen=True)
class LutSamplingWindow:
    """A movement phase in one feedback history and logical coordinate epoch."""

    history: TimestampedFeedbackHistory
    cursor: FeedbackCursor
    joint: int
    motors: tuple[int, ...]
    direction: int


class HostCalibrationIO:
    """机械算法只调用此端口；所有下行位置均为 logical ABS counts。"""

    def __init__(self, controller, cancel_check, selected_joint_mask,
                 selected_motor_mask, *, reset_motor_mask=0, snapshot_store=None):
        self.controller = controller
        self.cancel_check = cancel_check
        self.selected_joint_mask = int(selected_joint_mask)
        self.active_joint_mask = self.selected_joint_mask
        self.selected_motor_mask = int(selected_motor_mask) & VALID_MOTOR_MASK
        self.preparation_reset_mask = int(reset_motor_mask or selected_motor_mask) & VALID_MOTOR_MASK
        self.snapshot_store = snapshot_store or CalibrationSnapshotStore(CALIB_ACTIVE_SNAPSHOT_PATH)
        self.snapshot_identity = {}
        self._cancel = threading.Event()
        self._initial_prepared = False
        self._motion_started = False
        self._workspace_mask = 0
        self._motion_scope = self.selected_motor_mask
        self._limits = {}
        self._committed_joint_results = {}
        self._aa_hold = None
        self._aa_drift = (None, 0)
        self.control_loss_reason = ''
        self._lut_sampling_windows = {}
        self._lut_sampling_diagnostics = {}
        self.lut_sampling_error = ''
        self._directions = [default_encoder_direction(j) for j in range(ENCODER_COUNT)]
        self._manual = None
        self._last_diag = None
        self.feedback_recovery_error = ''
        self.motion_publish_error = ''
        self.wait_exit_reason = ''
        self._rebase_zero_applied = False
        self._rebase_windows = []
        self._target_token = None
        self._session_finished = False
        self._saved_mode = controller.get_control_mode()

    def cancel(self):
        self._cancel.set()

    def is_cancelled(self):
        return self._cancel.is_set() or bool(self.cancel_check())

    def _state(self):
        with self.controller.state_lock:
            state = HandModel()
            state.copy_state_from(self.controller.current_state)
            state.calib_status = self.controller.current_state.calib_status
        return state

    def now_ms(self):
        return int(time.monotonic() * 1000)

    def sleep_ms(self, milliseconds):
        self._cancel.wait(max(0, milliseconds) / 1000.0)

    def owns_control(self):
        reason = ''
        if (self._cancel.is_set() or self.cancel_check() or self.controller.comm is None
                or self.controller._host_calibration_io is not self):
            reason = 'session cancelled, replaced or disconnected'
        else:
            state = self._state()
            age = time.monotonic() - state.received_at('system_status_last_rx_ts')
            if not state.has_system_status_data or age > 1.0:
                reason = f'system status stale or missing (age={age:.3f}s)'
            elif self._motion_started:
                scope = self._motion_scope
                faults = state.system_fault_bitmap & ~SYSTEM_FAULT_CALIBRATION
                if not (scope & ~state.servo_online_bitmap):
                    faults &= ~SYSTEM_FAULT_SERVO_OFFLINE
                if not (scope & state.multi_turn_unknown_bitmap):
                    faults &= ~SYSTEM_FAULT_MULTITURN_UNKNOWN
                if faults or scope & state.servo_route_conflict_bitmap:
                    reason = f'faults=0x{faults:X}, route conflicts=0x{scope & state.servo_route_conflict_bitmap:X}'
                elif (state.system_state == SYSTEM_STATE_RUN_DIRECT and
                      state.control_enabled and state.servo_target_owner == 1):
                    return True
                elif not (state.system_state in (SYSTEM_STATE_IDLE, SYSTEM_STATE_READY,
                        SYSTEM_STATE_RECOVERY_REQUIRED) and not state.control_enabled and
                        state.servo_target_owner == 0 and state.direct_target_token == self._target_token
                        and state.direct_target_status == 2):
                    reason = (f'state={state.system_state}, owner={state.servo_target_owner}, '
                              f'token={state.direct_target_token}/{self._target_token}, result={state.direct_target_status}')
            elif (state.system_state not in (SYSTEM_STATE_UNCONTROLLABLE, SYSTEM_STATE_IDLE,
                    SYSTEM_STATE_READY, SYSTEM_STATE_RECOVERY_REQUIRED)
                    or state.control_enabled or state.servo_target_owner):
                reason = f'not stopped: state={state.system_state}, owner={state.servo_target_owner}'
        if reason:
            if reason != self.control_loss_reason:
                log_session('HOST_CALIB', f'control lost: {reason}')
            self.control_loss_reason = reason
        return not reason

    @staticmethod
    def _age(state, timestamp, received_at):
        if not received_at:
            return float('inf')
        now = time.monotonic()
        host_age = max(0.0, (now - received_at) * 1000)
        if state.has_p4_time:
            # p4_time_ms 是最近一个包的时间；任务诊断/安全事件可携带历史时间。
            # SYSTEM_STATUS 携带当前设备时间，配合其接收时刻估计现在的样本年龄。
            reference = state.p4_time_ms
            elapsed = 0.0
            if state.has_system_status_data and state.received_at('system_status_last_rx_ts'):
                reference = state.system_status_p4_time_ms
                elapsed = max(0.0, (now - state.received_at('system_status_last_rx_ts')) * 1000)
            delta = (reference - timestamp) & 0xFFFFFFFF
            if delta >= 0x80000000:
                delta -= 0x100000000
            return max(host_age, max(0.0, delta + elapsed))
        return host_age

    def snapshot(self):
        state = self._state()
        age = self._age(state, state.servo_feedback_p4_time_ms,
                        state.received_at('servo_abs_last_rx_ts'))
        return {
            'servo_feedback_age_ms': age,
            'servo_online_bitmap': state.servo_online_bitmap,
            'servo_route_conflict_bitmap': state.servo_route_conflict_bitmap,
            'multi_turn_unknown_bitmap': state.multi_turn_unknown_bitmap,
            'servo_feedback_timestamp_ms': self.now_ms() - age,
            'system_state': state.system_state,
            'servo_target_owner': state.servo_target_owner,
            'control_enabled': state.control_enabled,
            'active_joint_mask': state.active_joint_mask,
            'calibration_target_apply_status': state.direct_target_status,
        }

    @property
    def target_apply_status(self):
        return self._state().direct_target_status

    def encoder_raw(self, joint):
        state = self._state()
        bit = 1 << joint
        age = self._age(state, getattr(state, 'encoder_raw_p4_time_ms', 0),
                        state.received_at('encoder_raw_last_rx_ts'))
        if (not state.has_encoder_raw_data or age > kEncoderTrustedSampleMaxAgeMs
                or not state.can_link_online or not (state.encoder_online_bitmap & bit)
                or state.encoder_fault_bitmap & bit or state.encoders[joint].raw_error):
            return None
        return int(state.encoders[joint].raw_sensor) % 16384

    def encoder(self, joint):
        raw = self.encoder_raw(joint)
        return None if raw is None else (raw * self.direction(joint)) % 16384

    def servo_abs(self, motor):
        state = self._state()
        if (not state.has_servo_angle_data or
                not state.servo_online_bitmap & (1 << motor) or
                self._age(state, state.servo_feedback_p4_time_ms,
                          state.received_at('servo_abs_last_rx_ts')) > kFeedbackReadyServoTimeoutMs):
            return None
        return int(state.servo_angles[motor])

    def servo_raw(self, motor):
        state = self._state()
        if (self.servo_abs(motor) is None or not state.has_servo_raw_data
                or not state.servo_raw_online[motor]):
            return None
        return int(state.servo_raw_positions[motor]) % 4096

    def servo_load(self, motor):
        state = self._state()
        if (not state.servo_telem_online[motor] or self._age(
                state, getattr(state, 'servo_telem_p4_time_ms', 0),
                state.received_at('servo_telem_last_rx_ts')) > kFeedbackReadyServoTimeoutMs):
            return None
        return int(state.servo_load[motor])

    def begin_lut_sampling(self, joint, motors):
        """Start independent raw(t)/ABS(t) tables without polling latest values."""
        joint, motors = int(joint), tuple(int(motor) for motor in motors)
        self.lut_sampling_error = ''
        self._lut_sampling_diagnostics.pop(joint, None)
        if (not 0 <= joint < ENCODER_COUNT or
                not self.selected_joint_mask & (1 << joint) or
                motors != joint_motors(joint)):
            self.lut_sampling_error = 'time-aligned LUT sampling scope does not match joint'
            raise TimeAlignmentError(self.lut_sampling_error)
        history = getattr(self.controller.comm, 'feedback_history', None)
        if not isinstance(history, TimestampedFeedbackHistory):
            self.lut_sampling_error = 'time-aligned LUT sampling requires P4 packet history'
            raise TimeAlignmentError(self.lut_sampling_error)
        window = LutSamplingWindow(history, history.mark(), joint, motors,
                                   self.direction(joint))
        self._lut_sampling_windows[joint] = window
        return window

    def finish_lut_sampling(self, window):
        """Fit both time streams and retain the old 5-degree spatial sample rule.

        This is pure processing of the packets already received. In particular,
        it must not wait here: the single-tendon caller has not published its
        final release-position hold until its step returns.
        """
        try:
            if (not isinstance(window, LutSamplingWindow) or
                    self._lut_sampling_windows.get(window.joint) is not window):
                raise TimeAlignmentError('time-aligned LUT sampling window is not active')
            self._lut_sampling_windows.pop(window.joint)
            if (getattr(self.controller.comm, 'feedback_history', None) is not window.history or
                    self.direction(window.joint) != window.direction):
                raise TimeAlignmentError('time-aligned LUT sampling source or direction changed')
            aligned = align_feedback(
                window.history.read(window.cursor), window.joint, window.motors,
                window.direction, raw_max_gap_ms=kEncoderTrustedSampleMaxAgeMs,
                abs_max_gap_ms=kFeedbackReadyServoTimeoutMs)
            fit = TendonRawFitBuffer()
            first = aligned.samples[0]
            fit.append(first.encoder, first.primary_abs, first.secondary_abs)
            fit.next_travel_deg = kTendonFitSampleStepDeg
            selected = [first]
            for sample in aligned.samples[1:]:
                count = fit.count
                travel = abs(encoder_count_to_deg(
                    wrap_encoder_delta(sample.encoder, first.encoder)))
                fit.maybe_append(travel, sample.encoder, sample.primary_abs,
                                 sample.secondary_abs)
                if fit.count > count:
                    selected.append(sample)
            last = aligned.samples[-1]
            count = fit.count
            if not fit.append(last.encoder, last.primary_abs, last.secondary_abs):
                raise TimeAlignmentError('time-aligned LUT endpoint buffer is full')
            if fit.count > count:
                selected.append(last)
            summary = dict(raw_points=aligned.raw_point_count,
                           abs_points=aligned.abs_point_count,
                           paired_points=len(aligned.samples), fit_points=fit.count,
                           start_ms=aligned.overlap_start_ms,
                           end_ms=aligned.overlap_end_ms,
                           raw_gap_ms=aligned.max_raw_gap_ms,
                           abs_gap_ms=aligned.max_abs_gap_ms)
            self._lut_sampling_diagnostics[window.joint] = dict(
                summary, samples=tuple(selected))
            self.trace('LUT_TIME_FIT', window.joint, window.motors[0], **summary)
            for sample in selected:
                self.trace('LUT_TIME_SAMPLE', window.joint, window.motors[0],
                           t=sample.timestamp_ms, encoder=sample.encoder,
                           primary_abs=sample.primary_abs,
                           secondary_abs=sample.secondary_abs,
                           raw_t=sample.raw_bracket_ms, abs_t=sample.abs_bracket_ms)
            if fit.count < 3:
                raise TimeAlignmentError(
                    f'insufficient common-time LUT samples: raw={aligned.raw_point_count} '
                    f'ABS={aligned.abs_point_count} paired={len(aligned.samples)} fit={fit.count}')
            return fit
        except (FeedbackHistoryError, TimeAlignmentError) as exc:
            self.lut_sampling_error = str(exc)
            raise TimeAlignmentError(str(exc)) from exc

    def direction(self, joint):
        return self._directions[joint]

    def set_direction(self, joint, direction):
        self._directions[joint] = -1 if direction < 0 else 1

    def manual_reference(self, joint):
        if self._manual is None:
            self._manual = self._state()
        state = self._manual
        return {'valid': bool(state.manual_reference_valid_bitmap & (1 << joint)),
                'minRaw': int(state.manual_reference_min_raw[joint]),
                'maxRaw': int(state.manual_reference_max_raw[joint])}

    def urdf_limits(self, joint):
        return get_spec(joint).urdf_range_deg

    def urdf_config_crc(self):
        return urdf_manual_scaled_mapping_metadata()[1]

    def logical_range(self, motor):
        state = self._state()
        if not state.servo_logical_range_valid_bitmap & (1 << motor):
            return None
        return (int(state.servo_logical_range_min[motor]),
                int(state.servo_logical_range_max[motor]))

    def inside_limit(self, motor, target):
        bounds = self.logical_range(motor)
        return bounds is not None and bounds[0] <= target <= bounds[1]

    def clear_limits(self, joint):
        for motor in joint_motors(joint):
            self._limits.pop(motor, None)

    def set_limit(self, motor, minimum, maximum):
        self._limits[int(motor)] = (int(minimum), int(maximum))

    def hold_targets_except(self, joints, active_joint, skip_joint_mask, params):
        targets = {}
        for joint in joints:
            if joint == active_joint or skip_joint_mask & (1 << joint):
                continue
            for motor in joint_motors(joint):
                actual = self.servo_abs(motor)
                if actual is None:
                    return None
                targets[motor] = actual
        if self._aa_hold:
            aa, result, center, fixed = self._aa_hold
            try:
                state, angle = self._finger_aa_feedback(aa, result)
                stamp = state.encoder_raw_p4_time_ms
                last, count = self._aa_drift
                if stamp != last:
                    count = count + 1 if abs(angle - center) > 3.0 else 0
                    self._aa_drift = (stamp, count)
                if count >= 3:
                    raise ValueError(f'AA drifted from midpoint {center:.2f} deg (actual {angle:.2f})')
            except ValueError as exc:
                self.motion_publish_error = f'J{aa:02d}: {exc}'
                self.trace('AA_MIDPOINT_FAILED', aa, reason=str(exc))
                return None
            targets.update(fixed)
        return targets

    @staticmethod
    def finger_aa_motor_mask(joints):
        aas = {finger_aa_joint(j) for j in joints} - {None}
        return sum(1 << m for aa in aas for m in joint_motors(aa))

    def _host_snapshot_result(self, aa):
        """Full AA result from the host snapshot for a cross-session dependency."""
        try:
            snapshot = self.snapshot_store.load()
        except (OSError, ValueError, KeyError, TypeError):
            return None
        result = usable_snapshot_aa_result(
            snapshot, aa, self._state(), self.urdf_config_crc()
        )
        if result is not None:
            log_session(
                "CALIB_AA",
                f"J{aa:02d} using host snapshot AA result for dependency resolution",
            )
        return result

    def _aa_mapping_usable(self, aa, result) -> bool:
        """Validate the AA encoder mapping without requiring fresh feedback."""
        state = self._state()
        if not state.has_encoder_raw_data or not state.has_manual_reference_encoder_directions:
            return False
        if not state.encoder_online_bitmap & (1 << aa) or state.encoder_fault_bitmap & (1 << aa):
            return False
        if not state.manual_reference_valid_bitmap & (1 << aa):
            return False
        direction = int(state.manual_reference_encoder_directions[aa])
        return map_manual_scaled_encoder_to_urdf(
            self, aa, result, int(state.encoders[aa].raw_sensor), direction
        ) is not None

    def _finger_aa_plan(self, aa):
        state = self._state()
        result = self.calibration_result(aa)
        if result is not None:
            state.joint_calib_results[aa] = result
            result['hasManualScaledV4Fields'] = True
            state.auto_calibration_valid_bitmap |= 1 << aa
        else:
            # Cross-session dependency: use the full host snapshot result (with
            # host-only manualSpanCounts) instead of the incomplete P4 readback.
            result = self._host_snapshot_result(aa)
            if result is not None:
                # Snapshot JSON stores only the persisted fields; the live model
                # uses this marker to accept a full V4 result. The snapshot
                # loader already validated mapping/CRC/LUT, so mark it here.
                result['hasManualScaledV4Fields'] = True
                state.joint_calib_results[aa] = result
                state.auto_calibration_valid_bitmap |= 1 << aa
            else:
                result = state.joint_calib_results[aa]
        workspace = resolve_joint_workspace_range(state, aa, *self.urdf_limits(aa),
                                                  4, self.urdf_config_crc())
        if not workspace.ready:
            raise ValueError(f'load or calibrate AA first: {workspace.reason}')
        if not state.has_manual_reference_encoder_directions:
            raise ValueError('AA encoder installation direction is unavailable')
        motors = joint_motors(aa)
        mask = sum(1 << m for m in motors)
        if (not state.has_servo_route_status or mask & ~state.servo_route_controllable_bitmap
                or mask & state.servo_route_conflict_bitmap or mask & state.multi_turn_unknown_bitmap):
            raise ValueError('AA motor routes or multi-turn coordinates are untrusted')
        first = float(result['tendonLutAngleStartDeg'])
        # 末点按静态 angleMax 截断（与 P4 的取样角规则一致）。
        last = tendon_lut_sample_angle(result, int(result['tendonLutCount']) - 1)
        # 工作区始终使用静态 URDF Min/Max；currentMin/Max 概念已删除。
        low = max(workspace.urdf_min_deg, first)
        high = min(workspace.urdf_max_deg, last)
        center = (low + high) / 2
        if low >= high or dual_lut_targets(result, motors, center) is None:
            raise ValueError('AA midpoint requires a valid dual LUT; extrapolation is forbidden')
        limits = {}
        for motor in motors:
            bounds = self._limits.get(motor)
            if bounds is None and state.motor_abs_limit_valid_bitmap & (1 << motor):
                bounds = state.motor_abs_limit_min[motor], state.motor_abs_limit_max[motor]
            hardware = self.logical_range(motor)
            if bounds is None or hardware is None:
                raise ValueError('AA midpoint requires trusted motor limits')
            limits[motor] = max(bounds[0], hardware[0]), min(bounds[1], hardware[1])
            if limits[motor][0] >= limits[motor][1]:
                raise ValueError('AA motor limits are invalid')
        return result, center, limits

    def preflight_finger_aa(self, joints):
        """Check unselected dependencies before any selected motor is reset."""
        for aa in sorted({finger_aa_joint(j) for j in joints} - {None}):
            if self.selected_joint_mask & (1 << aa):
                continue
            try:
                result, _center, _limits = self._finger_aa_plan(aa)
                # Validate mapping before the flow resets/invalidate any selected
                # joint; a rejected dependency must not destroy a good result.
                if not self._aa_mapping_usable(aa, result):
                    raise ValueError('AA encoder mapping is invalid')
            except (ValueError, TypeError, KeyError, IndexError) as exc:
                self.feedback_recovery_error = f'J{aa:02d}: {exc}'
                return False
        return True

    def _finger_aa_feedback(self, aa, result):
        state = self._state()
        mask = sum(1 << m for m in joint_motors(aa))
        if (not self._fresh_feedback(state, mask) or mask & ~state.servo_route_controllable_bitmap
                or mask & state.servo_route_conflict_bitmap
                or not state.has_encoder_raw_data or not state.can_link_online
                or not state.encoder_online_bitmap & (1 << aa)
                or state.encoder_fault_bitmap & (1 << aa) or state.encoders[aa].raw_error
                or self._age(state, state.encoder_raw_p4_time_ms,
                             state.received_at('encoder_raw_last_rx_ts')) > kEncoderTrustedSampleMaxAgeMs):
            raise ValueError('AA feedback is stale, offline or untrusted')
        mapped = map_manual_scaled_encoder_to_urdf(
            self, aa, result, state.encoders[aa].raw_sensor,
            state.manual_reference_encoder_directions[aa])
        if mapped is None or not mapped['valid']:
            raise ValueError('AA encoder mapping is invalid')
        return state, mapped['urdf_deg']

    def prepare_finger_aa(self, joint):
        self._aa_hold = None
        self._aa_drift = (None, 0)
        aa = finger_aa_joint(joint)
        if aa is None:
            return True, ''
        try:
            result, center, limits = self._finger_aa_plan(aa)
            state, angle = self._finger_aa_feedback(aa, result)
            params = default_auto_calibration_params(aa)
            motors = joint_motors(aa)
            count = int(result['tendonLutCount'])
            # LUT determines tendon direction only; reverse travel has backlash.
            response = [(row[count - 1] - row[0]) * shorten_sign(m)
                        for m, row in zip(motors, result['tendonLutMotorAbs'])]
            if not (response[0] * response[1] < 0):
                raise ValueError('AA tendon directions are invalid')
            targets = {m: state.servo_angles[m] for m in motors}
            speed = min(100, params.servoSpeed)
            last_command = progress_at = self.now_ms()
            progress_angle = angle
            deadline = last_command + 60000
            self.trace('AA_MIDPOINT', aa, target_deg=center)
            from .dual_motor import invalidate_dual_workspace_cache_joint
            invalidate_dual_workspace_cache_joint(self, aa)
            stable, last = 0, (state.encoder_raw_p4_time_ms, state.servo_feedback_p4_time_ms)
            while self.owns_control() and self.now_ms() <= deadline:
                state, angle = self._finger_aa_feedback(aa, result)
                stamp = state.encoder_raw_p4_time_ms, state.servo_feedback_p4_time_ms
                if stamp[0] != last[0] and stamp[1] != last[1]:
                    last = stamp
                    now = self.now_ms()
                    if abs(angle - progress_angle) >= 0.5:
                        progress_at, progress_angle = now, angle
                    if abs(angle - center) <= 2:
                        targets = {m: state.servo_angles[m] for m in motors}
                        stable += 1
                    else:
                        stable = 0
                        if now - progress_at >= 20000:
                            raise ValueError('AA angle did not follow tendon movement for 20 seconds')
                        step = max(1, speed * min(now - last_command, 100) // 1000)
                        for motor, direction in zip(motors, response):
                            delta = step if (direction > 0) == (angle < center) else -step
                            command = motor_apply_tendon_delta(motor, targets[motor], delta)
                            # Never accumulate a target far ahead of actual feedback.
                            actual = state.servo_angles[motor]
                            targets[motor] = max(actual - 40, min(actual + 40, command))
                    # Trusted-window overshoot at the biased ends is expected; only
                    # forbid moving further outside than the current actual position.
                    if any(max(limits[m][0] - value, value - limits[m][1], 0)
                           > max(limits[m][0] - state.servo_angles[m],
                                 state.servo_angles[m] - limits[m][1], 0)
                           for m, value in targets.items()):
                        raise ValueError('AA midpoint movement exceeded trusted motor limits')
                    if not self.publish(targets, speed, params.servoAcc):
                        raise ValueError(self.motion_publish_error or 'AA midpoint target publish failed')
                    last_command = now
                    if stable >= 3:
                        self._aa_hold = (aa, result, center, targets)
                        return True, ''
                self.sleep_ms(params.loopMs)
            raise ValueError('AA midpoint arrival timed out or control was lost')
        except (ValueError, TypeError, KeyError, IndexError) as exc:
            message = f'J{aa:02d}: {exc}'
            self.trace('AA_MIDPOINT_FAILED', aa, reason=str(exc))
            return False, message

    def _wait(self, predicate, timeout_ms):
        self.wait_exit_reason = ''
        deadline = self.now_ms() + timeout_ms
        while self.now_ms() <= deadline:
            if self.is_cancelled():
                self.wait_exit_reason = 'cancelled'
                return False
            if predicate():
                self.wait_exit_reason = 'ready'
                return True
            self.sleep_ms(5)
        self.wait_exit_reason = 'cancelled' if self.is_cancelled() else 'timeout'
        return False

    def _ack(self, command, value, timeout=1.0):
        if self._cancel.is_set() or self.cancel_check() or not self.controller.comm:
            return False
        sent_at = time.monotonic()
        if not self.controller.comm.send_command(value):
            return False
        ack = self.controller.comm.wait_for_ack(command, timeout, received_after_monotonic=sent_at)
        if ack is None or int(ack[2]) not in (PROTO_ACK_STATUS_OK, PROTO_ACK_STATUS_QUEUED):
            return False
        if command == CMD_MANUAL_ENCODER_SINGLE and int(ack[2]) == PROTO_ACK_STATUS_QUEUED:
            # Manual 是异步提交；LUT 的 entry CRC 必须对应提交后的端点。
            return self.controller._wait_for_manual_reference_commit_result(sent_at, timeout)
        return True

    def _release_motion(self):
        state = self._state()
        if state.control_enabled or state.servo_target_owner != 0:
            # 所有普通直控共用结束接口；急停/保护故障不由标定解除。
            token = int(getattr(state, 'direct_target_token', 0))
            if not self._ack(CMD_MOTOR_DIRECT_RELEASE,
                             ('motor_direct_release', token)):
                return False
            if not self._wait(lambda: not self._state().control_enabled and
                              self._state().servo_target_owner == 0, 1000):
                return False
        self._motion_started = False
        self.controller.invalidate_direct_commands()
        return True

    def _rebase(self, mask):
        self._rebase_zero_applied = False
        self.feedback_recovery_error = ''
        if not self._release_motion():
            self.feedback_recovery_error = 'could not release the calibration motion owner'
            return False
        state = self._state()
        window = {'mask': int(mask), 'start_ms': state.system_status_p4_time_ms,
                  'start_rx': time.monotonic(), 'active': True, 'end_ms': None}
        with self.controller.state_lock:
            self._rebase_windows.append(window)
        try:
            return self._rebase_and_wait(mask, window)
        finally:
            with self.controller.state_lock:
                window['active'] = False

    def is_expected_calibration_turn_unknown(self, state):
        """只识别本轮静止 F0 中间帧；不改变真实故障位或运动准入。"""
        if (self.controller._host_calibration_io is not self or
                state.system_fault_bitmap != SYSTEM_FAULT_MULTITURN_UNKNOWN or
                state.control_enabled or state.servo_target_owner != 0 or
                state.system_state not in (SYSTEM_STATE_UNCONTROLLABLE, SYSTEM_STATE_IDLE,
                    SYSTEM_STATE_READY, SYSTEM_STATE_RECOVERY_REQUIRED)):
            return False
        unknown = int(state.multi_turn_unknown_bitmap)
        if not unknown:
            return False
        with self.controller.state_lock:
            windows = [dict(window) for window in self._rebase_windows]
        for window in windows:
            if (unknown & ~window['mask'] or
                    state.received_at('system_status_last_rx_ts') < window['start_rx']):
                continue
            delta = (state.system_status_p4_time_ms - window['start_ms']) & 0xFFFFFFFF
            if delta >= 0x80000000:
                continue
            if window['active'] and not self._cancel.is_set() and not self.cancel_check():
                return True
            # GUI 可晚于 worker 收到该帧；只抑制已经完成归零且当前已 Known 的旧中间帧。
            end_ms = window['end_ms']
            if (end_ms is not None and delta <= ((end_ms - window['start_ms']) & 0xFFFFFFFF)
                    and not (self._state().multi_turn_unknown_bitmap & unknown)):
                return True
        return False

    def _rebase_and_wait(self, mask, window):
        def submit(callback):
            # controller 的命令锁与 STOP 串行；等待最终结果不持该锁。
            if self.is_cancelled() or not self.owns_control():
                return False
            return callback()

        result = self.controller.rebase_servo_abs_zero(
            mask, cancelled=self.is_cancelled, send_gate=submit,
        )
        self._clear_result_scope(mask)
        if not result.ok:
            self.feedback_recovery_error = result.message
            log_session('HOST_CALIB', result.message)
            return False
        self._rebase_zero_applied = True
        completed = self._state()
        after_ms = completed.servo_rebase_zero_result_p4_time_ms
        requested_at = time.monotonic()
        self.controller.request_state_sync()
        accepted_anchor_ms = None
        # 新坐标的 ABS/raw 和硬件行程都必须来自这次结果之后。
        def rebased_feedback():
            nonlocal accepted_anchor_ms
            state = self._state()
            delta = (state.servo_feedback_p4_time_ms - after_ms) & 0xFFFFFFFF
            range_delta = (getattr(state, 'servo_logical_range_p4_time_ms', 0)
                           - after_ms) & 0xFFFFFFFF
            ready = (self._fresh_anchor(state, mask) and 0 < delta < 0x80000000
                     and 0 < range_delta < 0x80000000
                     and state.received_at('servo_logical_range_last_rx_ts') >= requested_at
                     and state.servo_logical_range_valid_bitmap & mask == mask)
            if ready:
                accepted_anchor_ms = state.servo_feedback_p4_time_ms
            return ready
        if not self._wait(rebased_feedback, kHostRebaseFeedbackTimeoutMs):
            state = self._state()
            stopped = self.is_cancelled() or self.wait_exit_reason == 'cancelled'
            reason = 'cancelled/stopped' if stopped else 'timeout'
            unmet = self._rebase_feedback_unmet(state, mask, after_ms, requested_at)
            self.feedback_recovery_error = (
                f'feedback wait {reason} after logical ABS zero was applied: '
                f'mask=0x{mask:08X} result_ms={after_ms} '
                f'ABS_ms={state.servo_feedback_p4_time_ms} raw_ms={state.servo_raw_p4_time_ms} '
                f'range_ms={state.servo_logical_range_p4_time_ms} '
                f'unknown=0x{mask & state.multi_turn_unknown_bitmap:08X} '
                f'unmet={",".join(unmet) if unmet else "none"}'
            )
            log_session('HOST_CALIB', self.feedback_recovery_error)
            return False
        # The new turn-zero anchor must be persisted before prepare_joint
        # invalidates this joint's scope.  If the invalidation runs first, the
        # anchor save sees "motors not active in calibration snapshot", the
        # capture mask never clears, and the next rebase is rejected with
        # "previous servo ABS rebase is awaiting its result/new feedback".
        def anchor_captured():
            return not getattr(self.controller, '_servo_rebase_zero_capture_mask', 0)
        if not self._wait(anchor_captured, kHostRebaseFeedbackTimeoutMs):
            capture_mask = getattr(self.controller, '_servo_rebase_zero_capture_mask', 0)
            self.feedback_recovery_error = (
                'logical ABS zero applied but the new feedback anchor was not saved '
                f'(capture_mask=0x{capture_mask:08X})')
            log_session('HOST_CALIB', self.feedback_recovery_error)
            return False
        with self.controller.state_lock:
            # SYSTEM_STATUS 可以比归零后的 Known 锚点更旧；以实际验收的
            # post-result ABS/raw 快照为截止，才能识别 NiceGUI 延迟的 F0 中间帧。
            window['end_ms'] = accepted_anchor_ms
        return True

    def _rebase_feedback_unmet(self, state, mask, after_ms, requested_at):
        """列出归零等待的实际缺项，保留同戳、可信和坐标更新门槛。"""
        unmet = []
        if not state.has_servo_angle_data:
            unmet.append('missing_abs')
        unknown = state.multi_turn_unknown_bitmap & mask
        offline = mask & ~state.servo_online_bitmap
        unroutable = mask & ~state.servo_route_controllable_bitmap
        if unknown:
            unmet.append(f'unknown=0x{unknown:08X}')
        if offline:
            unmet.append(f'offline=0x{offline:08X}')
        if unroutable:
            unmet.append(f'unroutable=0x{unroutable:08X}')
        age = self._age(state, state.servo_feedback_p4_time_ms,
                        state.received_at('servo_abs_last_rx_ts'))
        if age > kFeedbackReadyServoTimeoutMs:
            unmet.append(f'stale_abs(age={age:.0f}ms)')
        if not state.has_servo_raw_data:
            unmet.append('missing_raw')
        raw_offline = sum(1 << motor for motor in range(MOTOR_COUNT)
                          if mask & (1 << motor) and not state.servo_raw_online[motor])
        if raw_offline:
            unmet.append(f'raw_offline=0x{raw_offline:08X}')
        if state.servo_feedback_p4_time_ms != state.servo_raw_p4_time_ms:
            unmet.append('abs_raw_timestamp_mismatch')
        delta = (state.servo_feedback_p4_time_ms - after_ms) & 0xFFFFFFFF
        if not 0 < delta < 0x80000000:
            unmet.append('abs_not_after_zero_result')
        range_delta = (state.servo_logical_range_p4_time_ms - after_ms) & 0xFFFFFFFF
        if not 0 < range_delta < 0x80000000:
            unmet.append('range_not_after_zero_result')
        if state.received_at('servo_logical_range_last_rx_ts') < requested_at:
            unmet.append('range_before_sync_request')
        missing_range = mask & ~state.servo_logical_range_valid_bitmap
        if missing_range:
            unmet.append(f'missing_range=0x{missing_range:08X}')
        return unmet

    def _clear_result_scope(self, mask):
        state = self._state()
        results = deepcopy(state.joint_calib_results)
        joint_mask = 0
        mins, maxs = list(state.motor_abs_limit_min), list(state.motor_abs_limit_max)
        for motor in range(MOTOR_COUNT):
            if mask & (1 << motor):
                joint = MOTOR_TO_JOINT_INDEX[motor]
                joint_mask |= 1 << joint
                results[joint] = None
                mins[motor] = maxs[motor] = 0
        self._overlay(joint_calib_results=results, motor_abs_limit_min=mins,
                      motor_abs_limit_max=maxs,
                      motor_abs_limit_valid_bitmap=state.motor_abs_limit_valid_bitmap & ~mask,
                      auto_calibration_valid_bitmap=state.auto_calibration_valid_bitmap & ~joint_mask,
                      calibration_warning_bitmap=state.calibration_warning_bitmap & ~joint_mask)

    def _fresh_feedback(self, state, mask):
        return (state.has_servo_angle_data and
                not state.multi_turn_unknown_bitmap & mask and
                state.servo_online_bitmap & mask == mask and
                state.servo_route_controllable_bitmap & mask == mask and
                self._age(state, state.servo_feedback_p4_time_ms,
                          state.received_at('servo_abs_last_rx_ts')) <= kFeedbackReadyServoTimeoutMs)

    def _fresh_anchor(self, state, mask):
        return (self._fresh_feedback(state, mask) and state.has_servo_raw_data and
                state.servo_feedback_p4_time_ms == state.servo_raw_p4_time_ms and
                all(state.servo_raw_online[m] for m in range(MOTOR_COUNT) if mask & (1 << m)))

    def _prepare_device(self):
        # 与原 prepare_calibration_turn_zero 相同：每次校准入口一次维护 RESET。
        # 直接走通信端口，controller.reset() 会取消当前宿主工作线程。
        if not self.owns_control():
            self.feedback_recovery_error = 'calibration preparation lost control or fresh system status'
            return False
        # The startup restore may already be in flight when calibration takes
        # ownership. Let its verified result drain before changing coordinates;
        # the controller will not start another restore during this session.
        if not self._wait(lambda: not getattr(self.controller, '_multiturn_pending_restore_mask', 0), 5000):
            self.feedback_recovery_error = 'startup coordinate restore has not completed; calibration not started'
            return False
        self.controller.invalidate_direct_commands()
        sent_at = time.monotonic()
        if not self._ack(CMD_RESET, 'reset'):
            self.feedback_recovery_error = 'maintenance RESET timeout/reject'
            return False
        self.sleep_ms(300)  # 原 CALIBRATION_TURN_RESET_SETTLE_S = 0.30。
        if not self._wait(lambda: self._state().received_at('system_status_last_rx_ts') >= sent_at
                          and self.owns_control(), 1000):
            self.feedback_recovery_error = 'waiting for stopped P4 state after maintenance RESET'
            return False
        return True

    def _prepare_workspace_reset_scope(self):
        # Never reset or preposition fingers outside the explicit calibration scope.
        self.preparation_reset_mask &= self.selected_motor_mask
        return True

    def wait_servo_recovery(self, mask):
        self.feedback_recovery_error = ''
        if not self._initial_prepared:
            state = self._state()
            if not getattr(state, 'has_direct_target_result', False):
                self.controller.request_state_sync()
                if not self._wait(lambda: getattr(self._state(),
                                  'has_direct_target_result', False), 2500):
                    self.feedback_recovery_error = 'P4 direct execution result capability missing (0x1A)'
                    return False
            if not self._prepare_device():
                return False
            if not self._prepare_workspace_reset_scope():
                return False
            if self.preparation_reset_mask and not self._rebase(self.preparation_reset_mask):
                return False
            self._initial_prepared = True
        stable = 0
        last_timestamp = None
        deadline = self.now_ms() + kServoRecoveryTimeoutMs
        while self.now_ms() <= deadline and self.owns_control():
            state = self._state()
            # 原 waitForCalibrationServoRecovery 只按新 ABS 快照累计稳定帧。
            # Raw/ABS 是独立上行包；同戳要求只用于归零后的锚点采集。
            ready = (self._fresh_feedback(state, mask) and
                     state.servo_logical_range_valid_bitmap & mask == mask)
            if ready and state.servo_feedback_p4_time_ms != last_timestamp:
                stable += 1
                last_timestamp = state.servo_feedback_p4_time_ms
            elif not ready:
                stable = 0
            if stable >= kServoRecoveryStableSamples:
                return True
            self.sleep_ms(20)
        self.diag_if_empty(CALIB_DIAG_REASON_SERVO_RECOVERY_TIMEOUT,
                           CALIB_DIAG_PHASE_SEQUENCE, 255, 255, 0, 0, 0, mask)
        state = self._state()
        age = self._age(state, state.servo_feedback_p4_time_ms, state.received_at('servo_abs_last_rx_ts'))
        self.feedback_recovery_error = (
            f'ABS age={age:.0f}ms stable={stable}/{kServoRecoveryStableSamples} '
            f'offline=0x{mask & ~state.servo_online_bitmap:08X} '
            f'unknown=0x{mask & state.multi_turn_unknown_bitmap:08X} '
            f'unroutable=0x{mask & ~state.servo_route_controllable_bitmap:08X} '
            f'missing_range=0x{mask & ~state.servo_logical_range_valid_bitmap:08X} '
            f'state={state.system_state} owner={state.servo_target_owner}'
        )
        log_session('HOST_CALIB', self.feedback_recovery_error)
        return False

    def wait_encoder_recovery(self, mask):
        stable = 0
        last_timestamp = None
        deadline = self.now_ms() + kEncoderRecoveryTimeoutMs
        while self.now_ms() <= deadline and self.owns_control():
            valid = sum(1 << j for j in range(ENCODER_COUNT)
                        if mask & (1 << j) and self.encoder(j) is not None)
            timestamp = getattr(self._state(), 'encoder_raw_p4_time_ms', 0)
            if valid == mask and timestamp != last_timestamp:
                stable += 1
                last_timestamp = timestamp
            elif valid != mask:
                stable = 0
            if stable >= kEncoderRecoveryStableSamples:
                return True
            self.sleep_ms(20)
        state = self._state()
        self.diag_if_empty(CALIB_DIAG_REASON_ENCODER_ERROR,
                          CALIB_DIAG_PHASE_SEQUENCE, 255, 255, 0,
                          state.encoder_online_bitmap, state.encoder_fault_bitmap, mask)
        return False

    def set_workspace_read_mask(self, mask):
        self._workspace_mask = int(mask)

    def prepare_joint(self, joint, mask):
        self.clear_limits(joint)
        self._lut_sampling_windows.pop(joint, None)
        self.lut_sampling_error = ''
        self._committed_joint_results.pop(joint, None)
        if not self._rebase(mask):
            if self._rebase_zero_applied:
                return False, f'J{joint:02d} logical ABS zero applied; {self.feedback_recovery_error}'
            if self.is_cancelled():
                return False, f'J{joint:02d} calibration cancelled before logical ABS zero completed: {self.feedback_recovery_error}'
            return False, f'J{joint:02d} logical ABS zero failed: {self.feedback_recovery_error}'
        self.snapshot_store.invalidate_scope(motor_mask=mask, joint_mask=1 << joint,
                                             reason='host_calibration_rebase')
        return True, ''

    def _ensure_motion(self):
        if self._motion_started:
            return self.owns_control()
        scope = self.selected_motor_mask | self._workspace_mask
        if not self.owns_control() or not self.controller.set_active_scope(
                scope, self.selected_joint_mask, True):
            return False
        self._motion_scope = scope
        self.controller.set_control_mode(ControlMode.DIRECT_MOTOR)
        if not self._ack(CMD_START, 'start'):
            return False
        return self._wait(lambda: self._state().system_state in (
            SYSTEM_STATE_IDLE, SYSTEM_STATE_READY, SYSTEM_STATE_RECOVERY_REQUIRED), 1000)

    def publish(self, targets, speed, acc):
        self.motion_publish_error = ''
        if not targets:
            return True
        if not self._ensure_motion():
            return False
        scope = self.selected_motor_mask | self._workspace_mask
        requested_mask = sum(1 << m for m in targets)
        if any(not scope & (1 << m) or not self.inside_limit(m, target)
               for m, target in targets.items()):
            return False
        for motor in targets:
            load = self.servo_load(motor)
            if load is not None and load_magnitude(load) >= kOverloadLoadThreshold:
                self.diag_if_empty(CALIB_DIAG_REASON_OVERLOAD,
                                  CALIB_DIAG_PHASE_SEQUENCE,
                                  MOTOR_TO_JOINT_INDEX[motor], motor,
                                  load_magnitude(load), kOverloadLoadThreshold,
                                  load, 1 << motor)
                return False
        state = self._state()
        values = list(state.servo_angles)
        for motor in range(MOTOR_COUNT):
            if requested_mask & (1 << motor) and self.servo_abs(motor) is None:
                return False
        for motor, target in targets.items():
            values[motor] = int(target)
        if self._target_token is None:
            self._target_token = int(getattr(state, 'direct_target_token', 0))
        token = (self._target_token + 1) & 0xFFFFFFFF
        token = token or 1
        self._target_token = token
        sent_at = time.monotonic()
        if not self.controller.comm.send_command((
                'motor_pos_abs', token, requested_mask,
                values, int(speed), int(acc))):
            return False
        # 仅执行层完整同步写结果才推进机械状态机；命令入队 ACK 不算执行。
        rejected_ack = []

        def applied():
            latest = self._state()
            if (getattr(latest, 'direct_target_token', 0) == token and
                    latest.received_at('direct_target_result_rx_ts') >= sent_at):
                return True
            ack = self.controller.comm.wait_for_ack(
                CMD_MOTOR_POS_ABS, 0, received_after_monotonic=sent_at)
            if ack is not None and int(ack[2]) not in (PROTO_ACK_STATUS_OK, PROTO_ACK_STATUS_QUEUED):
                rejected_ack.append(int(ack[2]))
                return True
            return False

        if not self._wait(applied, kHostTargetResultTimeoutMs):
            self.motion_publish_error = f'D3 token={token} execution result timeout ({kHostTargetResultTimeoutMs}ms)'
            self.diag_if_empty(CALIB_DIAG_REASON_TARGET_APPLY_TIMEOUT,
                              CALIB_DIAG_PHASE_SEQUENCE, 255, 255,
                              token, 0, kHostTargetResultTimeoutMs, requested_mask)
            self.controller.request_state_sync(STATE_SYNC_MULTITURN_STATE)
            return False
        if rejected_ack:
            self.motion_publish_error = f'D3 token={token} command rejected (ACK status={rejected_ack[0]})'
            self.diag_if_empty(CALIB_DIAG_REASON_TARGET_PUBLISH_FAILED,
                              CALIB_DIAG_PHASE_SEQUENCE, 255, 255,
                              token, 0, rejected_ack[0], requested_mask)
            return False
        state = self._state()
        success = (state.direct_target_status == 2 and
                   state.direct_target_requested_mask == requested_mask and
                   state.direct_target_applied_mask == requested_mask and
                   not state.direct_target_rejected_mask)
        if success:
            self._motion_started = True
            if self._wait(self.owns_control, kHostTargetResultTimeoutMs):
                return True
            self.motion_publish_error = f'D3 token={token} applied; waiting for direct execution state failed'
            return False
        self.motion_publish_error = (
            f'D3 token={token} execution status={state.direct_target_status} '
            f'requested=0x{state.direct_target_requested_mask:08X} '
            f'applied=0x{state.direct_target_applied_mask:08X} '
            f'rejected=0x{state.direct_target_rejected_mask:08X}'
        )
        return False

    def failure_message(self, reason):
        if reason == 'control lost' and self.control_loss_reason:
            return f'control lost: {self.control_loss_reason}'
        if self.lut_sampling_error:
            return self.lut_sampling_error
        if self.motion_publish_error:
            return self.motion_publish_error
        if isinstance(reason, int):
            from ..calibration_diagnostics import CALIBRATION_DIAGNOSTICS
            spec = CALIBRATION_DIAGNOSTICS.get(int(reason))
            return (spec.title if spec else '') or f'calibration failed (reason={reason})'
        return str(reason)

    def diag(self, reason, phase, joint, motor=255, a=0, b=0, c=0, bitmap=0):
        self._last_diag = (reason, phase, joint, motor, a, b, c, bitmap)

    def diag_if_empty(self, *args):
        if self._last_diag is None:
            self.diag(*args)

    def trace(self, state, joint, motor=255, **details):
        log_session('HOST_CALIB', f'J{joint:02d} M{motor:02d} {state} {details}')

    def session_state(self, status, progress=0, joint=255, completed=0, failed=0, **details):
        if status == 'PENDING':
            # 新一轮标定开始时清空上一轮的诊断与错误信息。
            self._last_diag = None
            self.motion_publish_error = ''
        self._overlay(calib_status=status)
        log_session('HOST_CALIB', f'{status} J{joint:02d} {progress}% '
                    f'completed=0x{completed:X} failed=0x{failed:X}')

    def _overlay(self, **fields):
        with self.controller.state_lock:
            if (self.controller._host_calibration_io is not self or
                    self._cancel.is_set() or self.cancel_check()):
                return
            # 实际 LUT/限位由 P4 遥测拥有；不能用永久整表覆盖后续 Manual/F0 失效。
            telemetry_fields = {
                'joint_calib_results', 'motor_abs_limit_min', 'motor_abs_limit_max',
                'motor_abs_limit_valid_bitmap', 'auto_calibration_valid_bitmap',
                'calibration_warning_bitmap', 'has_calibration_warning_status',
            }
            self.controller._host_calibration_overlay.update(deepcopy({
                key: value for key, value in fields.items() if key not in telemetry_fields
            }))
            for key, value in fields.items():
                setattr(self.controller.current_state, key, deepcopy(value))

    def commit_joint(self, joint, result, limits):
        if self.snapshot_identity and self.controller.calibration_data_identity != self.snapshot_identity:
            return False, 'device or model identity changed before calibration commit'
        if not result or not result.get('success'):
            return False, 'invalid host joint calibration result'
        if not result.get('tendonLutValid') or int(result.get('tendonLutCount', 0)) < 2:
            # 原机械结果仍在 flow 的关节记录里；不能上传无效 LUT 或伪造设备成功。
            return False, 'mechanical result retained in host diagnostics; LUT build invalid, not uploaded'
        payload = deepcopy(result)
        payload['joint'] = joint
        build_calib_load_joint_cmd(payload)  # 在发送任何数据前完整验证 V4 字段。
        if not self._release_motion():
            return False, 'could not release motion before Manual/LUT commit'
        manual = self.manual_reference(joint)
        if not manual['valid'] or not self._ack(CMD_MANUAL_ENCODER_SINGLE, (
                'manual_encoder_single', joint, manual['minRaw'], manual['maxRaw'])):
            return False, 'Manual writeback failed'
        # 不改变用户 Manual 端点；写回后才上传依赖该 entry CRC 的 LUT。
        if not self._ack(CMD_CALIB_LOAD_JOINT, ('calib_load_joint', payload)):
            return False, 'joint LUT writeback failed'
        state = self._state()
        mins, maxs = list(state.motor_abs_limit_min), list(state.motor_abs_limit_max)
        mask = sum(1 << m for m in joint_motors(joint))
        local_limits = limits or {m: self._limits[m] for m in joint_motors(joint)
                                 if m in self._limits}
        if any(m not in local_limits for m in joint_motors(joint)):
            return False, 'calibration motor limits incomplete'
        for motor, bounds in local_limits.items():
            if mask & (1 << int(motor)):
                mins[int(motor)], maxs[int(motor)] = int(bounds[0]), int(bounds[1])
        if not self._ack(CMD_CALIB_LOAD_LIMITS, (
                'calib_load_limits', mask, mask, mins, maxs)):
            return False, 'joint ABS limits writeback failed'
        if (self._cancel.is_set() or self.cancel_check() or
                self.controller._host_calibration_io is not self):
            return False, 'calibration cancelled after device writeback'
        with self.controller.state_lock:
            results = deepcopy(self.controller.current_state.joint_calib_results)
        results[joint] = payload
        warning_mask = state.calibration_warning_bitmap
        reason = int(payload.get('assemblyWarningReason', 0))
        warning_mask = warning_mask | (1 << joint) if reason else warning_mask & ~(1 << joint)
        self._overlay(joint_calib_results=results, motor_abs_limit_min=mins,
                      motor_abs_limit_max=maxs,
                      motor_abs_limit_valid_bitmap=state.motor_abs_limit_valid_bitmap | mask,
                      auto_calibration_valid_bitmap=state.auto_calibration_valid_bitmap | (1 << joint),
                      calibration_warning_bitmap=warning_mask,
                      has_calibration_warning_status=True)
        self.snapshot_store.merge_calibration({
            **self.snapshot_identity,
            'saved_at_host_time': time.time(), 'hand_profile': HAND_PROFILE,
            'feedback_mapping_version': 4, 'manual_scaled_mapping_version': 4,
            'urdf_config_crc': f'0x{self.urdf_config_crc():08X}',
            'manual_reference_crc': f'0x{state.manual_reference_crc:08X}',
            'active_joint_mask': hex(1 << joint), 'active_motor_mask': hex(mask),
            'motor_abs_limit_valid_bitmap': hex(mask),
            'motor_abs_limit_min': mins, 'motor_abs_limit_max': maxs,
            'manual_reference_valid_bitmap': hex(state.manual_reference_valid_bitmap),
            'manual_reference_min_raw': list(state.manual_reference_min_raw),
            'manual_reference_max_raw': list(state.manual_reference_max_raw),
            'joint_calib_results': [payload if j == joint else None for j in range(ENCODER_COUNT)],
            'source': {'transport': 'host_calibration',
                       'identity_contract': 'hand_profile+urdf_crc+manual_entry_crc'},
        }, 1 << joint)
        if not self._wait(lambda: self._fresh_anchor(self._state(), mask), 3000):
            return False, 'new joint result saved; waiting for trusted ABS/raw anchors'
        state = self._state()
        with self.controller._multiturn_io_lock:
            with self.controller._multiturn_anchor_capture_lock:
                self.controller._multiturn_anchor_capture_blocked_mask &= ~mask
            try:
                if not self.controller._persist_multiturn_state(state, force=True):
                    with self.controller._multiturn_anchor_capture_lock:
                        self.controller._multiturn_anchor_capture_blocked_mask |= mask
                    return False, 'new joint result saved; runtime anchor persistence failed'
                update_snapshot_multiturn_anchors(self.snapshot_store.path, motor_mask=mask,
                                                  logical_abs=list(state.servo_angles),
                                                  raw=list(state.servo_raw_positions))
            except Exception:
                with self.controller._multiturn_anchor_capture_lock:
                    self.controller._multiturn_anchor_capture_blocked_mask |= mask
                raise
        self._committed_joint_results[joint] = deepcopy(payload)
        return True, f'J{joint:02d} Manual/LUT/limits and multi-turn anchors saved'

    def calibration_result(self, joint):
        """本轮 host 生成且已成功写回的数据，收尾不依赖 P4 异步回显。"""
        return deepcopy(self._committed_joint_results.get(int(joint)))

    def finish_session(self, success, completed, failed):
        finished_ok = False
        try:
            if not success or self._cancel.is_set() or self.cancel_check():
                if self.controller.comm and self.controller._host_calibration_io is self:
                    self.controller.invalidate_direct_commands()
                    self.controller.comm.send_command('stop')
                    # 取消时仍完成安全收口，但不能改写已切换的另一个会话。
                    deadline = time.monotonic() + 1.0
                    while time.monotonic() < deadline:
                        if self.controller._host_calibration_io is not self:
                            break
                        state = self._state()
                        if not state.control_enabled and state.servo_target_owner == 0:
                            if (state.active_motor_mask ==
                                    self._motion_scope and
                                    state.active_joint_mask == self.selected_joint_mask):
                                self.controller.set_active_scope(
                                    self.selected_motor_mask, self.selected_joint_mask, True)
                            break
                        time.sleep(0.005)
                return False, 'calibration stopped; completed joints retained'
            if not self._release_motion():
                return False, 'calibration complete; motion owner release failed'
            if not self.controller.set_active_scope(self.selected_motor_mask,
                                                    self.selected_joint_mask, True):
                return False, 'calibration complete; active scope restore failed'
            if not self._ack(CMD_START, 'start'):
                return False, 'calibration complete; P4 preparation failed'
            baseline = self._state().rope_init_sequence
            if not self._ack(CMD_ROPE_INIT, ('rope_init', self.selected_joint_mask)):
                return False, 'calibration complete; Rope Init start failed'
            if not self._wait(lambda: self._state().rope_init_sequence != baseline and
                              self._state().rope_init_status in (2, 3), 30000):
                return False, 'calibration complete; Rope Init result timeout'
            if self._state().rope_init_status != 2:
                return False, 'Rope Init failed'
            # 原 Auto 任务的双腱 LUT0 收尾由 host 执行，P4 仍是普通直控。
            from .finish import finish_dual_zero
            finished_ok, message = finish_dual_zero(self, completed)
            return finished_ok, message
        finally:
            self._lut_sampling_windows.clear()
            if (success and not finished_ok and self.controller.comm and
                    self.controller._host_calibration_io is self):
                self.controller.invalidate_direct_commands()
                self.controller.comm.send_command('stop')
            self.controller.set_control_mode(self._saved_mode)
            self._motion_started = False
            self._session_finished = True
