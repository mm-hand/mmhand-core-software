"""application_actions.py - 非阻塞应用操作和只读展示快照。

职责：在现有 HandBackend 内管理设备发现、操作、Manual 草稿和展示。
线程：耗时操作串行执行；停止立即撤权；通信回调只更新有界曲线。
输入：用户请求、P4 新鲜快照和现有工作流事件。
输出：操作编号、只读快照及明确失败原因。
状态所有权：HandBackend 实例拥有操作和草稿；P4 拥有执行状态。
禁止依赖：禁止依赖 GUI、ROS2、Replay 实现或另建整机状态机。
"""

from __future__ import annotations

from collections import deque
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
from dataclasses import asdict, dataclass
import math
from pathlib import Path
import threading
import time
from uuid import uuid4

from desktop.paths import RUN_DATA_DIR
from .angle_mapping import get_spec
from .calibration_diagnostics import format_calibration_diagnostic, format_calibration_warning
from .data_recording.recorder import DataSessionRecorder
from .host_calibration.config import default_encoder_direction, joint_motors, shorten_sign
from .session_log import log_session, session_log_since
from .console_snapshot import enrich_snapshot, motor_quality
from .protocol import (
    ControlMode, ENCODER_COUNT, MOTOR_COUNT, SYSTEM_STATE_UNCONTROLLABLE,
    SYSTEM_STATE_IDLE, SYSTEM_STATE_READY, SYSTEM_STATE_RUN_DIRECT,
    SYSTEM_STATE_RUN_FEEDBACK, SYSTEM_STATE_ROPE_INIT_RUNNING,
)


@dataclass(frozen=True)
class OperationResult:
    accepted: bool
    operation_id: str | None
    message: str


@dataclass(frozen=True)
class DeviceChoice:
    port: str
    description: str


def validate_manual_pair(minimum: int, maximum: int, direction: int) -> int:
    if direction not in (-1, 1) or not all(0 <= v < 16384 for v in (minimum, maximum)):
        raise ValueError('Invalid Manual raw value or encoder direction')
    span = ((maximum - minimum) * direction) % 16384
    if not 0 < span < 8192:
        raise ValueError('Manual endpoints are equal or the direction appears reversed. Capture again using the configured direction; endpoints are never swapped automatically.')
    return span


class ApplicationActions:
    """HandBackend 的应用能力；复用同一控制器和标定事务。"""

    def _init_application_actions(self):
        self._application_lock = threading.RLock()
        self._application_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='mmhand-operation')
        self._application_epoch = 0
        self._application_busy = None
        self._application_operations = deque(maxlen=64)
        self._manual_drafts = {}
        self._manual_saved_pairs = {}
        self._selected_joints = (0,)
        self._devices = ()
        self._discovery_error = ''
        self._application_closed = False
        self._application_stop = threading.Event()
        self._discovery_thread = None
        self._ros_available = False
        self._ros_reason = 'ROS2 is not running'
        self._replay_service = None
        self._curve_history = deque(maxlen=1800)
        self._curve_stamp = None
        self._recorder = DataSessionRecorder()
        self._recording_error = ''
        self._last_operation_message = 'Select a device to connect. Startup does not authorize motion.'
        self._calibration_operation_id = None
        self._estop_requested_at = 0.0
        self._direct_sequence = 0
        self._selected_motors = (0,)
        self._save_data = True
        self._record_full_tactile = False
        self._recording_started_at = 0.0
        self._recording_lock = threading.Lock()
        self._recording_thread = None
        self._import_previews = {}
        self._preview_generation = 0
        self._recording_path = ''
        self._import_operation_id = None
        self._motor_transactions = {}
        self._joint_transactions = {}
        self._import_scope = 0
        self.register_calibration_workflow_callback(self._application_calibration_event)

    def open_application(self, *, discover=None, replay_service=None):
        """进程级启动；页面刷新不调用此方法。"""
        with self._application_lock:
            if self._discovery_thread or self._application_closed:
                return
            self._replay_service = replay_service
            if discover is None:
                from .comm_layer import list_ports
                discover = list_ports

            def poll():
                while not self._application_stop.is_set():
                    try:
                        choices = tuple(DeviceChoice(str(port), str(description))
                                        for port, description in discover())
                        with self._application_lock:
                            self._devices = choices
                            self._discovery_error = ''
                    except Exception as exc:
                        with self._application_lock:
                            self._discovery_error = f'Device discovery failed: {exc}'
                    comm = self.tactile_comm
                    if comm is not None and self.tactile_port and not self.is_tactile_serial_connected() and time.time() >= self._next_tactile_reconnect_ts:
                        self._next_tactile_reconnect_ts = time.time() + 1.0
                        try:
                            comm.connect()
                        except Exception as exc:
                            log_session('COMM', f'S3 reconnect failed: {exc}')
                    self._reconcile_recording()
                    self._application_stop.wait(0.2)

            self._discovery_thread = threading.Thread(target=poll, name='mmhand-devices', daemon=True)
            self._discovery_thread.start()

    def close_application(self):
        with self._application_lock:
            if self._application_closed:
                return
            self._application_closed = True
        self.stop_motion(emergency=True)
        self._application_stop.set()
        if self._discovery_thread:
            self._discovery_thread.join(timeout=2)
        if self._recording_thread:
            self._recording_thread.join(timeout=5)
        self._application_executor.shutdown(wait=True, cancel_futures=True)
        if self._replay_service:
            self._replay_service.shutdown()
        self._recorder.stop()
        self.set_recording_packet_sink(None)
        self.disconnect_tactile_serial()
        self.shutdown(stop_lower=True)
        self._calibration_workflow._executor.shutdown(wait=True, cancel_futures=True)

    def log_operation(self, message):
        log_session('APP', message)

    def log_since(self, sequence=0):
        return session_log_since(sequence)

    def set_ros_availability(self, available, reason=''):
        with self._application_lock:
            self._ros_available = bool(available)
            self._ros_reason = str(reason)



    def request_task_diagnostics(self, enabled):
        return self.submit_operation('Task Diagnostics Subscription', lambda _: (self.set_task_diag_subscription(bool(enabled)), 'Task diagnostics subscription requested'))

    def request_replay_upload(self, name, content):
        def load(_):
            if not self._replay_service:
                return False, 'Replay service is not running'
            directory = RUN_DATA_DIR / 'imports'
            directory.mkdir(parents=True, exist_ok=True)
            path = directory / (uuid4().hex + '_' + Path(name).name)
            path.write_bytes(content)
            trace = self._replay_service.load_csv(path)
            return True, f'Motion imported: {trace.frame_count} frames, {trace.duration_s:.2f} s'
        return self.submit_operation('Import Motion', load, requires_device=False)

    def _application_observe(self, state):
        stamp = (state.encoder_mapped_p4_time_ms, state.servo_feedback_p4_time_ms)
        with self._application_lock:
            if stamp != self._curve_stamp and state.has_sensor_data:
                self._curve_stamp = stamp
                now = time.time()
                if self._curve_history and now - self._curve_history[-1]['host_time'] > 0.6:
                    self._curve_history.append({'host_time': now - 0.001, 'p4_time_ms': state.p4_time_ms,
                        'angles': [None] * ENCODER_COUNT, 'targets': [None] * ENCODER_COUNT})
                self._curve_history.append({
                    'host_time': now, 'p4_time_ms': state.p4_time_ms,
                    'angles': [e.current_angle_deg if not e.angle_error and self._mapped_feedback_fresh(state) else None for e in state.encoders],
                    'targets': list(state.target_angles),
                })

    def _finish_application_operation(self, operation_id, ok, message, *, phase=None):
        with self._application_lock:
            for item in self._application_operations:
                if item['operation_id'] == operation_id:
                    item.update(status='SUCCESS' if ok else 'FAILED', message=str(message),
                                phase=phase or ('complete' if ok else 'failed'))
                    break
            self._last_operation_message = str(message)
            log_session('APP', f'{operation_id} {"SUCCESS" if ok else "FAILED"}: {message}')
            self._recorder.event('operation_result', source='APP', detail=f'{operation_id}: {message}')
            if self._application_busy == operation_id:
                self._application_busy = None

    def submit_operation(self, name, action, *, requires_device=True, cleanup=False, scope='Global'):
        with self._application_lock:
            if self._application_closed:
                return OperationResult(False, None, 'Application is shutting down')
            if not cleanup and (self._application_busy or self._calibration_workflow.is_host_calibration_active() or self._calibration_workflow.is_import_active()):
                return OperationResult(False, None, 'An operation is still finishing. Use STOP to cancel.')
            if requires_device and not self._fresh_application_state():
                return OperationResult(False, None, 'P4 is disconnected or status data is stale. Check the connection.')
            operation_id = uuid4().hex[:12]
            epoch = self._application_epoch
            self._application_busy = operation_id
            self._application_operations.append({'operation_id': operation_id, 'name': name,
                                                 'time': time.strftime('%H:%M:%S'), 'scope': scope,
                                                 'status': 'PENDING', 'message': 'Accepted', 'phase': 'queued'})

            def worker():
                try:
                    with self._application_lock:
                        if epoch != self._application_epoch or self._application_closed:
                            self._finish_application_operation(operation_id, False, 'Operation cancelled')
                            return
                        for item in self._application_operations:
                            if item['operation_id'] == operation_id:
                                item.update(status='RUNNING', phase='running')
                    ok, message = action(epoch)
                    if epoch != self._application_epoch:
                        ok, message = False, 'Operation cancelled. A device write may have completed; check the readback.'
                    self._finish_application_operation(operation_id, ok, message)
                except Exception as exc:
                    self._finish_application_operation(operation_id, False, f'{name} failed: {exc}')

            try:
                self._application_executor.submit(worker)
            except RuntimeError:
                self._finish_application_operation(operation_id, False, 'The operation worker is closed')
                return OperationResult(False, None, 'The operation worker is closed')
            log_session('APP', f'{operation_id} {name} queued')
            return OperationResult(True, operation_id, f'{name} accepted')

    def _fresh_application_state(self):
        state = self.get_state_snapshot()
        return (self.is_comm_connected() and state.has_system_status_data and
                0 <= time.time() - state.system_status_last_rx_ts <= 0.6)

    @staticmethod
    def _mapped_feedback_fresh(state):
        return (state.has_sensor_data and 0 <= time.time() - state.encoder_mapped_last_rx_ts <= 0.6
                and (state.p4_time_ms - state.encoder_mapped_p4_time_ms) & 0xffffffff <= 600)

    def _wait_application(self, predicate, epoch, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and epoch == self._application_epoch:
            if predicate(self.get_state_snapshot()):
                return True
            self._application_stop.wait(0.01)
        return False

    def request_connect(self, port, *, tactile=False):
        if not str(port).strip():
            return OperationResult(False, None, 'Select a device. The port is never changed automatically.')

        def connect(epoch):
            ok = self.connect_tactile_serial(port) if tactile else self.connect(port)
            if epoch != self._application_epoch:
                if tactile:
                    self.disconnect_tactile_serial()
                else:
                    self.disconnect(stop_lower=True)
                return False, 'Connection cancelled'
            if not tactile:
                saved_pairs = {}
                try:
                    from .calibration_snapshot import parse_mask
                    payload = self._calibration_workflow._store.load()
                    valid = parse_mask(payload.get('manual_reference_valid_bitmap'))
                    saved_pairs = {j: (payload['manual_reference_min_raw'][j], payload['manual_reference_max_raw'][j])
                                   for j in range(ENCODER_COUNT) if valid & (1 << j)}
                except (OSError, ValueError, KeyError):
                    pass
                with self._application_lock:
                    self._manual_drafts.clear()
                    self._motor_transactions.clear()
                    self._joint_transactions.clear()
                    self._manual_saved_pairs = saved_pairs
            return bool(ok), 'S3 connected' if ok and tactile else 'P4 connected; waiting for valid status' if ok else 'Connection failed. Check the device and port availability.'

        return self.submit_operation('Connect S3' if tactile else 'Connect P4', connect, requires_device=False)

    def request_disconnect(self, *, tactile=False):
        self.stop_motion()

        def disconnect(_):
            while self._calibration_workflow.is_host_calibration_active() or self._calibration_workflow.is_import_active():
                if self._application_stop.wait(0.01):
                    return False, 'Application is shutting down'
            if tactile:
                self.disconnect_tactile_serial()
            else:
                self._stop_recording()
                self.disconnect(stop_lower=True)
                with self._application_lock:
                    self._manual_drafts.clear()
                    self._motor_transactions.clear()
                    self._joint_transactions.clear()
                    self._manual_saved_pairs.clear()
            return True, 'S3 disconnected' if tactile else 'P4 disconnected; motion authorization revoked'

        return self.submit_operation('Disconnect S3' if tactile else 'Disconnect P4', disconnect, requires_device=False, cleanup=True)

    def select_joints(self, joints):
        chosen = tuple(sorted(set(int(j) for j in joints)))
        if any(j < 0 or j >= ENCODER_COUNT for j in chosen):
            return OperationResult(False, None, 'Joint selection is out of range')
        with self._application_lock:
            changed = chosen != self._selected_joints
            busy = bool(self._application_busy or self._calibration_workflow.is_host_calibration_active())
        if changed and (busy or self.is_started() or self.ros2_output_armed()):
            self.stop_motion()
        with self._application_lock:
            self._selected_joints = chosen
        return OperationResult(True, None, 'Scope updated; the active operation was cancelled' if changed and busy else 'Scope updated')

    def request_enable(self):
        joints = self._selected_joints

        def enable(epoch):
            mask = sum(1 << j for j in joints)
            motors = sum(1 << m for j in joints for m in joint_motors(j))
            if not mask or not self.set_active_scope(motors, mask, True):
                return False, 'Scope setup failed. Check online devices, feedback and calibration.'
            with self._application_lock:
                if epoch != self._application_epoch:
                    return False, 'Enable cancelled'
                started = time.time()
                self.start()
            ok = self._wait_application(lambda s: s.system_state in (SYSTEM_STATE_IDLE, SYSTEM_STATE_READY)
                                        and s.control_enabled and s.system_status_last_rx_ts > started, epoch, 5)
            return ok, 'P4 preparation confirmed. Check device state and calibration before proceeding.' if ok else 'Enable was not confirmed. Check the P4 rejection reason.'

        return self.submit_operation('Enable', enable)

    def stop_motion(self, *, emergency=False):
        # 不排在任何阻塞事务之后；新操作仍须等旧 worker 收尾。
        with self._application_lock:
            self._application_epoch += 1
            self._estop_requested_at = time.time()
            self._last_operation_message = 'Stop requested; waiting for P4 confirmation'
        self.stop()
        self.cancel_calibration_workflow()
        if self._replay_service:
            self._replay_service.stop('E-STOP' if emergency else 'STOP')
        log_session('APP', self._last_operation_message)
        self._recorder.event('stop', source='APP', detail=self._last_operation_message)
        return OperationResult(True, None, self._last_operation_message)

    def request_clear_faults(self):
        def reset(epoch):
            sent_at = time.time()
            with self._application_lock:
                if epoch != self._application_epoch:
                    return False, 'Fault clear cancelled'
                self.reset()
            ok = self._wait_application(lambda s: s.system_status_last_rx_ts > sent_at
                                        and not s.system_fault_bitmap and not s.control_enabled, epoch)
            if ok:
                self._estop_requested_at = 0
            return ok, 'Recoverable faults cleared. Motion has not been restored automatically.' if ok else 'Fault clear was not confirmed. Check Diagnostics.'
        return self.submit_operation('Clear Faults', reset)

    def request_pose(self, targets, *, joint_mask=None):
        values = list(targets)
        if len(values) != ENCODER_COUNT or not all(math.isfinite(float(v)) for v in values):
            return OperationResult(False, None, 'Targets must contain 21 finite angle values')
        mask = int(joint_mask) if joint_mask is not None else sum(1 << j for j in self._selected_joints)
        if not mask or mask & ~((1 << ENCODER_COUNT) - 1):
            return OperationResult(False, None, 'Select a valid joint scope')
        for j in range(ENCODER_COUNT):
            if mask & (1 << j):
                lo, hi = get_spec(j).urdf_range_deg
                if not lo <= float(values[j]) <= hi:
                    return OperationResult(False, None, f'J{j:02d} target exceeds URDF limits [{lo}, {hi}]')

        def send(epoch):
            with self._application_lock:
                if epoch != self._application_epoch:
                    return False, 'Target cancelled'
                result = self.send_joint_targets(values, source='GUI', live=True, command_joint_mask=mask)
            return result.sent, result.message or ('Joint targets sent. Check fresh feedback for actual movement.' if result.sent else 'Target rejected')
        return self.submit_operation('Joint Targets', send, scope=f'Joints 0x{mask:06X}')

    def request_targets(self, targets):
        if not targets or any(j not in range(ENCODER_COUNT) for j in targets):
            return OperationResult(False, None, 'Select valid joints')
        values = list(self.get_state_snapshot().target_angles)
        for j, value in targets.items():
            values[j] = value
        return self.request_pose(values, joint_mask=sum(1 << j for j in targets))

    def request_ros_input(self, enabled):
        if enabled and not self._ros_available:
            return OperationResult(False, None, self._ros_reason)
        if enabled and (self._application_busy or self._calibration_workflow.is_host_calibration_active()):
            return OperationResult(False, None, 'An operation is running. Wait for it to finish before authorizing ROS Input.')
        ok, message = self.set_ros2_output_enabled(bool(enabled))
        return OperationResult(ok, None, message or ('ROS input authorized' if enabled else 'ROS input revoked'))

    def request_manual_capture(self, joint, endpoint):
        joint = int(joint)
        if joint not in range(ENCODER_COUNT) or endpoint not in ('min', 'max'):
            return OperationResult(False, None, 'Invalid Manual capture arguments')

        def capture(epoch):
            state = self.get_state_snapshot()
            encoder = state.encoders[joint]
            p4_age = (state.p4_time_ms - state.encoder_raw_p4_time_ms) & 0xFFFFFFFF
            if (not state.has_encoder_raw_data or encoder.raw_error or state.encoder_fault_bitmap & (1 << joint)
                    or not state.encoder_online_bitmap & (1 << joint)
                    or not 0 <= time.time() - state.encoder_raw_last_rx_ts <= 0.3
                    or p4_age > 300 or not 0 <= encoder.raw_sensor < 16384):
                return False, f'J{joint:02d} raw feedback is invalid or stale. Check the encoder before capturing.'
            if state.control_enabled or state.servo_target_owner or state.system_state == SYSTEM_STATE_ROPE_INIT_RUNNING:
                return False, 'Press STOP and wait until P4 is stationary before capturing Manual endpoints.'
            if not state.has_manual_reference_encoder_directions:
                return False, 'P4 encoder installation direction is unavailable. Wait for valid device status.'
            direction = int(state.manual_reference_encoder_directions[joint])
            if direction not in (-1, 1):
                return False, 'P4 encoder installation direction is invalid. Check device configuration.'
            with self._application_lock:
                if epoch != self._application_epoch:
                    return False, 'Capture cancelled'
                self._joint_transactions.pop(joint, None)
                self._motor_transactions[joint] = 'PENDING'
                draft = self._manual_drafts.setdefault(joint, {})
                draft.update({endpoint: int(encoder.raw_sensor), 'direction': direction, 'status': 'DRAFT'})
                pair = deepcopy(draft)
            if 'min' not in pair or 'max' not in pair:
                return True, f'J{joint:02d} {endpoint} draft captured. The other endpoint is missing; nothing has been saved.'
            return self._save_manual_draft(joint, epoch)
        return self.submit_operation(f'J{joint:02d} Set {endpoint}', capture, scope=f'J{joint:02d}')

    def _save_manual_draft(self, joint, epoch):
        with self._application_lock:
            pair = deepcopy(self._manual_drafts[joint])
        try:
            validate_manual_pair(pair['min'], pair['max'], pair['direction'])
        except (ValueError, KeyError) as exc:
            with self._application_lock:
                self._manual_drafts[joint]['status'] = 'VALIDATION_FAILED'
            return False, f'J{joint:02d} not saved: {exc}'
        if epoch != self._application_epoch:
            return False, 'Manual save cancelled'
        ok = self.set_manual_encoder_reference(joint, pair['min'], pair['max'])
        stage = getattr(self._controller, 'last_manual_write_phase', 'INSTALL_FAILED')
        with self._application_lock:
            self._manual_drafts[joint]['status'] = 'CANCELLED' if epoch != self._application_epoch else 'SAVED' if ok else stage
            if ok and epoch == self._application_epoch:
                self._manual_saved_pairs[joint] = (pair['min'], pair['max'])
        return ok, (f'J{joint:02d} Manual saved; automatic calibration must be repeated' if ok else
                    f'J{joint:02d} Manual was not fully saved ({stage}); draft retained. Retry explicitly.')


    def request_clear_manual(self):
        joints = self._selected_joints
        def clear(epoch):
            if not joints:
                return False, 'Select joints'
            state = self.get_state_snapshot()
            if state.control_enabled or state.servo_target_owner:
                return False, 'Press STOP and wait until the device is stationary'
            if not self.query_manual_encoder_references():
                return False, 'Manual readback failed'
            state = self.get_state_snapshot()
            mins, maxs = list(state.manual_reference_min_raw), list(state.manual_reference_max_raw)
            for j in joints:
                mins[j] = maxs[j] = 0
            if epoch != self._application_epoch:
                return False, 'Clear cancelled'
            ok = self.set_manual_encoder_references(mins, maxs)
            if ok:
                with self._application_lock:
                    for j in joints:
                        self._manual_drafts.pop(j, None)
                        self._joint_transactions.pop(j, None)
                        self._motor_transactions[j] = 'PENDING'
                        self._manual_saved_pairs.pop(j, None)
            return ok, 'Selected Manual references cleared; other joints preserved' if ok else 'Clear or save failed. Check the device readback.'
        return self.submit_operation('Clear Manual selected', clear)

    def request_calibration(self):
        with self._application_lock:
            if self._application_busy or self._calibration_workflow.is_host_calibration_active():
                return OperationResult(False, None, 'The current operation is still finishing')
            state = self.get_state_snapshot()
            joints = self._selected_joints
            mask = sum(1 << j for j in joints)
            if not mask or not self._fresh_application_state():
                return OperationResult(False, None, 'Select joints and connect P4 with fresh status')
            for j in joints:
                if not state.manual_reference_valid_bitmap & (1 << j):
                    return OperationResult(False, None, f'J{j:02d} is missing Manual Min / Max')
                direction = (state.manual_reference_encoder_directions[j] if state.has_manual_reference_encoder_directions
                             else default_encoder_direction(j))
                try:
                    validate_manual_pair(state.manual_reference_min_raw[j], state.manual_reference_max_raw[j], direction)
                except ValueError as exc:
                    return OperationResult(False, None, f'J{j:02d}: {exc}')
            motors = sum(1 << m for j in joints for m in joint_motors(j))
            operation_id = uuid4().hex[:12]
            self._calibration_operation_id = operation_id
            self._application_busy = operation_id
            for j in joints:
                self._motor_transactions[j] = 'PENDING'
            self._application_operations.append({'operation_id': operation_id, 'name': 'Selected-joint Calibration',
                                                 'time': time.strftime('%H:%M:%S'), 'scope': f'Joints 0x{mask:06X}',
                                                 'status': 'PENDING', 'message': 'Preparing', 'phase': 'prepare'})
            ok = self.begin_calibration_workflow(joint_mask=mask, motor_mask=motors, reset_motor_mask=motors)
            if not ok:
                self._finish_application_operation(operation_id, False, 'Calibration transaction rejected')
                self._calibration_operation_id = None
                return OperationResult(False, None, 'Calibration transaction rejected')
            return OperationResult(True, operation_id, 'Selected-joint calibration accepted')

    def _application_calibration_event(self, event):
        log_session('CALIB', f'{event.phase}: {event.message}')
        self._recorder.event('calibration_phase', source='CALIB', detail=f'{event.phase}: {event.message}')
        with self._application_lock:
            if self._import_operation_id and event.phase == 'import':
                for j in range(ENCODER_COUNT):
                    if self._import_scope & (1 << j):
                        if event.ok:
                            self._manual_drafts.pop(j, None)
                            self._joint_transactions.pop(j, None)
                            self._motor_transactions.pop(j, None)
                            pair = self._import_manual_pairs.get(j)
                            if pair is not None:
                                self._manual_saved_pairs[j] = pair
                            else:
                                self._manual_saved_pairs.pop(j, None)
                        else:
                            self._joint_transactions[j] = self._motor_transactions[j] = 'FAILED'
                self._finish_application_operation(self._import_operation_id, event.ok, event.message, phase='import')
                self._import_operation_id = None
                return
            operation_id = self._calibration_operation_id
            if not operation_id:
                return
            if event.phase == 'phase_complete':
                state = self.get_state_snapshot()
                for j in range(ENCODER_COUNT):
                    if event.joint_mask & (1 << j):
                        self._motor_transactions[j] = 'SAVED' if event.ok else 'FAILED'
                        result = state.joint_calib_results[j] or {}
                        labels = motor_quality(bool(state.auto_calibration_valid_bitmap & (1 << j)), result) if event.ok else ['FAILED']
                        log_session('CALIB', f"J{j:02d} completed: {' / '.join(labels)}; return offset={result.get('autoMinOffsetCounts', 'unknown')} (≥350); travel shortfall={result.get('autoMaxShortfallCounts', 'unknown')} (≥300) counts. Possible causes: spring return, tendon friction or preload; force is not measured.")
            if event.phase == 'calibration_complete' and not event.ok:
                failed = event.details.get('failed_joint_mask', 0) or (event.joint_mask & ~event.details.get('completed_joint_mask', 0))
                for j in range(ENCODER_COUNT):
                    if failed & (1 << j):
                        self._motor_transactions[j] = 'FAILED'
            if event.phase == 'calibration_complete':
                self._finish_application_operation(operation_id, event.ok, event.message,
                                                   phase=event.details.get('terminal_status', event.phase))
                self._calibration_operation_id = None
            else:
                for item in self._application_operations:
                    if item['operation_id'] == operation_id:
                        item.update(status='RUNNING', message=event.message, phase=event.phase)
                self._last_operation_message = event.message


    def select_motors(self, motors):
        chosen = tuple(sorted(set(int(m) for m in motors)))
        if any(m not in range(MOTOR_COUNT) for m in chosen):
            return OperationResult(False, None, 'Motor selection is out of range')
        if chosen != self._selected_motors and (self.is_started() or self._application_busy):
            self.stop_motion()
        self._selected_motors = chosen
        return OperationResult(True, None, 'Motor scope updated')

    def request_motors(self, targets):
        try:
            if not targets or any(int(m) != m or int(m) not in range(MOTOR_COUNT) or not math.isfinite(float(v)) or int(v) != float(v) for m, v in targets.items()):
                raise ValueError('Select valid integer motor IDs and integer logical ABS targets')
            values = {int(m): int(v) for m, v in targets.items()}
        except (ValueError, TypeError, OverflowError) as exc:
            return OperationResult(False, None, str(exc))
        return self.submit_operation('Motor Targets', lambda epoch: self._application_direct(values, epoch), scope=' / '.join(f'M{m:02d}' for m in values))

    def request_direct_enable(self):
        mask = sum(1 << m for m in self._selected_motors)
        def enable(epoch):
            from .motor_mapping import MOTOR_TO_JOINT_INDEX
            joints = sum(1 << j for j in {MOTOR_TO_JOINT_INDEX[m] for m in self._selected_motors})
            if not self.set_active_scope(mask, joints, False):
                return False, 'Motor scope setup failed'
            with self._application_lock:
                if not mask or epoch != self._application_epoch:
                    return False, 'Select motors; enable was not submitted'
                ok, message, _ = self.enable_direct_hold(mask)
            return ok, message
        return self.submit_operation('Enable Direct Control', enable)

    def request_pause(self):
        with self._application_lock:
            self._application_epoch += 1
        self.pause()
        log_session('APP', 'Pause requested')
        return OperationResult(True, None, 'Pause requested')

    def request_tendon_step(self, tighten):
        from .motor_mapping import motor_abs_after_tendon_action
        motors = self._selected_motors
        def step(epoch):
            state = self.get_state_snapshot()
            targets = {m: motor_abs_after_tendon_action(state.servo_angles[m], m, 2048 if tighten else -2048) for m in motors}
            if not targets:
                return False, 'Select motors'
            return self._application_direct(targets, epoch)
        return self.submit_operation('Tighten Half Turn' if tighten else 'Release Half Turn', step)

    def request_release_rope(self):
        def release(epoch):
            state = self.get_state_snapshot()
            if state.control_enabled or state.servo_target_owner or epoch != self._application_epoch:
                return False, 'Stop motion before releasing rope'
            result = self.manual_prepare_all_motors()
            with self._application_lock:
                for j in range(ENCODER_COUNT):
                    if result.invalidated_joint_mask & (1 << j):
                        self._motor_transactions[j] = 'PENDING'
            return bool(result.ok), result.message
        return self.submit_operation('Release Rope', release)

    def clear_calibration_preview(self):
        with self._application_lock:
            self._import_previews.clear()
            self._preview_generation += 1

    def request_calibration_preview(self, name, content):
        self.clear_calibration_preview()
        generation = self._preview_generation
        def preview(_):
            if len(content) > 10_000_000:
                return False, 'Calibration file exceeds 10 MB'
            directory = RUN_DATA_DIR / 'imports'
            directory.mkdir(parents=True, exist_ok=True)
            path = directory / (uuid4().hex + '.mmcal.json')
            path.write_bytes(content)
            try:
                payload = self.inspect_calibration_snapshot(path)
            finally:
                path.unlink(missing_ok=True)
            token = uuid4().hex
            with self._application_lock:
                if generation != self._preview_generation:
                    return False, 'Calibration preview cancelled'
                self._import_previews.clear()
                self._import_previews[token] = {'payload': payload, 'name': Path(name).name}
            return True, 'Calibration validated; confirm Load to P4 to apply'
        return self.submit_operation('Inspect Calibration', preview, requires_device=False)

    def request_calibration_import(self, token):
        with self._application_lock:
            preview = self._import_previews.get(token)
            state = self.get_state_snapshot()
            if not preview or self._application_busy or not self._fresh_application_state():
                return OperationResult(False, None, 'Import requires a validated file and fresh, idle P4 connection')
            if state.control_enabled or state.servo_target_owner or state.system_state not in (0, 1, 5, 7, 9):
                return OperationResult(False, None, 'Stop motion before loading calibration')
            operation_id = uuid4().hex[:12]
            from .calibration_snapshot import parse_mask
            payload = preview['payload']
            valid = parse_mask(payload.get('manual_reference_valid_bitmap'))
            self._import_scope = valid | state.manual_reference_valid_bitmap | parse_mask(payload.get('active_joint_mask'))
            self._import_manual_pairs = {j: (payload['manual_reference_min_raw'][j], payload['manual_reference_max_raw'][j])
                for j in range(ENCODER_COUNT) if valid & (1 << j)}
            self._import_operation_id = self._application_busy = operation_id
            self._application_operations.append({'operation_id': operation_id, 'name': 'Load Calibration',
                'time': time.strftime('%H:%M:%S'), 'scope': f"File joints 0x{self._import_scope:06X}",
                'status': 'RUNNING', 'phase': 'import', 'message': 'Restoring device calibration'})
            if not self.begin_calibration_import(deepcopy(preview['payload']), preview['name']):
                self._finish_application_operation(operation_id, False, 'Import transaction is busy')
                self._import_operation_id = None
                return OperationResult(False, None, 'Import transaction is busy')
            return OperationResult(True, operation_id, 'Calibration import started')

    def _application_direct(self, targets, epoch):
        state = self.get_state_snapshot()
        now = time.time()
        if (not state.has_servo_angle_data or not 0 <= now - state.servo_abs_last_rx_ts <= 0.6
                or not 0 <= now - state.servo_telem_last_rx_ts <= 0.6
                or not state.has_servo_route_status):
            return False, 'Fresh motor position, telemetry and route information are required'
        mask = sum(1 << m for m in targets)
        if state.servo_route_conflict_bitmap & mask or state.servo_route_controllable_bitmap & mask != mask:
            return False, 'Selected motor route is unavailable or conflicting'
        for m, target in targets.items():
            if (not state.servo_logical_range_valid_bitmap & (1 << m)
                    or not state.servo_online_bitmap & (1 << m)
                    or state.multi_turn_unknown_bitmap & (1 << m)
                    or not state.servo_logical_range_min[m] <= target <= state.servo_logical_range_max[m]
                    or state.servo_load[m] & 0x3ff >= 800):
                return False, f'M{m:02d} position is unknown, offline, overloaded or outside limits. Check Diagnostics.'
            if state.motor_abs_limit_valid_bitmap & (1 << m) and not state.motor_abs_limit_min[m] <= target <= state.motor_abs_limit_max[m]:
                return False, f'M{m:02d} target exceeds calibrated limits'
        from .motor_mapping import MOTOR_TO_JOINT_INDEX
        joints = sum(1 << j for j in {MOTOR_TO_JOINT_INDEX[m] for m in targets})
        if not self.set_active_scope(mask, joints, False):
            return False, 'Motor scope setup failed'
        with self._application_lock:
            if epoch != self._application_epoch:
                return False, 'Direct control cancelled'
            ok, message, _ = self.enable_direct_hold(mask)
        if not ok:
            return False, message
        state = self.get_state_snapshot()
        values = list(state.servo_angles)
        for m, target in targets.items():
            values[m] = target
        with self._application_lock:
            if epoch != self._application_epoch:
                return False, 'Direct control cancelled'
            self._direct_sequence = (max(self._direct_sequence, state.direct_target_token) + 1) & 0xffffffff or 1
            token = self._direct_sequence
            sent_at = time.time()
            if not self.comm.send_command(('motor_pos_abs', token, mask, values, 400, 20)):
                return False, 'Motor target send failed'
        ok = self._wait_application(lambda s: s.direct_target_token == token
                                    and s.direct_target_result_rx_ts >= sent_at, epoch, 1)
        result = self.get_state_snapshot()
        ok = bool(ok and result.direct_target_status == 2 and result.direct_target_applied_mask == mask
                  and not result.direct_target_rejected_mask)
        return ok, 'Motor target accepted by the execution layer. Check feedback for physical completion.' if ok else 'Motor execution rejected or confirmation timed out. No automatic retry.'



    def request_replay_start(self):
        def start(epoch):
            with self._application_lock:
                if epoch != self._application_epoch:
                    return False, 'Playback cancelled'
                if not self._replay_service:
                    return False, 'Replay is not running'
            result = self._replay_service.send(expected_application_epoch=epoch)
            if epoch != self._application_epoch:
                self._replay_service.stop('Playback startup cancelled')
                self.stop()
                return False, 'Playback startup cancelled'
            return result
        return self.submit_operation('Play Motion', start)

    def request_recording(self, enabled):
        self._save_data = bool(enabled)
        log_session('RECORD', 'Save Data ON' if enabled else 'Save Data OFF; finalizing')
        if enabled:
            self._recording_error = ''
        self._reconcile_recording()
        return OperationResult(True, None, 'Save Data ON' if enabled else 'Save Data OFF; recording is finalizing')

    def set_recording_options(self, full_tactile):
        if self._recorder.is_running():
            return OperationResult(False, None, 'Turn Save Data OFF before changing recording options')
        self._record_full_tactile = bool(full_tactile)
        return OperationResult(True, None, 'Recording options updated')

    def _stop_recording(self):
        with self._recording_lock:
            self.set_recording_packet_sink(None)
            path = self._recorder.path
            if self._recorder.is_running() or self._recorder.path:
                self.set_control_trace_subscription(False)
            self._recorder.stop()
            if path:
                log_session('RECORD', f'Recording finalized: {path}')

    def _reconcile_recording(self):
        if self._application_closed or self._recording_thread and self._recording_thread.is_alive():
            return
        error = self._recorder.consume_error()
        if error:
            self._recording_error = error
            self._save_data = False
            log_session('RECORD', f'Recording failed: {error}')
        should_run = self._save_data and self._fresh_application_state()
        # An open but stale port retains the session; never create many files during a brief gap.
        should_stop = not self._save_data or not self.is_comm_connected()
        if not (should_run and not self._recorder.is_running() or should_stop and self._recorder.is_running() or error):
            return
        def reconcile():
            try:
                if should_stop or error:
                    self._stop_recording()
                elif should_run:
                    with self._recording_lock:
                        if self._application_closed or not self._save_data or not self._fresh_application_state():
                            return
                        from .data_recording.topic_schema import DEFAULT_TOPICS
                        topics = [topic for topic in DEFAULT_TOPICS if topic != 'tactile_full' or self._record_full_tactile]
                        self._recorder.start(str(RUN_DATA_DIR), topics=topics,
                            meta={'fingerprint': self.get_state_snapshot().product_fingerprint})
                        self.set_recording_packet_sink(self._recorder.record_packets)
                        if not self.set_control_trace_subscription(True):
                            raise RuntimeError('P4 control-trace subscription failed')
                        self._recording_started_at = time.time()
                        self._recording_path = self._recorder.path
                        log_session('RECORD', f'Recording started: {self._recorder.path}')
            except Exception as exc:
                self._recording_error = str(exc)
                self._save_data = False
                self._stop_recording()
                log_session('RECORD', f'Recording failed: {exc}')
        self._recording_thread = threading.Thread(target=reconcile, name='mmhand-recording', daemon=True)
        self._recording_thread.start()

    def application_snapshot(self, *, include_curves=True):
        state = self.get_state_snapshot()
        self.consume_tactile_state_into(state)
        now = time.time()
        fresh = (self.is_comm_connected() and state.has_system_status_data and
                 0 <= now - state.system_status_last_rx_ts <= 0.6)
        communication = 'Online' if fresh else 'Stale data' if self.is_comm_connected() else 'Disconnected'
        host_active = self._calibration_workflow.is_host_calibration_active()
        if not fresh:
            device = 'Unknown'
        elif (self._estop_requested_at
                and state.system_status_last_rx_ts >= self._estop_requested_at
                and not state.control_enabled and not state.servo_target_owner):
            device = 'Stopped'
        elif host_active:
            device = 'Calibrating'
        elif state.control_enabled and state.servo_target_owner and state.system_state in (SYSTEM_STATE_RUN_DIRECT, SYSTEM_STATE_RUN_FEEDBACK):
            device = 'Controlling'
        elif state.system_state == SYSTEM_STATE_READY:
            device = 'Ready'
        else:
            device = 'Disabled'
        problems = []
        if self._recording_error:
            problems.append(f'Recording: {self._recording_error}. This session may be incomplete. Check the disk and recording directory.')
        if not fresh:
            problems.append('P4: disconnected or stale status. Check the connection and wait for valid status.')
        elif state.system_fault_bitmap:
            problems.append(f'P4: protection fault 0x{state.system_fault_bitmap:08X}. Check Diagnostics, resolve the cause, then clear faults.')
        if state.servo_route_conflict_bitmap:
            problems.append(f'Servo: route conflict 0x{state.servo_route_conflict_bitmap:08X}. Check motor IDs and bus connections.')
        if state.multi_turn_unknown_bitmap:
            problems.append(f'Motor: multi-turn position unknown 0x{state.multi_turn_unknown_bitmap:08X}. Use the controlled recovery or calibration procedure.')
        if self.tactile_port and not self.is_tactile_serial_connected():
            problems.append('S3: tactile serial offline. Check its independent connection; P4 status is evaluated separately.')
        warning = (format_calibration_warning(state, state.calibration_warning_bitmap)
                   if state.calibration_warning_bitmap else {'ready': False})
        if warning.get('ready'):
            problems.append(warning.get('body', warning.get('message', 'Assembly warning detected. Check calibration details.')))
        with self._application_lock:
            busy = bool(self._application_busy or host_active)
            ros_reason = self._ros_reason if not self._ros_available else ('P4 is not ready' if not fresh or not self.is_started() else
                         'Device protection fault is active' if state.system_fault_bitmap else
                         'Calibration or another operation is running' if busy else 'Replay owns the input' if self._replay_motion_active else '')
            joints = []
            for j, encoder in enumerate(state.encoders):
                spec = get_spec(j)
                draft = deepcopy(self._manual_drafts.get(j, {}))
                direction = (state.manual_reference_encoder_directions[j] if state.has_manual_reference_encoder_directions
                             else default_encoder_direction(j))
                joints.append({'joint': j, 'name': spec.joint_name, 'angle': encoder.current_angle_deg if fresh and self._mapped_feedback_fresh(state) and not encoder.angle_error else None,
                               'raw': encoder.raw_sensor if fresh and state.has_encoder_raw_data and 0 <= now - state.encoder_raw_last_rx_ts <= 0.3 and not encoder.raw_error else None,
                               'target': encoder.target_angle_deg, 'min': spec.urdf_range_deg[0], 'max': spec.urdf_range_deg[1],
                               'direction': direction, 'manual_min': state.manual_reference_min_raw[j],
                               'manual_max': state.manual_reference_max_raw[j], 'manual_valid': bool(state.manual_reference_valid_bitmap & (1 << j)),
                               'auto_valid': bool(state.auto_calibration_valid_bitmap & (1 << j)), 'draft': draft})
            # 被取消的标定 worker 先收尾，然后释放应用操作门。
            if self._calibration_operation_id and not host_active and self._application_epoch and state.calib_status not in ('PENDING', 'RUNNING'):
                self._finish_application_operation(self._calibration_operation_id, False, 'Calibration cancelled; completed joints preserved')
                self._calibration_operation_id = None
            snapshot = {
                'communication': communication, 'device': device, 'problems': problems,
                'message': self._last_operation_message, 'busy': busy, 'selected_joints': list(self._selected_joints),
                'devices': [asdict(d) for d in self._devices], 'discovery_error': self._discovery_error,
                'operations': deepcopy(list(self._application_operations)), 'joints': joints,
                'motors': [{'motor': m, 'abs': state.servo_angles[m] if fresh and state.has_servo_angle_data and state.servo_online[m] and 0 <= now - state.servo_abs_last_rx_ts <= 0.6 else None,
                            'online': state.servo_online[m] if fresh else None,
                            'current': state.servo_current[m] if fresh and state.servo_telem_online[m] and 0 <= now - state.servo_telem_last_rx_ts <= 0.6 else None,
                            'load': state.servo_load[m] if fresh and state.servo_telem_online[m] and 0 <= now - state.servo_telem_last_rx_ts <= 0.6 else None,
                            'telem_age_ms': max(0, int((now - state.servo_telem_last_rx_ts) * 1000)) if state.servo_telem_last_rx_ts else None}
                           for m in range(MOTOR_COUNT)],
                'ros': {'available': self._ros_available, 'enabled': self.ros2_output_armed(), 'reason': ros_reason},
                'recording': {'running': self._recorder.is_running(), 'path': self._recorder.path or self._recording_path},
                'curves': deepcopy(list(self._curve_history)) if include_curves else [],
                'replay': asdict(self._replay_service.status()) if self._replay_service else {},
                'calibration': {'status': state.calib_status, 'results': deepcopy(state.joint_calib_results),
                                'warning': warning, 'diagnostic': format_calibration_diagnostic(state)},
                'diagnostics': {'state': state.system_state, 'owner': state.servo_target_owner, 'faults': state.system_fault_bitmap,
                                'status_age_ms': int((now - state.system_status_last_rx_ts) * 1000) if state.system_status_last_rx_ts else None,
                                'tasks': deepcopy(state.task_runtime), 'safety_events': deepcopy(state.safety_event_history),
                                'fingerprint': deepcopy(state.product_fingerprint)},
                'tactile': {'source': state.tactile_source, 'online': state.tactile_online,
                            'age_ms': int((now - state.tactile_last_rx_ts) * 1000) if state.tactile_last_rx_ts else None,
                            'forces': [[s.resultant.tolist() for s in finger.sensors] for finger in state.tactile_data]},
            }
            return enrich_snapshot(self, state, snapshot, now, fresh)
