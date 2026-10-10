"""Console transactions, cancellation and recording lifecycle with simulated hardware."""
import threading
import time
from unittest.mock import Mock
import pytest
from test_application_actions import hand, wait_operation
from desktop.core.hand_backend import HandBackend
from desktop.core.calibration_workflow import CalibrationWorkflowEvent
from desktop.core.core_logic import ManualPrepareResult
from desktop.core.comm_layer import LowerComputerComm
from desktop.core.host_calibration.config import joint_motors
from desktop.core.protocol import (
    ControlMode, SYSTEM_STATE_IDLE, SYSTEM_STATE_READY,
    SYSTEM_STATE_PAUSED, SYSTEM_STATE_RUN_DIRECT, SYSTEM_STATE_RUN_FEEDBACK,
)


def joint_preparation_device(hand, monkeypatch, *, ready_state=SYSTEM_STATE_READY, fault=None):
    """Emulate P4 scope/START replies, keeping the actual host command queue."""
    controller = hand._controller
    state = controller.current_state
    controller.set_control_mode(ControlMode.DIRECT_MOTOR)
    comm = LowerComputerComm('')
    comm._protocol_compatible = True
    monkeypatch.setattr(comm, 'is_connected', lambda: True)
    monkeypatch.setattr(controller, 'comm', comm)

    def scope(motors, joints, require_can):
        state.has_active_scope_status = True
        state.active_motor_mask, state.active_joint_mask = motors, joints
        state.require_can_feedback = require_can
        state.control_mode = ControlMode.JOINT_ANGLE.value
        state.system_status_last_rx_ts = time.time()
        return True

    scope_writer = Mock(side_effect=scope)
    monkeypatch.setattr(controller, 'set_active_scope', scope_writer)
    send_command = comm.send_command

    def reply(command):
        sent = send_command(command)
        if sent and command == 'start':
            state.system_state = ready_state
            state.control_enabled = False
            state.servo_target_owner = 0
            if fault != 'old_status':
                state.system_status_last_rx_ts = time.time()
            if fault == 'wrong_motor_scope':
                state.active_motor_mask ^= 1 << 26
            elif fault == 'wrong_joint_scope':
                state.active_joint_mask ^= 1 << 20
            elif fault == 'no_can_feedback':
                state.require_can_feedback = False
            elif fault == 'direct_mode':
                state.control_mode = ControlMode.DIRECT_MOTOR.value
            elif fault == 'active_control':
                state.control_enabled = True
            elif fault == 'active_owner':
                state.servo_target_owner = 1
            elif fault == 'missing_scope':
                state.has_active_scope_status = False
            elif fault == 'missing_status':
                state.has_system_status_data = False
            elif fault == 'handshake':
                state.protocol_compatible = False
            elif fault == 'offline':
                monkeypatch.setattr(hand, 'is_comm_connected', lambda: False)
        return sent

    monkeypatch.setattr(comm, 'send_command', reply)
    return comm, scope_writer


@pytest.mark.parametrize('ready_state', [SYSTEM_STATE_IDLE, SYSTEM_STATE_READY])
def test_joint_enable_confirms_preparation_without_claiming_motion(hand, monkeypatch, ready_state):
    comm, scope = joint_preparation_device(hand, monkeypatch, ready_state=ready_state)
    hand._selected_joints = (3,)
    comm.tx_queue.put_nowait(('motor_pos_abs_one', 3, 500))
    comm.tx_queue.put_nowait('start')

    item = wait_operation(hand, hand.request_enable())

    assert item['status'] == 'SUCCESS'
    assert 'preparation confirmed' in item['message']
    assert hand.get_control_mode() == ControlMode.JOINT_ANGLE
    scope.assert_called_once_with(sum(1 << m for m in joint_motors(3)), 1 << 3, True)
    assert list(comm.tx_queue.queue) == ['start']
    assert not hand._controller.current_state.control_enabled
    assert hand._controller.current_state.servo_target_owner == 0


@pytest.mark.parametrize('fault', [
    'old_status', 'wrong_motor_scope', 'wrong_joint_scope', 'no_can_feedback',
    'direct_mode', 'active_control', 'active_owner', 'missing_scope',
    'missing_status', 'handshake', 'offline',
])
def test_joint_enable_does_not_confirm_old_or_mismatched_p4_status(hand, monkeypatch, fault):
    comm, _ = joint_preparation_device(hand, monkeypatch, fault=fault)
    # Evaluate the real confirmation predicate once without a five-second wait.
    monkeypatch.setattr(hand, '_wait_application',
                        lambda predicate, epoch, timeout: predicate(hand.get_state_snapshot()))

    item = wait_operation(hand, hand.request_enable())

    assert item['status'] == 'FAILED'
    assert 'not confirmed' in item['message']
    assert list(comm.tx_queue.queue) == ['start']


@pytest.mark.parametrize('p4_state,owner,enabled', [
    (SYSTEM_STATE_PAUSED, 0, False),
    (SYSTEM_STATE_RUN_DIRECT, 1, True),
    (SYSTEM_STATE_RUN_FEEDBACK, 1, True),
    (SYSTEM_STATE_READY, 1, False),
    (SYSTEM_STATE_READY, 0, True),
])
def test_joint_enable_never_resumes_or_resets_active_motion(hand, monkeypatch, p4_state, owner, enabled):
    comm, scope = joint_preparation_device(hand, monkeypatch)
    state = hand._controller.current_state
    state.system_state, state.servo_target_owner, state.control_enabled = p4_state, owner, enabled

    item = wait_operation(hand, hand.request_enable())

    assert item['status'] == 'FAILED'
    assert 'Stop active motion' in item['message']
    scope.assert_not_called()
    assert hand.get_control_mode() == ControlMode.DIRECT_MOTOR
    assert comm.tx_queue.empty()

def test_joint_ready_requires_matching_persisted_pair(hand):
    state = hand._controller.current_state
    state.manual_reference_valid_bitmap = 1
    state.manual_reference_committed = True
    state.manual_reference_min_raw[0], state.manual_reference_max_raw[0] = 12000, 11000
    assert hand.application_snapshot()['joints'][0]['joint_quality'] == ['UNKNOWN']
    hand._manual_saved_pairs[0] = (12000, 11000)
    assert hand.application_snapshot()['joints'][0]['joint_quality'][0] in ('READY', 'OFFSET')
    hand._manual_saved_pairs[0] = (12000, 10999)
    assert hand.application_snapshot()['joints'][0]['joint_quality'] == ['UNKNOWN']


def test_release_selected_tendons_freezes_only_selected_joint_motors(hand, monkeypatch):
    hand._selected_joints = (0, 7)
    mask = sum(1 << m for j in (0, 7) for m in joint_motors(j))
    writer = Mock(return_value=ManualPrepareResult(True, 'released', mask, 0, (1 << 0) | (1 << 7), mask))
    monkeypatch.setattr(hand, 'manual_prepare_all_motors', writer)
    item = wait_operation(hand, hand.request_release_rope())
    assert item['status'] == 'SUCCESS'
    writer.assert_called_once_with(motor_mask=mask)
    assert set(hand._motor_transactions) == {0, 7}


def test_release_selected_tendons_rejects_empty_selection_without_writing(hand, monkeypatch):
    hand._selected_joints = ()
    writer = Mock()
    monkeypatch.setattr(hand, 'manual_prepare_all_motors', writer)
    assert not hand.request_release_rope().accepted
    writer.assert_not_called()

def test_overload_uses_fresh_fault_packet_instead_of_inferred_load(hand):
    state = hand._controller.current_state
    state.has_fault_status_data = state.has_servo_route_status = True
    state.fault_status_last_rx_ts = time.time()
    state.servo_overload_fault[0] = True
    assert hand.application_snapshot()['badges']['Overload'] == 'ACTIVE'
    state.servo_overload_fault[0] = False
    assert hand.application_snapshot()['badges']['Overload'] == 'CLEAR'
    state.fault_status_last_rx_ts = time.time() - 1
    assert hand.application_snapshot()['badges']['Overload'] == 'UNKNOWN'

def test_manual_transaction_blocks_other_motion_owners(hand):
    hand._manual_transaction_active = True
    assert not hand.begin_replay_motion()[0]
    assert not hand.begin_calibration_import({})
    assert not hand.begin_calibration_workflow(joint_mask=1, motor_mask=3, reset_motor_mask=3)
    assert not hand.set_active_scope(3, 1, True)
    hand._manual_transaction_active = False

def test_stop_does_not_wait_for_scope_ack(hand, monkeypatch):
    entered, release = threading.Event(), threading.Event()
    def scope(*args):
        entered.set()
        assert release.wait(2)
        return True
    monkeypatch.setattr(hand, 'stop', HandBackend.stop.__get__(hand))
    monkeypatch.setattr(hand._controller, 'stop', Mock())
    monkeypatch.setattr(hand._controller, 'set_active_scope', scope)
    monkeypatch.setattr(hand._controller, 'start', Mock())
    hand._controller.set_control_mode(ControlMode.DIRECT_MOTOR)
    result = hand.request_enable()
    assert entered.wait(1)
    started = time.monotonic()
    hand.stop_motion()
    assert time.monotonic() - started < 0.1
    release.set()
    assert wait_operation(hand, result)['status'] == 'FAILED'
    hand._controller.start.assert_not_called()
    assert hand.get_control_mode() == ControlMode.DIRECT_MOTOR

def test_import_full_manual_scope_resets_confirmations_outside_auto_scope(hand, monkeypatch):
    monkeypatch.setattr('desktop.core.calibration_workflow.validate_calibration_snapshot_for_load', lambda _: None)
    monkeypatch.setattr(hand._calibration_workflow._store, 'activate', Mock())
    monkeypatch.setattr(hand._controller, 'load_calibration_snapshot', lambda _: (True, 'readback OK'))
    hand._manual_drafts = {0: {'min': 12000}, 1: {'min': 13000}, 2: {'min': 14000}}
    payload = dict(active_joint_mask=1, active_motor_mask=3, manual_reference_valid_bitmap=3,
                   manual_reference_min_raw=[12000] * 21, manual_reference_max_raw=[11000] * 21)
    hand._import_previews['token'] = dict(name='test.json', payload=payload)
    assert wait_operation(hand, hand.request_calibration_import('token'))['status'] == 'SUCCESS'
    assert set(hand._manual_drafts) == {2}
    assert hand._manual_saved_pairs[1] == (12000, 11000)

def test_max_first_and_repeated_same_end_require_the_other_end(hand, monkeypatch):
    writer = Mock(return_value=True)
    monkeypatch.setattr(hand, 'set_manual_encoder_reference', writer)
    for _ in range(2):
        assert wait_operation(hand, hand.request_manual_capture(0, 'max'))['status'] == 'SUCCESS'
    writer.assert_not_called()
    hand._controller.current_state.encoders[0].raw_sensor = 13000
    wait_operation(hand, hand.request_manual_capture(0, 'min'))
    writer.assert_called_once_with(0, 13000, 12000)
    hand._controller.current_state.encoders[0].raw_sensor = 11500
    wait_operation(hand, hand.request_manual_capture(0, 'max'))
    writer.assert_called_with(0, 13000, 11500)

def test_save_failure_retains_pair_and_next_set_retries_once(hand, monkeypatch):
    writer = Mock(side_effect=[False, True])
    monkeypatch.setattr(hand, 'set_manual_encoder_reference', writer)
    hand._controller.last_manual_write_phase = 'INSTALLED_NOT_SAVED'
    wait_operation(hand, hand.request_manual_capture(0, 'min'))
    hand._controller.current_state.encoders[0].raw_sensor = 11000
    assert wait_operation(hand, hand.request_manual_capture(0, 'max'))['status'] == 'FAILED'
    assert hand.application_snapshot()['joints'][0]['joint_quality'] == ['FAILED']
    time.sleep(0.03)
    assert writer.call_count == 1
    assert wait_operation(hand, hand.request_manual_capture(0, 'max'))['status'] == 'SUCCESS'
    assert writer.call_count == 2
    assert set(('min', 'max')) <= hand._manual_drafts[0].keys()

def test_stop_bypasses_real_manual_backend_transaction_lock(hand, monkeypatch):
    entered, release = threading.Event(), threading.Event()
    def blocked(*args, cancelled):
        entered.set()
        assert release.wait(2)
        assert cancelled()
        return False
    monkeypatch.setattr(hand, 'stop', HandBackend.stop.__get__(hand))
    monkeypatch.setattr(hand._controller, 'stop', Mock())
    monkeypatch.setattr(hand._controller, 'set_manual_encoder_reference', blocked)
    wait_operation(hand, hand.request_manual_capture(0, 'min'))
    hand._controller.current_state.encoders[0].raw_sensor = 11000
    result = hand.request_manual_capture(0, 'max')
    assert entered.wait(1)
    started = time.monotonic()
    hand.stop_motion()
    assert time.monotonic() - started < 0.1
    assert not hand.request_enable().accepted
    release.set()
    assert wait_operation(hand, result)['status'] == 'FAILED'
    assert hand._manual_drafts[0]['status'] == 'CANCELLED'

def test_preview_only_validates_and_bad_new_file_clears_old_token(hand, monkeypatch, tmp_path):
    monkeypatch.setattr('desktop.core.application_actions.RUN_DATA_DIR', tmp_path)
    inspect = Mock(return_value={'active_joint_mask': 1, 'active_motor_mask': 3})
    activate = Mock()
    monkeypatch.setattr(hand, 'inspect_calibration_snapshot', inspect)
    monkeypatch.setattr(hand._calibration_workflow._store, 'activate', activate)
    wait_operation(hand, hand.request_calibration_preview('test.mmcal.json', b'{}'))
    assert len(hand._import_previews) == 1
    activate.assert_not_called()
    inspect.side_effect = ValueError('bad integrity')
    assert wait_operation(hand, hand.request_calibration_preview('bad.json', b'{}'))['status'] == 'FAILED'
    assert not hand._import_previews
    assert not list((tmp_path / 'imports').glob('*.json'))

@pytest.mark.parametrize('ok', [False, True])
def test_import_activates_only_after_readback_and_resets_only_file_scope(hand, monkeypatch, ok):
    monkeypatch.setattr('desktop.core.calibration_workflow.validate_calibration_snapshot_for_load', lambda _: None)
    activate = Mock()
    monkeypatch.setattr(hand._calibration_workflow._store, 'activate', activate)
    monkeypatch.setattr(hand._controller, 'load_calibration_snapshot', lambda _: (ok, 'readback result'))
    hand._manual_drafts = {0: {'min': 12000}, 1: {'min': 13000}}
    hand._import_previews['token'] = dict(name='test.json', payload={'active_joint_mask': 1, 'active_motor_mask': 3})
    result = hand.request_calibration_import('token')
    assert wait_operation(hand, result)['status'] == ('SUCCESS' if ok else 'FAILED')
    assert activate.call_count == int(ok)
    assert (0 not in hand._manual_drafts) == ok
    assert 1 in hand._manual_drafts
    if not ok:
        assert hand.application_snapshot()['joints'][0]['joint_quality'] == ['FAILED']

def test_cancelled_import_remains_busy_until_worker_leaves_and_never_activates(hand, monkeypatch):
    monkeypatch.setattr('desktop.core.calibration_workflow.validate_calibration_snapshot_for_load', lambda _: None)
    entered, release, activate = threading.Event(), threading.Event(), Mock()
    def load(_):
        entered.set(); assert release.wait(2)
        return True, 'read back'
    monkeypatch.setattr(hand._controller, 'load_calibration_snapshot', load)
    monkeypatch.setattr(hand._calibration_workflow._store, 'activate', activate)
    hand._import_previews['token'] = dict(name='test.json', payload={'active_joint_mask': 1, 'active_motor_mask': 3})
    result = hand.request_calibration_import('token')
    assert entered.wait(1)
    hand.stop_motion()
    assert not hand.request_enable().accepted
    release.set()
    assert wait_operation(hand, result)['status'] == 'FAILED'
    activate.assert_not_called()

class Recorder:
    def __init__(self):
        self.running, self.starts, self.stops, self.error = False, 0, 0, ''
        self.path, self.events, self.topics = '', [], []
    def is_running(self): return self.running
    def start(self, directory, *, topics, meta):
        self.running = True; self.starts += 1; self.path = f'session-{self.starts}'; self.topics = topics
    def stop(self):
        if self.running: self.stops += 1
        self.running = False
    def consume_error(self):
        error, self.error = self.error, ''
        return error
    def record_packets(self, *args): pass
    def event(self, name, **kwargs):
        if self.running: self.events.append(name)

def reconcile(hand):
    hand._reconcile_recording()
    if hand._recording_thread: hand._recording_thread.join(1)

def test_recording_waits_for_fresh_p4_reconnects_and_keeps_stop_event(hand, monkeypatch):
    recorder = Recorder()
    monkeypatch.setattr(hand, '_recorder', recorder)
    monkeypatch.setattr(hand, 'set_control_trace_subscription', lambda _: True)
    connected = [False]
    monkeypatch.setattr(hand, 'is_comm_connected', lambda: connected[0])
    assert hand._save_data
    reconcile(hand); assert recorder.starts == 0
    connected[0] = True
    reconcile(hand); reconcile(hand)
    assert recorder.starts == 1 and 'tactile_full' not in recorder.topics
    hand.stop_motion(); assert recorder.running and 'stop' in recorder.events
    connected[0] = False
    reconcile(hand); assert recorder.stops == 1 and hand._save_data
    connected[0] = True
    reconcile(hand); assert recorder.starts == 2
    hand.request_recording(False)
    if hand._recording_thread: hand._recording_thread.join(1)
    reconcile(hand); assert recorder.stops == 2

def test_recording_error_turns_save_off_without_snapshot_consuming_error(hand, monkeypatch):
    recorder = Recorder()
    monkeypatch.setattr(hand, '_recorder', recorder)
    monkeypatch.setattr(hand, 'set_control_trace_subscription', lambda _: True)
    reconcile(hand)
    recorder.error = 'disk full'
    hand.application_snapshot()
    reconcile(hand)
    assert not hand._save_data and not recorder.running
    assert hand._recording_error == 'disk full'

def test_motor_workflow_failure_overrides_cached_success(hand):
    hand._calibration_operation_id = 'run'
    hand._application_busy = 'run'
    hand._application_calibration_event(CalibrationWorkflowEvent(1, 'phase_complete', False, 'readback failed', joint_mask=1))
    assert hand.application_snapshot()['joints'][0]['motor_quality'] == ['FAILED']
