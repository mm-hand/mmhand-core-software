"""Console transactions, cancellation and recording lifecycle with simulated hardware."""
import threading
import time
from unittest.mock import Mock
import pytest
from test_application_actions import hand, wait_operation
from desktop.core.hand_backend import HandBackend
from desktop.core.calibration_workflow import CalibrationWorkflowEvent

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
    result = hand.request_enable()
    assert entered.wait(1)
    started = time.monotonic()
    hand.stop_motion()
    assert time.monotonic() - started < 0.1
    release.set()
    assert wait_operation(hand, result)['status'] == 'FAILED'

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
