"""Console transactions, cancellation and recording lifecycle with simulated hardware."""

from _clock_helpers import live_status_feedback, stamp
import threading
import time
from unittest.mock import ANY, Mock
import pytest
from test_application_actions import hand, wait_operation
from desktop.core.hand_backend import HandBackend
from desktop.core.calibration_workflow import CalibrationWorkflowEvent
from desktop.core.core_logic import ManualPrepareResult
from desktop.core.comm_layer import LowerComputerComm
from desktop.core.host_calibration.config import joint_motors
from desktop.core.protocol import (
    ControlMode, CMD_CONTROL_RELEASE, SYSTEM_STATE_IDLE, SYSTEM_STATE_READY,
    SYSTEM_STATE_RUN_DIRECT, SYSTEM_STATE_RUN_FEEDBACK,
)


def test_s3_rx_newer_than_snapshot_clock_is_online_instead_of_no_valid_frames(hand, monkeypatch):
    monkeypatch.setattr(time, 'time', lambda: 1000.0)
    comm = Mock()
    comm.get_latest_model.return_value = None
    comm.connection_diagnostics.return_value = dict(last_valid_rx=time.time()+0.01,
        opened_at=time.time()-20, bytes=100, frames=1)
    hand.tactile_comm = comm
    monkeypatch.setattr(hand, 'is_tactile_serial_connected', lambda: True)
    connection = hand.application_snapshot()['connections']['s3']
    assert connection['age_ms'] == 0
    assert connection['data_status'] == 'Online'


def joint_preparation_device(hand, monkeypatch, *, ready_state=SYSTEM_STATE_READY,
                             fault=None, live_status=True):
    """Emulate P4 scope/START replies, keeping the actual host command queue.

    ``live_status`` streams synthetic status frames for the whole test, which is
    what real hardware does and what the enable predicate needs. Tests that
    exercise a stale status frame pass ``live_status=False``.
    """
    controller = hand._controller
    state = controller.current_state
    controller.set_control_mode(ControlMode.DIRECT_MOTOR)
    comm = LowerComputerComm('')
    comm._protocol_compatible = True
    monkeypatch.setattr(comm, 'is_connected', lambda: True)
    monkeypatch.setattr(controller, 'comm', comm)
    hand._selected_joints = (3,)  # Explicit user selection for these enable scenarios.
    monkeypatch.setattr(comm, 'apply_control_parameters', Mock(return_value=(1, 0, 1, 0)))

    def scope(motors, joints, require_can):
        state.has_active_scope_status = True
        state.active_motor_mask, state.active_joint_mask = motors, joints
        state.require_can_feedback = require_can
        state.control_mode = ControlMode.JOINT_ANGLE.value
        state.system_status_last_rx_ts = time.time()
        stamp(state, 'system_status_last_rx_ts')
        return True

    scope_writer = Mock(side_effect=scope)
    monkeypatch.setattr(controller, 'set_active_scope', scope_writer)
    state.has_sensor_data = True
    state.encoder_mapped_last_rx_ts = time.time()
    stamp(state, 'encoder_mapped_last_rx_ts')
    state.encoder_mapped_p4_time_ms = state.p4_time_ms
    for encoder in state.encoders:
        encoder.angle_error = False
        encoder.raw = 0
    monkeypatch.setattr(hand, '_apply_joint_workspace_guard',
        lambda state, values, **kwargs: (list(values), [], [], ''))
    send_command = comm.send_command

    def reply(command):
        sent = send_command(command)
        if sent and command == 'start':
            state.system_state = ready_state
            state.control_enabled = False
            state.servo_target_owner = 0
            if fault != 'old_status':
                state.system_status_last_rx_ts = time.time()
                stamp(state, 'system_status_last_rx_ts')
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
        if sent and command == 'control_release':
            received = time.time()
            comm._ack_tracker.publish(CMD_CONTROL_RELEASE, state.system_state, 0, received)
            state.system_state = SYSTEM_STATE_READY
            state.servo_target_owner = 0; state.control_enabled = False
            state.system_status_last_rx_ts = received + .000001
            stamp(state, 'system_status_last_rx_ts')
        if sent and isinstance(command, tuple) and command[0] == 'joint_target_run_masked':
            state.system_state = SYSTEM_STATE_RUN_FEEDBACK
            state.control_enabled = True
            state.servo_target_owner = 1
            state.system_status_last_rx_ts = time.time()
            stamp(state, 'system_status_last_rx_ts')
        return sent

    monkeypatch.setattr(comm, 'send_command', reply)
    if live_status:
        # Real hardware streams status frames; a synchronous mock would go stale
        # during the multi-second enable wait and never satisfy its predicate.
        feed = live_status_feedback(state)
        feed.__enter__()
        # Keep the generator alive so its finally clause can stop the thread.
        monkeypatch.setattr(comm, 'status_feed', feed, raising=False)
    return comm, scope_writer


def test_joint_enable_holds_current_pose_and_confirms_actual_motion_owner(hand, monkeypatch):
    comm, scope = joint_preparation_device(hand, monkeypatch)
    hand._selected_joints = (3,)
    comm.tx_queue.put_nowait(('motor_pos_abs_one', 3, 500))
    comm.tx_queue.put_nowait('start')

    item = wait_operation(hand, hand.request_enable())

    assert item['status'] == 'SUCCESS'
    assert 'closed-loop holding the current pose' in item['message']
    assert hand.get_control_mode() == ControlMode.JOINT_ANGLE
    scope.assert_called_once_with(sum(1 << m for m in joint_motors(3)), 1 << 3, True)
    commands = list(comm.tx_queue.queue)
    assert commands[0] == 'start'
    assert commands[1][0] == 'joint_target_run_masked'
    assert commands[1][1][3] == 0.0
    assert commands[1][2] == 1 << 3
    assert hand._controller.current_state.control_enabled
    assert hand._controller.current_state.servo_target_owner == 1


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


def test_joint_enable_requires_parameter_application_before_sending_a_target(hand, monkeypatch):
    comm, _ = joint_preparation_device(hand, monkeypatch)
    monkeypatch.setattr(comm, 'apply_control_parameters', Mock(side_effect=ValueError('parameter CRC mismatch')))
    item = wait_operation(hand, hand.request_enable())
    assert item['status'] == 'FAILED'
    assert 'parameter CRC mismatch' in item['message']
    assert not any(isinstance(c, tuple) and c[0] == 'joint_target_run_masked'
                   for c in comm.tx_queue.queue)


@pytest.mark.parametrize('after_ack', ['moved', 'stale', 'invalid', 'stop'])
def test_joint_enable_rechecks_current_pose_after_parameter_sync(hand, monkeypatch, after_ack):
    comm, _ = joint_preparation_device(hand, monkeypatch)
    hand._selected_joints = (3,)
    state = hand._controller.current_state

    def change_pose(*args, **kwargs):
        if after_ack == 'moved':
            state.encoders[3].raw = 512
        elif after_ack == 'stale':
            state.encoder_mapped_last_rx_ts = time.time() - 2
            stamp(state, 'encoder_mapped_last_rx_ts')
        elif after_ack == 'invalid':
            state.encoders[3].angle_error = True
        else:
            hand.stop_motion()
        return (1,0,1,0)

    monkeypatch.setattr(comm, 'apply_control_parameters', change_pose)
    item = wait_operation(hand, hand.request_enable())
    targets = [c for c in comm.tx_queue.queue
               if isinstance(c, tuple) and c[0] == 'joint_target_run_masked']
    if after_ack == 'moved':
        assert item['status'] == 'SUCCESS', item['message']
        assert targets[0][1][3] == state.encoders[3].current_angle_deg
    else:
        assert item['status'] == 'FAILED'
        assert not targets


def test_joint_enable_never_clamps_the_current_pose(hand, monkeypatch):
    comm, _ = joint_preparation_device(hand, monkeypatch)
    monkeypatch.setattr(hand, '_apply_joint_workspace_guard',
                        lambda state, values, **kwargs: (values, [], [3], 'J03 outside limits'))
    # 回正被 P4 拒绝时 Enable 仍然只能失败：绝不把夹回后的目标当成“保持当前姿态”。
    monkeypatch.setattr(hand, 'pull_joint_into_workspace', lambda mask: False)
    item = wait_operation(hand, hand.request_enable())
    assert item['status'] == 'FAILED'
    assert 'J03 outside limits' in item['message']
    assert not any(isinstance(c, tuple) and c[0] == 'joint_target_run_masked'
                   for c in comm.tx_queue.queue)


def test_joint_enable_waits_for_ready_not_idle(hand, monkeypatch):
    """P4 在 IDLE 与 JOINT_BEYOND_WORKSPACE 之间抖动时，主机不得把 IDLE 当成“已就绪”。"""
    comm, _ = joint_preparation_device(hand, monkeypatch, ready_state=SYSTEM_STATE_IDLE)
    original = hand._wait_application
    monkeypatch.setattr(hand, '_wait_application',
                        lambda predicate, epoch, timeout=0: original(predicate, epoch, 0.1))
    item = wait_operation(hand, hand.request_enable())
    assert item['status'] == 'FAILED'
    assert 'not confirmed' in item['message']
    assert not any(isinstance(c, tuple) and c[0] == 'joint_target_run_masked'
                   for c in comm.tx_queue.queue)


@pytest.mark.parametrize('status_ms,sample_ms,age,expected', [
    (1000, 1200, 0.1, True), (1200, 1000, 0.1, True),
    (0xfffffff0, 10, 0.1, True), (10, 0xfffffff0, 0.1, True),
    (1000, 1800, 0.1, False), (1000, 1000, 1.0, False),
])
def test_mapped_angle_freshness_accepts_heartbeat_skew_and_clock_wrap(
        hand, status_ms, sample_ms, age, expected):
    state = hand._controller.current_state
    state.has_sensor_data = True
    state.p4_time_ms = status_ms
    state.encoder_mapped_p4_time_ms = sample_ms
    state.encoder_mapped_last_rx_ts = time.time() - age
    stamp(state, 'encoder_mapped_last_rx_ts')
    assert hand._mapped_feedback_fresh(state) is expected


@pytest.mark.parametrize('p4_state,owner,enabled', [
    (SYSTEM_STATE_RUN_DIRECT, 1, True),
    (SYSTEM_STATE_RUN_FEEDBACK, 1, True),
    (SYSTEM_STATE_READY, 1, False),
    (SYSTEM_STATE_READY, 0, True),
])
def test_joint_enable_hands_over_existing_normal_control_without_stop(hand, monkeypatch, p4_state, owner, enabled):
    comm, scope = joint_preparation_device(hand, monkeypatch)
    state = hand._controller.current_state
    state.system_state, state.servo_target_owner, state.control_enabled = p4_state, owner, enabled

    item = wait_operation(hand, hand.request_enable())

    assert item['status'] == 'SUCCESS', item['message']
    scope.assert_called_once()
    commands = list(comm.tx_queue.queue)
    assert 'control_release' in commands and 'stop' not in commands and 'reset' not in commands
    assert hand.get_control_mode() == ControlMode.JOINT_ANGLE


def test_calibrate_selected_auto_releases_active_owner(hand, monkeypatch):
    comm, _scope = joint_preparation_device(hand, monkeypatch)
    state = hand._controller.current_state
    state.system_state = SYSTEM_STATE_RUN_FEEDBACK
    state.control_enabled = True
    state.servo_target_owner = 1
    state.manual_reference_valid_bitmap |= 1 << 3
    state.manual_reference_min_raw[3] = 6000
    state.manual_reference_max_raw[3] = 10000
    hand._selected_joints = (3,)
    entered = threading.Event()
    monkeypatch.setattr(
        hand, 'begin_calibration_workflow',
        Mock(side_effect=lambda **kwargs: entered.set() or True))

    result = hand.request_calibration(origin='Motor Calibration')

    assert result.accepted
    assert entered.wait(2)
    assert 'control_release' in list(comm.tx_queue.queue)
    operation = next(item for item in hand._application_operations
                     if item['operation_id'] == result.operation_id)
    assert operation['origin'] == 'Motor Calibration'
    hand._finish_application_operation(result.operation_id, False, 'test cleanup')

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


def test_saved_manual_not_loaded_in_p4_is_distinguished_from_missing(hand):
    hand._manual_saved_pairs[11] = (12000, 11000)
    rows = hand.application_snapshot()['joints']
    assert rows[11]['persistence'] == 'Saved in JSON; not loaded in P4'
    assert not rows[11]['manual_valid']
    assert rows[10]['persistence'] == 'RAM / JSON unknown'


def test_clear_manual_selected_passes_only_selected_json_scope(hand, monkeypatch):
    hand._selected_joints = (12, 13)
    state = hand._controller.current_state
    state.manual_reference_min_raw = [100] * 21
    state.manual_reference_max_raw = [5000] * 21
    hand._manual_saved_pairs[11] = (100, 5000)
    monkeypatch.setattr(hand, 'query_manual_encoder_references', lambda: True)
    writer = Mock(return_value=True)
    monkeypatch.setattr(hand, 'set_manual_encoder_references', writer)
    item = wait_operation(hand, hand.request_clear_manual())
    assert item['status'] == 'SUCCESS'
    assert writer.call_args.kwargs == {'updated_joint_mask': (1 << 12) | (1 << 13)}
    assert writer.call_args.args[0][11:14] == [100, 0, 0]
    assert writer.call_args.args[1][11:14] == [5000, 0, 0]
    assert hand._manual_saved_pairs[11] == (100, 5000)


@pytest.mark.parametrize('faults', [0, 0x50, 0x51])
def test_clear_faults_releases_host_stop_even_when_live_hardware_faults_recur(hand, monkeypatch, faults):
    comm, _ = joint_preparation_device(hand, monkeypatch)
    state = hand._controller.current_state
    hand._estop_requested_at = time.time() - 1
    state.system_fault_bitmap = faults
    assert hand.application_snapshot()['device'] == 'Stopped'

    def reset_ack(*args, **kwargs):
        received = time.time()
        state.system_status_last_rx_ts = received + 0.000001
        stamp(state, 'system_status_last_rx_ts')
        return (206, 0, 2, received)

    monkeypatch.setattr(comm, 'wait_for_ack', reset_ack)
    item = wait_operation(hand, hand.request_clear_faults())
    assert item['status'] == 'SUCCESS'
    assert hand._estop_requested_at == 0
    assert hand.application_snapshot()['device'] == ('Fault' if faults else 'Ready')
    assert list(comm.tx_queue.queue) == ['reset']
    if faults:
        assert f'0x{faults:08X}' in item['message']


@pytest.mark.parametrize('failure', ['no_ack', 'rejected', 'old_status', 'new_stop'])
def test_clear_faults_cannot_clear_stop_without_ack_and_fresh_stationary_status(hand, monkeypatch, failure):
    comm, _ = joint_preparation_device(hand, monkeypatch, live_status=False)
    hand._estop_requested_at = time.time() - 1
    def reset_ack(*args, **kwargs):
        if failure == 'no_ack':
            return None
        received = time.time()
        if failure == 'new_stop':
            hand.stop_motion()
        if failure == 'old_status':
            # The request gate already saw fresh status. This case is about the
            # post-ACK confirmation, so the frame goes stale right after the ACK.
            state = hand._controller.current_state
            state.system_status_last_rx_ts = received - 5
            stamp(state, 'system_status_last_rx_ts')
        return (206, 0, 1 if failure == 'rejected' else 2, received)

    monkeypatch.setattr(comm, 'wait_for_ack', reset_ack)
    monkeypatch.setattr(hand, '_wait_application',
                        lambda predicate, epoch: epoch == hand._application_epoch and predicate(hand.get_state_snapshot()))
    item = wait_operation(hand, hand.request_clear_faults())
    assert item['status'] == 'FAILED'
    assert hand._estop_requested_at != 0


def test_release_selected_tendons_freezes_only_selected_joint_motors(hand, monkeypatch):
    hand._selected_joints = (0, 7)
    mask = sum(1 << m for j in (0, 7) for m in joint_motors(j))
    writer = Mock(return_value=ManualPrepareResult(True, 'released', mask, 0, (1 << 0) | (1 << 7), mask))
    monkeypatch.setattr(hand, 'manual_prepare_all_motors', writer)
    item = wait_operation(hand, hand.request_release_rope())
    assert item['status'] == 'SUCCESS'
    writer.assert_called_once_with(motor_mask=mask, cancelled=ANY)
    assert set(hand._motor_transactions) == {0, 7}


def test_backend_manual_prepare_combines_external_and_manual_cancellation(hand, monkeypatch):
    forwarded = {}

    def fake_controller_release(**kwargs):
        forwarded.update(kwargs)
        return ManualPrepareResult(True, 'released', 1 << 3, 0, 1 << 3, 1 << 3)

    monkeypatch.setattr(hand._controller, 'manual_prepare_all_motors', fake_controller_release)
    external = Mock(return_value=True)
    result = hand.manual_prepare_all_motors(motor_mask=1 << 3, cancelled=external)
    assert result.ok
    assert forwarded['cancelled']() is True
    external.assert_called_once_with()


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
    stamp(state, 'fault_status_last_rx_ts')
    state.servo_overload_fault[0] = True
    assert hand.application_snapshot()['badges']['Overload'] == 'ACTIVE'
    state.servo_overload_fault[0] = False
    assert hand.application_snapshot()['badges']['Overload'] == 'CLEAR'
    state.fault_status_last_rx_ts = time.time() - 1
    stamp(state, 'fault_status_last_rx_ts')
    assert hand.application_snapshot()['badges']['Overload'] == 'UNKNOWN'

def test_workspace_badges_and_motor_row_expose_both_workspaces(hand):
    state = hand._controller.current_state
    state.encoders[4].angle_out_of_workspace = True
    state.has_servo_angle_data = True
    state.servo_abs_last_rx_ts = time.time()
    stamp(state, 'servo_abs_last_rx_ts')
    state.servo_online[25] = True
    state.servo_angles[25] = 250
    state.servo_logical_range_valid_bitmap |= 1 << 25
    state.servo_logical_range_min[25] = 100
    state.servo_logical_range_max[25] = 200
    snapshot = hand.application_snapshot()
    assert snapshot['badges']['Joint workspace'] == 'BEYOND'
    assert snapshot['badges']['Motor workspace'] == 'BEYOND'
    assert snapshot['joints'][4]['angle_out_of_workspace'] is True
    assert snapshot['motors'][25]['motor_beyond_workspace'] is True
    assert snapshot['motors'][24]['motor_beyond_workspace'] is False
    state.encoders[4].angle_out_of_workspace = False
    state.servo_angles[25] = 150
    assert hand.application_snapshot()['badges']['Joint workspace'] == 'CLEAR'
    assert hand.application_snapshot()['badges']['Motor workspace'] == 'CLEAR'
    state.system_state = 9
    assert hand.application_snapshot()['badges']['Joint workspace'] == 'BEYOND'
    state.system_state = 1
    assert hand.application_snapshot()['badges']['Joint workspace'] == 'CLEAR'

def test_manual_transaction_blocks_other_motion_owners(hand):
    hand._manual_transaction_active = True
    assert not hand.begin_replay_motion()[0]
    assert not hand.begin_calibration_import({})
    assert not hand.begin_calibration_workflow(joint_mask=1, motor_mask=3)
    assert not hand.set_active_scope(3, 1, True)
    hand._manual_transaction_active = False

def test_stop_does_not_wait_for_scope_ack(hand, monkeypatch):
    hand._selected_joints = (3,)
    monkeypatch.setattr(hand, '_synchronize_control_parameters', lambda epoch: (True, 'applied'))
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
    monkeypatch.setattr(hand._controller, 'wait_for_multiturn_recovery', Mock(return_value=(True, 'recovered')))
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
    hand._controller.current_state.encoders[0].raw_sensor = 11000
    wait_operation(hand, hand.request_manual_capture(0, 'min'))
    assert writer.call_count == 1 and writer.call_args.args == (0, 11000, 12000)
    hand._controller.current_state.encoders[0].raw_sensor = 12500
    wait_operation(hand, hand.request_manual_capture(0, 'max'))
    assert writer.call_args.args == (0, 11000, 12500)

def test_save_failure_retains_pair_and_next_set_retries_once(hand, monkeypatch):
    writer = Mock(side_effect=[False, True])
    monkeypatch.setattr(hand, 'set_manual_encoder_reference', writer)
    hand._controller.last_manual_write_phase = 'INSTALLED_NOT_SAVED'
    wait_operation(hand, hand.request_manual_capture(0, 'min'))
    hand._controller.current_state.encoders[0].raw_sensor = 13000
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
    hand._controller.current_state.encoders[0].raw_sensor = 13000
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
    monkeypatch.setattr(hand._controller, 'wait_for_multiturn_recovery', Mock(return_value=(True, 'recovered')))
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
        # 整包加载失败时没有关节被实际尝试；不能把关节标成校准失败。
        assert hand.application_snapshot()['joints'][0]['joint_quality'] == ['PENDING']
        assert 0 not in hand._joint_transactions

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


@pytest.mark.parametrize('p4_state,expected', [
    (0, 'Stopped'), (1, 'Idle'), (5, 'Ready'), (8, 'Pull Joint into Workspace'), (9, 'Joint Beyond Workspace')])
def test_system_label_reports_each_p4_state_instead_of_disabled(hand, monkeypatch, p4_state, expected):
    """四个不同 P4 状态过去全显示成 'Disabled'，让急停锁死看起来像待机。"""
    state = hand._controller.current_state
    comm = LowerComputerComm('')
    comm._protocol_compatible = True
    monkeypatch.setattr(comm, 'is_connected', lambda: True)
    monkeypatch.setattr(hand._controller, 'comm', comm)
    hand._estop_requested_at = 0.0
    state.has_system_status_data = True
    state.system_fault_bitmap = 0
    state.control_enabled = 0
    state.servo_target_owner = 0
    state.system_state = p4_state
    state.system_status_last_rx_ts = time.time()
    stamp(state, 'system_status_last_rx_ts')
    assert hand.application_snapshot()['device'] == expected


def test_clear_faults_applicable_only_when_there_is_something_to_clear(hand, monkeypatch):
    state = hand._controller.current_state
    comm = LowerComputerComm('')
    comm._protocol_compatible = True
    monkeypatch.setattr(comm, 'is_connected', lambda: True)
    monkeypatch.setattr(hand._controller, 'comm', comm)
    state.has_system_status_data = True
    state.control_enabled = 0
    state.servo_target_owner = 0
    state.system_status_last_rx_ts = time.time()
    stamp(state, 'system_status_last_rx_ts')

    hand._estop_requested_at = 0.0
    state.system_fault_bitmap = 0
    state.system_state = SYSTEM_STATE_READY
    snapshot = hand.application_snapshot()
    assert snapshot['p4_state_code'] == SYSTEM_STATE_READY
    assert snapshot['clear_faults_applicable'] is False

    state.system_state = 0  # Stopped：必须显式 RESET 才能离开
    assert hand.application_snapshot()['clear_faults_applicable'] is True

    state.system_state = SYSTEM_STATE_READY
    state.system_fault_bitmap = 0x10
    assert hand.application_snapshot()['clear_faults_applicable'] is True

    state.system_fault_bitmap = 0
    hand._estop_requested_at = time.time()
    assert hand.application_snapshot()['clear_faults_applicable'] is True
