"""用户操作边界回归；不连接真实硬件。"""

from _clock_helpers import stamp
import threading
import time
import queue
from unittest.mock import Mock
import pytest
from desktop.core.application_actions import validate_manual_pair
from desktop.core.hand_backend import HandBackend
from desktop.core.protocol import (SYSTEM_STATE_READY, SYSTEM_STATE_RUN_DIRECT,
                                   SYSTEM_STATE_UNCONTROLLABLE, HOST_PROTOCOL_VERSION, STATE_SYNC_ALL)
from desktop.core.core_logic import HandController, ServoRebaseZeroResult
from _conftest_support import wait_operation


@pytest.mark.parametrize('kind',['joint','motor'])
@pytest.mark.parametrize('active',[False,True])
def test_selection_uses_actual_motion_not_ready_started_flag(hand,monkeypatch,kind,active):
    hand._controller._started=True
    state=hand._controller.current_state
    state.control_enabled=active
    state.servo_target_owner=1 if active else 0
    stopped=Mock()
    monkeypatch.setattr(hand,'stop_motion',stopped)
    if kind=='joint': hand.select_joints([13])
    else: hand.select_motors([17])
    assert stopped.call_count==int(active)


def test_received_feedback_is_published_before_periodic_disk_save(monkeypatch):
    from desktop.core.data_models import HandModel
    c=HandController(); c.running=True
    q=queue.Queue(); sample=HandModel(); sample.p4_time_ms=12345
    q.put(sample)
    c.comm=Mock(rx_queue=q,drain_ack_events=lambda:[],drain_safety_events=lambda:[])
    for name in ('_service_startup_state_sync',
                 '_service_servo_rebase_zero_capture','_service_multiturn_anchor_capture'):
        monkeypatch.setattr(c,name,Mock())
    observed=[]
    def save(_state):
        observed.append(c.current_state.p4_time_ms)
        c.running=False
    monkeypatch.setattr(c,'_queue_multiturn_persistence',save)
    c._update_loop()
    assert observed==[12345]


def test_slow_periodic_anchor_writer_does_not_block_feedback_publication(monkeypatch):
    from desktop.core.data_models import HandModel
    c=HandController(); entered=threading.Event(); finish=threading.Event()
    def slow(_state):
        entered.set(); assert finish.wait(2)
    monkeypatch.setattr(c,'_persist_multiturn_state_locked',slow)
    c._start_persistence_worker()
    try:
        c._queue_multiturn_persistence(HandModel()); assert entered.wait(1)
        q=queue.Queue(); sample=HandModel(); sample.p4_time_ms=54321; q.put(sample)
        c.comm=Mock(rx_queue=q,drain_ack_events=lambda:[],drain_safety_events=lambda:[])
        c.running=True
        monkeypatch.setattr(c,'_service_startup_state_sync',Mock())
        monkeypatch.setattr(c,'_queue_multiturn_persistence',lambda _s:setattr(c,'running',False))
        before=time.monotonic(); c._update_loop()
        assert time.monotonic()-before<.15
        assert c.current_state.p4_time_ms==54321
    finally:
        finish.set(); c._stop_persistence_worker()


def test_queued_anchor_older_than_verified_zero_feedback_is_discarded(monkeypatch):
    from desktop.core.data_models import HandModel
    c=HandController(); c._servo_abs_zero_generation_feedback_ms[0]=200
    saved=Mock(); monkeypatch.setattr(c,'_persist_multiturn_state_locked',saved)
    c._start_persistence_worker()
    try:
        sample=HandModel(); sample.servo_feedback_p4_time_ms=150
        c._queue_multiturn_persistence(sample)
        c._persistence_queue.join()
        assert not saved.called
    finally: c._stop_persistence_worker()



def direct_target_device(hand, monkeypatch, *, load=0, reject_reason=0):
    """Keep the real request/worker path and emulate execution-layer readback."""
    state = hand._controller.current_state
    state.has_servo_angle_data = state.has_servo_route_status = True
    state.has_connectivity_status = True
    state.servo_abs_last_rx_ts = state.servo_telem_last_rx_ts = time.time()
    stamp(state, 'servo_abs_last_rx_ts')
    stamp(state, 'servo_telem_last_rx_ts')
    state.servo_online_bitmap = state.servo_route_controllable_bitmap = (1 << 27) - 1
    state.servo_online = state.servo_telem_online = [True] * 27
    state.servo_logical_range_valid_bitmap = (1 << 27) - 1
    state.servo_logical_range_min = [-30719] * 27
    state.servo_logical_range_max = [30719] * 27
    state.servo_load[11] = load
    monkeypatch.setattr(hand, 'set_active_scope', Mock(return_value=True))
    monkeypatch.setattr(hand, 'enable_direct_hold',
                        Mock(return_value=(True, 'Direct hold enabled', list(state.servo_angles))))
    comm = Mock()

    def execution_reply(command):
        if not isinstance(command, tuple):
            return True
        assert command[0] == 'motor_pos_abs'
        _, token, mask, values, speed, acceleration = command
        from desktop.core.protocol import DIRECT_SERVO_SPEED, DIRECT_SERVO_ACC
        assert (speed, acceleration) == (DIRECT_SERVO_SPEED, DIRECT_SERVO_ACC)
        state.direct_target_token = token
        state.direct_target_applied_mask = 0 if reject_reason else mask
        state.direct_target_rejected_mask = mask if reject_reason else 0
        state.direct_target_status = 3 if reject_reason else 2
        state.direct_reject_reason = reject_reason
        state.direct_target_result_rx_ts = time.time()
        stamp(state, 'direct_target_result_rx_ts')
        state.system_status_last_rx_ts = state.servo_abs_last_rx_ts = state.servo_telem_last_rx_ts = time.time()
        stamp(state, 'system_status_last_rx_ts')
        stamp(state, 'servo_abs_last_rx_ts')
        stamp(state, 'servo_telem_last_rx_ts')
        for motor in range(27):
            if mask & (1 << motor):
                state.servo_angles[motor] = values[motor]
        return True

    comm.send_command.side_effect = execution_reply
    monkeypatch.setattr(hand._controller, 'comm', comm)
    return comm


@pytest.mark.parametrize('load', [-1023, -800, -799, 0, 799, 800, 1023])
def test_direct_targets_forward_signed_loads_to_p4(hand, monkeypatch, load):
    comm = direct_target_device(hand, monkeypatch, load=load)
    assert wait_operation(hand, hand.request_motors({11: 3000}))['status'] == 'SUCCESS'
    command = comm.send_command.call_args.args[0]
    assert command[0] == 'motor_pos_abs' and command[2] == 1 << 11 and command[3][11] == 3000


@pytest.mark.parametrize('reason', [1, 2, 3, 6])
def test_direct_target_uses_p4_rejection(hand, monkeypatch, reason):
    comm = direct_target_device(hand, monkeypatch, load=1023, reject_reason=reason)
    result = wait_operation(hand, hand.request_motors({11: 3000}))
    assert result['status'] == 'FAILED'
    assert f'reason={reason}' in result['message']
    assert comm.send_command.call_args.args[0][2] == 1 << 11


@pytest.mark.parametrize('fault', [None, 'stale', 'unknown', 'offline', 'missing'])
def test_half_turn_uses_fresh_abs_after_direct_preparation(hand, monkeypatch, fault):
    from desktop.core.motor_mapping import motor_abs_after_tendon_action

    comm = direct_target_device(hand, monkeypatch)
    hand._selected_motors = (11,)
    state = hand._controller.current_state
    state.servo_angles[11] = 2000

    def prepare(_mask):
        state.has_p4_time = True
        state.has_servo_angle_data = fault != 'missing'
        state.p4_time_ms = state.servo_feedback_p4_time_ms = 1500
        state.monotonic_received['servo_abs_last_rx_ts'] = time.monotonic() - (1 if fault == 'stale' else 0)
        state.servo_angles[11] = 4000
        if fault == 'unknown':
            state.multi_turn_unknown_bitmap = 1 << 11
        elif fault == 'offline':
            state.servo_online_bitmap &= ~(1 << 11)
        return True, '', []

    monkeypatch.setattr(hand, 'enable_direct_hold', prepare)
    result = wait_operation(hand, hand.request_tendon_step(tighten=True))
    if fault:
        assert result['status'] == 'FAILED'
        assert 'stale or untrusted' in result['message']
        comm.send_command.assert_not_called()
    else:
        assert result['status'] == 'SUCCESS'
        assert comm.send_command.call_args.args[0][3][11] == motor_abs_after_tendon_action(4000, 11, 2048)


def direct_zero_device(hand, monkeypatch, *, success=None, fault='', change_selection=False):
    """模拟 P4 发布；不会让 UI 或业务操作直接改写反馈。"""
    state = hand._controller.current_state
    state.has_servo_route_status = True
    state.has_product_fingerprint = True
    state.product_fingerprint = {"protocol_version": HOST_PROTOCOL_VERSION, "device_id": "0000123456789ABC"}
    state.servo_online_bitmap = state.servo_route_controllable_bitmap = (1 << 27) - 1
    state.servo_angles = [7000 + m for m in range(27)]
    state.servo_online = [True] * 27
    state.system_state = SYSTEM_STATE_RUN_DIRECT
    state.control_enabled = True
    state.servo_target_owner = 1

    def stop_readback():
        state.system_status_last_rx_ts = time.time()
        stamp(state, 'system_status_last_rx_ts')
        state.system_state = SYSTEM_STATE_READY
        state.control_enabled = False
        state.servo_target_owner = 0
        if change_selection:
            hand._selected_motors = (0,)
        if fault == 'cancel':
            hand.stop_motion()

    monkeypatch.setattr(hand._controller, 'stop', Mock(side_effect=stop_readback))
    monkeypatch.setattr(hand._controller, 'invalidate_direct_commands', Mock())

    state.has_servo_online_status = True
    state.servo_online_check = [1] * 27

    def zero_readback(mask, *, cancelled, send_gate):
        assert not cancelled() and send_gate(lambda: True)
        done = mask if success is None else success
        if fault == 'rejected':
            return ServoRebaseZeroResult(False, 'hardware rejected', 0, 0, mask)
        state.has_servo_angle_data = True
        state.servo_rebase_zero_result_token = 18
        state.servo_rebase_zero_result_p4_time_ms = 1000
        state.servo_feedback_p4_time_ms = 1001
        state.servo_abs_last_rx_ts = time.time()
        stamp(state, 'servo_abs_last_rx_ts')
        for motor in range(27):
            if done & (1 << motor):
                state.servo_angles[motor] = 2048
        if fault == 'old_feedback':
            state.servo_feedback_p4_time_ms = 999
        if fault == 'unknown':
            state.multi_turn_unknown_bitmap = done
        if fault == 'one_offline':
            state.servo_online_bitmap &= ~(1 << 9)
        hand._controller._servo_rebase_zero_capture_mask = done
        hand._controller._servo_rebase_zero_capture_after_ms = 1000
        hand._controller._servo_rebase_zero_device_id = state.product_fingerprint["device_id"]
        hand._controller._service_servo_rebase_zero_capture(state)
        return ServoRebaseZeroResult(done == mask, 'hardware final result', 18, done, mask & ~done)

    writer = Mock(side_effect=zero_readback)
    monkeypatch.setattr(hand._controller, 'zero_direct_motor_abs', writer)
    monkeypatch.setattr(hand._controller, '_persist_multiturn_state', Mock(return_value=fault != 'save_pending'))
    monkeypatch.setattr(hand._controller, 'servo_abs_zero_anchor_saved', Mock(return_value=fault != 'save_pending'))
    monkeypatch.setattr(hand._controller, 'request_state_sync', Mock(return_value=True))
    waits = 0

    def fresh_wait(predicate, epoch, timeout=3.0):
        nonlocal waits
        waits += 1
        if fault == 'stop_unconfirmed' and waits == 1:
            return False
        return epoch == hand._application_epoch and bool(predicate(hand.get_state_snapshot()))

    monkeypatch.setattr(hand, '_wait_application', fresh_wait)
    return writer


def test_direct_targets_continue_after_zero_and_a_light_reverse_load(hand, monkeypatch):
    direct_zero_device(hand, monkeypatch)
    assert wait_operation(hand, hand.request_direct_zero(tuple(range(27))))['status'] == 'SUCCESS'
    state = hand._controller.current_state
    assert state.servo_angles == [2048] * 27
    comm = direct_target_device(hand, monkeypatch)
    assert wait_operation(hand, hand.request_motors({11: 3000}))['status'] == 'SUCCESS'
    # Recorded M11 feedback at the failing second request: known ABS=2997,
    # online, light reverse load=-32; another motor's unknown bit is irrelevant.
    state.servo_angles[11] = 2997
    state.servo_load[11] = -32
    state.multi_turn_unknown_bitmap = 1 << 10
    assert wait_operation(hand, hand.request_motors({11: 2500}))['status'] == 'SUCCESS'
    assert comm.send_command.call_count == 2
    commands = [call.args[0] for call in comm.send_command.call_args_list]
    assert commands[0][1] != commands[1][1]
    assert [command[3][11] for command in commands] == [3000, 2500]
    assert all(command[2] == 1 << 11 for command in commands)


@pytest.mark.parametrize('recovers', [True, False])
def test_direct_zero_checks_routes_after_stopping(hand, monkeypatch, recovers):
    writer = direct_zero_device(hand, monkeypatch)
    controller = hand._controller
    state = controller.current_state
    state.servo_route_scan_state = 1
    state.servo_online_bitmap = state.servo_route_controllable_bitmap = 0
    stop = controller.stop.side_effect
    publish_result = writer.side_effect

    def stop_and_update_routes():
        stop()
        if recovers:
            state.servo_route_scan_state = 0
            state.servo_online_bitmap = state.servo_route_controllable_bitmap = 1 << 9

    def zero_if_ready(mask, **kwargs):
        if controller._coordinate_safe_state(hand.get_state_snapshot(), mask):
            return publish_result(mask, **kwargs)
        return ServoRebaseZeroResult(False, 'hardware not ready', 0, 0, mask)

    controller.stop.side_effect = stop_and_update_routes
    writer.side_effect = zero_if_ready
    result = wait_operation(hand, hand.request_direct_zero((9,)))
    assert result['status'] == ('SUCCESS' if recovers else 'FAILED')
    writer.assert_called_once()
    assert state.servo_angles[9] == (2048 if recovers else 7009)


def test_direct_zero_only_selected_motor_without_expanding_joint_scope(hand, monkeypatch):
    hand._selected_motors = (9,)
    writer = direct_zero_device(hand, monkeypatch)
    assert wait_operation(hand, hand.request_direct_zero())['status'] == 'SUCCESS'
    assert writer.call_args.args == (1 << 9,)
    assert hand._controller.current_state.servo_angles[9] == 2048
    assert hand._controller.current_state.servo_angles[8] == 7008
    assert list(hand._controller.get_servo_abs_zero_generations()) == [0] * 9 + [1] + [0] * 17
    assert hand.application_snapshot()['motors'][9]['abs_zero_generation'] == 1


def test_direct_zero_all_zeros_every_online_routable_motor(hand, monkeypatch):
    writer = direct_zero_device(hand, monkeypatch)
    state = hand._controller.current_state
    state.servo_online_bitmap &= ~((1 << 3) | (1 << 4))
    state.servo_route_controllable_bitmap &= ~(1 << 5)
    state.servo_route_conflict_bitmap = 1 << 6
    assert wait_operation(hand, hand.request_direct_zero_all())['status'] == 'SUCCESS'
    expected = ((1 << 27) - 1) & ~((1 << 3) | (1 << 4) | (1 << 5) | (1 << 6))
    assert writer.call_args.args == (expected,)
    # 归零时会真停一次：P4 锁存 E-STOP，UI 必须显示 Clear Faults。
    hand._controller.stop.assert_called_once()


def test_direct_zero_while_idle_does_not_latch_estop(hand, monkeypatch):
    """未标定进 Direct 时设备本来就静止，归零不得再发 STOP 锁存 E-STOP。"""
    writer = direct_zero_device(hand, monkeypatch)
    state = hand._controller.current_state
    state.system_state = SYSTEM_STATE_READY
    state.control_enabled = False
    state.servo_target_owner = 0

    def next_status_frame(predicate, epoch, timeout=3.0):
        state.system_status_last_rx_ts = time.time()
        stamp(state, 'system_status_last_rx_ts')
        return epoch == hand._application_epoch and bool(predicate(hand.get_state_snapshot()))

    monkeypatch.setattr(hand, '_wait_application', next_status_frame)
    assert wait_operation(hand, hand.request_direct_zero_all())['status'] == 'SUCCESS'
    hand._controller.stop.assert_not_called()
    assert writer.called and hand._estop_requested_at == 0


def test_direct_zero_all_rejects_when_no_motor_is_eligible(hand, monkeypatch):
    writer = direct_zero_device(hand, monkeypatch)
    hand._controller.current_state.servo_online_bitmap = 0
    result = hand.request_direct_zero_all()
    assert not result.accepted and 'No online, routable motors' in result.message
    writer.assert_not_called()


def test_direct_zero_clears_a_latched_stop_before_waiting_for_the_motor_check(hand, monkeypatch):
    writer = direct_zero_device(hand, monkeypatch)
    state = hand._controller.current_state
    state.system_state = SYSTEM_STATE_UNCONTROLLABLE
    state.control_enabled = False
    state.servo_target_owner = 0
    state.servo_online_check = [0] * 27
    hand._estop_requested_at = time.time()
    resets = []

    def reset(epoch):
        resets.append(epoch)
        hand._estop_requested_at = 0
        state.servo_online_check = [1] * 27
        return True, 'RESET acknowledged'

    monkeypatch.setattr(hand, '_reset_for_operation', reset)

    def next_status_frame(predicate, epoch, timeout=3.0):
        state.system_status_last_rx_ts = time.time()
        stamp(state, 'system_status_last_rx_ts')
        return epoch == hand._application_epoch and bool(predicate(hand.get_state_snapshot()))

    monkeypatch.setattr(hand, '_wait_application', next_status_frame)
    assert wait_operation(hand, hand.request_direct_zero_all())['status'] == 'SUCCESS'
    assert resets and writer.called and hand._estop_requested_at == 0


def test_direct_zero_reports_a_motor_check_that_never_finishes(hand, monkeypatch):
    writer = direct_zero_device(hand, monkeypatch)
    state = hand._controller.current_state
    state.system_state = SYSTEM_STATE_UNCONTROLLABLE
    state.control_enabled = False
    state.servo_target_owner = 0
    state.servo_online_check = [0] * 27
    hand._estop_requested_at = time.time()
    monkeypatch.setattr(hand, '_reset_for_operation', lambda epoch: (True, 'RESET acknowledged'))
    monkeypatch.setattr(hand, '_wait_application', lambda predicate, epoch, timeout=3.0: False)
    item = wait_operation(hand, hand.request_direct_zero_all())
    assert item['status'] == 'FAILED'
    assert item['user_issue']['code'] == 'HOST_MOTORS_NOT_READY'
    writer.assert_not_called()


def test_direct_zero_real_backend_sync_wrapper_waits_for_feedback_anchor_save(hand, monkeypatch):
    hand._selected_motors = (9,)
    direct_zero_device(hand, monkeypatch, fault='save_pending')
    controller = hand._controller
    monkeypatch.setattr(controller, 'request_state_sync',
                        HandController.request_state_sync.__get__(controller))
    monkeypatch.setattr(controller, 'servo_abs_zero_anchor_saved',
                        HandController.servo_abs_zero_anchor_saved.__get__(controller))
    lower_comm = Mock()
    monkeypatch.setattr(controller, 'comm', lower_comm)

    def sync_reply(command):
        assert command == ('state_sync', STATE_SYNC_ALL)
        assert not controller.servo_abs_zero_anchor_saved(1 << 9)
        assert controller._servo_rebase_zero_capture_mask == 1 << 9
        controller._persist_multiturn_state.return_value = True
        controller._multiturn_store_payload = {'valid_motor_mask': hex(1 << 9)}
        controller.current_state.servo_feedback_p4_time_ms = 1002
        controller._service_servo_rebase_zero_capture(controller.current_state)
        assert controller.servo_abs_zero_anchor_saved(1 << 9)
        return True

    lower_comm.send_command.side_effect = sync_reply
    assert hand.request_state_sync.__func__ is HandBackend.request_state_sync
    assert wait_operation(hand, hand.request_direct_zero())['status'] == 'SUCCESS'
    lower_comm.send_command.assert_called_once_with(('state_sync', STATE_SYNC_ALL))
    assert controller._persist_multiturn_state.call_count == 2
    assert controller.get_servo_abs_zero_generations()[9] == 1
    lower_comm.send_command.side_effect = None


def test_direct_zero_freezes_the_motor_selection_when_clicked(hand, monkeypatch):
    hand._selected_motors = (1, 9)
    writer = direct_zero_device(hand, monkeypatch, change_selection=True)
    assert wait_operation(hand, hand.request_direct_zero())['status'] == 'SUCCESS'
    assert writer.call_args.args == ((1 << 1) | (1 << 9),)
    generations = hand._controller.get_servo_abs_zero_generations()
    assert generations[0] == 0
    assert generations[1] == generations[9] == 1


@pytest.mark.parametrize('fault', ['stop_unconfirmed', 'cancel'])
def test_direct_zero_not_sent_without_stop_confirmation_or_after_cancel(hand, monkeypatch, fault):
    hand._selected_motors = (9,)
    writer = direct_zero_device(hand, monkeypatch, fault=fault)
    assert wait_operation(hand, hand.request_direct_zero())['status'] == 'FAILED'
    writer.assert_not_called()
    assert hand._controller.current_state.servo_angles[9] == 7009
    assert not any(hand._controller.get_servo_abs_zero_generations())


def test_direct_zero_failure_keeps_host_feedback_and_target_generation(hand, monkeypatch):
    hand._selected_motors = (9,)
    writer = direct_zero_device(hand, monkeypatch, fault='rejected')
    assert wait_operation(hand, hand.request_direct_zero())['status'] == 'FAILED'
    writer.assert_called_once()
    assert hand._controller.current_state.servo_angles[9] == 7009
    assert not any(hand._controller.get_servo_abs_zero_generations())


@pytest.mark.parametrize('fault', ['old_feedback', 'unknown', 'save_pending'])
def test_direct_zero_needs_trusted_new_matching_feedback_and_json_save(hand, monkeypatch, fault):
    hand._selected_motors = (9,)
    writer = direct_zero_device(hand, monkeypatch, fault=fault)
    item = wait_operation(hand, hand.request_direct_zero())
    assert item['status'] == 'FAILED'
    assert 'unconfirmed' in item['message']
    writer.assert_called_once()
    assert not any(hand._controller.get_servo_abs_zero_generations())


def test_direct_zero_partial_result_refreshes_only_successfully_saved_motor(hand, monkeypatch):
    hand._selected_motors = (1, 9)
    direct_zero_device(hand, monkeypatch, success=1 << 1)
    item = wait_operation(hand, hand.request_direct_zero())
    assert item['status'] == 'FAILED' and 'partly' in item['message']
    generations = hand._controller.get_servo_abs_zero_generations()
    assert generations[1] == 1
    assert generations[9] == 0
    assert hand._controller.current_state.servo_angles[9] == 7009


def test_direct_zero_accepts_cumulative_saved_success_when_last_result_only_contains_retried_subset(hand, monkeypatch):
    hand._selected_motors = (1, 9)
    direct_zero_device(hand, monkeypatch)
    original = hand._controller.zero_direct_motor_abs

    def completed(mask, *, cancelled, send_gate):
        result = original(mask, cancelled=cancelled, send_gate=send_gate)
        state = hand._controller.current_state
        # 最后一轮只重试 M09；M01 的归零锚点已在第一轮保存。
        state.servo_rebase_zero_success_mask = 1 << 9
        state.servo_rebase_zero_failed_mask = 0
        return result

    monkeypatch.setattr(hand._controller, 'zero_direct_motor_abs', completed)
    item = wait_operation(hand, hand.request_direct_zero())
    assert item['status'] == 'SUCCESS'
    assert hand._controller.get_servo_abs_zero_generations()[1] == 1
    assert hand._controller.get_servo_abs_zero_generations()[9] == 1


def test_direct_zero_late_offline_does_not_hide_another_saved_motor(hand, monkeypatch):
    hand._selected_motors = (1, 9)
    direct_zero_device(hand, monkeypatch, fault='one_offline')
    assert wait_operation(hand, hand.request_direct_zero())['status'] == 'FAILED'
    generations = hand._controller.get_servo_abs_zero_generations()
    assert generations[1] == 1
    assert generations[9] == 0


def test_snapshot_waits_for_the_saved_zero_feedback_frame_before_refreshing_draft(hand, monkeypatch):
    hand._selected_motors = (9,)
    direct_zero_device(hand, monkeypatch)
    assert wait_operation(hand, hand.request_direct_zero())['status'] == 'SUCCESS'
    state = hand._controller.current_state
    # 已完成保存的 new_model 尚未复制到 current_state 的短暂窗口。
    state.servo_feedback_p4_time_ms = 999
    state.servo_angles[9] = 7009
    assert hand.application_snapshot()['motors'][9]['abs_zero_generation'] == 0
    state.servo_feedback_p4_time_ms = 1001
    state.servo_angles[9] = 2048
    row = hand.application_snapshot()['motors'][9]
    assert row['abs_zero_generation'] == 1 and row['abs'] == 2048
    hand._controller._reset_local_runtime_state('SYNCING')
    assert hand.application_snapshot()['motors'][9]['abs_zero_generation'] == 1


def test_snapshot_refreshes_late_success_after_application_worker_reported_save_timeout(hand, monkeypatch):
    hand._selected_motors = (9,)
    direct_zero_device(hand, monkeypatch, fault='save_pending')
    assert wait_operation(hand, hand.request_direct_zero())['status'] == 'FAILED'
    assert hand.application_snapshot()['motors'][9]['abs_zero_generation'] == 0
    hand._controller._persist_multiturn_state.return_value = True
    hand._controller._service_servo_rebase_zero_capture(hand._controller.current_state)
    assert hand.application_snapshot()['motors'][9]['abs_zero_generation'] == 1
    hand._controller._service_servo_rebase_zero_capture(hand._controller.current_state)
    assert hand.application_snapshot()['motors'][9]['abs_zero_generation'] == 1


def test_snapshot_refreshes_saved_zero_even_after_stop_cancelled_application_epoch(hand, monkeypatch):
    hand._selected_motors = (9,)
    direct_zero_device(hand, monkeypatch, fault='save_pending')
    assert wait_operation(hand, hand.request_direct_zero())['status'] == 'FAILED'
    hand.stop_motion()
    hand._controller._persist_multiturn_state.return_value = True
    hand._controller._service_servo_rebase_zero_capture(hand._controller.current_state)
    assert hand.application_snapshot()['motors'][9]['abs_zero_generation'] == 1


def test_snapshot_never_publishes_saved_generation_to_a_different_device(hand, monkeypatch):
    hand._selected_motors = (9,)
    direct_zero_device(hand, monkeypatch)
    assert wait_operation(hand, hand.request_direct_zero())['status'] == 'SUCCESS'
    state = hand._controller.current_state
    state.product_fingerprint["device_id"] = "0000FEDCBA987654"
    state.servo_feedback_p4_time_ms = 5000
    state.servo_angles[9] = 11200
    row = hand.application_snapshot()['motors'][9]
    assert row['abs_zero_generation'] == 0 and row['abs'] == 11200


def test_snapshot_never_pairs_unpublished_old_generation_with_a_new_connection_clock(hand, monkeypatch):
    hand._selected_motors = (9,)
    direct_zero_device(hand, monkeypatch)
    assert wait_operation(hand, hand.request_direct_zero())['status'] == 'SUCCESS'
    from desktop.core.data_models import HandModel
    new_session = HandModel()
    new_session.copy_state_from(hand._controller.current_state)
    hand._controller._reset_local_runtime_state('SYNCING')
    new_session.servo_feedback_p4_time_ms = 5000
    new_session.servo_angles[9] = 11200
    hand._controller.current_state.copy_state_from(new_session)
    assert hand.application_snapshot()['motors'][9]['abs_zero_generation'] == 0
    hand._controller._servo_rebase_zero_device_id = new_session.product_fingerprint["device_id"]
    hand._controller._servo_rebase_zero_capture_mask = 1 << 9
    hand._controller._servo_rebase_zero_capture_after_ms = 4999
    hand._controller._service_servo_rebase_zero_capture(hand._controller.current_state)
    assert hand.application_snapshot()['motors'][9]['abs_zero_generation'] == 2


def test_snapshot_rechecks_connection_if_reconnect_occurs_between_metadata_and_state_reads(hand, monkeypatch):
    hand._selected_motors = (9,)
    direct_zero_device(hand, monkeypatch)
    assert wait_operation(hand, hand.request_direct_zero())['status'] == 'SUCCESS'
    original_snapshot = hand.get_state_snapshot
    reconnected = False

    def reconnect_during_state_read():
        nonlocal reconnected
        if not reconnected:
            reconnected = True
            new_session = original_snapshot()
            hand._controller._reset_local_runtime_state('SYNCING')
            new_session.servo_feedback_p4_time_ms = 5000
            new_session.servo_angles[9] = 11200
            hand._controller.current_state.copy_state_from(new_session)
        return original_snapshot()

    monkeypatch.setattr(hand, 'get_state_snapshot', reconnect_during_state_read)
    row = hand.application_snapshot()['motors'][9]
    assert reconnected and row['abs'] == 11200
    assert row['abs_zero_generation'] == 0


@pytest.mark.parametrize('motors', [(), (-1,), (27,), ('9',), (True,)])
def test_direct_zero_invalid_scope_never_stops_or_writes(hand, monkeypatch, motors):
    writer = direct_zero_device(hand, monkeypatch)
    assert not hand.request_direct_zero(motors).accepted
    hand._controller.stop.assert_not_called()
    writer.assert_not_called()


@pytest.mark.parametrize('minimum,maximum,span', [(16000, 100, 484)])
def test_manual_pair_preserves_positive_travel_across_wrap(minimum, maximum, span):
    assert validate_manual_pair(minimum, maximum) == span


@pytest.mark.parametrize('minimum,maximum', [(10, 10), (0, 8192), (-1, 1), (10, 0)])
def test_invalid_manual_pair_does_not_swap_endpoints(minimum, maximum):
    with pytest.raises(ValueError):
        validate_manual_pair(minimum, maximum)


def test_two_fresh_captures_save_once_then_single_endpoint_resaves(hand, monkeypatch):
    writer = Mock(return_value=True)
    monkeypatch.setattr(hand, 'set_manual_encoder_reference', writer)
    state = hand._controller.current_state
    state.encoders[0].raw_sensor = 11000
    assert wait_operation(hand, hand.request_manual_capture(0, 'min'))['status'] == 'SUCCESS'
    writer.assert_not_called()
    state.encoders[0].raw_sensor = 12000
    saved = wait_operation(hand, hand.request_manual_capture(0, 'max'))
    assert saved['status'] == 'SUCCESS'
    assert saved['message'] == ''
    assert hand.application_snapshot()['joints'][0]['capture_status'] == ''
    assert writer.call_count == 1 and writer.call_args.args == (0, 11000, 12000)
    assert writer.call_args.kwargs['cancelled']() is False
    state.encoders[0].raw_sensor = 10500
    wait_operation(hand, hand.request_manual_capture(0, 'min'))
    assert hand._manual_drafts[0]['max'] == 12000
    assert writer.call_count == 2
    assert writer.call_args.args == (0, 10500, 12000)


def test_manual_installed_but_not_saved_keeps_draft_until_next_set(hand, monkeypatch):
    hand._manual_drafts[0] = dict(min=11000, max=12000, status='DRAFT')
    hand._controller.current_state.encoders[0].raw_sensor = 11000
    hand._controller.last_manual_write_phase = 'INSTALLED_NOT_SAVED'
    writer = Mock(side_effect=[False, True])
    monkeypatch.setattr(hand, 'set_manual_encoder_reference', writer)
    failed = wait_operation(hand, hand.request_manual_capture(0, 'min'))
    assert failed['status'] == 'FAILED'
    assert 'INSTALLED_NOT_SAVED' in failed['message']
    assert hand._manual_drafts[0]['status'] == 'INSTALLED_NOT_SAVED'
    assert wait_operation(hand, hand.request_manual_capture(0, 'min'))['status'] == 'SUCCESS'
    assert hand._manual_drafts[0]['status'] == 'SAVED'


def test_stale_raw_cannot_be_captured(hand, monkeypatch):
    hand._controller.current_state.encoder_raw_last_rx_ts = time.time() - 1
    stamp(hand._controller.current_state, 'encoder_raw_last_rx_ts')
    writer = Mock()
    monkeypatch.setattr(hand, 'set_manual_encoder_reference', writer)
    assert wait_operation(hand, hand.request_manual_capture(0, 'min'))['status'] == 'FAILED'
    assert not hand._manual_drafts
    writer.assert_not_called()


def test_invalid_mapped_angle_does_not_block_healthy_raw_manual_capture(hand):
    encoder = hand._controller.current_state.encoders[0]
    encoder.angle_error = encoder.error = True
    encoder.raw_error = False
    assert wait_operation(hand, hand.request_manual_capture(0, 'min'))['status'] == 'SUCCESS'
    assert hand._manual_drafts[0]['min'] == 12000


def test_healthy_status_does_not_make_stale_mapped_angle_current(hand):
    state = hand._controller.current_state
    state.has_sensor_data = True
    state.encoders[0].angle_error = False
    state.encoder_mapped_last_rx_ts = time.time() - 1
    stamp(state, 'encoder_mapped_last_rx_ts')
    assert hand.application_snapshot()['joints'][0]['angle'] is None


def test_ros_invalid_frame_is_rejected_before_any_output(hand, monkeypatch):
    sender = Mock()
    monkeypatch.setattr(hand, 'send_joint_targets', sender)
    frame = [0.0] * 21
    frame[0] = 1000
    assert not hand.send_ros2_joint_targets(frame, source='test').sent
    sender.assert_not_called()


def test_cancelled_replay_preflight_cannot_claim_motion_after_stop(hand, monkeypatch):
    epoch = hand._application_epoch
    monkeypatch.setattr(hand, 'is_started', lambda: True)
    hand.stop_motion()
    ok, token, message = hand.begin_replay_motion(expected_application_epoch=epoch)
    assert not ok and token == 0 and 'cancelled' in message
    assert not hand._replay_motion_active


def test_stop_bypasses_blocked_operation_and_busy_remains_until_cleanup(hand):
    entered, release = threading.Event(), threading.Event()
    def action(epoch):
        entered.set()
        release.wait(2)
        return True, 'finished'
    result = hand.submit_operation('blocked', action)
    assert entered.wait(1)
    started = time.monotonic()
    hand.stop_motion(emergency=True)
    assert time.monotonic() - started < 0.1
    hand.stop.assert_called_once()
    assert not hand.submit_operation('second', action).accepted
    release.set()
    assert wait_operation(hand, result)['status'] == 'FAILED'
    assert hand._application_busy is None


def test_disconnect_is_queued_after_cancelled_worker(hand, monkeypatch):
    entered, release = threading.Event(), threading.Event()
    def blocked(epoch):
        entered.set(); release.wait(2)
        return True, 'finished'
    hand.submit_operation('blocked', blocked)
    assert entered.wait(1)
    disconnect = Mock()
    monkeypatch.setattr(hand, 'disconnect', disconnect)
    result = hand.request_disconnect()
    assert result.accepted
    disconnect.assert_not_called()
    release.set()
    assert wait_operation(hand, result)['status'] == 'SUCCESS'
    disconnect.assert_called_once_with(stop_lower=True)


def test_stale_status_is_unknown_and_zero_warning_is_not_an_alarm(hand):
    hand._controller.current_state.system_status_last_rx_ts = time.time() - 1
    stamp(hand._controller.current_state, 'system_status_last_rx_ts')
    snapshot = hand.application_snapshot()
    assert snapshot['communication'] == 'Stale data'
    assert snapshot['device'] == 'Unknown'
    assert snapshot['calibration']['warning'] == {'ready': False}


def test_port_discovery_does_not_connect_or_create_second_thread(hand, monkeypatch):
    hand._save_data = False
    connect = Mock()
    monkeypatch.setattr(hand, 'connect', connect)
    hand.open_application(discover=lambda: [('COM7', 'P4')])
    worker = hand._discovery_thread
    hand.open_application(discover=lambda: [('COM8', 'other')])
    worker.join(0.02)
    assert hand._discovery_thread is worker
    assert hand.application_snapshot()['devices'] == [{'port': 'COM7', 'description': 'P4'}]
    connect.assert_not_called()


def test_port_discovery_failure_clears_stale_choices(hand):
    hand._save_data = False
    calls = []

    def discover():
        calls.append(True)
        if len(calls) == 1:
            return [('COM7', 'P4')]
        raise OSError('USB enumeration failed')

    hand.open_application(discover=discover)
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        snapshot = hand.application_snapshot()
        if snapshot['discovery_error']:
            break
        time.sleep(0.01)
    assert snapshot['devices'] == []
    assert 'USB enumeration failed' in snapshot['discovery_error']



@pytest.mark.parametrize('outcome', ['success', 'failed', 'stale', 'wrong_scope', 'ack', 'stop', 'reconnect'])
def test_enable_recovery_uses_p4_result(hand, monkeypatch, outcome):
    state = hand._controller.current_state
    state.pull_joint_into_workspace_sequence = 7
    comm = Mock()
    hand._controller.comm = comm
    comm.wait_for_ack.return_value = (0, 0, 9 if outcome == 'ack' else 2)
    epoch = hand._application_epoch

    def rope(mask):
        state.pull_joint_into_workspace_sequence = 7 if outcome == 'stale' else 8
        state.pull_joint_into_workspace_joint_mask = 2 if outcome == 'wrong_scope' else mask
        state.pull_joint_into_workspace_status = 3 if outcome == 'failed' else 2
        state.pull_joint_into_workspace_reason = 4
        state.monotonic_received['system_status_last_rx_ts'] = time.monotonic()
        if outcome == 'stop':
            hand._application_epoch += 1
        if outcome == 'reconnect':
            hand._controller.comm = Mock()
        return True

    pull = Mock(side_effect=rope)
    monkeypatch.setattr(hand, 'pull_joint_into_workspace', pull)
    monkeypatch.setattr(hand, '_wait_application', lambda predicate, *_: predicate(state))
    ok, message = hand._recover_enable_workspace(1, epoch, comm)

    pull.assert_called_once_with(1)
    assert ok == (outcome == 'success')
    if outcome == 'failed':
        assert 'reason=4' in message
    if outcome in ('stale', 'wrong_scope'):
        assert 'timed out' in message
    if outcome == 'ack':
        assert 'ACK=9' in message


@pytest.mark.parametrize('recover', [False, True])
def test_enable_uses_p4_readiness_and_holds_actual_angle(hand, monkeypatch, recover):
    from desktop.core.hand_backend import JointCommandResult
    from desktop.core.protocol import (
        ControlMode, SYSTEM_STATE_READY, SYSTEM_STATE_JOINT_BEYOND_WORKSPACE,
        SYSTEM_STATE_RUN_FEEDBACK, SERVO_TARGET_OWNER_CONTROL,
    )
    state = hand._controller.current_state
    state.has_active_scope_status = state.has_system_status_data = state.protocol_compatible = True
    state.require_can_feedback = True
    state.active_joint_mask, state.active_motor_mask = 1, 3
    state.control_mode = ControlMode.JOINT_ANGLE.value
    state.encoders[0].raw = -8192
    state.encoders[0].angle_error = False
    monkeypatch.setattr(hand, 'is_comm_connected', lambda: True)
    monkeypatch.setattr(hand, '_mapped_feedback_fresh', lambda _: True)
    monkeypatch.setattr(hand, '_prepare_control_switch', lambda _: (True, ''))
    monkeypatch.setattr(hand, 'set_active_scope', lambda *_: True)

    def start():
        state.system_state = SYSTEM_STATE_JOINT_BEYOND_WORKSPACE if recover else SYSTEM_STATE_READY
        state.monotonic_received['system_status_last_rx_ts'] = time.monotonic()

    def recovery(*_):
        state.system_state = SYSTEM_STATE_READY
        state.monotonic_received['system_status_last_rx_ts'] = time.monotonic()
        return True, ''

    def hold(values, **kwargs):
        assert values[0] == -180.0 and kwargs['command_joint_mask'] == 1
        state.system_state = SYSTEM_STATE_RUN_FEEDBACK
        state.control_enabled = True
        state.servo_target_owner = SERVO_TARGET_OWNER_CONTROL
        return JointCommandResult(sent=True)

    monkeypatch.setattr(hand, 'start', start)
    pull = Mock(side_effect=recovery)
    monkeypatch.setattr(hand, '_recover_enable_workspace', pull)
    monkeypatch.setattr(hand, 'send_joint_targets', hold)
    monkeypatch.setattr(hand, '_wait_application', lambda predicate, *_: predicate(state))
    ok, message = hand._enable_selected_joints([0], hand._application_epoch)
    assert ok, message
    assert pull.call_count == int(recover)


def test_calibration_rejects_unapplied_parameters_before_reset_or_motion(hand, monkeypatch):
    from desktop.core.user_issues import operation_issue
    hand._controller._applied_output_margins = None
    hand._selected_joints = (0,)
    prepared = Mock()
    monkeypatch.setattr(hand, '_prepare_maintenance_switch', prepared)
    result = hand.request_calibration()
    assert not result.accepted
    assert operation_issue(result.message).code == 'HOST_PARAMETERS_PENDING'
    prepared.assert_not_called()
    assert not hand._application_operations


@pytest.mark.parametrize('action', ['save', 'defaults'])
def test_parameter_changes_cannot_change_calibration_margins_mid_operation(hand, monkeypatch, action):
    hand._calibration_operation_id = 'preparing-calibration'
    saved = Mock()
    monkeypatch.setattr(hand._control_parameters, 'save', saved)
    monkeypatch.setattr(hand._control_parameters, 'restore_defaults', saved)
    before = hand._controller._applied_output_margins
    result = hand.request_save_control_parameters({}) if action == 'save' else hand.request_restore_control_defaults()
    assert not result.accepted
    saved.assert_not_called()
    assert hand._controller._applied_output_margins == before


def test_save_parameters_requires_fresh_application_before_new_calibration(hand, tmp_path):
    from desktop.core.control_parameters import ControlParameterStore
    hand._control_parameters = ControlParameterStore(tmp_path / 'parameters.json')
    data = hand._control_parameters.snapshot()
    data.pop('crc32'); data.pop('ramp_crc32')
    assert hand.request_save_control_parameters(data).accepted
    assert hand._controller._applied_output_margins is None


def test_synchronous_rejection_retains_evidence_without_changing_motion_ownership(hand):
    from desktop.core.user_issues import IssueMessage
    hand._application_busy = 'existing-action'
    before = hand.get_state_snapshot()
    issue = hand.record_operation_issue(IssueMessage('F1 status=9 rejected', 'ACT-1005'), scope='M02')
    assert issue['code'] == 'ACT-1005'
    assert issue['details']['original_message'] == 'F1 status=9 rejected'
    item = hand.application_snapshot()['operations'][-1]
    assert item['status'] == 'FAILED' and item['notification_shown']
    assert item['scope'] == 'M02'
    assert hand._application_busy == 'existing-action'
    assert hand.get_state_snapshot().servo_target_owner == before.servo_target_owner


@pytest.mark.parametrize('unknown,recovery_ok', [(True, True), (True, False), (False, True)])
def test_enable_recovers_unknown_coordinates_before_setting_scope(hand, monkeypatch, unknown, recovery_ok):
    state = hand._controller.current_state
    state.multi_turn_unknown_bitmap = 3 if unknown else 0
    monkeypatch.setattr(hand, '_prepare_control_switch', lambda _epoch: (True, ''))
    calls = []
    def recover(mask, **kwargs):
        calls.append(('recover', mask))
        return recovery_ok, 'test recovery'
    def scope(*args):
        calls.append(('scope', args[0]))
        return False  # Stop here: this test must not start motion.
    monkeypatch.setattr(hand._controller, 'wait_for_multiturn_recovery', recover)
    monkeypatch.setattr(hand, 'set_active_scope', scope)
    ok, message = hand._enable_selected_joints([0], hand._application_epoch)
    assert not ok
    assert calls == ([('recover', 3)] if unknown and not recovery_ok else
                     [('recover', 3), ('scope', 3)] if unknown else [('scope', 3)])
    if unknown and not recovery_ok:
        assert 'coordinate recovery failed' in message


def _install_contact_history(hand):
    from desktop.core.communication.feedback_history import TimestampedFeedbackHistory
    history = TimestampedFeedbackHistory()
    comm = Mock()
    comm.feedback_history = history
    hand._controller.comm = comm
    state = hand.current_state
    mask = (1 << 21) - 1
    state.manual_reference_valid_bitmap = mask
    state.manual_reference_min_raw = [1000] * 21
    state.manual_reference_max_raw = [5000] * 21
    state.has_encoder_raw_data = True
    state.encoder_raw_last_rx_ts = time.time()
    return history


def test_contact_capture_reads_only_fresh_valid_frames(hand):
    history = _install_contact_history(hand)
    mask = (1 << 21) - 1
    history.record('encoder_raw', 100, tuple([1200] * 21), mask,
                   time.time() - 1.0)
    started = hand.begin_contact_capture()
    assert started['ok'], started
    assert hand.application_snapshot(include_curves=False)['busy']
    history.record('encoder_raw', 110, tuple([1250] * 21), mask, time.time())
    history.record('encoder_raw', 120, tuple([1300] * 21), mask, time.time())
    history.record('encoder_raw', 130, tuple([1350] * 21), 0, time.time())
    done = hand.end_contact_capture()
    assert done['ok'], done
    assert [frame[0] for frame in done['frames']] == [110, 120]
    assert done['manual_min'] == [1000] * 21
    assert done['manual_max'] == [5000] * 21
    assert not hand.application_snapshot(include_curves=False)['busy']
    assert not hand.end_contact_capture()['ok']


def test_contact_capture_rejects_duplicates_resets_and_missing_p4(hand):
    history = _install_contact_history(hand)
    assert hand.begin_contact_capture()['ok']
    assert not hand.begin_contact_capture()['ok']
    history.reset()
    invalidated = hand.end_contact_capture()
    assert not invalidated['ok'] and 'invalidated' in invalidated['message']
    hand._controller.comm = None
    missing = hand.begin_contact_capture()
    assert not missing['ok'] and 'not connected' in missing['message']


def test_contact_capture_requires_fresh_manual_endpoints(hand):
    _install_contact_history(hand)
    hand.current_state.manual_reference_valid_bitmap = 1
    assert not hand.begin_contact_capture()['ok']
    hand.current_state.manual_reference_valid_bitmap = (1 << 21) - 1
    hand.current_state.encoder_raw_last_rx_ts = time.time() - 5
    assert not hand.begin_contact_capture()['ok']
