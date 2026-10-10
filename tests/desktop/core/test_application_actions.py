"""用户操作边界回归；不连接真实硬件。"""
import threading
import time
from unittest.mock import Mock
import pytest
from desktop.core.application_actions import validate_manual_pair
from desktop.core.hand_backend import HandBackend
from desktop.core.protocol import SYSTEM_STATE_READY


@pytest.fixture
def hand(monkeypatch):
    hand = HandBackend()
    monkeypatch.setattr(hand, 'is_comm_connected', lambda: True)
    monkeypatch.setattr(hand, 'stop', Mock())
    state = hand._controller.current_state
    state.has_system_status_data = True
    state.system_status_last_rx_ts = time.time()
    state.system_state = SYSTEM_STATE_READY
    state.has_encoder_raw_data = True
    state.encoder_raw_last_rx_ts = time.time()
    state.p4_time_ms = state.encoder_raw_p4_time_ms = 1000
    state.encoder_online_bitmap = (1 << 21) - 1
    state.encoders[0].raw_error = state.encoders[0].error = False
    state.encoders[0].raw_sensor = 12000
    yield hand
    hand.close_application()


def wait_operation(hand, result):
    assert result.accepted, result.message
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        item = next(row for row in hand._application_operations if row['operation_id'] == result.operation_id)
        if item['status'] in ('SUCCESS', 'FAILED'):
            return item
        time.sleep(0.005)
    pytest.fail('operation did not finish')


@pytest.mark.parametrize('minimum,maximum,direction,span', [(16000, 100, 1, 484), (100, 16000, -1, 484)])
def test_manual_pair_preserves_direction_across_wrap(minimum, maximum, direction, span):
    assert validate_manual_pair(minimum, maximum, direction) == span


@pytest.mark.parametrize('minimum,maximum,direction', [(10, 10, 1), (100, 10, 1), (0, 8192, 1), (-1, 1, 1), (0, 10, 0)])
def test_invalid_manual_pair_does_not_swap_endpoints(minimum, maximum, direction):
    with pytest.raises(ValueError):
        validate_manual_pair(minimum, maximum, direction)


def test_two_fresh_captures_save_once_then_single_endpoint_resaves(hand, monkeypatch):
    writer = Mock(return_value=True)
    monkeypatch.setattr(hand, 'set_manual_encoder_reference', writer)
    assert wait_operation(hand, hand.request_manual_capture(0, 'min'))['status'] == 'SUCCESS'
    writer.assert_not_called()
    state = hand._controller.current_state
    state.encoders[0].raw_sensor = 11000  # J00 安装方向 -1
    assert wait_operation(hand, hand.request_manual_capture(0, 'max'))['status'] == 'SUCCESS'
    writer.assert_called_once_with(0, 12000, 11000)
    state.encoders[0].raw_sensor = 12500
    wait_operation(hand, hand.request_manual_capture(0, 'min'))
    assert hand._manual_drafts[0]['max'] == 11000
    assert writer.call_count == 2
    writer.assert_called_with(0, 12500, 11000)


def test_manual_installed_but_not_saved_keeps_draft_until_next_set(hand, monkeypatch):
    hand._manual_drafts[0] = dict(min=12000, max=11000, direction=-1, status='DRAFT')
    hand._controller.last_manual_write_phase = 'INSTALLED_NOT_SAVED'
    writer = Mock(side_effect=[False, True])
    monkeypatch.setattr(hand, 'set_manual_encoder_reference', writer)
    assert wait_operation(hand, hand.request_manual_capture(0, 'min'))['status'] == 'FAILED'
    assert hand._manual_drafts[0]['status'] == 'INSTALLED_NOT_SAVED'
    assert wait_operation(hand, hand.request_manual_capture(0, 'min'))['status'] == 'SUCCESS'
    assert hand._manual_drafts[0]['status'] == 'SAVED'


def test_stale_raw_cannot_be_captured(hand, monkeypatch):
    hand._controller.current_state.encoder_raw_last_rx_ts = time.time() - 1
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
