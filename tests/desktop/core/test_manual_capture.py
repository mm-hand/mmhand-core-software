"""Manual capture uses current asynchronous GUI operations and retains confirmed endpoints."""
import time
import threading
from unittest.mock import Mock
import pytest
from desktop.core.hand_backend import HandBackend
from desktop.core.protocol import SYSTEM_STATE_READY

@pytest.fixture
def capture(monkeypatch):
    hand = HandBackend()
    monkeypatch.setattr(hand, 'is_comm_connected', lambda: True)
    state = hand._controller.current_state
    state.has_system_status_data = state.has_encoder_raw_data = True
    state.system_status_last_rx_ts = state.encoder_raw_last_rx_ts = time.time()
    state.system_state = SYSTEM_STATE_READY
    state.encoder_online_bitmap = 1
    state.has_manual_reference_encoder_directions = True
    state.manual_reference_encoder_directions = [1] * 21
    state.encoders[0].raw_error = False
    writer = Mock(return_value=True)
    monkeypatch.setattr(hand, 'set_manual_encoder_reference', writer)
    yield hand, state, writer
    hand.close_application()

def run(hand, endpoint):
    result = hand.request_manual_capture(0, endpoint)
    assert result.accepted, result.message
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        row = next(r for r in hand._application_operations if r['operation_id'] == result.operation_id)
        if row['status'] in ('SUCCESS', 'FAILED'):
            return row['status'] == 'SUCCESS'
        time.sleep(.005)
    pytest.fail('capture operation did not finish')

def test_wrap_pair_and_subsequent_single_endpoint_save(capture):
    hand, state, writer = capture
    state.encoders[0].raw_sensor = 16000
    assert run(hand, 'min')
    writer.assert_not_called()
    state.encoders[0].raw_sensor = 200
    assert run(hand, 'max')
    assert writer.call_count == 1 and writer.call_args.args == (0, 16000, 200)
    assert writer.call_args.kwargs['cancelled']() is False
    state.encoders[0].raw_sensor = 400
    assert run(hand, 'max')
    assert writer.call_args.args == (0, 16000, 400)
    assert writer.call_count == 2

@pytest.mark.parametrize('second', [100, 99, 8292])
def test_equal_reverse_or_half_turn_pair_is_not_installed(capture, second):
    hand, state, writer = capture
    state.encoders[0].raw_sensor = 100
    assert run(hand, 'min')
    state.encoders[0].raw_sensor = second
    assert not run(hand, 'max')
    writer.assert_not_called()

@pytest.mark.parametrize('fault', ['stale', 'raw_error', 'missing_direction', 'bad_direction'])
def test_invalid_feedback_or_direction_never_enters_draft(capture, fault):
    hand, state, writer = capture
    if fault == 'stale': state.encoder_raw_last_rx_ts -= 1
    elif fault == 'raw_error': state.encoders[0].raw_error = True
    elif fault == 'missing_direction': state.has_manual_reference_encoder_directions = False
    else: state.manual_reference_encoder_directions[0] = 0
    assert not run(hand, 'min')
    assert not hand._manual_drafts
    writer.assert_not_called()

def test_install_failure_retains_confirmed_pair_for_explicit_set(capture):
    hand, state, writer = capture
    writer.return_value = False
    state.encoders[0].raw_sensor = 100
    assert run(hand, 'min')
    state.encoders[0].raw_sensor = 200
    assert not run(hand, 'max')
    assert hand._manual_drafts[0]['min'] == 100
    assert hand._manual_drafts[0]['max'] == 200
    writer.return_value = True
    assert run(hand, 'max')
    assert writer.call_count == 2


def test_single_click_waits_for_a_fresh_raw_frame(capture):
    hand,state,writer=capture
    state.encoder_raw_last_rx_ts=time.time()-1
    def feedback():
        time.sleep(.03)
        state.encoders[0].raw_sensor=1234
        state.encoder_raw_last_rx_ts=time.time()
    thread=threading.Thread(target=feedback); thread.start()
    assert run(hand,'min')
    thread.join()
    assert hand._manual_drafts[0]['min']==1234
    writer.assert_not_called()


def test_stop_during_save_is_visible_to_write_callback(capture):
    hand,state,writer=capture
    hand._manual_drafts[0]=dict(min=100,max=200,direction=1,status='DRAFT')
    state.encoders[0].raw_sensor=200
    def saving(*_args,**kwargs):
        assert not kwargs['cancelled']()
        hand.stop_motion()
        assert kwargs['cancelled']()
        return False
    writer.side_effect=saving
    assert not run(hand,'max')
    assert hand._manual_drafts[0]['status']=='CANCELLED'
