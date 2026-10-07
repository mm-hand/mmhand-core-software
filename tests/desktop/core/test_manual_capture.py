"""Manual endpoint capture: fresh feedback, draft isolation and install outcome."""

import threading
from types import SimpleNamespace

import pytest

from desktop.core.hand_backend import HandBackend
from desktop.core.data_models import HandModel


@pytest.fixture
def capture(monkeypatch):
    state = HandModel()
    state.has_encoder_raw_data = True
    state.encoder_raw_last_rx_ts = 100.0
    state.has_manual_reference_encoder_directions = True
    state.manual_reference_encoder_directions = [1] * 21
    state.encoders[0].raw_error = False
    calls = []
    backend = HandBackend.__new__(HandBackend)
    backend._manual_capture_lock = threading.RLock()
    backend._manual_drafts = {}
    backend.is_comm_connected = lambda: True
    backend.get_state_snapshot = lambda: state
    backend._controller = SimpleNamespace(set_manual_encoder_reference=lambda *args: calls.append(args) or True)
    monkeypatch.setattr('desktop.core.hand_backend.time.time', lambda: 100.1)
    return backend, state, calls


def test_cross_zero_pair_saves_only_after_both_new_endpoints(capture):
    backend, state, calls = capture
    state.encoders[0].raw_sensor = 16000
    assert not backend.capture_manual_endpoint(0, True).saved
    assert calls == []
    state.encoders[0].raw_sensor = 200
    assert backend.capture_manual_endpoint(0, False).saved
    assert calls == [(0, 16000, 200)]
    state.encoders[0].raw_sensor = 400
    assert not backend.capture_manual_endpoint(0, False).saved
    assert len(calls) == 1  # An old endpoint cannot complete a new pair.


@pytest.mark.parametrize('second', [100, 99, 8292])
def test_equal_reverse_or_half_turn_pair_is_not_installed(capture, second):
    backend, state, calls = capture
    state.encoders[0].raw_sensor = 100
    backend.capture_manual_endpoint(0, True)
    state.encoders[0].raw_sensor = second
    assert not backend.capture_manual_endpoint(0, False).saved
    assert calls == []


def test_stale_or_error_feedback_never_enters_draft(capture):
    backend, state, calls = capture
    state.encoder_raw_last_rx_ts = 99
    assert not backend.capture_manual_endpoint(0, True).accepted
    state.encoder_raw_last_rx_ts = 100
    state.encoders[0].raw_error = True
    assert not backend.capture_manual_endpoint(0, True).accepted
    assert backend._manual_drafts == {}
    assert calls == []


def test_install_failure_preserves_draft_for_retry(capture):
    backend, state, _ = capture
    backend._controller.set_manual_encoder_reference = lambda *args: False
    state.encoders[0].raw_sensor = 100
    backend.capture_manual_endpoint(0, True)
    state.encoders[0].raw_sensor = 200
    result = backend.capture_manual_endpoint(0, False)
    assert result.accepted and not result.saved
    assert backend._manual_drafts[0] == {'min': 100, 'max': 200}
    backend._controller.set_manual_encoder_reference = lambda *args: True
    assert backend.capture_manual_endpoint(0, False).saved
