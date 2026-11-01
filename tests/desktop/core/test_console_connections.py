"""Connection clocks belong to each physical link, independently of relay data."""

from _clock_helpers import stamp
import time
from unittest.mock import Mock
import pytest

from desktop.core.data_models import HandModel

def test_p4_connection_age_uses_recent_packets_not_slow_system_status(hand):
    state = hand._controller.current_state
    now = time.time()
    state.system_status_last_rx_ts = now - 0.5
    stamp(state, 'system_status_last_rx_ts')
    state.host_rx_time = now - 0.015
    snapshot = hand.application_snapshot()
    assert snapshot['connections']['p4']['age_ms'] < 100
    assert snapshot['diagnostics']['status_age_ms'] >= 500

def test_unconnected_s3_does_not_display_p4_relay_clock(hand):
    state = hand._controller.current_state
    state.tactile_last_rx_ts = time.time()
    stamp(state, 'tactile_last_rx_ts')
    state.tactile_online = True
    assert hand.application_snapshot()['tactile']['age_ms'] is not None
    assert hand.application_snapshot()['connections']['s3']['age_ms'] is None

def test_s3_merge_never_overwrites_p4_connection_clock(hand, monkeypatch):
    now = time.time()
    hand._controller.current_state.host_rx_time = now - 0.4
    tactile = HandModel()
    tactile.tactile_online = True
    tactile.tactile_last_rx_ts = now - 0.05
    stamp(tactile, 'tactile_last_rx_ts')
    tactile.host_rx_time = now - 0.005
    comm = Mock()
    comm.is_connected.return_value = True
    comm.get_latest_model.return_value = tactile
    monkeypatch.setattr(hand, 'tactile_comm', comm)
    snapshot = hand.application_snapshot()
    assert snapshot['connections']['p4']['age_ms'] >= 400
    assert 50 <= snapshot['connections']['s3']['age_ms'] < 200

@pytest.mark.parametrize('name', ['p4', 's3'])
def test_disconnected_link_does_not_display_a_cached_clock(hand, monkeypatch, name):
    hand._controller.current_state.host_rx_time = time.time()
    hand._tactile_state = HandModel()
    hand._tactile_state.tactile_last_rx_ts = time.time()
    stamp(hand._tactile_state, 'tactile_last_rx_ts')
    monkeypatch.setattr(hand, 'is_comm_connected' if name == 'p4' else 'is_tactile_serial_connected', lambda: False)
    assert hand.application_snapshot()['connections'][name]['age_ms'] is None
