"""Clock corrections must not affect control freshness or transaction ordering."""
import struct
import time

import pytest

from desktop.core.comm_layer import LowerComputerComm
from desktop.core.communication.ack_tracker import AckTracker
from desktop.core.host_calibration.runtime import HostCalibrationIO
from desktop.core.protocol import PACKET_TYPE_SERVO_ANGLE
from test_host_calibration_runtime import io


@pytest.mark.parametrize('jump', [2.816, 60.0, -60.0])
def test_calibration_feedback_and_owner_ignore_wall_clock_jumps(io, monkeypatch, jump):
    state = io.controller.current_state
    before = time.time()
    monkeypatch.setattr(time, 'time', lambda: before + jump)
    assert io.owns_control()
    assert io.servo_abs(2) == 0
    assert io.encoder_raw(1) == 10000
    # A genuine monotonic outage still fails and identifies the cause.
    state.monotonic_received['system_status_last_rx_ts'] -= 2
    assert not io.owns_control()
    assert 'system status stale' in io.control_loss_reason
    state.monotonic_received['encoder_raw_last_rx_ts'] -= 1
    assert io.encoder_raw(1) is None


@pytest.mark.parametrize('jump', [2.816, 60.0, -60.0])
def test_rx_keeps_wall_dates_and_monotonic_freshness_separate(monkeypatch, jump):
    comm = LowerComputerComm('')
    wall = time.time()
    payload = struct.pack('>27iI', *([12] * 27), 100)
    first = comm._process_packets([(PACKET_TYPE_SERVO_ANGLE, payload)])
    monkeypatch.setattr(time, 'time', lambda: wall + jump)
    second = comm._process_packets([(PACKET_TYPE_SERVO_ANGLE, payload)])
    assert second.servo_abs_last_rx_ts == wall + jump
    assert abs(HostCalibrationIO._age(second, 100, second.received_at('servo_abs_last_rx_ts'))) < 100
    assert second.received_at('servo_abs_last_rx_ts') >= first.received_at('servo_abs_last_rx_ts')
    second.monotonic_received['servo_abs_last_rx_ts'] = 0
    assert first.received_at('servo_abs_last_rx_ts') > 0  # Independent snapshots.


@pytest.mark.parametrize('jump', [2.816, 60.0, -60.0])
def test_ack_filter_uses_monotonic_time_and_retains_four_wall_fields(monkeypatch, jump):
    tracker = AckTracker()
    clock = [100.0]
    monkeypatch.setattr(time, 'monotonic', lambda: clock[0])
    tracker.publish(1, 0, 0, 1000.0)
    clock[0] = 101.0
    assert tracker.wait_for(1, 0, received_after_monotonic=101.0) is None
    tracker.publish(1, 0, 0, 1000.0 + jump)
    assert tracker.wait_for(1, 0, received_after_monotonic=101.0) == (1, 0, 0, 1000.0 + jump)
    tracker.prepare_command(1)
    assert tracker.wait_for(1, 0, received_after_monotonic=101.0) is None


def test_ack_timeout_does_not_read_wall_clock(monkeypatch):
    tracker = AckTracker()
    monkeypatch.setattr(time, 'time', lambda: pytest.fail('timeout read wall clock'))
    start = time.monotonic()
    assert tracker.wait_for(1, .01) is None
    assert .005 <= time.monotonic() - start < .5
