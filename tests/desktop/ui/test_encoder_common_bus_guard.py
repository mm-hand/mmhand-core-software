from types import SimpleNamespace

from desktop.ui.ui_control_state import ControlPanelStateMixin


ALL_ENCODERS_MASK = (1 << 21) - 1


def _state(raw_values, timestamp, *, all_online=True):
    return SimpleNamespace(
        has_encoder_raw_data=True,
        has_encoder_health_status=True,
        encoder_raw_last_rx_ts=float(timestamp),
        encoder_online_bitmap=ALL_ENCODERS_MASK if all_online else 0,
        encoder_warn_bitmap=0,
        encoder_fault_bitmap=0,
        encoders=[
            SimpleNamespace(raw_sensor=int(value), raw_error=False, error=False)
            for value in raw_values
        ],
    )


class _GuardHarness(ControlPanelStateMixin):
    def __init__(self):
        self._encoder_common_zero_first_ts = 0.0
        self._encoder_common_zero_last_sample_ts = 0.0
        self._encoder_common_zero_recovery_frames = 0
        self._encoder_common_bus_fault = False
        self.actions = []
        self.statuses = []
        self.context = SimpleNamespace(emit_action=self.actions.append)

    def _set_status(self, message):
        self.statuses.append(message)


def test_all_zero_all_online_is_confirmed_and_overlays_red_health():
    guard = _GuardHarness()

    first = _state([0] * 21, 10.0)
    guard._update_encoder_common_bus_guard(first)
    assert not guard._encoder_common_bus_fault

    confirmed = _state([0] * 21, 10.21)
    guard._update_encoder_common_bus_guard(confirmed)

    assert guard._encoder_common_bus_fault
    assert confirmed.encoder_online_bitmap == 0
    assert confirmed.encoder_fault_bitmap == ALL_ENCODERS_MASK
    assert len(guard.actions) == 1
    assert "encoder FPC to S3" in guard.actions[0]


def test_zero_snapshot_is_not_fault_when_firmware_does_not_report_all_online():
    guard = _GuardHarness()

    guard._update_encoder_common_bus_guard(_state([0] * 21, 20.0, all_online=False))
    guard._update_encoder_common_bus_guard(_state([0] * 21, 20.5, all_online=False))

    assert not guard._encoder_common_bus_fault
    assert guard.actions == []


def test_fault_recovers_after_three_valid_nonzero_raw_snapshots():
    guard = _GuardHarness()
    guard._update_encoder_common_bus_guard(_state([0] * 21, 30.0))
    guard._update_encoder_common_bus_guard(_state([0] * 21, 30.21))
    assert guard._encoder_common_bus_fault

    for timestamp in (30.22, 30.23):
        state = _state([1] + [0] * 20, timestamp)
        guard._update_encoder_common_bus_guard(state)
        assert guard._encoder_common_bus_fault
        assert state.encoder_fault_bitmap == ALL_ENCODERS_MASK

    recovered = _state([1] + [0] * 20, 30.24)
    guard._update_encoder_common_bus_guard(recovered)

    assert not guard._encoder_common_bus_fault
    assert recovered.encoder_online_bitmap == ALL_ENCODERS_MASK
    assert recovered.encoder_fault_bitmap == 0
    assert "fault has cleared" in guard.actions[-1]
