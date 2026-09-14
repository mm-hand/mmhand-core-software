import math
from types import SimpleNamespace

from desktop.core.feedback_gate import FeedbackGate


class FakeClock:
    def __init__(self) -> None:
        self.now_ns = 0

    def __call__(self) -> int:
        return self.now_ns

    def advance_ms(self, milliseconds: int) -> None:
        self.now_ns += milliseconds * 1_000_000


def make_state(clock: FakeClock, *, sequence: int = 1, valid: bool = True):
    return SimpleNamespace(
        angles=[float(i) for i in range(21)],
        encoders=[SimpleNamespace(angle_error=False) for _ in range(21)],
        joint_state_valid=valid,
        joint_state_sequence=sequence,
        joint_state_host_rx_ns=1_700_000_000_000_000_000 + sequence,
        joint_state_host_rx_monotonic_ns=clock.now_ns,
    )


def test_armed_gate_waits_300_ms_then_requires_a_post_cooldown_frame() -> None:
    clock = FakeClock()
    gate = FeedbackGate("armed", clock=clock)
    old_state = make_state(clock)

    clock.advance_ms(299)
    assert gate.next_sample(old_state, requested=True) is None

    clock.advance_ms(1)
    assert gate.next_sample(old_state, requested=True) is None
    assert gate.fresh_after == clock.now_ns

    clock.advance_ms(1)
    fresh_state = make_state(clock, sequence=2)
    stamp, positions = gate.next_sample(fresh_state, requested=True)
    assert stamp == fresh_state.joint_state_host_rx_ns
    assert positions == [math.radians(float(i)) for i in range(21)]


def test_reopen_waits_from_later_of_disable_and_last_publish() -> None:
    clock = FakeClock()
    gate = FeedbackGate("armed", clock=clock)
    clock.advance_ms(300)
    assert gate.next_sample(make_state(clock), requested=True) is None
    clock.advance_ms(1)
    sample = gate.next_sample(make_state(clock, sequence=2), requested=True)
    assert sample is not None
    gate.published(sample[0])

    clock.advance_ms(20)
    gate.disabled()
    clock.advance_ms(299)
    assert gate.next_sample(make_state(clock, sequence=3), requested=True) is None
    clock.advance_ms(1)
    assert gate.next_sample(make_state(clock, sequence=4), requested=True) is None
    clock.advance_ms(1)
    assert gate.next_sample(make_state(clock, sequence=5), requested=True) is not None


def test_gate_rejects_stale_invalid_duplicate_and_angle_error_frames() -> None:
    clock = FakeClock()
    gate = FeedbackGate("always", clock=clock)

    valid = make_state(clock)
    sample = gate.next_sample(valid, requested=False)
    assert sample is not None
    gate.published(sample[0])
    assert gate.next_sample(valid, requested=False) is None

    invalid = make_state(clock, sequence=2, valid=False)
    assert gate.next_sample(invalid, requested=False) is None

    clock.advance_ms(201)
    stale = make_state(clock, sequence=3)
    stale.joint_state_host_rx_monotonic_ns = 0
    assert gate.next_sample(stale, requested=False) is None

    errored = make_state(clock, sequence=4)
    errored.encoders[7].angle_error = True
    assert gate.next_sample(errored, requested=False) is None


def test_off_mode_never_publishes() -> None:
    clock = FakeClock()
    gate = FeedbackGate("off", clock=clock)
    clock.advance_ms(500)
    assert gate.next_sample(make_state(clock), requested=True) is None
