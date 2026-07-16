import pytest


ENCODER_MODULO = 16384


def directed_delta(from_raw: int, to_raw: int, direction: int) -> int:
    if direction not in (-1, 1):
        raise ValueError("direction must be -1 or 1")
    delta = to_raw - from_raw if direction > 0 else from_raw - to_raw
    return delta % ENCODER_MODULO


def build_range(manual_min, manual_max, auto_min, auto_max, direction, urdf_min, urdf_max):
    manual_span = directed_delta(manual_min, manual_max, direction)
    auto_span = directed_delta(auto_min, auto_max, direction)
    if not 0 < manual_span < ENCODER_MODULO:
        raise ValueError("invalid manual span")
    if not 0 < auto_span < ENCODER_MODULO:
        raise ValueError("invalid auto span")
    current_max = urdf_max
    current_min = current_max - (urdf_max - urdf_min) * auto_span / manual_span
    return manual_span, auto_span, current_min, current_max


def test_directed_delta_handles_both_wrap_directions():
    assert directed_delta(16000, 400, 1) == 784
    assert directed_delta(400, 16000, -1) == 784


def test_manual_scaled_range_uses_requested_formula():
    manual_span, auto_span, current_min, current_max = build_range(
        1000, 5000, 1800, 4600, 1, -15.0, 90.0
    )
    assert manual_span == 4000
    assert auto_span == 2800
    assert current_max == 90.0
    assert current_min == pytest.approx(16.5)


def test_auto_span_larger_than_manual_span_is_allowed():
    _, _, current_min, current_max = build_range(
        1000, 4000, 500, 4500, 1, 0.0, 60.0
    )
    assert current_max == 60.0
    assert current_min == pytest.approx(-20.0)
