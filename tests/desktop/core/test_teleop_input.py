import math

from desktop.core.teleop_input import InputFilterConfig, StationaryAwareOneEuro, TeleopInputFilter


def test_stationary_jitter_is_anchored_and_real_motion_exits_smoothly():
    item = StationaryAwareOneEuro(InputFilterConfig())
    values = []
    for index in range(100):
        raw = 0.02 * (-1 if index % 2 else 1)
        output, valid = item.update(raw, index * .01)
        assert valid
        values.append(output)
    assert item.state == 'STATIONARY'
    assert max(values[-30:]) - min(values[-30:]) < 1e-9
    held = values[-1]
    for index in range(100, 150):
        output, valid = item.update(2.0, index * .01)
        assert valid
    assert item.state == 'MOVING'
    assert output > held


def test_invalid_time_and_nan_do_not_advance_state():
    item = StationaryAwareOneEuro(InputFilterConfig())
    first, valid = item.update(0.2, 1.0)
    assert valid
    assert item.update(math.nan, 1.1) == (first, False)
    assert item.update(0.3, 0.9) == (first, False)
    assert item.previous_time == 1.0


def test_batch_requires_21_degrees_and_first_frame_is_not_ramped():
    batch = TeleopInputFilter()
    assert batch.update_degrees([0.0] * 20, 0.0) is None
    result = batch.update_degrees([10.0] * 21, 1.0)
    assert result == [10.0] * 21
    assert batch.update_degrees([10.0] * 21, 1.0) is None
