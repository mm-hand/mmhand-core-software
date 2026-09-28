from pathlib import Path

import numpy as np

from desktop.visualize.plot_step_response import analyze_steps, detect_step_indices


def test_detect_step_indices_ignores_platform_samples() -> None:
    time_s = np.arange(0.0, 2.0, 0.05)
    target = np.where(time_s < 0.5, 10.0, np.where(time_s < 1.2, 30.0, 15.0))

    assert detect_step_indices(time_s, target, 0.5) == [10, 24]


def test_analyze_steps_handles_positive_and_negative_steps() -> None:
    time_s = np.arange(0.0, 3.0, 0.05)
    target = np.where(time_s < 0.5, 10.0, np.where(time_s < 1.7, 30.0, 15.0))
    actual = np.full_like(time_s, 10.0)
    rising = (time_s >= 0.6) & (time_s < 1.0)
    actual[rising] = 10.0 + 20.0 * (time_s[rising] - 0.6) / 0.4
    actual[(time_s >= 1.0) & (time_s < 1.8)] = 30.0
    falling = (time_s >= 1.8) & (time_s < 2.2)
    actual[falling] = 30.0 - 15.0 * (time_s[falling] - 1.8) / 0.4
    actual[time_s >= 2.2] = 15.0

    steps = analyze_steps(
        Path("session"),
        time_s,
        target,
        time_s,
        actual,
        min_step_deg=0.5,
        tolerance_deg=0.5,
        settle_hold_s=0.2,
    )

    assert len(steps) == 2
    assert steps[0].step_deg == 20.0
    assert steps[1].step_deg == -15.0
    assert 50.0 <= steps[0].response_delay_ms <= 150.0
    assert 300.0 <= steps[0].rise_time_ms <= 400.0
    assert steps[0].overshoot_deg < 0.01
