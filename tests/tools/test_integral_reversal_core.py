import math

import numpy as np

from tools.debug import integral_reversal_core as core


def _hold_samples(points, droop=0.0, noise=0.0):
    samples = []
    for index, point in enumerate(points):
        t = (index + 1) * 10.0
        actual = point.target_deg + droop + noise * math.sin(index * 0.7)
        samples.append(dict(t=t, target=point.target_deg, actual=actual,
                            encoder_ms=t, slow=actual,
                            i_counts=0.0, reversal_counts=0, saturated=False,
                            segment=point.segment, motor_command_abs=[1000],
                            motor_actual_abs=[1000]))
    return samples


def test_hold_plan_has_six_holds_and_two_approaches():
    points = core.build_hold_plan(-25.8, 23.8)
    labels = core.hold_labels(points)
    assert len(labels) == 6
    assert {label['position'] for label in labels} == {0.2, 0.5, 0.8}
    for position in (0.2, 0.5, 0.8):
        approaches = {label['approach'] for label in labels if label['position'] == position}
        assert approaches == {'up', 'down'}
    assert min(point.target_deg for point in points) >= -25.8 - 1e-9
    assert max(point.target_deg for point in points) <= 23.8 + 1e-9


def test_detector_requires_confirmation_distance_and_time():
    target = [0.0] * 20
    target += [2.0 * step / 100 for step in range(1, 101)]
    target += [2.0 - 4.0 * step / 200 for step in range(1, 201)]
    t = [index * 0.01 for index in range(len(target))]
    events = core.simulate_detector(t, target, confirm_deg=0.6, confirm_ms=20)
    assert len(events) == 1
    assert events[0].direction == -1
    assert 1.2 <= events[0].t_s <= 1.9


def test_detector_suite_selection_has_no_false_positive():
    suite = core.build_detector_suite()
    scores = core.evaluate_detector_grid(suite)
    best = core.choose_detector(scores)
    assert best.fp == 0
    assert best.fn_needs_takeup == 0
    assert best.p90_latency_s is not None


def test_replay_reports_gate_after_noise_and_confirm_time():
    samples = []
    for index in range(120):
        t = (index + 1) * 10.0
        samples.append(dict(t=t, target=0.0, actual=0.5, encoder_ms=t, slow=0.5,
                            i_counts=0.0, reversal_counts=0, saturated=False))
    params = dict(ki=0.1, fast_window_ms=20, slow_window_ms=500,
                  integral_limit_counts=100, integral_noise_deg=0.1,
                  integral_confirm_ms=200, confirm_deg=1.0, confirm_ms=20,
                  takeup_counts=0, exit_travel_deg=0.25)
    replayed = core.replay_control_state(samples, params)
    assert replayed[0]['integral_gate_reason'] == 'feedback_stale'
    assert any(row['integral_enabled'] for row in replayed[30:])
    assert replayed[-1]['integral_gate_reason'] == 'ok'


def test_replay_detects_reversal_and_takeup_phase():
    params = dict(ki=0.1, fast_window_ms=20, slow_window_ms=500,
                  integral_limit_counts=100, integral_noise_deg=0.1,
                  integral_confirm_ms=200, confirm_deg=0.6, confirm_ms=20,
                  takeup_counts=20, exit_travel_deg=0.25)
    targets = [0.0] * 60
    for sign in (1, -1):
        targets += [sign * 2.0 * step / 30 for step in range(1, 31)]
    samples = []
    for index, target in enumerate(targets):
        t = (index + 1) * 10.0
        samples.append(dict(t=t, target=target, actual=target + 0.1,
                            encoder_ms=t, slow=target + 0.1, i_counts=0.0,
                            reversal_counts=0, saturated=False))
    replayed = core.replay_control_state(samples, params)
    detected = [row for row in replayed if row['reversal_detected']]
    assert len(detected) == 1
    assert detected[0]['control_phase'] == 'Takeup'
    assert any(row['takeup_active'] for row in replayed)


def test_integral_metrics_scores_droop_and_keeps_perfect_hold_valid():
    points = core.build_hold_plan(-25.8, 23.8)
    params = dict(ki=0.0, fast_window_ms=20, slow_window_ms=500,
                  integral_limit_counts=100, integral_noise_deg=0.1,
                  integral_confirm_ms=200, confirm_deg=1.0, confirm_ms=20,
                  takeup_counts=0, exit_travel_deg=0.25)
    perfect = core.integral_metrics(_hold_samples(points), points, [0], params)
    droop = core.integral_metrics(_hold_samples(points, droop=0.4), points, [0], params)
    assert perfect.valid and droop.valid
    assert perfect.score < droop.score


def test_choose_integral_prefers_smaller_ki_within_noise():
    baseline = core.IntegralTrial('aa', 8, 0.0, True, 0.50, '')
    close = core.IntegralTrial('aa', 8, 0.1, True, 0.47, '')
    better = core.IntegralTrial('aa', 8, 0.2, True, 0.20, '')
    choice = core.choose_integral([baseline, close, better], noise=0.02)
    assert choice['selected_ki'] == 0.2
    choice = core.choose_integral([baseline, close], noise=0.02)
    assert choice['selected_ki'] == 0.0


def test_reversal_metrics_requires_response_and_scores_latency():
    points, truth = core.build_reversal_plan(-25.8, 23.8, speeds=(('low', 5.0),))
    samples = []
    for index, point in enumerate(points):
        t = (index + 1) * 10.0
        samples.append(dict(t=t, target=point.target_deg,
                            actual=point.target_deg + 0.05, encoder_ms=t,
                            slow=point.target_deg + 0.05, i_counts=0.0,
                            reversal_counts=0, saturated=False,
                            segment=point.segment, motor_command_abs=[1000],
                            motor_actual_abs=[1000]))
    params = dict(ki=0.0, fast_window_ms=20, slow_window_ms=500,
                  integral_limit_counts=100, integral_noise_deg=0.1,
                  integral_confirm_ms=200, confirm_deg=0.6, confirm_ms=20,
                  takeup_counts=0, exit_travel_deg=0.25)
    trial = core.reversal_metrics(samples, truth, [0], params)
    assert trial.valid
    assert trial.score is not None
    assert len(trial.events) == len(truth)


def test_takeup_requires_both_single_and_aa_results():
    single = core.ReversalTrial('mcp', 9, 20, 0.25, True, 0.2, '')
    dual = core.ReversalTrial('aa', 8, 20, 0.25, True, 0.25, '')
    baseline_single = core.ReversalTrial('mcp', 9, 0, 0.25, True, 0.5, '')
    baseline_dual = core.ReversalTrial('aa', 8, 0, 0.25, True, 0.5, '')
    choice = core.choose_takeup([single, dual, baseline_single, baseline_dual], noise=0.02)
    assert choice['selected_takeup'] == 20
    choice = core.choose_takeup([single, baseline_single, baseline_dual], noise=0.02)
    assert choice['selected_takeup'] == 0.0
    # Missing single-tendon baseline: never deploy a global nonzero takeup.
    choice = core.choose_takeup([single, dual, baseline_dual], noise=0.02)
    assert choice['selected_takeup'] == 0.0


def test_validate_combined_rejects_degraded_reversal():
    baseline = {name: dict(integral_score=0.5, reversal_score=0.5,
                           dynamic_rmse_deg=1.0) for name in core.TYPES}
    candidate = {name: dict(integral_score=0.5, reversal_score=0.9,
                            dynamic_rmse_deg=1.0) for name in core.TYPES}
    ok, reasons = core.validate_combined(baseline, candidate)
    assert not ok
    assert reasons


def test_mcp_test_plans_stay_below_75_degrees():
    low, high = -3.6, 79.6
    cap = core.TEST_HIGH_CAP_DEG['mcp']
    assert cap == 74.0
    for plan in (core.build_hold_plan(low, high, high_cap=cap),
                 core.build_reversal_plan(low, high, high_cap=cap)[0],
                 core.build_validation_plan(low, high, high_cap=cap)[0]):
        assert max(point.target_deg for point in plan) < 75.0
        assert max(point.target_deg for point in plan) <= cap + 1e-9
