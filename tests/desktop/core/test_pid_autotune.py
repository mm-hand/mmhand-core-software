from desktop.core.pid_autotune import (
    DYNAMIC_BINS, POSITION_BINS, DiscreteEI, GridPoint, ScoreThresholds,
    build_composite_trajectory, evaluate_trial,
)


def test_composite_trajectory_covers_twelve_dynamic_bins_and_three_holds():
    points = build_composite_trajectory(-10.0, 50.0)
    dynamic = {(point.position_bin, point.speed_bin, point.direction) for point in points
               if point.phase == 'dynamic'}
    assert dynamic == set(DYNAMIC_BINS)
    holds = {point.position_bin for point in points if point.phase == 'hold'}
    assert holds == set(POSITION_BINS)
    assert all(-10.0 - 1e-9 <= point.target_deg <= 50.0 + 1e-9 for point in points)
    assert points[-1].target_deg == -10.0 + 0.1 * 60.0


def test_perfect_trial_scores_zero_and_is_valid():
    rows = []
    for position, speed, direction in DYNAMIC_BINS:
        target = -10.0 + position * 60.0
        rows.append(dict(phase='dynamic', position_bin=position, speed_bin=speed,
                         direction=direction, target=target, actual=target))
    for position in POSITION_BINS:
        target = -10.0 + position * 60.0
        for index in range(60):
            rows.append(dict(phase='hold', position_bin=position, time=index * 0.01,
                             target=target, actual=target, motor_cmd=1000))
    score = evaluate_trial(rows, ScoreThresholds())
    assert score.valid
    assert score.dynamic_bins == 12 and score.hold_bins == 3
    assert score.score == 0.0


def test_missing_bin_is_invalid():
    rows = []
    for position, speed, direction in DYNAMIC_BINS[:-1]:
        target = -10.0 + position * 60.0
        rows.append(dict(phase='dynamic', position_bin=position, speed_bin=speed,
                         direction=direction, target=target, actual=target))
    for position in POSITION_BINS:
        target = -10.0 + position * 60.0
        for index in range(60):
            rows.append(dict(phase='hold', position_bin=position, time=index * 0.01,
                             target=target, actual=target, motor_cmd=1000))
    score = evaluate_trial(rows, ScoreThresholds())
    assert not score.valid
    assert 'missing 1 dynamic bins' in score.failures


def test_discrete_ei_suggests_only_untried_points_and_learns():
    grid = [GridPoint(kp, kd) for kp in (0.8, 1.0, 1.2) for kd in (0.0, 0.04)]
    start = grid[0]
    optimizer = DiscreteEI(grid, [start])
    assert optimizer.suggest() == start
    optimizer.observe(start, 1.0, True)
    next_point = optimizer.suggest()
    assert next_point is not None
    assert next_point != start
    optimizer.observe(next_point, 0.2, True)
    third = optimizer.suggest()
    assert third is not None
    assert third not in (start, next_point)
    for point in grid:
        optimizer.observe(point, 0.1, True)
    assert optimizer.suggest() is None
    assert optimizer.best is not None
    assert optimizer.best.score == 0.1


def test_discrete_ei_skips_invalid_candidates():
    grid = [GridPoint(kp, kd) for kp in (0.8, 1.0, 1.2) for kd in (0.0, 0.04)]
    optimizer = DiscreteEI(grid, [])
    optimizer.observe(grid[0], None, False, 'saturated')
    suggested = optimizer.suggest()
    assert suggested == grid[1]
