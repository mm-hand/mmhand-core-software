"""V7 滑动轨迹接触标定：自动指尖、抽稀、non-contact、连续性和标定包字段。"""

import math

import numpy as np
import pytest

from desktop.core import contact_calibration as contact_module
from desktop.core.angle_mapping import get_spec
from desktop.core.calibration_snapshot import (
    _empty_v7_snapshot,
    atomic_write_snapshot,
    read_snapshot,
    update_manual_snapshot,
)
from desktop.core.contact_calibration import (
    CONTACT_LINKS,
    CONTACT_MARGIN_M,
    CONTACT_PAIRS,
    FINGER_JOINTS,
    NONCONTACT_MIN_GAP_M,
    add_non_contact_samples,
    build_trajectory,
    clear_contact,
    contact_mappings,
    counts_from_snapshot,
    infer_pair,
    initial_mappings,
    model_angles,
    relabel_lut,
    save_contact,
    signed_counts,
    solve,
    solve_contact,
    tip_travel,
    trajectory_events,
)
from desktop.core.kinematics import Kinematics
from desktop.core.tactile_geometry import module_points
from desktop.paths import DEFAULT_URDF_PATH


def _snapshot(with_manual=True):
    return {'control_mode': 'IDLE', 'communication': 'Online', 'joints': [
        {'raw': (1000 + joint * 7) % 16384, 'manual_min': 1000,
         'manual_max': 5000, 'manual_valid': with_manual}
        for joint in range(21)]}


def test_signed_counts_matches_the_p4_window_and_rejects_bad_manual():
    assert signed_counts(1000, 1000, 5000) == 0
    assert signed_counts(3000, 1000, 5000) == 2000
    assert signed_counts(5000, 5000, 1000) == 0
    assert signed_counts(4000, 5000, 1000) == -1000
    assert signed_counts(1000, 5000, 1000) == 16384 - 4000
    assert signed_counts(3000, 1000, 1000) is None
    assert signed_counts(None, 1000, 5000) is None
    assert signed_counts(3000, None, 5000) is None


def test_counts_from_snapshot_names_the_missing_field():
    counts, reason = counts_from_snapshot(_snapshot())
    assert reason is None and len(counts) == 21
    snapshot = _snapshot()
    snapshot['joints'][0]['raw'] = None
    assert counts_from_snapshot(snapshot) == (None, 'encoder raw data is not fresh')
    snapshot = _snapshot()
    snapshot['joints'][0]['manual_valid'] = False
    assert counts_from_snapshot(snapshot) == (None, 'a joint is missing Manual Min/Max')
    snapshot = _snapshot()
    snapshot['joints'][0]['manual_max'] = snapshot['joints'][0]['manual_min']
    assert counts_from_snapshot(snapshot) == (None, "a joint's Manual Min/Max is invalid")
    assert counts_from_snapshot({'joints': []})[1] == 'joint data is incomplete'


def test_initial_mappings_use_manual_span():
    mappings = initial_mappings([1000] * 21, [9000] * 21)
    low, high = get_spec(1).urdf_range_deg
    assert mappings[1] == {'k': (high - low) / 8000.0, 'b': low}
    assert model_angles([0, 100], [2.0], [-10.0]).tolist() == [-10.0, 190.0]


def test_contact_links_are_the_physical_finger_links():
    assert CONTACT_LINKS[:3] == ('link_finger_tip_5', 'link_distal_phalanx_5',
                                 'link_proximal_phalanx_5')
    assert CONTACT_LINKS[12:] == ('link_finger_tip_1', 'link_distal_phalanx_1',
                                  'link_proximal_phalanx_1')
    assert not any(name.endswith('_sensor') for name in CONTACT_LINKS)


def _clouds():
    return contact_module._clouds(DEFAULT_URDF_PATH)


def _pair_gap(frames, pair, clouds):
    finger, other, segment = pair
    tip = clouds[0] @ frames[finger * 3].T
    target = clouds[segment] @ frames[other * 3 + segment].T
    return float(np.linalg.norm(tip[:, None, :] - target[None, :, :],
                                axis=-1).min())


def _truth(model, rng):
    mappings = initial_mappings()
    k0 = np.asarray([item['k'] for item in mappings])
    b0 = np.asarray([item['b'] for item in mappings])
    k = k0 * (1.0 + rng.uniform(-0.02, 0.02, 21))
    b = b0 + rng.uniform(-2.0, 2.0, 21)
    translation = rng.uniform(-0.001, 0.001, (21, 3))
    rotation = rng.uniform(-math.radians(1.0), math.radians(1.0), (21, 2))
    origins = {
        name: model.correction_origin(name, translation[joint],
                                      rotation[joint, 0], rotation[joint, 1])
        for joint, name in enumerate(model.names)}
    return mappings, k, b, origins


def _touching_events(model, origins, k, b, rng, contacts):
    """用真值摆出真接触，再在同指同段上方 10% 量程处取“接近但未接触”。"""
    clouds = _clouds()
    events = []
    for _attempt in range(400):
        if sum(1 for event in events if event['y'] == 1) >= contacts:
            break
        counts = rng.uniform(0.1, 0.9, 21) * 4000.0
        frames = model.link_transforms(
            model_angles(counts, k, b), origins, CONTACT_LINKS)[0]
        gaps = np.asarray([_pair_gap(frames, pair, clouds) for pair in CONTACT_PAIRS])
        pair = CONTACT_PAIRS[int(np.argmin(gaps))]
        joint = FINGER_JOINTS[pair[0]][1]
        best_counts, best_gap = counts.copy(), float(gaps.min())
        for value in np.linspace(0.0, 4000.0, 41):
            trial = counts.copy()
            trial[joint] = value
            frames = model.link_transforms(
                model_angles(trial, k, b), origins, CONTACT_LINKS)[0]
            gap = _pair_gap(frames, pair, clouds)
            if gap < best_gap:
                best_gap, best_counts = gap, trial
        step = 4000.0 / 40.0
        for value in np.linspace(best_counts[joint] - step,
                                 best_counts[joint] + step, 21):
            trial = best_counts.copy()
            trial[joint] = value
            frames = model.link_transforms(
                model_angles(trial, k, b), origins, CONTACT_LINKS)[0]
            gap = _pair_gap(frames, pair, clouds)
            if gap < best_gap:
                best_gap, best_counts = gap, trial
        if best_gap > CONTACT_MARGIN_M:
            continue
        events.append({'c': best_counts.tolist(), 'y': 1,
                       'fingertip': pair[0], 'pair': pair})
        close = best_counts.copy()
        close[joint] = best_counts[joint] + 400.0
        frames = model.link_transforms(
            model_angles(close, k, b), origins, CONTACT_LINKS)[0]
        if _pair_gap(frames, pair, clouds) > 2.0 * CONTACT_MARGIN_M:
            events.append({'c': close.tolist(), 'y': 0,
                           'fingertip': pair[0], 'pair': pair})
    return events


_SYNTHETIC = {}


def _synthetic():
    if not _SYNTHETIC:
        model = Kinematics(DEFAULT_URDF_PATH)
        rng = np.random.default_rng(7)
        mappings, k, b, origins = _truth(model, rng)
        events = _touching_events(model, origins, k, b, rng, contacts=12)
        _SYNTHETIC.update(events=events, mappings=mappings)
    return _SYNTHETIC['events'], _SYNTHETIC['mappings']


def test_synthetic_contact_data_fits_the_two_class_model():
    events, mappings = _synthetic()
    assert sum(1 for event in events if event['y'] == 1) >= 8
    assert sum(1 for event in events if event['y'] == 0) >= 4

    solution = solve(events, mappings, iterations=4, optimize_geometry=True)
    assert solution['base_k'] == [item['k'] for item in mappings]
    assert solution['base_b'] == [item['b'] for item in mappings]
    assert solution['optimized'] == {'kb': True, 'geometry': True}
    assert solution['clearance_after_mm'] <= solution['clearance_before_mm']
    assert solution['contact_pass'] + solution['noncontact_pass'] >= (
        solution['contact_total'] + solution['noncontact_total']) // 2
    for joint in range(21):
        base_k, base_b = mappings[joint]['k'], mappings[joint]['b']
        assert 0.9 * base_k <= solution['k'][joint] <= 1.1 * base_k
        assert abs(solution['b'][joint] - base_b) <= 5.0 + 1e-9
        params = solution['params'][joint]
        assert np.linalg.norm(params[:3]) <= 0.005 + 1e-9
        assert abs(params[3]) <= math.radians(3.0) + 1e-9
        assert abs(params[4]) <= math.radians(3.0) + 1e-9


def test_solve_kb_only_leaves_the_urdf_origins_alone():
    events, mappings = _synthetic()
    solution = solve(events, mappings, iterations=3, optimize_geometry=False)
    assert solution['optimized'] == {'kb': True, 'geometry': False}
    assert solution['origins'] == {}
    assert all(not any(params[:5]) for params in solution['params'])
    for joint in range(21):
        base_k, base_b = mappings[joint]['k'], mappings[joint]['b']
        assert 0.9 * base_k <= solution['k'][joint] <= 1.1 * base_k
        assert abs(solution['b'][joint] - base_b) <= 5.0 + 1e-9


def test_solve_geometry_only_keeps_the_base_kb():
    events, mappings = _synthetic()
    solution = solve(events, mappings, iterations=3, optimize_kb=False,
                     optimize_geometry=True)
    assert solution['optimized'] == {'kb': False, 'geometry': True}
    assert solution['k'] == solution['base_k']
    assert solution['b'] == solution['base_b']


def test_solve_requires_at_least_one_correction_and_one_contact():
    mappings = initial_mappings()
    with pytest.raises(ValueError, match='select at least one'):
        solve([{'c': [0.0] * 21, 'y': 1}], mappings,
              optimize_kb=False, optimize_geometry=False)
    with pytest.raises(ValueError, match='at least one contact'):
        solve([{'c': [0.0] * 21, 'y': 0}], mappings)
    with pytest.raises(ValueError, match='invalid contact events'):
        solve([{'y': 1}], mappings)


def _fist_counts():
    mappings = initial_mappings()
    k0 = np.asarray([item['k'] for item in mappings])
    b0 = np.asarray([item['b'] for item in mappings])
    angles = b0.copy()
    for finger, joints in enumerate(FINGER_JOINTS):
        if finger == 4:
            angles[joints[0]] = -20.0
            angles[joints[1]] = 45.0
            angles[joints[4]] = 40.0
        else:
            angles[joints[1]] = 70.0
            angles[joints[2]] = 75.0
            angles[joints[3]] = 45.0
    return (angles - b0) / k0, mappings


def _base_frames(counts):
    mappings = initial_mappings()
    model = Kinematics(DEFAULT_URDF_PATH)
    return model.link_transforms(
        model_angles(counts, [item['k'] for item in mappings],
                     [item['b'] for item in mappings]),
        None, CONTACT_LINKS)[0]


def test_infer_pair_is_limited_to_the_fingertip():
    counts, mappings = _fist_counts()
    finger, other, segment, clearance = infer_pair(counts, mappings, 0)
    frames = _base_frames(counts)
    clouds = _clouds()
    expected = min((_pair_gap(frames, pair, clouds), pair)
                   for pair in CONTACT_PAIRS if pair[0] == 0)
    assert (finger, other, segment) == expected[1]
    assert math.isclose(clearance, expected[0] * 1000.0, abs_tol=1e-9)


def test_solve_uses_only_the_selected_fingertip():
    counts, mappings = _fist_counts()
    frames = _base_frames(counts)
    clouds = _clouds()
    global_gap = min(_pair_gap(frames, pair, clouds) for pair in CONTACT_PAIRS)
    pinky_gap = min(_pair_gap(frames, pair, clouds)
                    for pair in CONTACT_PAIRS if pair[0] == 0)
    assert pinky_gap > global_gap  # 全局最近对不是小指，必须按 fingertip 收窄
    tagged = solve([{'c': counts.tolist(), 'y': 1, 'fingertip': 0}],
                   mappings, iterations=0)
    untagged = solve([{'c': counts.tolist(), 'y': 1}], mappings, iterations=0)
    assert math.isclose(tagged['clearance_before_mm'], pinky_gap * 1000.0,
                        abs_tol=1e-9)
    assert math.isclose(untagged['clearance_before_mm'], global_gap * 1000.0,
                        abs_tol=1e-9)


def test_solve_rejects_an_invalid_fingertip():
    with pytest.raises(ValueError, match='invalid fingertip'):
        solve([{'c': [0.0] * 21, 'y': 1, 'fingertip': 9}],
              initial_mappings(), iterations=0)


def test_physical_link_points_are_the_sensor_points():
    model = Kinematics(DEFAULT_URDF_PATH)
    mappings = initial_mappings()
    counts = np.linspace(200.0, 3800.0, 21)
    angles = model_angles(counts, [item['k'] for item in mappings],
                          [item['b'] for item in mappings])
    link_frames = model.link_transforms(angles, None, CONTACT_LINKS)
    sensor_frames = model.sensor_transforms(angles)
    clouds = _clouds()
    for index in range(15):
        segment = index % 3
        sensor_points = np.column_stack([module_points(segment),
                                         np.ones(len(module_points(segment)))])
        assert np.allclose(clouds[segment] @ link_frames[0, index].T,
                           sensor_points @ sensor_frames[0, index].T,
                           atol=1e-12)


def test_relabel_lut_keeps_motor_values_and_tracks_same_counts():
    rows = [[1000 + 37 * index for index in range(8)],
            [5000 - 41 * index for index in range(8)]]
    result = {
        "success": True, "scaled": True, "mappingVersion": 5,
        "manualSpanCounts": 4000, "angleK": 0.014, "angleB": -45.0,
        "angleMin": -45.0, "angleMax": 11.0,
        "tendonLutValid": True, "tendonLutCount": 8,
        "tendonLutMotor": [0, 1],
        "tendonLutAngleStartDeg": -45.0, "tendonLutAngleStepDeg": 5.0,
        "tendonLutMotorAbs": rows,
    }
    updated = relabel_lut(result, 0.014, -45.0, 0.01407, -44.8)
    assert updated["tendonLutMotorAbs"] == rows
    assert updated["angleK"] == 0.01407 and updated["angleB"] == -44.8
    scale = 0.01407 / 0.014
    offset = -44.8 - scale * -45.0
    for index in range(8):
        assert math.isclose(updated["tendonLutAngleStartDeg"]
                            + updated["tendonLutAngleStepDeg"] * index,
                            scale * (-45.0 + 5.0 * index) + offset,
                            abs_tol=1e-4)


def test_contact_solution_lives_in_the_calibration_bundle(tmp_path, monkeypatch):
    path = tmp_path / "bundle.mmcal.json"
    atomic_write_snapshot(path, _empty_v7_snapshot())
    names = [get_spec(joint).urdf_joint_name for joint in range(21)]
    solution = {"names": names, "k": [0.01] * 21, "b": [0.5] * 21,
                "base_k": [0.02] * 21, "base_b": [0.4] * 21,
                "origins": {names[3]: {"xyz": [0.001, 0.0, 0.0],
                                       "rpy": [0.0, 0.0, 0.0]}}}
    save_contact(path, solution)
    payload = read_snapshot(path)
    stored = contact_mappings(payload)
    assert len(stored) == 21 and stored[names[3]] == (0.01, 0.5)
    assert payload["contact_calibration"][names[3]]["base_k"] == 0.02
    assert payload["contact_calibration"][names[3]]["base_b"] == 0.4
    assert "xyz" not in payload["contact_calibration"][names[0]]
    assert "rpy" not in payload["contact_calibration"][names[0]]
    # contact 结果只写自己的字段，不动 Joint Calibration 结果。
    assert payload["joint_calib_results"] == [None] * 21

    monkeypatch.setattr(contact_module, "CALIB_ACTIVE_SNAPSHOT_PATH", path)
    origins = contact_module.active_origins()
    assert set(origins) == {names[3]}
    assert origins[names[3]]["xyz"] == [0.001, 0.0, 0.0]

    # 其它标定写入必须保留 contact 字段。
    update_manual_snapshot(path, valid_mask=1 << 3, min_raw=[0] * 21,
                           max_raw=[5000] * 21, updated_joint_mask=1 << 3)
    assert contact_mappings(read_snapshot(path))[names[3]] == (0.01, 0.5)

    clear_contact(path)
    payload = read_snapshot(path)
    assert contact_mappings(payload) == {}
    assert payload["joint_calib_results"] == [None] * 21


# ---- V7 滑动轨迹 / 非接触样本 ----

_SLIDE_PAIR = (3, 4, 0)
_SLIDE_BASE = np.array([
    703, 1384, 2484, 2981, 2099, 2602, 3008, 2815, 1948, 3339, 2285,
    2291, 3221, 2557, 2776, 675, 1367, 2309, 1012, 2658, 1832], float)
_MANUAL_MIN = [1000] * 21
_MANUAL_MAX = [5000] * 21


def _synthetic_slide(seed=0, steps=30):
    """真值模型下让食指指尖沿拇指滑动，返回 raw 帧和标定上下文。"""
    model = Kinematics(DEFAULT_URDF_PATH)
    clouds = _clouds()
    rng = np.random.default_rng(seed)
    mappings, k, b, origins = _truth(model, rng)
    dips = np.linspace(300.0, 1600.0, 41)
    frames = []
    for step in range(steps):
        t = step / max(1, steps - 1)
        counts = _SLIDE_BASE.copy()
        counts[12] += (t - 0.5) * 500.0
        counts[13] += (t - 0.5) * 400.0
        candidates = np.repeat(counts[None, :], len(dips), axis=0)
        candidates[:, 15] = dips
        poses = model.link_transforms(
            model_angles(candidates, k, b), origins, CONTACT_LINKS)
        gaps = np.array([_pair_gap(poses[i], _SLIDE_PAIR, clouds)
                         for i in range(len(candidates))])
        best = int(np.argmin(gaps))
        if gaps[best] > CONTACT_MARGIN_M:
            continue
        frames.append((step * 10, tuple(
            ((candidates[best] + 1000) % 16384).astype(int).tolist())))
    return model, clouds, mappings, frames


def test_trajectory_events_selects_the_moving_fingertip_and_decimates():
    _model, _clouds_, _mappings, frames = _synthetic_slide()
    events, info = trajectory_events(frames, _MANUAL_MIN, _MANUAL_MAX, 0)
    assert info["fingertip"] == 3
    assert info["contacts"] == len(events) >= 5
    assert info["frames"] == len(frames)
    assert all(event["y"] == 1 and event["pair"][0] == 3 for event in events)
    assert all(event["traj"] == 0 for event in events)


def test_non_contact_samples_have_a_clear_model_gap():
    model, clouds, mappings, frames = _synthetic_slide()
    contacts, _info = trajectory_events(frames, _MANUAL_MIN, _MANUAL_MAX, 0)
    events, info = add_non_contact_samples(
        contacts, mappings, _MANUAL_MIN, _MANUAL_MAX)
    negatives = [event for event in events if event["y"] == 0]
    assert negatives and info["noncontacts"] == len(negatives)
    geometry = contact_module._geometry_from_mappings(mappings)
    for event in negatives:
        gap = contact_module._pair_gap(
            model, clouds, geometry, event["c"], event["pair"])
        assert gap >= NONCONTACT_MIN_GAP_M - 1e-9


def test_build_trajectory_end_to_end_returns_finite_metrics():
    _model, _clouds_, mappings, frames = _synthetic_slide()
    result = build_trajectory(frames, _MANUAL_MIN, _MANUAL_MAX, 2)
    events, info = result["events"], result["info"]
    assert info["contacts"] >= 5
    assert info["noncontacts"] >= 1
    assert sum(1 for event in events if event["y"] == 1) == info["contacts"]
    assert all(event["traj"] == 2 for event in events)
    solution = solve(events, mappings)
    assert solution["iterations_used"] <= contact_module.SOLVE_ITERATIONS
    assert math.isfinite(solution["continuity_before_mm"])
    assert math.isfinite(solution["continuity_after_mm"])
    assert solution["contact_total"] == info["contacts"]
    assert solution["noncontact_total"] == info["noncontacts"]


def test_continuity_pairs_stay_inside_one_trajectory():
    event = {"c": [0.0] * 21, "y": 1, "pair": _SLIDE_PAIR}
    pairs = contact_module._continuity_pairs([
        dict(event, traj=0), dict(event, traj=0), dict(event, traj=1)])
    assert pairs.tolist() == [[0, 1]]


def test_tip_travel_selects_the_moving_finger():
    counts = np.full((3, 21), 2000.0)
    for offset, joint in enumerate(FINGER_JOINTS[3]):
        counts[1, joint] += 120.0 + 40.0 * offset
        counts[2, joint] += 240.0 + 80.0 * offset
    travel = tip_travel(counts, initial_mappings())
    assert int(np.argmax(travel)) == 3


def test_trajectory_rejects_a_recording_without_motion():
    frames = [(index * 10, tuple([2000] * 21)) for index in range(10)]
    with pytest.raises(ValueError, match="sliding motion is too short"):
        trajectory_events(frames, _MANUAL_MIN, _MANUAL_MAX, 0)


def test_solve_contact_refreshes_correspondences_in_an_outer_loop():
    events, mappings = _synthetic()
    result = solve_contact(events, mappings, outer=2, inner=2)
    assert result["iterations_used"] <= 2
    assert math.isfinite(result["clearance_after_mm"])
    assert math.isfinite(result["continuity_after_mm"])
    assert result["contact_total"] >= 1
