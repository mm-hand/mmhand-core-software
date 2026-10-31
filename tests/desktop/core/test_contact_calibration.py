"""V5 接触标定：合成恢复、采集规则、LUT 重标和结果 JSON。"""

import math

import numpy as np

from desktop.core.contact_calibration import (
    CaptureConfig,
    ContactRecorder,
    _initial_mappings,
    load_solution,
    relabel_lut,
    save_solution,
    solve,
)
from desktop.core.kinematics import Kinematics, origin_matrix
from desktop.core.tactile_geometry import module_points
from desktop.paths import DEFAULT_URDF_PATH


def _synthetic(model, rng, truth=None):
    mappings = _initial_mappings()
    k0 = np.asarray([item["k"] for item in mappings])
    b0 = np.asarray([item["b"] for item in mappings])
    if truth is None:
        k = k0 * (1.0 + rng.uniform(-0.005, 0.005, 21))
        b = b0 + rng.uniform(-0.5, 0.5, 21)
        translation = rng.uniform(-0.0005, 0.0005, (21, 3))
        rotation = rng.uniform(-math.radians(0.3), math.radians(0.3), (21, 2))
    else:
        k, b, translation, rotation = truth
    origins = {
        name: model.correction_origin(name, translation[j], rotation[j, 0],
                                      rotation[j, 1])
        for j, name in enumerate(model.names)
    }
    pairs = ((0, 1), (0, 2), (0, 3), (0, 4), (1, 2),
             (1, 3), (1, 4), (2, 3), (2, 4), (3, 4))
    events = []
    for _ in range(150):
        finger_a, finger_b = pairs[int(rng.integers(len(pairs)))]
        segment_a = int(rng.integers(3))
        segment_b = int(rng.integers(3))
        counts = rng.uniform(0.15, 0.85, 21) * 4000.0
        q = counts * k + b
        transforms = model.sensor_transforms(q[None], origins)[0]
        index_a = finger_a * 3 + segment_a
        index_b = finger_b * 3 + segment_b
        points_a = module_points(segment_a)
        point_a = points_a[int(rng.integers(len(points_a)))]
        world = transforms[index_a] @ np.array([*point_a, 1.0])
        point_b = (np.linalg.inv(transforms[index_b]) @ world)[:3]
        events.append({
            "c": counts.tolist(),
            "a": {"finger": finger_a, "segment": segment_a,
                  "point": point_a.tolist(), "force": 100.0},
            "b": {"finger": finger_b, "segment": segment_b,
                  "point": point_b.tolist(), "force": 100.0},
        })
    return events, (k, b, translation, rotation), mappings


def test_synthetic_contact_data_recovers_kb_and_fits_held_out_contacts():
    model = Kinematics(DEFAULT_URDF_PATH)
    rng = np.random.default_rng(7)
    events, truth, mappings = _synthetic(model, rng)
    solution = solve(events, mappings, iterations=20)
    k_true, b_true, translation_true, rotation_true = truth
    params = np.asarray(solution["params"])
    # k 在接触数据里可辨识；几何/ b 有一部分落在“无传感器中间 link”的规范零空间，
    # 因此用 held-out 接触残差验证模型，而不是要求每个随机真值逐项还原。
    assert np.max(np.abs(params[:, 5] / k_true - 1.0)) < 0.01
    assert solution["rms_after_mm"] < 0.1

    held_rng = np.random.default_rng(99)
    held, _, _ = _synthetic(model, held_rng, truth)
    recovered = {
        name: origin_matrix(value["xyz"], value["rpy"])
        for name, value in solution["origins"].items()
    }
    residuals = []
    for event in held:
        counts = np.asarray(event["c"], float)
        angles = counts * np.asarray(solution["k"]) + np.asarray(solution["b"])
        transforms = model.sensor_transforms(angles[None], recovered)[0]
        index_a = event["a"]["finger"] * 3 + event["a"]["segment"]
        index_b = event["b"]["finger"] * 3 + event["b"]["segment"]
        pa = transforms[index_a] @ np.array([*event["a"]["point"], 1.0])
        pb = transforms[index_b] @ np.array([*event["b"]["point"], 1.0])
        residuals.append(np.linalg.norm((pa - pb)[:3]))
    assert np.sqrt(np.mean(np.square(residuals))) * 1000.0 < 0.1


def _snapshot(with_contact=True):
    rows = []
    for joint in range(21):
        rows.append({
            "angle": 0.0, "raw": (1000 + joint * 7) % 16384,
            "manual_min": 1000, "manual_max": 5000, "manual_valid": True,
        })
    modules = []
    for finger, segment in ((0, 0), (1, 0)):
        points = module_points(segment)
        shape = (4, 13, 3) if segment == 0 else (5, 5, 3)
        taxels = np.zeros(shape)
        taxels.reshape(-1, 3)[0] = (60.0, 0.0, 0.0)
        modules.append({
            "finger": finger, "module": segment, "valid": True,
            "force": [60.0, 0.0, 0.0] if with_contact else [0.0, 0.0, 0.0],
            "taxels": taxels.tolist(),
        })
    return {"control_mode": "IDLE", "joints": rows,
            "tactile": {"source": "S3_SERIAL", "level": "FULL",
                        "modules": modules}}


def test_recorder_requires_static_two_finger_full_contact():
    recorder = ContactRecorder(CaptureConfig(force_min=10.0, weight_min=1.0))
    snapshot = _snapshot()
    recorder.observe(snapshot, now=1.0)
    recorder.observe(snapshot, now=1.1)
    recorder.observe(snapshot, now=1.2)
    assert len(recorder.events) == 1
    event = recorder.events[0]
    assert event["a"]["finger"] != event["b"]["finger"]
    assert len(event["c"]) == 21
    recorder.observe(snapshot, now=2.0)
    assert len(recorder.events) == 1  # 同姿态去重


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


def test_solution_json_round_trips_and_rejects_changed_urdf(tmp_path):
    model = Kinematics(DEFAULT_URDF_PATH)
    names = list(model.names)
    solution = {
        "names": names, "k": [0.01] * 21, "b": [0.0] * 21, "origins": {},
        "joint_events": [10] * 21, "weak": [], "rejected": [],
        "events": 100, "rms_before_mm": 1.0, "rms_after_mm": 0.2,
    }
    path = tmp_path / "contact.json"
    save_solution(path, solution, calibration_path=tmp_path / "missing.json")
    payload, reason = load_solution(path, calibration_path=tmp_path / "missing.json")
    assert reason is None and payload["events"] == 100
    assert len(payload["joints"]) == 21
    # 用不同的 URDF 哈希模拟模型变化。
    payload2, reason2 = load_solution(
        path, urdf_path=tmp_path / "other.urdf",
        calibration_path=tmp_path / "missing.json")
    assert payload2 is None and "urdf_sha256" in reason2
