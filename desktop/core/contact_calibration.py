"""contact_calibration.py - V5 跨指接触标定。

职责：采集跨指 FULL 触觉接触事件，反解每关节 5 个几何自由度 + counts->角度 k/b，
并生成可下发到 P4 的标定结果/LUT 重标和只读模型修正 JSON。
线程：采集由 UI 刷新线程调用；solve 为纯函数，可在 worker 进程运行。
输入：application_snapshot 字典、当前 21 关节 k/b 初值、URDF。
输出：接触事件、k/b、关节原点修正、RMS 与持久化 JSON。
状态所有权：调用方持有事件和结果；本模块只读资源文件。
禁止依赖：禁止导入 UI、通信、协议或直接操作设备。
"""

from __future__ import annotations

import hashlib
import json
import math
import struct
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from desktop.paths import (CALIB_ACTIVE_SNAPSHOT_PATH, DATA_DIR, DEFAULT_URDF_PATH,
                           atomic_write_json)

from .angle_mapping import get_spec
from .kinematics import Kinematics, decompose_origin
from .tactile_geometry import GEOMETRY_PATH, module_points

CORRECTION_PATH = DATA_DIR / "contact_calibration.json"
ENCODER_MODULO = 16384


def _f32(value):
    return struct.unpack("=f", struct.pack("=f", float(value)))[0]


def _sha256(path):
    try:
        return hashlib.sha256(Path(path).read_bytes()).hexdigest()
    except OSError:
        return ""


def _signed_counts(raw, manual_min, manual_max):
    """与 P4 signedDirectedCoordinateFromWindowStart 完全一致。"""
    span = (int(manual_max) - int(manual_min)) % ENCODER_MODULO
    if span <= 0 or span >= ENCODER_MODULO:
        return None
    forward = (int(raw) - int(manual_min)) % ENCODER_MODULO
    if forward <= span:
        return forward
    below = ENCODER_MODULO - forward
    above = forward - span
    return -below if below < above else forward


@dataclass
class CaptureConfig:
    force_min: float = 50.0
    weight_min: float = 8.0
    static_deg: float = 0.1
    cooldown_s: float = 0.5
    max_events: int = 5000


@dataclass
class ContactRecorder:
    config: CaptureConfig = field(default_factory=CaptureConfig)
    events: list = field(default_factory=list)
    skips: dict = field(default_factory=dict)
    previous_theta: list | None = None
    last_pose: dict = field(default_factory=dict)
    last_time: dict = field(default_factory=dict)

    def reset(self):
        self.events.clear()
        self.skips.clear()
        self.previous_theta = None
        self.last_pose.clear()
        self.last_time.clear()

    def _skip(self, reason):
        self.skips[reason] = self.skips.get(reason, 0) + 1

    def observe(self, snapshot, now=None):
        now = time.time() if now is None else float(now)
        tactile = snapshot.get("tactile") or {}
        rows = snapshot.get("joints") or []
        if snapshot.get("control_mode") != "IDLE":
            return self._skip("not_idle")
        if tactile.get("source") != "S3_SERIAL" or tactile.get("level") != "FULL":
            return self._skip("not_full")
        theta, counts = [], []
        for row in rows:
            angle, raw = row.get("angle"), row.get("raw")
            if (angle is None or raw is None or not row.get("manual_valid")):
                return self._skip("stale_joint")
            coordinate = _signed_counts(raw, row.get("manual_min"), row.get("manual_max"))
            if coordinate is None:
                return self._skip("stale_joint")
            theta.append(float(angle))
            counts.append(coordinate)
        if len(theta) != len(rows) or len(theta) != 21:
            return self._skip("stale_joint")
        if self.previous_theta is None:
            self.previous_theta = theta
            return self._skip("warmup")
        if max(abs(a - b) for a, b in zip(theta, self.previous_theta)) > self.config.static_deg:
            self.previous_theta = theta
            return self._skip("moving")
        self.previous_theta = theta

        active = []
        for module in tactile.get("modules") or []:
            if not module.get("valid"):
                continue
            force = module.get("force")
            taxels = module.get("taxels")
            if force is None or taxels is None:
                continue
            magnitude = float(np.linalg.norm(force))
            weights = np.linalg.norm(np.asarray(taxels, float), axis=2).reshape(-1)
            points = module_points(int(module["module"]))
            if len(weights) != len(points) or float(weights.sum()) < self.config.weight_min:
                continue
            if magnitude < self.config.force_min:
                continue
            centroid = (points * weights[:, None]).sum(axis=0) / weights.sum()
            active.append((int(module["finger"]), int(module["module"]),
                           centroid, magnitude))
        if len(active) != 2:
            return self._skip("multi_module")
        first, second = active
        if first[0] == second[0]:
            return self._skip("same_finger")
        pair = tuple(sorted(((first[0], first[1]), (second[0], second[1]))))
        pose = np.asarray(theta)
        previous = self.last_pose.get(pair)
        if previous is not None and np.max(np.abs(pose - previous)) < 0.5:
            return self._skip("duplicate")
        if now - self.last_time.get(pair, -1e9) < self.config.cooldown_s:
            return self._skip("cooldown")
        self.last_pose[pair] = pose
        self.last_time[pair] = now
        self.events.append({
            "t": now,
            "c": counts,
            "a": {"finger": first[0], "segment": first[1],
                  "point": first[2].tolist(), "force": first[3]},
            "b": {"finger": second[0], "segment": second[1],
                  "point": second[2].tolist(), "force": second[3]},
        })
        if len(self.events) > self.config.max_events:
            self.events.pop(0)
        return None


def _event_arrays(events):
    c = np.asarray([event["c"] for event in events], float)
    index_a = np.asarray([event["a"]["finger"] * 3 + event["a"]["segment"]
                          for event in events], int)
    index_b = np.asarray([event["b"]["finger"] * 3 + event["b"]["segment"]
                          for event in events], int)
    point_a = np.asarray([event["a"]["point"] for event in events], float)
    point_b = np.asarray([event["b"]["point"] for event in events], float)
    return c, index_a, index_b, point_a, point_b


def _initial_mappings():
    """URDF 限制 + Manual 4000 counts 的初始 k/b；仅用于没有现成 result 的场景。"""
    mappings = []
    for joint in range(21):
        low, high = get_spec(joint).urdf_range_deg
        mappings.append({"k": (high - low) / 4000.0, "b": low})
    return mappings


def _model_angles(counts, k, b):
    return counts * np.asarray(k)[None, :] + np.asarray(b)[None, :]


def solve(events, mappings=None, *, urdf_path=DEFAULT_URDF_PATH,
          iterations=20, max_events=300):
    """纯求解器：返回 k/b、几何修正、RMS 和可观测性标记。"""
    events = list(events)[:max_events]
    if len(events) < 20:
        raise ValueError("need at least 20 contact events")
    model = Kinematics(urdf_path)
    mappings = list(mappings or _initial_mappings())
    if len(mappings) != 21:
        raise ValueError("need 21 k/b mappings")
    k0 = np.asarray([float(item["k"]) for item in mappings])
    b0 = np.asarray([float(item["b"]) for item in mappings])
    if np.any(k0 <= 0) or not np.isfinite(k0).all() or not np.isfinite(b0).all():
        raise ValueError("invalid initial k/b")

    c, index_a, index_b, point_a, point_b = _event_arrays(events)
    count = len(events)
    point_a_h = np.concatenate([point_a, np.ones((count, 1))], axis=1)
    point_b_h = np.concatenate([point_b, np.ones((count, 1))], axis=1)
    names = model.names

    def unpack(x):
        geometry = x.reshape(21, 7)
        return geometry

    def compose(x):
        origins = {}
        for joint, name in enumerate(names):
            t = x[joint * 7:joint * 7 + 3]
            alpha, beta = x[joint * 7 + 3], x[joint * 7 + 4]
            if np.any(t) or alpha or beta:
                origins[name] = model.correction_origin(name, t, alpha, beta)
        return origins

    def residual(x):
        geometry = unpack(x)
        angles = _model_angles(c, geometry[:, 5], geometry[:, 6])
        transforms = model.sensor_transforms(angles, compose(x))
        pa = np.einsum("nij,nj->ni", transforms[np.arange(count), index_a], point_a_h)
        pb = np.einsum("nij,nj->ni", transforms[np.arange(count), index_b], point_b_h)
        return (pa - pb)[:, :3].reshape(-1)

    # 剔除明显离群事件（用初值残差）。
    nominal = residual(np.concatenate([np.zeros((21, 5)), k0[:, None], b0[:, None]],
                                      axis=1).reshape(-1))
    norms = np.linalg.norm(nominal.reshape(-1, 3), axis=1)
    median = float(np.median(norms))
    keep = np.ones(count, dtype=bool)
    if median > 1e-6:
        keep = norms <= 3.0 * median
    c, index_a, index_b = c[keep], index_a[keep], index_b[keep]
    point_a, point_b = point_a[keep], point_b[keep]
    point_a_h = np.concatenate([point_a, np.ones((len(c), 1))], axis=1)
    point_b_h = np.concatenate([point_b, np.ones((len(c), 1))], axis=1)
    count = len(c)
    if count < 20:
        raise ValueError("too few contact events after outlier rejection")

    def residual(x):
        geometry = unpack(x)
        angles = _model_angles(c, geometry[:, 5], geometry[:, 6])
        transforms = model.sensor_transforms(angles, compose(x))
        pa = np.einsum("nij,nj->ni", transforms[np.arange(count), index_a], point_a_h)
        pb = np.einsum("nij,nj->ni", transforms[np.arange(count), index_b], point_b_h)
        return (pa - pb)[:, :3].reshape(-1)

    x0 = np.concatenate([np.zeros((21, 5)), k0[:, None], b0[:, None]],
                        axis=1).reshape(-1)
    steps = np.empty(21 * 7)
    steps[0::7] = steps[1::7] = steps[2::7] = 1e-5
    steps[3::7] = steps[4::7] = 1e-5
    steps[5::7] = 1e-4
    steps[6::7] = 1e-4

    # ponytail: 关节链里“无触觉模块的中间 link”使其相邻关节原点存在规范自由度
    # （A 关节的原点可搬到下一关节）。147 参数接触 Jacobian 实测 rank≈129；
    # 这里用朝初值的 ridge 取最小修正解，未观测分量保持 joint calibration 的
    # k/b 和零几何。若以后需要唯一分解，再给相邻关节加相对变换参数化。
    before = residual(x0)
    cost = float(before @ before)
    x = x0.copy()
    lam = 1e-3
    for _ in range(iterations):
        r = residual(x)
        jacobian = np.empty((len(r), len(x)))
        for column in range(len(x)):
            delta = steps[column]
            xp = x.copy()
            xp[column] += delta
            xm = x.copy()
            xm[column] -= delta
            jacobian[:, column] = (residual(xp) - residual(xm)) / (2.0 * delta)
        # 参数量纲差异极大（t~1e-4 m, k~1e-2, b~1e2）；按列范数归一化后再解 LM。
        column_scale = np.linalg.norm(jacobian, axis=0)
        column_scale = np.maximum(column_scale, 1e-6)
        scaled = jacobian / column_scale[None, :]
        improved = False
        for _attempt in range(5):
            rows = np.vstack([scaled, math.sqrt(lam) * np.eye(len(x))])
            rhs = np.concatenate([-r, -math.sqrt(lam) * (x - x0) * column_scale])
            step_scaled = np.linalg.lstsq(rows, rhs, rcond=None)[0]
            step = step_scaled / column_scale
            trial = x + step
            trial_r = residual(trial)
            trial_cost = float(trial_r @ trial_r)
            if trial_cost < cost:
                x, cost, improved = trial, trial_cost, True
                # 不把 ridge 降到 0：规范零空间分量保持贴近初值。
                lam = max(lam / 3.0, 1e-6)
                break
            lam *= 10.0
        if not improved or float(np.linalg.norm(step)) < 1e-9:
            break

    geometry = unpack(x)
    origins = {}
    rejected = []
    for joint, name in enumerate(names):
        t = geometry[joint, :3]
        alpha, beta = geometry[joint, 3], geometry[joint, 4]
        k, b = geometry[joint, 5], geometry[joint, 6]
        bad = (abs(alpha) > math.radians(3.0) or abs(beta) > math.radians(3.0)
               or np.linalg.norm(t) > 0.005
               or not (0.9 * k0[joint] <= k <= 1.1 * k0[joint])
               or abs(b - b0[joint]) > 5.0)
        if bad:
            rejected.append(joint)
            x[joint * 7:joint * 7 + 7] = x0[joint * 7:joint * 7 + 7]
    geometry = unpack(x)
    for joint, name in enumerate(names):
        t = geometry[joint, :3]
        alpha, beta = geometry[joint, 3], geometry[joint, 4]
        if np.any(t) or alpha or beta:
            origins[name] = model.correction_origin(name, t, alpha, beta)
    after = residual(x)
    before_mm = float(np.sqrt(np.mean(np.sum(before.reshape(-1, 3) ** 2, axis=1))) * 1000.0)
    after_mm = float(np.sqrt(np.mean(np.sum(after.reshape(-1, 3) ** 2, axis=1))) * 1000.0)

    theta0 = _model_angles(c, k0, b0)
    joint_events = []
    for joint in range(21):
        column = theta0[:, joint]
        joint_events.append(int(np.sum(np.abs(column - np.median(column)) > 2.0)))
    weak = [joint for joint, count_events in enumerate(joint_events) if count_events < 3]
    return {
        "names": list(names),
        "params": geometry.tolist(),
        "k": geometry[:, 5].tolist(),
        "b": geometry[:, 6].tolist(),
        "origins": {
            name: {"xyz": list(decompose_origin(matrix)[0]),
                   "rpy": list(decompose_origin(matrix)[1])}
            for name, matrix in origins.items()
        },
        "rms_before_mm": before_mm,
        "rms_after_mm": after_mm,
        "events": count,
        "joint_events": joint_events,
        "weak": weak,
        "rejected": rejected,
    }


def relabel_lut(result, old_k, old_b, new_k, new_b):
    """k/b 变化时对 LUT 角度轴做仿射重标；counts<->motor 关系不变。"""
    if old_k <= 0 or not math.isfinite(old_k):
        raise ValueError("invalid old k")
    scale = new_k / old_k
    offset = new_b - scale * old_b
    updated = dict(result)
    updated["tendonLutAngleStartDeg"] = _f32(
        scale * float(result.get("tendonLutAngleStartDeg", 0.0)) + offset)
    updated["tendonLutAngleStepDeg"] = _f32(
        scale * float(result.get("tendonLutAngleStepDeg", 0.0)))
    updated["angleK"] = float(new_k)
    updated["angleB"] = float(new_b)
    updated["angleMin"] = float(new_b)
    span = int(result.get("manualSpanCounts", 0) or 0)
    if span > 0:
        updated["angleMax"] = _f32(new_k * span + new_b)
    return updated


def build_updated_bundle(solution, snapshot_path):
    """把解出的 k/b 写进活动 bundle 的 21 个 result，返回可导入的 dict。"""
    from .calibration_snapshot import CalibrationSnapshotStore, seal_snapshot

    bundle = CalibrationSnapshotStore(snapshot_path).load()
    results = list(bundle.get("joint_calib_results") or [])
    for joint, name in enumerate(solution["names"]):
        result = results[joint] if joint < len(results) else None
        if not isinstance(result, dict) or not result.get("success"):
            continue
        old_k = float(result.get("angleK", 0.0) or 0.0)
        old_b = float(result.get("angleB", 0.0) or 0.0)
        if old_k <= 0:
            continue
        results[joint] = relabel_lut(result, old_k, old_b,
                                     solution["k"][joint], solution["b"][joint])
    bundle["joint_calib_results"] = results
    return seal_snapshot(bundle)


def save_solution(path, solution, *, urdf_path=DEFAULT_URDF_PATH,
                  tactile_path=GEOMETRY_PATH, calibration_path=CALIB_ACTIVE_SNAPSHOT_PATH):
    payload = {
        "version": 1,
        "saved_at": time.time(),
        "events": int(solution.get("events", 0)),
        "urdf_sha256": _sha256(urdf_path),
        "tactile_sha256": _sha256(tactile_path),
        "calibration_sha256": _sha256(calibration_path),
        "rms_before_mm": float(solution.get("rms_before_mm", 0.0)),
        "rms_after_mm": float(solution.get("rms_after_mm", 0.0)),
        "joints": {
            name: {
                "xyz": list(solution["origins"].get(name, {}).get("xyz", [0.0, 0.0, 0.0])),
                "rpy": list(solution["origins"].get(name, {}).get("rpy", [0.0, 0.0, 0.0])),
                "k": float(solution["k"][index]),
                "b": float(solution["b"][index]),
                "events": int(solution["joint_events"][index]),
                "weak": index in solution.get("weak", []),
                "rejected": index in solution.get("rejected", []),
            }
            for index, name in enumerate(solution["names"])
        },
    }
    atomic_write_json(path, payload)
    return payload


def load_solution(path=CORRECTION_PATH, *, urdf_path=DEFAULT_URDF_PATH,
                  tactile_path=GEOMETRY_PATH, calibration_path=CALIB_ACTIVE_SNAPSHOT_PATH):
    source = Path(path)
    try:
        payload = json.loads(source.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return None, f"missing or invalid contact calibration: {exc}"
    if int(payload.get("version", 0) or 0) != 1:
        return None, "unsupported contact calibration version"
    for key, expected in (
        ("urdf_sha256", _sha256(urdf_path)),
        ("tactile_sha256", _sha256(tactile_path)),
        ("calibration_sha256", _sha256(calibration_path)),
    ):
        if str(payload.get(key, "")) != expected:
            return None, f"contact calibration is stale ({key})"
    joints = payload.get("joints")
    if not isinstance(joints, dict):
        return None, "contact calibration has no joints"
    return payload, None


def active_origins(path=CORRECTION_PATH, **kwargs):
    """Hand 页使用：返回 {joint_name: {'xyz','rpy'}} 或空字典。"""
    payload, _reason = load_solution(path, **kwargs)
    if payload is None:
        return {}
    return {name: value for name, value in payload["joints"].items()
            if not value.get("rejected")}
