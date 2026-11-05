"""contact_calibration.py - V6 手动接触标定。

职责：用人工标注的“接触 / 接近但未接触”位姿，二分类拟合每关节 k/b + 5 个原点修正；
把结果写进活动标定包的 contact_calibration 字段（Motor Calibration 下发）。
线程：采集由 UI 刷新线程调用；solve 为纯函数，可在 worker 进程运行。
输入：application_snapshot 字典、21 关节 k/b 基值、URDF。
输出：k/b、关节原点修正、分类统计与标定包字段。
状态所有权：调用方持有事件和结果；本模块只读资源文件。
禁止依赖：禁止导入 UI、通信、协议或直接操作设备。
"""

from __future__ import annotations

import math
import struct
from functools import lru_cache
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from desktop.paths import CALIB_ACTIVE_SNAPSHOT_PATH, DEFAULT_URDF_PATH

from .angle_mapping import get_spec
from .calibration_snapshot import CalibrationSnapshotStore
from .kinematics import Kinematics, decompose_origin
from .tactile_geometry import module_points

# 活动标定包里的 contact 字段；不存在就是没做过 contact 标定。
CONTACT_KEY = "contact_calibration"
ENCODER_MODULO = 16384
COUNTS_PER_DEG = ENCODER_MODULO / 360.0

# 接触判定旋钮：点云最近点间隙在真接触时约 0.2–1.5 mm，margin 吸收离散化偏差。
CONTACT_MARGIN_M = 0.0008
CONTACT_SLOPE_M = 0.0004
# 先验把没有接触覆盖的方向钉在 Joint Calibration，比钳位带更紧。
PRIOR_TRANSLATION_M = 0.002
PRIOR_ROTATION_RAD = math.radians(1.0)
PRIOR_K_REL = 0.02
PRIOR_B_DEG = 2.0
# 安全钳位：与标定包加载门禁一致。
CLAMP_TRANSLATION_M = 0.005
CLAMP_ROTATION_RAD = math.radians(3.0)
CLAMP_K_REL = 0.10
CLAMP_B_DEG = 5.0
SOLVE_ITERATIONS = 6

# finger 顺序 G0..G4 = 小指/无名指/中指/食指/拇指。
FINGER_JOINTS = ((0, 1, 2, 3), (4, 5, 6, 7), (8, 9, 10, 11),
                 (12, 13, 14, 15), (16, 17, 18, 19, 20))
JOINT_FINGER = tuple(
    finger for finger, joints in enumerate(FINGER_JOINTS) for _joint in joints)
# 接触体是物理手指 link，不是挂在它上面的 tactile sensor link。
CONTACT_LINK_NAMES = ('finger_tip', 'distal_phalanx', 'proximal_phalanx')
CONTACT_LINKS = tuple(
    f"link_{CONTACT_LINK_NAMES[segment]}_{5 - finger}"
    for finger in range(5) for segment in range(3))
# 候选接触对：指尖 finger 对另一指 other 的 tip/middle/base 段。
CONTACT_PAIRS = tuple(
    (finger, other, segment)
    for finger in range(5) for other in range(5) if other != finger
    for segment in range(3))
# 每关节 7 个参数：t3, α, β, k, b；数值差分步长。
_PARAM_COLUMNS = np.arange(21 * 7).reshape(21, 7)
GEOMETRY_COLUMNS = _PARAM_COLUMNS[:, :5].reshape(-1)
ENCODER_COLUMNS = _PARAM_COLUMNS[:, 5:].reshape(-1)
_STEP_BY_COLUMN = np.array([1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-3])


def _f32(value):
    return struct.unpack("=f", struct.pack("=f", float(value)))[0]


def signed_counts(raw, manual_min, manual_max):
    """与 P4 signedDirectedCoordinateFromWindowStart 完全一致。"""
    if raw is None or manual_min is None or manual_max is None:
        return None
    try:
        span = (int(manual_max) - int(manual_min)) % ENCODER_MODULO
        if span <= 0 or span >= ENCODER_MODULO:
            return None
        forward = (int(raw) - int(manual_min)) % ENCODER_MODULO
    except (TypeError, ValueError):
        return None
    if forward <= span:
        return forward
    below = ENCODER_MODULO - forward
    above = forward - span
    return -below if below < above else forward


def initial_mappings(manual_min=None, manual_max=None):
    """Joint Calibration 基值：k = URDF span / Manual span, b = URDF min。"""
    mappings = []
    for joint in range(21):
        low, high = get_spec(joint).urdf_range_deg
        span = 4000
        if manual_min is not None and manual_max is not None:
            try:
                candidate = (int(manual_max[joint]) - int(manual_min[joint])) % ENCODER_MODULO
            except (TypeError, ValueError):
                candidate = 0
            if 0 < candidate < ENCODER_MODULO:
                span = candidate
        mappings.append({"k": (high - low) / span, "b": low})
    return mappings


def model_angles(counts, k, b):
    return np.asarray(counts, float) * np.asarray(k, float) + np.asarray(b, float)


def counts_from_snapshot(snapshot):
    """返回 (21 个 signed counts, 失败原因)；只读 P4 的 raw / Manual 端点。"""
    rows = snapshot.get("joints") or []
    if len(rows) != 21:
        return None, "joint data is incomplete"
    counts = []
    for row in rows:
        raw = row.get("raw")
        if raw is None:
            return None, "encoder raw data is not fresh"
        if not row.get("manual_valid"):
            return None, "a joint is missing Manual Min/Max"
        coordinate = signed_counts(raw, row.get("manual_min"), row.get("manual_max"))
        if coordinate is None:
            return None, "a joint's Manual Min/Max is invalid"
        counts.append(float(coordinate))
    return counts, None


@lru_cache(maxsize=1)
def _sensor_offsets(urdf_path):
    """物理 link → sensor link 的固定平移；测点用它换到物理 link 坐标系。"""
    root = ET.parse(Path(urdf_path)).getroot()
    offsets = {}
    for joint in root.findall('joint'):
        child = joint.find('child').attrib['link']
        if not child.endswith('_skeleton_sensor'):
            continue
        parent = joint.find('parent').attrib['link']
        if parent not in CONTACT_LINKS:
            continue
        origin = joint.find('origin')
        xyz = (origin.get('xyz') if origin is not None else None) or '0 0 0'
        offsets[CONTACT_LINKS.index(parent) % 3] = np.fromstring(xyz, sep=' ')
    if set(offsets) != {0, 1, 2}:
        raise ValueError('URDF is missing the tactile sensor mounting joints')
    return offsets


@lru_cache(maxsize=1)
def _clouds(urdf_path):
    """15 个物理 link 的表面点（齐次坐标）。"""
    offsets = _sensor_offsets(urdf_path)
    return tuple(np.column_stack([module_points(segment) + offsets[segment],
                                  np.ones(len(module_points(segment)))])
                 for segment in range(3))


def _transforms(model, geometry, counts):
    angles = model_angles(counts, geometry[:, 5], geometry[:, 6])
    origins = {}
    for joint, name in enumerate(model.names):
        translation = geometry[joint, :3]
        alpha, beta = geometry[joint, 3], geometry[joint, 4]
        if np.any(translation) or alpha or beta:
            origins[name] = model.correction_origin(
                name, translation, alpha, beta)
    return model.link_transforms(angles, origins, CONTACT_LINKS)


def _pair_distances(transforms, clouds, finger, other, segment):
    tip = np.einsum("nij,mj->nmi", transforms[:, finger * 3], clouds[0])
    target = np.einsum("nij,mj->nmi",
                       transforms[:, other * 3 + segment], clouds[segment])
    return np.linalg.norm(tip[:, :, None, :] - target[:, None, :, :], axis=-1)


def _scan_pairs(transforms, clouds, candidates):
    """在一组候选对里找每个位姿的最近对及其最近点。"""
    count = transforms.shape[0]
    gaps = np.full(count, np.inf)
    pairs = np.zeros((count, 3), dtype=int)
    point_a = np.zeros(count, dtype=int)
    point_b = np.zeros(count, dtype=int)
    for finger, other, segment in candidates:
        distances = _pair_distances(
            transforms, clouds, finger, other, segment)
        gap = distances.min(axis=(1, 2))
        better = gap < gaps
        if not np.any(better):
            continue
        gaps[better] = gap[better]
        pairs[better] = (finger, other, segment)
        nearest = distances.reshape(count, -1).argmin(axis=1)
        point_a[better], point_b[better] = np.unravel_index(
            nearest[better], distances.shape[1:])
    return gaps, pairs, point_a, point_b


def _fingertip_candidates(fingertip):
    return tuple(
        (fingertip, other, segment)
        for other in range(5) if other != fingertip
        for segment in range(3))


def _infer_pairs(transforms, clouds, events):
    """按每个事件给的候选接触对（或 fingertip / 全局）分组推断。"""
    count = transforms.shape[0]
    gaps = np.empty(count)
    pairs = np.empty((count, 3), dtype=int)
    point_a = np.empty(count, dtype=int)
    point_b = np.empty(count, dtype=int)
    groups = {}
    for row, event in enumerate(events):
        recorded = event.get("pair")
        if recorded is not None:
            try:
                finger, other, segment = (int(value) for value in recorded)
            except (TypeError, ValueError) as exc:
                raise ValueError("invalid contact pair") from exc
            if (not 0 <= finger < 5 or not 0 <= other < 5
                    or other == finger or not 0 <= segment < 3):
                raise ValueError("invalid contact pair")
            candidates = ((finger, other, segment),)
        else:
            fingertip = event.get("fingertip")
            if fingertip is None:
                candidates = CONTACT_PAIRS
            else:
                try:
                    fingertip = int(fingertip)
                except (TypeError, ValueError) as exc:
                    raise ValueError("invalid fingertip") from exc
                if not 0 <= fingertip < 5:
                    raise ValueError("invalid fingertip")
                candidates = _fingertip_candidates(fingertip)
        groups.setdefault(candidates, []).append(row)
    for candidates, rows in groups.items():
        group_gaps, group_pairs, group_a, group_b = _scan_pairs(
            transforms[rows], clouds, candidates)
        gaps[rows] = group_gaps
        pairs[rows] = group_pairs
        point_a[rows] = group_a
        point_b[rows] = group_b
    return gaps, pairs, point_a, point_b


def infer_pair(counts, mappings, fingertip, *, urdf_path=DEFAULT_URDF_PATH):
    """按 Joint Calibration 模型推断 fingertip 碰到的目标指/段（另外 4 指 × 3 段）。"""
    if len(mappings) != 21:
        raise ValueError("need 21 k/b mappings")
    values = np.asarray(counts, float)
    if values.shape != (21,) or not np.isfinite(values).all():
        raise ValueError("need 21 finite counts")
    try:
        fingertip = int(fingertip)
    except (TypeError, ValueError) as exc:
        raise ValueError("invalid fingertip") from exc
    if not 0 <= fingertip < 5:
        raise ValueError("invalid fingertip")
    model = Kinematics(urdf_path)
    geometry = np.zeros((21, 7))
    geometry[:, 5] = [float(item["k"]) for item in mappings]
    geometry[:, 6] = [float(item["b"]) for item in mappings]
    transforms = _transforms(model, geometry, values[None, :])
    gaps, pairs, _point_a, _point_b = _scan_pairs(
        transforms, _clouds(urdf_path), _fingertip_candidates(fingertip))
    return (int(pairs[0, 0]), int(pairs[0, 1]), int(pairs[0, 2]),
            float(gaps[0]) * 1000.0)


def _prior(geometry, k0, b0):
    return np.concatenate([
        geometry[:, :3].reshape(-1) / PRIOR_TRANSLATION_M,
        geometry[:, 3:5].reshape(-1) / PRIOR_ROTATION_RAD,
        (geometry[:, 5] / k0 - 1.0) / PRIOR_K_REL,
        (geometry[:, 6] - b0) / PRIOR_B_DEG,
    ])


def _revert_out_of_range(geometry, base):
    rejected = []
    for joint in range(21):
        translation = geometry[joint, :3]
        alpha, beta = geometry[joint, 3], geometry[joint, 4]
        k, b = geometry[joint, 5], geometry[joint, 6]
        if (np.linalg.norm(translation) > CLAMP_TRANSLATION_M
                or abs(alpha) > CLAMP_ROTATION_RAD
                or abs(beta) > CLAMP_ROTATION_RAD
                or not (1.0 - CLAMP_K_REL <= k / base[joint, 5]
                        <= 1.0 + CLAMP_K_REL)
                or abs(b - base[joint, 6]) > CLAMP_B_DEG):
            rejected.append(joint)
            geometry[joint] = base[joint]
    return rejected


def _levenberg_marquardt(residual, x, steps, iterations):
    r = residual(x)
    cost = float(r @ r)
    damping = 1e-3
    for _ in range(iterations):
        jacobian = np.empty((len(r), len(x)))
        for column in range(len(x)):
            delta = steps[column]
            up = x.copy()
            up[column] += delta
            down = x.copy()
            down[column] -= delta
            jacobian[:, column] = (residual(up) - residual(down)) / (2.0 * delta)
        scale = np.maximum(np.linalg.norm(jacobian, axis=0), 1e-9)
        scaled = jacobian / scale[None, :]
        improved = False
        step = np.zeros(len(x))
        for _attempt in range(6):
            rows = np.vstack([scaled, math.sqrt(damping) * np.eye(len(x))])
            rhs = np.concatenate([-r, np.zeros(len(x))])
            step_scaled = np.linalg.lstsq(rows, rhs, rcond=None)[0]
            step = step_scaled / scale
            trial = x + step
            trial_r = residual(trial)
            trial_cost = float(trial_r @ trial_r)
            if trial_cost < cost:
                x, r, cost = trial, trial_r, trial_cost
                damping = max(damping / 3.0, 1e-7)
                improved = True
                break
            damping *= 10.0
        if not improved or float(np.linalg.norm(step)) < 1e-10:
            break
    return x


def solve(events, mappings, *, urdf_path=DEFAULT_URDF_PATH,
          iterations=SOLVE_ITERATIONS,
          optimize_kb=True, optimize_geometry=False):
    """二分类求解；默认只拟合 k/b，可选关节轴平移/旋转。"""
    parts = []
    if optimize_geometry:
        parts.append(GEOMETRY_COLUMNS)
    if optimize_kb:
        parts.append(ENCODER_COLUMNS)
    if not parts:
        raise ValueError("select at least one correction")
    free = np.concatenate(parts)
    if len(mappings) != 21:
        raise ValueError("need 21 k/b mappings")
    k0 = np.asarray([float(item["k"]) for item in mappings])
    b0 = np.asarray([float(item["b"]) for item in mappings])
    if np.any(k0 <= 0) or not np.isfinite(k0).all() or not np.isfinite(b0).all():
        raise ValueError("invalid base k/b")
    try:
        counts = np.asarray([event["c"] for event in events], float)
        labels = np.asarray([int(event["y"]) for event in events], dtype=int)
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("invalid contact events") from exc
    if (counts.ndim != 2 or counts.shape[1] != 21
            or len(counts) != len(labels) or len(counts) == 0):
        raise ValueError("events need 21 counts and a 0/1 label")
    if not np.isfinite(counts).all() or np.any((labels != 0) & (labels != 1)):
        raise ValueError("invalid event data")
    if not np.any(labels == 1):
        raise ValueError("record at least one contact first")

    model = Kinematics(urdf_path)
    clouds = _clouds(urdf_path)
    base = np.zeros((21, 7))
    base[:, 5] = k0
    base[:, 6] = b0
    gaps, pairs, point_a, point_b = _infer_pairs(
        _transforms(model, base, counts), clouds, events)
    tip_points = clouds[0][point_a]
    target_points = np.asarray([
        clouds[int(pairs[index, 2])][int(point_b[index])]
        for index in range(len(events))])
    link_a = pairs[:, 0] * 3
    link_b = pairs[:, 1] * 3 + pairs[:, 2]
    rows = np.arange(len(events))
    base_x = base.reshape(-1)
    steps = _STEP_BY_COLUMN[free % 7]

    def full_free(free_x):
        x = base_x.copy()
        x[free] = free_x
        return x

    def residual(free_x):
        geometry = full_free(free_x).reshape(21, 7)
        frames = _transforms(model, geometry, counts)
        a = np.einsum("nij,nj->ni", frames[rows, link_a], tip_points)
        b = np.einsum("nij,nj->ni", frames[rows, link_b], target_points)
        gap = np.linalg.norm(a - b, axis=1)
        probability = 1.0 / (1.0 + np.exp(-np.clip(
            (CONTACT_MARGIN_M - gap) / CONTACT_SLOPE_M, -60.0, 60.0)))
        probability = np.clip(probability, 1e-9, 1.0 - 1e-9)
        bce = -np.where(labels == 1,
                        np.log(probability), np.log(1.0 - probability))
        return np.concatenate([np.sqrt(2.0 * bce), _prior(geometry, k0, b0)])

    free_x = _levenberg_marquardt(
        residual, base_x[free], steps, iterations)
    geometry = full_free(free_x).reshape(21, 7).copy()
    rejected = _revert_out_of_range(geometry, base)
    frames = _transforms(model, geometry, counts)
    a = np.einsum("nij,nj->ni", frames[rows, link_a], tip_points)
    b = np.einsum("nij,nj->ni", frames[rows, link_b], target_points)
    fitted = np.linalg.norm(a - b, axis=1)
    contact = labels == 1
    finger_contacts = np.zeros(5, dtype=int)
    for finger in pairs[contact, 0]:
        finger_contacts[int(finger)] += 1
    for finger in pairs[contact, 1]:
        finger_contacts[int(finger)] += 1
    origins = {}
    for joint, name in enumerate(model.names):
        translation = geometry[joint, :3]
        alpha, beta = geometry[joint, 3], geometry[joint, 4]
        if np.any(translation) or alpha or beta:
            xyz, rpy = decompose_origin(
                model.correction_origin(name, translation, alpha, beta))
            origins[name] = {"xyz": list(xyz), "rpy": list(rpy)}
    return {
        "names": list(model.names),
        "params": geometry.tolist(),
        "k": geometry[:, 5].tolist(),
        "b": geometry[:, 6].tolist(),
        "base_k": k0.tolist(),
        "base_b": b0.tolist(),
        "origins": origins,
        "optimized": {"kb": bool(optimize_kb),
                      "geometry": bool(optimize_geometry)},
        "events": int(len(events)),
        "clearance_before_mm": float(np.mean(gaps[contact]) * 1000.0),
        "clearance_after_mm": float(np.mean(fitted[contact]) * 1000.0),
        "contact_pass": int(np.sum(fitted[contact] <= CONTACT_MARGIN_M)),
        "contact_total": int(np.sum(contact)),
        "negative_pass": int(np.sum(fitted[~contact] > CONTACT_MARGIN_M)),
        "negative_total": int(np.sum(~contact)),
        "weak": [joint for joint in range(21)
                 if finger_contacts[JOINT_FINGER[joint]] < 3],
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


def save_contact(snapshot_path, solution):
    """把 k/b、原点修正和 Joint Calibration 基值写进活动标定包。"""
    # ponytail: load→activate 不在同一把锁里；页面只在 IDLE 保存，暂不处理并发丢更新。
    store = CalibrationSnapshotStore(snapshot_path)
    bundle = store.load()
    base_k = solution.get("base_k", solution["k"])
    base_b = solution.get("base_b", solution["b"])
    entries = {}
    for index, name in enumerate(solution["names"]):
        entry = {"k": float(solution["k"][index]),
                 "b": float(solution["b"][index]),
                 "base_k": float(base_k[index]),
                 "base_b": float(base_b[index])}
        origin = solution["origins"].get(name)
        if origin:
            entry["xyz"] = origin["xyz"]
            entry["rpy"] = origin["rpy"]
        entries[name] = entry
    bundle[CONTACT_KEY] = entries
    store.activate(bundle)


def clear_contact(snapshot_path):
    store = CalibrationSnapshotStore(snapshot_path)
    try:
        bundle = store.load()
    except (OSError, ValueError):
        return
    if bundle.pop(CONTACT_KEY, None) is not None:
        store.activate(bundle)


def contact_mappings(payload):
    """Motor Calibration 用：{urdf_joint_name: (k, b)}。"""
    return {name: (float(item["k"]), float(item["b"]))
            for name, item in (payload.get(CONTACT_KEY) or {}).items()}


def active_origins():
    """Hand 页用：从活动标定包读几何修正；没有就返回空。"""
    try:
        bundle = CalibrationSnapshotStore(CALIB_ACTIVE_SNAPSHOT_PATH).load()
    except (OSError, ValueError):
        return {}
    return {name: {"xyz": item["xyz"], "rpy": item["rpy"]}
            for name, item in (bundle.get(CONTACT_KEY) or {}).items()
            if "xyz" in item and "rpy" in item}
