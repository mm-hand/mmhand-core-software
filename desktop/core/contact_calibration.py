"""contact_calibration.py - V7 滑动轨迹接触标定。

职责：把“指尖贴着另一指滑动”的连续轨迹抽稀成接触样本，自动生成 non-contact 样本，
二分类拟合每关节 k/b（可选 5 个原点修正），并把结果写进活动标定包（Motor Calibration 下发）。
线程：采集由 UI 刷新线程调用；trajectory/add_non_contact/solve 为纯函数，可在 worker 进程运行。
输入：P4 encoder_raw 帧、21 关节 Manual 端点、k/b 基值、URDF。
输出：contact/non-contact 事件、k/b、关节原点修正、分类统计与标定包字段。
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
SOLVE_ITERATIONS = 12

# V7 轨迹：按指尖空间位移抽稀；连续性允许量约 3 个采样步长（仿真 E3 选值）。
STEP_M = 0.002
CONTINUITY_ALLOWANCE_M = 0.006
CONTINUITY_SIGMA_M = 0.003
# 程序生成的 non-contact 样本目标间隙；必须明显大于接触 margin。
NONCONTACT_GAP_M = 0.003
NONCONTACT_MIN_GAP_M = 2.0 * CONTACT_MARGIN_M
# 一次轨迹至少保留的接触样本数。
MIN_SAMPLES = 5
# 生成 non-contact 时求间隙梯度的计数差分步长；扰动最多 5°。
GRAD_STEP_COUNTS = 50.0
NONCONTACT_MAX_PERTURB_DEG = 5.0
# 直接间隙损失（可选）：接触样本拉向 0，non-contact 推到至少 NONCONTACT_MIN_GAP_M。
GAP_SIGMA_M = 0.0005

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


def _geometry_from_mappings(mappings):
    geometry = np.zeros((21, 7))
    geometry[:, 5] = [float(item["k"]) for item in mappings]
    geometry[:, 6] = [float(item["b"]) for item in mappings]
    return geometry


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
    transforms = _transforms(model, _geometry_from_mappings(mappings),
                             values[None, :])
    gaps, pairs, _point_a, _point_b = _scan_pairs(
        transforms, _clouds(urdf_path), _fingertip_candidates(fingertip))
    return (int(pairs[0, 0]), int(pairs[0, 1]), int(pairs[0, 2]),
            float(gaps[0]) * 1000.0)


def _tip_positions(clouds, frames):
    """(E, 5, 3) 指尖点云质心在手掌系的位置。"""
    centroid = clouds[0].mean(axis=0)
    return np.stack(
        [np.einsum("nij,j->ni", frames[:, finger * 3], centroid)[:, :3]
         for finger in range(5)], axis=1)


def tip_travel(counts, mappings, *, urdf_path=DEFAULT_URDF_PATH):
    """5 根手指的指尖质心路径长度（米）；UI 在线判定和离线抽稀共用。"""
    if len(mappings) != 21:
        raise ValueError("need 21 k/b mappings")
    values = np.asarray(counts, float)
    if values.ndim != 2 or values.shape[1] != 21 or len(values) < 2:
        raise ValueError("need at least two 21-count frames")
    if not np.isfinite(values).all():
        raise ValueError("invalid counts")
    model = Kinematics(urdf_path)
    frames = _transforms(model, _geometry_from_mappings(mappings), values)
    positions = _tip_positions(_clouds(urdf_path), frames)
    return [float(np.linalg.norm(np.diff(positions[:, finger], axis=0), axis=1).sum())
            for finger in range(5)]


def _checked_manual(manual_min, manual_max):
    if len(manual_min) != 21 or len(manual_max) != 21:
        raise ValueError("need 21 Manual Min/Max endpoints")
    checked = []
    for joint, (low, high) in enumerate(zip(manual_min, manual_max)):
        try:
            span = (int(high) - int(low)) % ENCODER_MODULO
        except (TypeError, ValueError) as exc:
            raise ValueError(f"invalid Manual Min/Max on J{joint:02d}") from exc
        if not 0 < span < ENCODER_MODULO:
            raise ValueError(f"invalid Manual Min/Max on J{joint:02d}")
        checked.append((int(low), int(high)))
    return checked


def trajectory_events(frames, manual_min, manual_max, traj_id=0, *,
                      step_m=STEP_M, min_samples=MIN_SAMPLES,
                      urdf_path=DEFAULT_URDF_PATH):
    """P4 encoder_raw 帧 -> 抽稀后的接触事件；自动选移动指尖。

    frames: [(p4_time_ms, raw_0..raw_20)]，按接收顺序。
    每个事件 {"c": 21 counts, "y": 1, "pair": (f, o, seg), "traj": traj_id}。
    """
    manual = _checked_manual(manual_min, manual_max)
    by_time = {}
    for p4_ms, raw in frames:
        try:
            values = [int(value) for value in raw]
        except (TypeError, ValueError) as exc:
            raise ValueError("invalid encoder frames") from exc
        if len(values) != 21:
            continue
        by_time[int(p4_ms)] = values
    cells = []
    for raw in by_time.values():
        row = [signed_counts(raw[joint], manual[joint][0], manual[joint][1])
               for joint in range(21)]
        if all(value is not None for value in row):
            cells.append(row)
    if len(cells) < 2:
        raise ValueError("not enough valid encoder frames")
    counts = np.asarray(cells, float)
    mappings = initial_mappings([item[0] for item in manual],
                                [item[1] for item in manual])
    model = Kinematics(urdf_path)
    clouds = _clouds(urdf_path)
    all_frames = _transforms(model, _geometry_from_mappings(mappings), counts)
    positions = _tip_positions(clouds, all_frames)
    travel = [float(np.linalg.norm(np.diff(positions[:, finger], axis=0), axis=1).sum())
              for finger in range(5)]
    fingertip = int(np.argmax(travel))
    kept = [0]
    last = positions[0, fingertip]
    for index in range(1, len(counts)):
        point = positions[index, fingertip]
        if float(np.linalg.norm(point - last)) >= step_m:
            kept.append(index)
            last = point
    if kept[-1] != len(counts) - 1:
        kept.append(len(counts) - 1)
    if len(kept) < min_samples:
        raise ValueError("sliding motion is too short; record a longer slide")
    _gaps, pairs, _point_a, _point_b = _scan_pairs(
        all_frames[kept], clouds, _fingertip_candidates(fingertip))
    events = [
        {"c": counts[index].tolist(), "y": 1,
         "pair": (int(pairs[row, 0]), int(pairs[row, 1]), int(pairs[row, 2])),
         "traj": int(traj_id)}
        for row, index in enumerate(kept)]
    info = {
        "fingertip": fingertip,
        "contacts": len(events),
        "frames": len(by_time),
        "valid_frames": len(counts),
        "travel_mm": [round(value * 1000.0, 2) for value in travel],
    }
    return events, info


def _pair_gap(model, clouds, geometry, counts, pair):
    frames = _transforms(model, geometry, np.asarray(counts, float)[None, :])
    return float(_pair_distances(frames, clouds, pair[0], pair[1], pair[2]).min())


def _clamped_counts(counts, joints, limits, delta, max_delta):
    candidate = counts.copy()
    for joint in joints:
        low, high = limits[joint]
        change = min(max_delta[joint], max(-max_delta[joint], delta[joint]))
        candidate[joint] = min(high, max(low, counts[joint] + change))
    return candidate


def add_non_contact_samples(events, mappings, manual_min, manual_max, *,
                            gap_m=NONCONTACT_GAP_M,
                            urdf_path=DEFAULT_URDF_PATH):
    """在接触位姿上沿“增大目标对间隙”的方向退开，生成 y=0 样本。"""
    _checked_manual(manual_min, manual_max)
    if len(mappings) != 21:
        raise ValueError("need 21 k/b mappings")
    model = Kinematics(urdf_path)
    clouds = _clouds(urdf_path)
    base = _geometry_from_mappings(mappings)
    limits = []
    for joint in range(21):
        low, high = get_spec(joint).urdf_range_deg
        k = float(mappings[joint]["k"])
        b = float(mappings[joint]["b"])
        if k <= 0 or not math.isfinite(k) or not math.isfinite(b):
            raise ValueError("invalid base k/b")
        limits.append(((low - b) / k, (high - b) / k))
    out = list(events)
    created = skipped = 0
    for event in events:
        pair = event.get("pair")
        if pair is None:
            raise ValueError("contact events need a pair")
        pair = (int(pair[0]), int(pair[1]), int(pair[2]))
        counts = np.asarray(event["c"], float)
        if counts.shape != (21,):
            raise ValueError("contact events need 21 counts")
        joints = FINGER_JOINTS[pair[0]]
        gradient = np.zeros(21)
        for joint in joints:
            up = counts.copy()
            down = counts.copy()
            up[joint] += GRAD_STEP_COUNTS
            down[joint] -= GRAD_STEP_COUNTS
            gradient[joint] = (
                _pair_gap(model, clouds, base, up, pair)
                - _pair_gap(model, clouds, base, down, pair)
            ) / (2.0 * GRAD_STEP_COUNTS)
        norm = float(np.linalg.norm(gradient))
        if norm <= 1e-12:
            skipped += 1
            continue
        direction = gradient / norm
        current = _pair_gap(model, clouds, base, counts, pair)
        max_delta = {
            joint: NONCONTACT_MAX_PERTURB_DEG / float(mappings[joint]["k"])
            for joint in joints}

        def step(scale):
            candidate = _clamped_counts(
                counts, joints, limits, direction * scale, max_delta)
            return candidate, _pair_gap(model, clouds, base, candidate, pair)

        # 沿梯度方向做一维搜索：在每关节最多 3° 的前提下，找最小步长达到 gap_m。
        wanted = max(gap_m - current, NONCONTACT_MIN_GAP_M * 0.5)
        limit_scale = min(
            (max_delta[joint] / abs(direction[joint])
             for joint in joints if abs(direction[joint]) > 1e-12),
            default=0.0)
        hi = min(max(1.0, wanted / max(norm, 1e-12)), limit_scale)
        candidate, gap = step(hi)
        if gap < gap_m:
            skipped += 1
            continue
        lo = 0.0
        for _attempt in range(8):
            mid = 0.5 * (lo + hi)
            trial, trial_gap = step(mid)
            if trial_gap >= gap_m:
                hi, candidate, gap = mid, trial, trial_gap
            else:
                lo = mid
        if gap < NONCONTACT_MIN_GAP_M:
            skipped += 1
            continue
        out.append({"c": candidate.tolist(), "y": 0, "pair": pair,
                    "traj": event.get("traj")})
        created += 1
    return out, {"noncontacts": created, "skipped": skipped}


def build_trajectory(frames, manual_min, manual_max, traj_id=0, *,
                     step_m=STEP_M, gap_m=NONCONTACT_GAP_M,
                     urdf_path=DEFAULT_URDF_PATH):
    """Stop 的 worker 入口：raw 帧 -> contact + non-contact 事件。"""
    contacts, info = trajectory_events(
        frames, manual_min, manual_max, traj_id, step_m=step_m,
        urdf_path=urdf_path)
    mappings = initial_mappings(manual_min, manual_max)
    events, close_info = add_non_contact_samples(
        contacts, mappings, manual_min, manual_max, gap_m=gap_m,
        urdf_path=urdf_path)
    info.update(close_info)
    return {"events": events, "info": info}


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


def _continuity_pairs(events):
    """同一 traj 内、列表相邻的 y=1 事件下标对。"""
    pairs, last = [], {}
    for index, event in enumerate(events):
        if int(event.get("y", 0)) != 1:
            continue
        traj = event.get("traj")
        if traj is None:
            last.clear()
            continue
        if traj in last:
            pairs.append((last[traj], index))
        last[traj] = index
    return np.asarray(pairs, dtype=int).reshape(-1, 2)


def _continuity_excess(points, pairs, allowance):
    if len(pairs) == 0:
        return 0.0
    delta = np.linalg.norm(points[pairs[:, 1]] - points[pairs[:, 0]], axis=1)
    return float(np.mean(np.maximum(0.0, delta - allowance)) * 1000.0)


def _continuity_residual(points, pairs, allowance, sigma):
    if len(pairs) == 0:
        return np.empty(0)
    delta = np.linalg.norm(points[pairs[:, 1]] - points[pairs[:, 0]], axis=1)
    return np.maximum(0.0, delta - allowance) / sigma


def _levenberg_marquardt(residual, x, steps, iterations):
    r = residual(x)
    cost = float(r @ r)
    damping = 1e-3
    used = 0
    for used in range(1, iterations + 1):
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
    return x, used


def _event_pairs(events, model, clouds, base_geometry, counts):
    """每个事件的 (指尖, 目标指, 段)；缺失时用起始模型的最近对补一次。"""
    pairs = np.zeros((len(events), 3), dtype=int)
    missing = []
    for index, event in enumerate(events):
        recorded = event.get("pair")
        if recorded is None:
            missing.append(index)
            continue
        try:
            finger, other, segment = (int(value) for value in recorded)
        except (TypeError, ValueError) as exc:
            raise ValueError("invalid contact pair") from exc
        if (not 0 <= finger < 5 or not 0 <= other < 5
                or other == finger or not 0 <= segment < 3):
            raise ValueError("invalid contact pair")
        pairs[index] = (finger, other, segment)
    if missing:
        sub_events = [events[index] for index in missing]
        _gaps, sub_pairs, _point_a, _point_b = _infer_pairs(
            _transforms(model, base_geometry, counts[missing]), clouds, sub_events)
        pairs[missing] = sub_pairs
    return pairs


def _gaps_and_points(model, clouds, counts, pairs, geometry):
    """按当前几何重新找每个事件在指定 pair 内的最近点和接触点。"""
    frames = _transforms(model, geometry, counts)
    gaps = np.empty(len(counts))
    points = np.empty((len(counts), 3))
    rows = np.arange(len(counts))
    for pair in np.unique(pairs, axis=0):
        group = rows[np.all(pairs == pair[None, :], axis=1)]
        distances = _pair_distances(
            frames[group], clouds, int(pair[0]), int(pair[1]), int(pair[2]))
        flat = distances.reshape(len(group), -1)
        nearest = flat.argmin(axis=1)
        gaps[group] = flat[np.arange(len(group)), nearest]
        index_a, index_b = np.unravel_index(nearest, distances.shape[1:])
        a = np.einsum("nij,nj->ni", frames[group, int(pair[0]) * 3],
                      clouds[0][index_a])[:, :3]
        b = np.einsum("nij,nj->ni",
                      frames[group, int(pair[1]) * 3 + int(pair[2])],
                      clouds[int(pair[2])][index_b])[:, :3]
        points[group] = 0.5 * (a + b)
    return gaps, points


def solve(events, mappings, *, urdf_path=DEFAULT_URDF_PATH,
          iterations=SOLVE_ITERATIONS,
          optimize_kb=True, optimize_geometry=False,
          continuity_allowance_m=None, continuity_sigma_m=None,
          initial=None, loss="bce", joints=None, prior_scale=1.0):
    """二分类求解；默认只拟合 k/b，可选关节轴平移/旋转。

    initial: 可选的起始 k/b（与 mappings 同格式）；先验仍锚定 mappings。
    外层重复调用并传上一轮结果，即可刷新最近点对应（ICP 式外循环）。
    loss: "bce"（V6 软接触概率）或 "gap"（直接间隙损失，便于对比）。
    joints: 可选的关节子集；只放开这些关节的 k/b/几何，其余固定在初始值。
    prior_scale: >1 放松先验，<1 收紧；只用于仿真调参。
    """
    allowance = (CONTINUITY_ALLOWANCE_M if continuity_allowance_m is None
                 else float(continuity_allowance_m))
    sigma = (CONTINUITY_SIGMA_M if continuity_sigma_m is None
             else float(continuity_sigma_m))
    if allowance < 0.0 or sigma <= 0.0:
        raise ValueError("invalid continuity limits")
    if loss not in ("bce", "gap"):
        raise ValueError("unsupported loss")
    if not math.isfinite(float(prior_scale)) or float(prior_scale) <= 0.0:
        raise ValueError("invalid prior scale")
    prior_scale = float(prior_scale)
    selected = None if joints is None else sorted({int(joint) for joint in joints})
    if selected is not None and any(not 0 <= joint < 21 for joint in selected):
        raise ValueError("invalid joint subset")
    parts = []
    if optimize_geometry:
        parts.append(GEOMETRY_COLUMNS if selected is None
                     else _PARAM_COLUMNS[selected, :5].reshape(-1))
    if optimize_kb:
        parts.append(ENCODER_COLUMNS if selected is None
                     else _PARAM_COLUMNS[selected, 5:].reshape(-1))
    if not parts:
        raise ValueError("select at least one correction")
    free = np.concatenate(parts)
    if len(mappings) != 21:
        raise ValueError("need 21 k/b mappings")
    if initial is not None and len(initial) != 21:
        raise ValueError("need 21 initial k/b mappings")
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
    prior_base = _geometry_from_mappings(mappings)
    start_base = prior_base if initial is None else _geometry_from_mappings(initial)
    continuity_pairs = _continuity_pairs(events)
    pairs = _event_pairs(events, model, clouds, start_base, counts)
    gaps, start_points = _gaps_and_points(
        model, clouds, counts, pairs, start_base)
    continuity_before = _continuity_excess(
        start_points, continuity_pairs, allowance)
    base_x = start_base.reshape(-1)
    steps = _STEP_BY_COLUMN[free % 7]

    def full_free(free_x):
        x = base_x.copy()
        x[free] = free_x
        return x

    def residual(free_x):
        geometry = full_free(free_x).reshape(21, 7)
        gap, points = _gaps_and_points(model, clouds, counts, pairs, geometry)
        if loss == "gap":
            data = np.where(labels == 1,
                            gap / GAP_SIGMA_M,
                            np.maximum(0.0, NONCONTACT_MIN_GAP_M - gap) / GAP_SIGMA_M)
        else:
            probability = 1.0 / (1.0 + np.exp(-np.clip(
                (CONTACT_MARGIN_M - gap) / CONTACT_SLOPE_M, -60.0, 60.0)))
            probability = np.clip(probability, 1e-9, 1.0 - 1e-9)
            bce = -np.where(labels == 1,
                            np.log(probability), np.log(1.0 - probability))
            data = np.sqrt(2.0 * bce)
        return np.concatenate([
            data,
            _continuity_residual(points, continuity_pairs, allowance, sigma),
            _prior(geometry, k0, b0) / prior_scale])

    free_x, iterations_used = _levenberg_marquardt(
        residual, base_x[free], steps, iterations)
    geometry = full_free(free_x).reshape(21, 7).copy()
    rejected = _revert_out_of_range(geometry, prior_base)
    fitted, fitted_points = _gaps_and_points(
        model, clouds, counts, pairs, geometry)
    continuity_after = _continuity_excess(
        fitted_points, continuity_pairs, allowance)
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
        "iterations_used": int(iterations_used),
        "clearance_before_mm": float(np.mean(gaps[contact]) * 1000.0),
        "clearance_after_mm": float(np.mean(fitted[contact]) * 1000.0),
        "continuity_before_mm": continuity_before,
        "continuity_after_mm": continuity_after,
        "contact_pass": int(np.sum(fitted[contact] <= CONTACT_MARGIN_M)),
        "contact_total": int(np.sum(contact)),
        "noncontact_pass": int(np.sum(fitted[~contact] > CONTACT_MARGIN_M)),
        "noncontact_total": int(np.sum(~contact)),
        "weak": [joint for joint in range(21)
                 if finger_contacts[JOINT_FINGER[joint]] < 3],
        "rejected": rejected,
    }


def solve_contact(events, mappings, *, outer=3, inner=6, **kwargs):
    """ICP 式外循环：每轮刷新最近点对应，把上一轮结果作为下一轮起点。

    返回的 clearance_before/continuity_before 取第一轮（基值模型）的值，
    这样页面显示的 before→after 是整次拟合的真实改善。
    """
    initial = None
    result = None
    before = None
    continuity_before = None
    for index in range(max(1, int(outer))):
        result = solve(events, mappings, iterations=int(inner),
                       initial=initial, **kwargs)
        if index == 0:
            before = result["clearance_before_mm"]
            continuity_before = result["continuity_before_mm"]
        initial = [{"k": result["k"][joint], "b": result["b"][joint]}
                   for joint in range(21)]
    if before is not None:
        result = dict(result)
        result["clearance_before_mm"] = before
        result["continuity_before_mm"] = continuity_before
    return result


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
