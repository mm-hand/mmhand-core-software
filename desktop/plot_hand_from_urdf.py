"""
从 URDF 读取运动学并在 matplotlib 3D 中绘制手部骨架（连杆原点连线）。

- 交互模式（默认）：21 个旋转关节滑块，实时更新姿态。
- 静态模式：命令行一次性绘图。

依赖：numpy、matplotlib（与 desktop/requirements.txt 一致）。
"""

from __future__ import annotations

import argparse
import os
import re
import sys
import xml.etree.ElementTree as ET
from typing import Dict, List, Optional, Tuple

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import to_rgba
from matplotlib.widgets import Slider


# 与 hku_hand_v2_urdf/config/joint_names_V2D18-URDF.yaml 中顺序一致（21 个）
# 显示坐标系（左乘 T @ T_urdf）：
# - Rx(-90°)：URDF +Z（掌心法向）→ 中间 +Y
# - Ry(-90°)：URDF +X（手指大致方向）→ 显示 +Z（屏幕竖直向上）
# - Rz(π)：将掌心法向从 +Y 翻成 -Y（垂直屏幕向外、朝向观察者），且保持 +Z（指尖向上）不变
# 配合 view_init(elev≈5, azim≈90) 从 -Y 侧观察。
def _make_display_transform() -> np.ndarray:
    T = np.eye(4, dtype=float)
    ax = -np.pi / 2
    cx, sx = np.cos(ax), np.sin(ax)
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]])
    ay = -np.pi / 2
    cy, sy = np.cos(ay), np.sin(ay)
    Ry = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]])
    Rz_pi = np.array([[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]])
    T[:3, :3] = Rz_pi @ Ry @ Rx
    return T


DISPLAY_TRANSFORM: np.ndarray = _make_display_transform()

# 姿态窗口 Figure / Axes 统一底色（与 ui_main 中 Figure 一致）。mplot3d 的 ax.clear() 会把 axes 恢复成默认白底，必须与 fig.patch 一起设回，否则宽屏两侧「信箱」与绘图区会错色。
POSE_VIEW_BG = "#e8eaee"


JOINT_ORDER: List[str] = [
    "1-1",
    "1-2",
    "1-3",
    "1-4",
    "2-1",
    "2-2",
    "2-3",
    "2-4",
    "3-1",
    "3-2",
    "3-3",
    "3-4",
    "4-1",
    "4-2",
    "4-3",
    "4-4",
    "5-1",
    "5-2",
    "5-3",
    "5-4",
    "5-5",
]

# 与 hand_visualizer_ui.DEFAULT_ANGLE_MAPPING 行顺序一致：thumb, index, middle, ring, pinky
# URDF 手指编号：1=index, 2=middle, 3=ring, 4=pinky, 5=thumb
URDF_PREFIX_BY_MAPPING_ROW: Tuple[str, ...] = ("5", "1", "2", "3", "4")

_FALLBACK_ANGLE_MAPPING: List[List[int]] = [
    [3, 0, 1, 2],
    [4, 5, 6, 7],
    [8, 9, 10, 11],
    [12, 13, 14, 15],
    [16, 17, 18, 19],
]


def load_angle_mapping_for_ui() -> List[List[int]]:
    """与 hand_visualizer_ui 相同：从 config/hand_geo.json 读取 angle_mapping。"""
    p = os.path.join(os.path.dirname(__file__), "config", "hand_geo.json")
    try:
        from hand_geometry import load_hand_geo

        geo = load_hand_geo(p)
        m = geo.get("angle_mapping")
        if m and isinstance(m, list) and len(m) >= 5:
            return [list(row) for row in m[:5]]
    except Exception:
        pass
    return [list(row) for row in _FALLBACK_ANGLE_MAPPING]


def motor_deg_to_urdf_joint_dict(
    angles_deg: List[float],
    *,
    angle_mapping: Optional[List[List[int]]] = None,
) -> Dict[str, float]:
    """21 路电机角度（度）→ URDF 关节名 → 弧度（与 hand_geo / UI 映射一致）。

    拇指 URDF 有 5 个关节：5-1..5-4 来自映射中 4 个电机；5-5 使用第 21 路（索引 20），
    若不可用则与 5-4 相同。
    """
    from protocol import ENCODER_COUNT

    m = angle_mapping if angle_mapping is not None else load_angle_mapping_for_ui()
    deg = list(angles_deg)
    while len(deg) < ENCODER_COUNT:
        deg.append(0.0)
    deg = deg[:ENCODER_COUNT]

    out: Dict[str, float] = {}
    for row_idx, motor_indices in enumerate(m):
        prefix = URDF_PREFIX_BY_MAPPING_ROW[row_idx]
        for ji, mi in enumerate(motor_indices):
            name = f"{prefix}-{ji + 1}"
            out[name] = float(np.deg2rad(deg[mi] if mi < len(deg) else 0.0))
        if prefix == "5":
            if len(deg) > 20:
                out["5-5"] = float(np.deg2rad(deg[20]))
            else:
                out["5-5"] = out.get("5-4", 0.0)
    return out


def load_motor_degrees_from_text(path: str) -> List[float]:
    """从文本读取 ENCODER_COUNT 路关节角（度）。支持 # 行注释，数字间空格/逗号/分号分隔。"""
    from protocol import ENCODER_COUNT

    nums: List[float] = []
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.split("#")[0].strip()
            if not line:
                continue
            for tok in re.split(r"[,;\s]+", line):
                if not tok:
                    continue
                nums.append(float(tok))

    n_in = len(nums)
    if n_in < ENCODER_COUNT:
        nums.extend([0.0] * (ENCODER_COUNT - n_in))
        print(
            f"load_motor_degrees_from_text: {path!r} 仅解析到 {n_in} 个数，已用 0 填充至 {ENCODER_COUNT}",
            flush=True,
        )
    elif n_in > ENCODER_COUNT:
        print(
            f"load_motor_degrees_from_text: {path!r} 有 {n_in} 个数，截断为 {ENCODER_COUNT}",
            flush=True,
        )
        nums = nums[:ENCODER_COUNT]
    return nums


def _parse_floats(s: str) -> np.ndarray:
    return np.array([float(x) for x in re.split(r"[\s,]+", s.strip()) if x], dtype=float)


def _rpy_to_R(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = float(rpy[0]), float(rpy[1]), float(rpy[2])
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def _homogeneous_from_origin(xyz: np.ndarray, rpy: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=float)
    T[:3, :3] = _rpy_to_R(rpy)
    T[:3, 3] = xyz
    return T


def _rodrigues(axis: np.ndarray, angle: float) -> np.ndarray:
    v = np.asarray(axis, dtype=float).reshape(3)
    n = np.linalg.norm(v)
    if n < 1e-12:
        return np.eye(3)
    k = v / n
    x, y, z = k
    c, s = np.cos(angle), np.sin(angle)
    K = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + s * K + (1 - c) * (K @ K)


def _parse_origin(elem) -> Tuple[np.ndarray, np.ndarray]:
    o = elem.find("origin")
    if o is None:
        return np.zeros(3), np.zeros(3)
    xyz = _parse_floats(o.attrib.get("xyz", "0 0 0"))
    rpy = _parse_floats(o.attrib.get("rpy", "0 0 0"))
    return xyz, rpy


class UrdfKinematics:
    """仅解析 link/joint 树与关节限位，用于正向运动学。"""

    def __init__(self, urdf_path: str):
        self.urdf_path = os.path.abspath(urdf_path)
        self.root_dir = os.path.dirname(self.urdf_path)
        tree = ET.parse(self.urdf_path)
        robot = tree.getroot()
        self.link_names: List[str] = []
        for lk in robot.findall("link"):
            n = lk.attrib.get("name")
            if n:
                self.link_names.append(n)

        self.joints: List[dict] = []
        for j in robot.findall("joint"):
            name = j.attrib.get("name", "")
            jtype = j.attrib.get("type", "fixed")
            parent_el = j.find("parent")
            child_el = j.find("child")
            if parent_el is None or child_el is None:
                continue
            parent = parent_el.attrib.get("link", "")
            child = child_el.attrib.get("link", "")
            xyz, rpy = _parse_origin(j)
            axis_el = j.find("axis")
            if axis_el is not None:
                axis = _parse_floats(axis_el.attrib.get("xyz", "0 0 1"))
            else:
                axis = np.array([0.0, 0.0, 1.0])
            lim = j.find("limit")
            lower, upper = 0.0, 0.0
            if lim is not None:
                lower = float(lim.attrib.get("lower", "0"))
                upper = float(lim.attrib.get("upper", "0"))
            self.joints.append(
                {
                    "name": name,
                    "type": jtype,
                    "parent": parent,
                    "child": child,
                    "xyz": xyz,
                    "rpy": rpy,
                    "axis": axis,
                    "limit": (lower, upper),
                }
            )

        self._children: Dict[str, List[dict]] = {}
        for j in self.joints:
            p = j["parent"]
            self._children.setdefault(p, []).append(j)

        self.root_link = self._find_root()

    def _find_root(self) -> str:
        childs = {j["child"] for j in self.joints}
        parents = {j["parent"] for j in self.joints}
        roots = parents - childs
        if not roots:
            return self.link_names[0] if self.link_names else "base_link"
        return sorted(roots)[0]

    def joint_by_name(self) -> Dict[str, dict]:
        return {j["name"]: j for j in self.joints if j["name"]}

    def compute_link_poses(self, joint_positions: Dict[str, float]) -> Dict[str, np.ndarray]:
        """返回每个 link 名称到 4x4 世界位姿（根连杆为原点）。"""
        world: Dict[str, np.ndarray] = {self.root_link: np.eye(4)}

        def visit(parent: str, T_world_parent: np.ndarray) -> None:
            for j in self._children.get(parent, []):
                T_origin = _homogeneous_from_origin(j["xyz"], j["rpy"])
                if j["type"] == "fixed":
                    T_pc = T_origin
                elif j["type"] == "revolute":
                    q = float(joint_positions.get(j["name"], 0.0))
                    R = _rodrigues(j["axis"], q)
                    T_rot = np.eye(4)
                    T_rot[:3, :3] = R
                    T_pc = T_origin @ T_rot
                else:
                    T_pc = T_origin
                child = j["child"]
                T_wc = T_world_parent @ T_pc
                world[child] = T_wc
                visit(child, T_wc)

        visit(self.root_link, world[self.root_link])
        return world


def _default_joint_positions(kin: UrdfKinematics, mode: str) -> Dict[str, float]:
    out: Dict[str, float] = {}
    for j in kin.joints:
        if j["type"] != "revolute":
            continue
        lo, hi = j["limit"]
        name = j["name"]
        if mode == "mid":
            out[name] = 0.5 * (lo + hi)
        else:
            out[name] = 0.0
    return out


def _apply_overrides(base: Dict[str, float], specs: List[str]) -> Dict[str, float]:
    pos = dict(base)
    for s in specs:
        if "=" not in s:
            raise ValueError(f"关节覆盖应为 NAME=rad，收到: {s!r}")
        k, v = s.split("=", 1)
        pos[k.strip()] = float(v.strip())
    return pos


def apply_display_transform(poses: Dict[str, np.ndarray], T: np.ndarray) -> Dict[str, np.ndarray]:
    """将 URDF 连杆位姿左乘显示变换 T（4x4）。"""
    return {name: T @ M for name, M in poses.items()}


def apply_pose_view_background(ax, *, bg: str = POSE_VIEW_BG) -> None:
    """统一 Figure 与 Axes 底色，隐藏 3D 坐标轴线（与背景同色），避免 clear 后白底与两侧信箱留白错色。"""
    fig = ax.figure
    fig.patch.set_facecolor(bg)
    ax.set_facecolor(bg)
    try:
        ax.patch.set_alpha(1.0)
    except (AttributeError, TypeError):
        pass
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        try:
            ln = axis.line
            ln.set_color(bg)
            ln.set_linewidth(0)
        except (AttributeError, TypeError):
            pass


def fill_3d_axes_to_figure(ax, *, has_title: bool = True) -> None:
    """压缩 Figure 边距，使单个 3D 子图尽量占满画布。

    仅适用于「整图只有一个 3D 轴」的情况；含 Slider 等多轴的 Figure 请勿调用。
    左右/底部留出少量空间，避免 mplot3d 刻度与坐标轴标签被裁切；子图仍尽量大。
    """
    fig = ax.figure
    fig.subplots_adjust(left=0.02, right=0.99, bottom=0.02, top=0.98)
    if has_title:
        # 左略留空给轴标签；右侧略少留白，减轻宽窗口下与下方相机列对齐的「右侧一条未绘满」感
        ax.set_position([0.05, 0.10, 0.945, 0.82])
    else:
        ax.set_position([0.05, 0.06, 0.945, 0.90])


def apply_mplot3d_camera_zoom(ax, dist: float = 6.0) -> None:
    """拉近 mplot3d 相机（Axes3D._dist，默认约 10），手部在视窗中占比更大。

    注意：鼠标旋转时/结束后 mplot3d 常会把 _dist 恢复成 ~10，需在交互回调里再次写入目标距离
    （见 ui_main：motion_notify 的 after_idle、button_release、scroll 同步）。
    """
    if hasattr(ax, "_dist"):
        try:
            ax._dist = float(dist)
        except (TypeError, AttributeError):
            pass


def compute_pose_view_camera_dist(
    *,
    floor_grid: bool = True,
    sim_floor_extent_mult: float = 4.5,
) -> float:
    """与 ``draw_skeleton`` 末尾一致的目标相机距离，供 Tk 在 resize 时写入 ``_pose_3d_target_dist``。

    大地板模式下会拉近相机，若 UI 仍用 6.0 作为占位，Configure 会在重绘前把场景拉远，手显小。
    """
    cam_dist = 6.0
    if floor_grid and float(sim_floor_extent_mult) > 1.0 + 1e-6:
        cam_dist = 6.0 / (float(sim_floor_extent_mult) ** 0.42)
        cam_dist = max(2.35, min(12.0, cam_dist))
    return float(cam_dist)


def _mplot3d_adaptive_box_zoom(ax, *, base: float = 1.32, cap: float = 3.45) -> float:
    """宽屏子图时提高 set_box_aspect 的 zoom，让 3D 投影尽量铺满轴矩形（减少左右大块空白）。"""
    try:
        pos = ax.get_position()
        w = float(pos.width)
        h = float(pos.height)
    except (AttributeError, TypeError, ValueError):
        return float(min(cap, base))
    if h < 1e-12:
        return float(min(cap, base))
    ar = w / h
    if ar >= 1.0:
        # 略强于 ar**0.58，并小幅右偏补偿（mplot3d 宽轴上偶见右侧多留一条空白）
        scale = float(np.clip(ar**0.62, 1.0, 2.55))
        z = base * scale * 1.055
        return float(min(cap, z))
    scale = float(np.clip((1.0 / ar) ** 0.38, 1.0, 1.95))
    return float(min(cap, base * scale))


def apply_mplot3d_box_aspect_fill_widget(ax, *, zoom: Optional[float] = None, zoom_cap: float = 3.45) -> None:
    """按数据范围设置 3D 盒的 x:y:z **显示长度比**，保持与 URDF 几何一致（各向同性）。

    若用子图宽高比去单独拉长 X 或 Y，会非均匀缩放数据轴、手指比例失真；故仍用 lim 跨度 (dx,dy,dz)。
    **zoom>1** 时调用 ``set_box_aspect(..., zoom=zoom)``，在 **不改变 x:y:z 比** 的前提下把 3D 场景在
    子图里整体放大。``zoom is None`` 时按子图在 Figure 中的宽高比自适应，减轻宽姿态窗口左右信箱留白。
    """
    set_box = getattr(ax, "set_box_aspect", None)
    if not callable(set_box):
        return
    eff = float(zoom) if zoom is not None else _mplot3d_adaptive_box_zoom(ax, cap=float(zoom_cap))
    eff = max(0.4, min(float(zoom_cap), eff))
    try:
        x0, x1 = ax.get_xlim3d()
        y0, y1 = ax.get_ylim3d()
        z0, z1 = ax.get_zlim3d()
        dx = max(float(x1 - x0), 1e-12)
        dy = max(float(y1 - y0), 1e-12)
        dz = max(float(z1 - z0), 1e-12)
        try:
            set_box((dx, dy, dz), zoom=float(eff))
        except TypeError:
            set_box((dx, dy, dz))
    except TypeError:
        try:
            set_box(None)
        except TypeError:
            set_box((1.0, 1.0, 1.0))


def _equal_3d_aspect(
    ax,
    poses: Dict[str, np.ndarray],
    *,
    pad_scale: float = 1.32,
    min_half_size: float = 0.055,
) -> None:
    """以手部为中心的正方体视景；略放大包围盒，避免网格与手挤在一起。"""
    pts = np.stack([T[:3, 3] for T in poses.values()], axis=0)
    if pts.size == 0:
        return
    c = 0.5 * (pts.min(axis=0) + pts.max(axis=0))
    r = float(np.max(np.linalg.norm(pts - c, axis=1)))
    r = max(r * pad_scale, min_half_size, 1e-6)
    ax.set_xlim(c[0] - r, c[0] + r)
    ax.set_ylim(c[1] - r, c[1] + r)
    ax.set_zlim(c[2] - r, c[2] + r)


def _major_grid_step(span: float) -> float:
    """根据跨度选取主网格步长（约 12～20 格）。"""
    if span <= 0:
        return 0.05
    for s in (0.01, 0.02, 0.025, 0.05, 0.1, 0.15, 0.2, 0.25, 0.5):
        if span / s <= 22:
            return float(s)
    return float(max(0.05, np.ceil(span / 18 / 0.05) * 0.05))


def _plot3d_segments_noclip(ax, xs, ys, zs, **kwargs):
    """3D 线段不参与 axes patch 裁剪，减轻宽屏下网格在左右边缘被切掉、只剩中间一段的现象。"""
    arts = ax.plot(xs, ys, zs, **kwargs)
    for a in arts:
        try:
            a.set_clip_on(False)
        except (AttributeError, TypeError):
            pass


def _draw_plane_grid_xy(
    ax,
    *,
    x0: float,
    x1: float,
    y0: float,
    y1: float,
    z: float,
    major_step: float,
    minor_div: int = 5,
    major_color: str = "#8a8f98",
    major_alpha: float = 0.55,
    major_lw: float = 0.75,
    minor_color: str = "#b8bcc4",
    minor_alpha: float = 0.28,
    minor_lw: float = 0.35,
) -> None:
    """XY 平面 (z 固定) 上主/次网格线。"""
    zf = float(z)
    ms = max(float(major_step), 1e-9)
    minor_step = ms / float(max(2, minor_div))
    k = 0
    while True:
        x = x0 + k * ms
        if x > x1 + 1e-9:
            break
        _plot3d_segments_noclip(ax, [x, x], [y0, y1], [zf, zf], color=major_color, alpha=major_alpha, lw=major_lw, zorder=0)
        k += 1
    k = 0
    while True:
        y = y0 + k * ms
        if y > y1 + 1e-9:
            break
        _plot3d_segments_noclip(ax, [x0, x1], [y, y], [zf, zf], color=major_color, alpha=major_alpha, lw=major_lw, zorder=0)
        k += 1
    i = 0
    while True:
        x = x0 + i * minor_step
        if x > x1 + 1e-9:
            break
        rel = (x - x0) / ms
        if abs(rel - round(rel)) > 1e-5:
            _plot3d_segments_noclip(ax, [x, x], [y0, y1], [zf, zf], color=minor_color, alpha=minor_alpha, lw=minor_lw, zorder=0)
        i += 1
    i = 0
    while True:
        y = y0 + i * minor_step
        if y > y1 + 1e-9:
            break
        rel = (y - y0) / ms
        if abs(rel - round(rel)) > 1e-5:
            _plot3d_segments_noclip(ax, [x0, x1], [y, y], [zf, zf], color=minor_color, alpha=minor_alpha, lw=minor_lw, zorder=0)
        i += 1


def _draw_plane_grid_xz(
    ax,
    *,
    y: float,
    x0: float,
    x1: float,
    z0: float,
    z1: float,
    major_step: float,
    minor_div: int = 5,
    major_color: str = "#9aa0a8",
    major_alpha: float = 0.35,
    major_lw: float = 0.55,
    minor_color: str = "#c5c9cf",
    minor_alpha: float = 0.2,
    minor_lw: float = 0.3,
) -> None:
    """X-Z 平面 (y 固定)，用于侧墙背景网格。"""
    yf = float(y)
    ms = max(float(major_step), 1e-9)
    mstep = ms / float(max(2, minor_div))
    k = 0
    while True:
        x = x0 + k * ms
        if x > x1 + 1e-9:
            break
        _plot3d_segments_noclip(ax, [x, x], [yf, yf], [z0, z1], color=major_color, alpha=major_alpha, lw=major_lw, zorder=0)
        k += 1
    k = 0
    while True:
        z = z0 + k * ms
        if z > z1 + 1e-9:
            break
        _plot3d_segments_noclip(ax, [x0, x1], [yf, yf], [z, z], color=major_color, alpha=major_alpha, lw=major_lw, zorder=0)
        k += 1
    i = 0
    while True:
        x = x0 + i * mstep
        if x > x1 + 1e-9:
            break
        rel = (x - x0) / ms
        if abs(rel - round(rel)) > 1e-5:
            _plot3d_segments_noclip(ax, [x, x], [yf, yf], [z0, z1], color=minor_color, alpha=minor_alpha, lw=minor_lw, zorder=0)
        i += 1
    i = 0
    while True:
        z = z0 + i * mstep
        if z > z1 + 1e-9:
            break
        rel = (z - z0) / ms
        if abs(rel - round(rel)) > 1e-5:
            _plot3d_segments_noclip(ax, [x0, x1], [yf, yf], [z, z], color=minor_color, alpha=minor_alpha, lw=minor_lw, zorder=0)
        i += 1


def _draw_plane_grid_yz(
    ax,
    *,
    x: float,
    y0: float,
    y1: float,
    z0: float,
    z1: float,
    major_step: float,
    minor_div: int = 5,
    major_color: str = "#9aa0a8",
    major_alpha: float = 0.35,
    major_lw: float = 0.55,
    minor_color: str = "#c5c9cf",
    minor_alpha: float = 0.2,
    minor_lw: float = 0.3,
) -> None:
    """Y-Z 平面 (x 固定)，用于侧墙背景网格。"""
    xf = float(x)
    ms = max(float(major_step), 1e-9)
    mstep = ms / float(max(2, minor_div))
    k = 0
    while True:
        y = y0 + k * ms
        if y > y1 + 1e-9:
            break
        _plot3d_segments_noclip(ax, [xf, xf], [y, y], [z0, z1], color=major_color, alpha=major_alpha, lw=major_lw, zorder=0)
        k += 1
    k = 0
    while True:
        z = z0 + k * ms
        if z > z1 + 1e-9:
            break
        _plot3d_segments_noclip(ax, [xf, xf], [y0, y1], [z, z], color=major_color, alpha=major_alpha, lw=major_lw, zorder=0)
        k += 1
    i = 0
    while True:
        y = y0 + i * mstep
        if y > y1 + 1e-9:
            break
        rel = (y - y0) / ms
        if abs(rel - round(rel)) > 1e-5:
            _plot3d_segments_noclip(ax, [xf, xf], [y, y], [z0, z1], color=minor_color, alpha=minor_alpha, lw=minor_lw, zorder=0)
        i += 1
    i = 0
    while True:
        z = z0 + i * mstep
        if z > z1 + 1e-9:
            break
        rel = (z - z0) / ms
        if abs(rel - round(rel)) > 1e-5:
            _plot3d_segments_noclip(ax, [xf, xf], [y0, y1], [z, z], color=minor_color, alpha=minor_alpha, lw=minor_lw, zorder=0)
        i += 1


def _expand_equal_aspect_for_sim_stage(ax, *, extent_mult: float) -> None:
    """在 `_equal_3d_aspect` 得到的立方体基础上，保持中心与同比例放大边长，让地坪/墙网格覆盖更大舞台（类似 Gazebo 大地板）。"""
    if extent_mult <= 1.0 + 1e-9:
        return
    try:
        x0, x1 = ax.get_xlim3d()
        y0, y1 = ax.get_ylim3d()
        z0, z1 = ax.get_zlim3d()
    except (AttributeError, TypeError):
        x0, x1 = ax.get_xlim()
        y0, y1 = ax.get_ylim()
        z0, z1 = ax.get_zlim()
    cx = 0.5 * (float(x0) + float(x1))
    cy = 0.5 * (float(y0) + float(y1))
    cz = 0.5 * (float(z0) + float(z1))
    r = 0.5 * max(float(x1) - float(x0), float(y1) - float(y0), float(z1) - float(z0))
    R = max(r * float(extent_mult), r, 1e-9)
    ax.set_xlim(cx - R, cx + R)
    ax.set_ylim(cy - R, cy + R)
    ax.set_zlim(cz - R, cz + R)


def _draw_sim_environment_grid(ax) -> None:
    """在视景包围盒内绘制地平面 + 两面角点墙网格（仿真器风格背景）。"""
    x0, x1 = ax.get_xlim()
    y0, y1 = ax.get_ylim()
    z0, z1 = ax.get_zlim()
    span_x = float(x1 - x0)
    span_y = float(y1 - y0)
    span_z = float(z1 - z0)
    sx = _major_grid_step(span_x)
    sy = _major_grid_step(span_y)
    sz = _major_grid_step(span_z)
    s_floor = max(sx, sy)

    _draw_plane_grid_xy(ax, x0=x0, x1=x1, y0=y0, y1=y1, z=z0, major_step=s_floor)
    _draw_plane_grid_xz(ax, y=y0, x0=x0, x1=x1, z0=z0, z1=z1, major_step=max(sx, sz))
    _draw_plane_grid_yz(ax, x=x0, y0=y0, y1=y1, z0=z0, z1=z1, major_step=max(sy, sz))


def _draw_inner_xyz_triad(
    ax,
    poses: Dict[str, np.ndarray],
    *,
    rel_scale: float = 0.11,
    margin: float = 0.07,
) -> None:
    """在视景盒内一角绘制小型 XYZ 方向标（显示坐标系），随视角旋转，不占外侧轴刻度区。"""
    if not poses:
        return
    x0, x1 = ax.get_xlim3d()
    y0, y1 = ax.get_ylim3d()
    z0, z1 = ax.get_zlim3d()
    span = np.array([x1 - x0, y1 - y0, z1 - z0], dtype=float)
    span = np.maximum(span, 1e-12)
    scale = float(rel_scale * float(np.min(span)))
    if scale <= 0:
        return

    pts = np.stack([T[:3, 3] for T in poses.values()], axis=0)
    c = np.mean(pts, axis=0)
    cand_lo = np.array([x0, y0, z0], dtype=float) + margin * span
    cand_hi = np.array([x1, y1, z1], dtype=float) - margin * span
    o = cand_lo if np.linalg.norm(c - cand_lo) >= np.linalg.norm(c - cand_hi) else cand_hi

    ex = np.array([scale, 0.0, 0.0])
    ey = np.array([0.0, scale, 0.0])
    ez = np.array([0.0, 0.0, scale])
    tip = 1.12
    lw = 2.0
    _plot3d_segments_noclip(ax, [o[0], o[0] + ex[0]], [o[1], o[1] + ex[1]], [o[2], o[2] + ex[2]], color="#c62828", lw=lw, zorder=50)
    _plot3d_segments_noclip(ax, [o[0], o[0] + ey[0]], [o[1], o[1] + ey[1]], [o[2], o[2] + ey[2]], color="#2e7d32", lw=lw, zorder=50)
    _plot3d_segments_noclip(ax, [o[0], o[0] + ez[0]], [o[1], o[1] + ez[1]], [o[2], o[2] + ez[2]], color="#1565c0", lw=lw, zorder=50)

    def _lbl(p0, p1, s: str, color: str) -> None:
        t = o + tip * (p1 - o)
        try:
            txt = ax.text(float(t[0]), float(t[1]), float(t[2]), s, color=color, fontsize=9, fontweight="bold")
            txt.set_clip_on(False)
        except (AttributeError, TypeError):
            pass

    _lbl(o, o + ex, "X", "#c62828")
    _lbl(o, o + ey, "Y", "#2e7d32")
    _lbl(o, o + ez, "Z", "#1565c0")


def _style_axes_sim_environment(ax) -> None:
    """弱化默认 3D 坐标轴立方体边框；pane 与底色同源，避免灰边/白块与 Figure 不一致。"""
    bg = POSE_VIEW_BG
    apply_pose_view_background(ax)
    ax.grid(False)
    try:
        prgba = to_rgba(bg, alpha=0.08)
        for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
            axis.pane.set_facecolor(prgba)
            axis.pane.set_edgecolor(bg)
    except (AttributeError, TypeError):
        pass
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        try:
            axis._axinfo["grid"]["linewidth"] = 0.0
        except (AttributeError, KeyError, TypeError):
            pass
    ax.tick_params(axis="both", which="major", labelsize=8, colors="0.35", length=0)
    ax.tick_params(axis="both", which="minor", labelsize=0)


def draw_skeleton(
    ax,
    kin: UrdfKinematics,
    joint_positions: Dict[str, float],
    *,
    axis_len: float = 0.012,
    show_axes: bool = False,
    display_transform: np.ndarray = DISPLAY_TRANSFORM,
    plot_title: Optional[str] = None,
    floor_grid: bool = True,
    sim_floor_extent_mult: float = 4.5,
    fill_figure: bool = True,
    inner_xyz_triad: bool = True,
    draw_links_nodes: bool = True,
) -> Dict[str, np.ndarray]:
    """在已有 3D axes 上绘制骨架，返回 **显示坐标系下** 的 link 位姿。

    floor_grid=True 时绘制仿真风格环境网格（地平面 + 两面角点墙）并弱化默认 3D 坐标轴面板。
    sim_floor_extent_mult>1 时在保持立方体各向同性前提下扩大轴范围，使网格像 Gazebo/RViz 那样铺满视窗（手仍居中）。
    fill_figure=True 时压缩 Figure 边距（单轴场景）；交互模式含 Slider 时应传 False。
    inner_xyz_triad=True 时在盒内画小型 RGB 方向标，并隐藏外侧 X/Y/Z 刻度文字。
    """
    ax.clear()
    apply_pose_view_background(ax)
    poses = apply_display_transform(kin.compute_link_poses(joint_positions), display_transform)

    if draw_links_nodes:
        for j in kin.joints:
            pa, ch = j["parent"], j["child"]
            if pa not in poses or ch not in poses:
                continue
            p0 = poses[pa][:3, 3]
            p1 = poses[ch][:3, 3]
            ax.plot([p0[0], p1[0]], [p0[1], p1[1]], [p0[2], p1[2]], color="black", linewidth=2.0)

        for _name, T in poses.items():
            p = T[:3, 3]
            ax.scatter([p[0]], [p[1]], [p[2]], s=12, c="red", depthshade=True)

    if show_axes:
        for name, T in poses.items():
            o = T[:3, 3]
            R = T[:3, :3]
            for col, color in zip(range(3), ["r", "g", "b"]):
                d = R[:, col] * axis_len
                ax.plot(
                    [o[0], o[0] + d[0]],
                    [o[1], o[1] + d[1]],
                    [o[2], o[2] + d[2]],
                    color=color,
                    linewidth=0.8,
                )

    if inner_xyz_triad:
        ax.set_xlabel("")
        ax.set_ylabel("")
        ax.set_zlabel("")
    else:
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_zlabel("Z (m)")
    if plot_title is not None:
        ax.set_title(plot_title)
    else:
        ax.set_title(
            f"URDF skeleton — root: {kin.root_link} | palm toward viewer (+Y disp)"
        )
    _equal_3d_aspect(ax, poses)
    if floor_grid:
        _expand_equal_aspect_for_sim_stage(ax, extent_mult=sim_floor_extent_mult)
        _draw_sim_environment_grid(ax)
    _style_axes_sim_environment(ax)
    # 从 -Y 方向看原点：掌心法向 (+Y) 指向屏幕外；Z 在屏幕上为竖直向上
    ax.view_init(elev=5, azim=90)
    if inner_xyz_triad:
        try:
            plt.setp(ax.get_xticklabels(), visible=False)
            plt.setp(ax.get_yticklabels(), visible=False)
            plt.setp(ax.get_zticklabels(), visible=False)
        except (AttributeError, TypeError):
            pass
        _draw_inner_xyz_triad(ax, poses)
    if fill_figure:
        fill_3d_axes_to_figure(ax, has_title=True)
    apply_mplot3d_box_aspect_fill_widget(ax)
    cam_dist = compute_pose_view_camera_dist(
        floor_grid=floor_grid,
        sim_floor_extent_mult=sim_floor_extent_mult,
    )
    apply_mplot3d_camera_zoom(ax, dist=cam_dist)
    try:
        ax._pose_view_cam_dist = float(cam_dist)  # type: ignore[attr-defined]
    except (TypeError, AttributeError):
        pass
    return poses


def plot_skeleton(
    kin: UrdfKinematics,
    joint_positions: Dict[str, float],
    *,
    axis_len: float = 0.012,
    show_axes: bool = False,
    display_transform: np.ndarray = DISPLAY_TRANSFORM,
) -> None:
    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")
    draw_skeleton(
        ax,
        kin,
        joint_positions,
        axis_len=axis_len,
        show_axes=show_axes,
        display_transform=display_transform,
    )
    plt.show()


def _joint_limits_map(kin: UrdfKinematics) -> Dict[str, Tuple[float, float]]:
    m: Dict[str, Tuple[float, float]] = {}
    for j in kin.joints:
        if j["type"] == "revolute" and j["name"]:
            m[j["name"]] = j["limit"]
    return m


def _slider_range_for_joint(
    lo: float,
    hi: float,
    *,
    use_degrees: bool,
) -> Tuple[float, float, float]:
    """返回 (vmin, vmax, v0) 用于 Slider；内部仍用弧度。"""
    if use_degrees:
        d_lo, d_hi = np.rad2deg(lo), np.rad2deg(hi)
        if d_hi < d_lo:
            d_lo, d_hi = d_hi, d_lo
        v0 = 0.5 * (d_lo + d_hi)
        return float(d_lo), float(d_hi), float(v0)
    if hi < lo:
        lo, hi = hi, lo
    v0 = 0.5 * (lo + hi)
    return float(lo), float(hi), float(v0)


def _slider_to_rad(val: float, use_degrees: bool) -> float:
    return float(np.deg2rad(val)) if use_degrees else float(val)


def run_interactive(
    kin: UrdfKinematics,
    *,
    pose: str,
    use_degrees: bool = True,
    axis_len: float = 0.012,
    show_axes: bool = False,
    display_transform: np.ndarray = DISPLAY_TRANSFORM,
) -> None:
    limits = _joint_limits_map(kin)
    for name in JOINT_ORDER:
        if name not in limits:
            raise RuntimeError(f"URDF 中缺少关节 {name!r}")

    base = _default_joint_positions(kin, "mid" if pose == "mid" else "zero")

    fig = plt.figure(figsize=(14, 10))
    fig.subplots_adjust(left=0.06, right=0.98, top=0.94, bottom=0.28)
    ax = fig.add_axes([0.06, 0.34, 0.92, 0.58], projection="3d")

    unit = "°" if use_degrees else "rad"
    sliders: List[Slider] = []
    ncols = 7
    nrows = 3
    slider_w = 0.12
    slider_h = 0.022
    gap_x = 0.015
    left0 = 0.05
    row_bottoms = [0.20, 0.11, 0.02]

    for i, name in enumerate(JOINT_ORDER):
        lo, hi = limits[name]
        vmin, vmax, v0 = _slider_range_for_joint(lo, hi, use_degrees=use_degrees)
        if name in base:
            br = base[name]
            v0 = float(np.rad2deg(br)) if use_degrees else float(br)
            v0 = min(max(v0, vmin), vmax)

        row = i // ncols
        col = i % ncols
        left = left0 + col * (slider_w + gap_x)
        bottom = row_bottoms[row]
        ax_sl = fig.add_axes([left, bottom, slider_w, slider_h])
        sl = Slider(
            ax_sl,
            f"{name} ({unit})",
            vmin,
            vmax,
            valinit=v0,
            valstep=None,
        )
        sliders.append(sl)

    def joint_positions_from_sliders() -> Dict[str, float]:
        pos = dict(base)
        for name, sl in zip(JOINT_ORDER, sliders):
            pos[name] = _slider_to_rad(sl.val, use_degrees)
        return pos

    def on_change(_val: float) -> None:
        draw_skeleton(
            ax,
            kin,
            joint_positions_from_sliders(),
            axis_len=axis_len,
            show_axes=show_axes,
            display_transform=display_transform,
            fill_figure=False,
        )
        fig.canvas.draw_idle()

    for sl in sliders:
        sl.on_changed(on_change)

    on_change(0.0)
    plt.show()


def main(argv: Optional[List[str]] = None) -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    default_urdf = os.path.join(here, "hku_hand_v2_urdf", "urdf", "hand.urdf")

    p = argparse.ArgumentParser(
        description="从 URDF 绘制 HKU 手骨架：默认交互 21 关节，可选静态快照。",
    )
    p.add_argument(
        "urdf",
        nargs="?",
        default=default_urdf,
        help=f"URDF 路径（默认: {default_urdf}）",
    )
    p.add_argument(
        "--static",
        action="store_true",
        help="非交互：只画一帧并退出（原行为）",
    )
    p.add_argument(
        "--pose",
        choices=("zero", "mid"),
        default="zero",
        help="静态模式或未指定滑块初值时：关节默认全 0 或限位中点（交互模式下滑块初值同此）",
    )
    p.add_argument(
        "-j",
        "--joint",
        action="append",
        default=[],
        metavar="NAME=RAD",
        help="仅静态模式：覆盖关节角（弧度）",
    )
    p.add_argument("--axes", action="store_true", help="在每个连杆系画 RGB 轴")
    p.add_argument(
        "--axis-len",
        type=float,
        default=0.012,
        help="show_axes 时轴长度（米）",
    )
    p.add_argument(
        "--radians",
        action="store_true",
        help="交互模式下滑块单位为弧度（默认为度）",
    )
    args = p.parse_args(argv)

    if not os.path.isfile(args.urdf):
        print(f"找不到 URDF: {args.urdf}", file=sys.stderr)
        return 1

    kin = UrdfKinematics(args.urdf)

    if args.static:
        base = _default_joint_positions(kin, "mid" if args.pose == "mid" else "zero")
        joint_positions = _apply_overrides(base, args.joint)
        plot_skeleton(
            kin,
            joint_positions,
            axis_len=args.axis_len,
            show_axes=args.axes,
            display_transform=DISPLAY_TRANSFORM,
        )
        return 0

    run_interactive(
        kin,
        pose=args.pose,
        use_degrees=not args.radians,
        axis_len=args.axis_len,
        show_axes=args.axes,
        display_transform=DISPLAY_TRANSFORM,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
