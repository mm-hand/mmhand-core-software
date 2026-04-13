# hand_visualizer_ui.py - 手部姿态可视化（集成到 UI）
# 功能拷贝自 plot_hand.py，几何参数从 config/hand_geo.json 加载
# 将 21 个电机角度（度）映射为 20 个关节角（弧度）并绘制左手 3D 模型

import os
import numpy as np
from typing import List, Optional, Sequence, Tuple
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from protocol import ENCODER_COUNT
from hand_geometry import load_hand_geo

CONFIG_DIR = os.path.join(os.path.dirname(__file__), "config")
HAND_GEO_PATH = os.path.join(CONFIG_DIR, "hand_geo.json")
# 21 电机索引 -> 20 关节角，可从 hand_geo.json 的 angle_mapping 覆盖
DEFAULT_ANGLE_MAPPING = [
    [3, 0, 1, 2],    # thumb: abduct=3, flex=0,1,2
    [4, 5, 6, 7],    # index
    [8, 9, 10, 11],  # middle
    [12, 13, 14, 15],  # ring
    [16, 17, 18, 19],  # pinky
]


def _load_angle_mapping(geo: dict) -> list:
    """从 hand_geo 加载 angle_mapping，缺失时用默认"""
    m = geo.get("angle_mapping")
    if m and isinstance(m, list) and len(m) >= 5:
        return [list(row) for row in m[:5]]
    return DEFAULT_ANGLE_MAPPING


def _darken_color(color, factor: float = 0.6):
    import matplotlib.colors as mcolors
    try:
        if isinstance(color, str):
            rgb = np.array(mcolors.to_rgb(color))
        elif hasattr(color, "__len__") and len(color) >= 3:
            rgb = np.array(color[:3])
        else:
            return color
        darker = rgb * factor
        return mcolors.to_hex(np.clip(darker, 0, 1))
    except Exception:
        return color


def _angles_21deg_to_20rad(angles_deg: List[float], mapping: Optional[list] = None) -> List[float]:
    """21 个电机角度（度）-> 20 个关节角（弧度）"""
    mapping = mapping or DEFAULT_ANGLE_MAPPING
    deg = list(angles_deg)
    while len(deg) < ENCODER_COUNT:
        deg.append(0.0)
    deg = deg[:ENCODER_COUNT]
    rad_list = []
    for finger_map in mapping:
        for idx in finger_map:
            rad_list.append(np.deg2rad(deg[idx] if idx < len(deg) else 0))
    return rad_list[:20]


class HandVisualizerUI:
    """手部可视化器 - 从 hand_geo.json 加载几何，根据 21 电机角度绘制左手"""

    def __init__(self, hand_geo_path: Optional[str] = None):
        geo = load_hand_geo(hand_geo_path or HAND_GEO_PATH)
        self._angle_mapping = _load_angle_mapping(geo)
        scale = geo.get("scale_display", 1.0) / 1000.0  # mm -> m
        # Visual tuning for UI:
        # - Slightly shorten thumb to better match perceived proportion.
        # - Move palm root downward so the full hand is easier to keep in frame.
        self._thumb_length_scale = 0.88
        self._palm_root_z_offset = -0.006

        # 从 config 加载指节长度 (m)
        self.finger_lengths = {}
        for fin in geo.get("fingers", []):
            name = fin["name"]
            lengths_mm = fin.get("segment_lengths_mm", [40, 32, 28])
            self.finger_lengths[name] = [l * scale for l in lengths_mm[:3]]
        if "thumb" in self.finger_lengths:
            self.finger_lengths["thumb"] = [l * self._thumb_length_scale for l in self.finger_lengths["thumb"]]

        # 默认长度（config 缺失时）
        defaults = {
            "thumb": [0.045, 0.030, 0.022],
            "index": [0.042, 0.024, 0.018],
            "middle": [0.046, 0.027, 0.020],
            "ring": [0.042, 0.025, 0.019],
            "pinky": [0.034, 0.017, 0.016],
        }
        for k, v in defaults.items():
            if k not in self.finger_lengths or len(self.finger_lengths[k]) < 3:
                self.finger_lengths[k] = v

        # 手掌尺寸 (m)
        palm = geo.get("palm", {})
        self.palm_thickness = palm.get("thickness_mm", 28) * scale
        self.palm_width = palm.get("width_mm", 90) * scale
        self.palm_length = palm.get("length_mm", 100) * scale

        # 指根相对掌心中心的偏移：从 hand_geo base_offset_mm 推导
        # hand_geo: 原点腕部，Y 朝上，X 拇指侧，Z 掌心；plot_hand: X 掌心，Y 拇指→小指，Z 向上
        # 转换: plot_hand(X,Y,Z) = (hand_geo.Z, -hand_geo.X, hand_geo.Y - palm_center.Y)
        palm_center_y = palm.get("center_mm", [0, 50, 0])[1] * scale
        self.finger_origins = {}
        for fin in geo.get("fingers", []):
            name = fin["name"]
            base_mm = np.array(fin.get("base_offset_mm", [0, 50, 0]))
            # 相对掌心中心
            rel = base_mm * scale - np.array([0, palm_center_y, 0])
            # 转为 plot_hand 系: X=geo.Z, Y=-geo.X, Z=rel.Y
            self.finger_origins[name] = np.array([rel[2], -rel[0], rel[1]])
        # 默认值（config 缺失某指时）
        defaults = {
            "thumb": np.array([0.008, -0.038, -0.032]),   # 拇指 CMC 在掌侧桡骨旁，略高于腕
            "index": np.array([0.0, 0.012, 0.0273]),
            "middle": np.array([0.0, 0.028, 0.0281]),
            "ring": np.array([0.0, 0.040, 0.0111]),
            "pinky": np.array([0.0, 0.045, -0.0075]),
        }
        for k, v in defaults.items():
            if k not in self.finger_origins:
                self.finger_origins[k] = v

        self.joint_angles = {
            "thumb": np.array([0.1, 0.0, 0.0, 0.0]),
            "index": np.array([0.0, 0.0, 0.0, 0.0]),
            "middle": np.array([0.0, 0.0, 0.0, 0.0]),
            "ring": np.array([0.0, 0.0, 0.0, 0.0]),
            "pinky": np.array([0.0, 0.0, 0.0, 0.0]),
        }
        self.palm_pos = np.array([0.0, 0.0, self._palm_root_z_offset])
        self.palm_rot = np.eye(3, dtype=float)
        self.is_left_hand = True

    def set_angles_from_motors(self, angles_deg: List[float]) -> None:
        """从 21 个电机角度（度）设置手姿"""
        rad_20 = _angles_21deg_to_20rad(angles_deg, self._angle_mapping)
        self.set_all_joint_angles(
            rad_20[0:4], rad_20[4:8], rad_20[8:12], rad_20[12:16], rad_20[16:20]
        )

    def set_all_joint_angles(
        self,
        thumb_angles: Sequence[float],
        index_angles: Sequence[float],
        middle_angles: Sequence[float],
        ring_angles: Sequence[float],
        pinky_angles: Sequence[float],
    ):
        for name, angles in [
            ("thumb", thumb_angles),
            ("index", index_angles),
            ("middle", middle_angles),
            ("ring", ring_angles),
            ("pinky", pinky_angles),
        ]:
            self.joint_angles[name] = np.array(angles, dtype=float)[:4]

    def _rotate_point(self, point: np.ndarray, R: np.ndarray, t: np.ndarray) -> np.ndarray:
        return R @ point + t

    def _compute_finger_joints(
        self, finger_name: str, base_pos: np.ndarray, base_rot: np.ndarray
    ) -> List[np.ndarray]:
        lengths = self.finger_lengths[finger_name]
        angles = self.joint_angles[finger_name]

        if finger_name == "thumb":
            abduct = angles[0]
            flex1, flex2, flex3 = angles[1], angles[2], angles[3]
            if self.is_left_hand:
                abduct = -abduct
            R_abduct = np.array(
                [[np.cos(abduct), -np.sin(abduct), 0], [np.sin(abduct), np.cos(abduct), 0], [0, 0, 1]]
            )
            R_flex1 = np.array(
                [[np.cos(flex1), 0, -np.sin(flex1)], [0, 1, 0], [np.sin(flex1), 0, np.cos(flex1)]]
            )
            R_flex2 = np.array(
                [[np.cos(flex2), 0, -np.sin(flex2)], [0, 1, 0], [np.sin(flex2), 0, np.cos(flex2)]]
            )
            R_flex3 = np.array(
                [[np.cos(flex3), 0, -np.sin(flex3)], [0, 1, 0], [np.sin(flex3), 0, np.cos(flex3)]]
            )
            j1 = np.zeros(3)
            j2 = j1 + R_abduct @ R_flex1 @ np.array([lengths[0], 0, 0])
            j3 = j2 + R_abduct @ R_flex1 @ R_flex2 @ np.array([lengths[1], 0, 0])
            tip = j3 + R_abduct @ R_flex1 @ R_flex2 @ R_flex3 @ np.array([lengths[2], 0, 0])
        else:
            abduct = angles[0]
            flex1, flex2, flex3 = angles[1], angles[2], angles[3]
            if self.is_left_hand:
                abduct = -abduct
            R_abduct = np.array(
                [[np.cos(abduct), 0, np.sin(abduct)], [0, 1, 0], [-np.sin(abduct), 0, np.cos(abduct)]]
            )
            R_flex1 = np.array(
                [[np.cos(flex1), 0, np.sin(flex1)], [0, 1, 0], [-np.sin(flex1), 0, np.cos(flex1)]]
            )
            R_flex2 = np.array(
                [[np.cos(flex2), 0, np.sin(flex2)], [0, 1, 0], [-np.sin(flex2), 0, np.cos(flex2)]]
            )
            R_flex3 = np.array(
                [[np.cos(flex3), 0, np.sin(flex3)], [0, 1, 0], [-np.sin(flex3), 0, np.cos(flex3)]]
            )
            seg = np.array([0.0, 0.0, 1.0])
            j1 = np.zeros(3)
            j2 = j1 + R_abduct @ R_flex1 @ (lengths[0] * seg)
            j3 = j2 + R_abduct @ R_flex1 @ R_flex2 @ (lengths[1] * seg)
            tip = j3 + R_abduct @ R_flex1 @ R_flex2 @ R_flex3 @ (lengths[2] * seg)

        joints_local = [j1, j2, j3, tip]
        return [self._rotate_point(j, base_rot, base_pos) for j in joints_local]

    def get_finger_joints(self) -> dict:
        palm_center = np.array([0.0, 0.0, self.palm_length / 2])
        R_thumb = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=float)
        finger_joints = {}
        for name, origin in self.finger_origins.items():
            base_local = palm_center + origin
            base_world = self._rotate_point(base_local, self.palm_rot, self.palm_pos)
            base_rot = self.palm_rot @ R_thumb if name == "thumb" else self.palm_rot
            finger_joints[name] = self._compute_finger_joints(name, base_world, base_rot)
        return finger_joints

    def draw(
        self,
        ax,
        tactile_values: Optional[List[float]] = None,
        show_tactile: bool = False,
    ) -> None:
        """在 matplotlib 3D 轴上绘制手。tactile_values: 15 个触觉值，用于着色（可选）"""
        finger_joints = self.get_finger_joints()
        palm_root = self.palm_pos

        colors = {
            "thumb": "red",
            "index": "blue",
            "middle": "green",
            "ring": "orange",
            "pinky": "purple",
        }

        for finger_name, joints in finger_joints.items():
            color = colors.get(finger_name, "gray")
            if show_tactile and tactile_values and len(tactile_values) >= 15:
                try:
                    import matplotlib.pyplot as plt
                    idx = {"thumb": 0, "index": 1, "middle": 2, "ring": 3, "pinky": 4}[finger_name]
                    val = max(tactile_values[idx * 3 : (idx + 1) * 3], default=0)
                    t_max = max(tactile_values) or 1.0
                    cmap = plt.colormaps.get("viridis", plt.cm.viridis)
                    color = cmap(np.clip(val / t_max, 0, 1))
                except Exception:
                    pass

            _c = color if isinstance(color, str) else tuple(color[:3]) if hasattr(color, "__len__") else color
            ax.plot(
                [palm_root[0], joints[0][0]],
                [palm_root[1], joints[0][1]],
                [palm_root[2], joints[0][2]],
                color=_c,
                linewidth=2,
                linestyle="--",
                alpha=0.7,
            )

            x = [j[0] for j in joints]
            y = [j[1] for j in joints]
            z = [j[2] for j in joints]
            ax.plot(
                x, y, z,
                color=_c,
                linewidth=3,
                marker="o",
                markersize=6,
                label=finger_name,
            )

            if len(joints) >= 2:
                tip = np.array(joints[-1])
                prev = np.array(joints[-2])
                direction = tip - prev
                d_norm = np.linalg.norm(direction)
                if d_norm > 1e-6:
                    direction = direction / d_norm
                    n_ref = np.array([0.0, 1.0, 0.0]) if abs(direction[1]) < 0.9 else np.array([1.0, 0.0, 0.0])
                    u = np.cross(direction, n_ref)
                    u_norm = np.linalg.norm(u)
                    if u_norm > 1e-6:
                        u = u / u_norm
                        v = np.cross(direction, u)
                        v = v / np.linalg.norm(v)
                        if finger_name == "thumb" and self.is_left_hand:
                            u = -u
                        R = np.column_stack([u, v, direction])
                        h_cone, r_cone = 0.006, 0.004
                        n_seg = 12
                        theta = np.linspace(0, 2 * np.pi, n_seg + 1)[:-1]
                        base_pts = tip + (
                            R
                            @ np.vstack(
                                [
                                    r_cone * np.cos(theta),
                                    r_cone * np.sin(theta),
                                    np.zeros(n_seg),
                                ]
                            )
                        ).T
                        apex_pt = tip + h_cone * direction
                        c = color if isinstance(color, str) else tuple(color[:3]) if hasattr(color, "__len__") else color
                        base_face = [base_pts.tolist()]
                        poly_base = Poly3DCollection(
                            base_face,
                            facecolor=_darken_color(c, 0.5),
                            alpha=0.85,
                            edgecolor=c,
                            linewidth=1.2,
                        )
                        ax.add_collection3d(poly_base)
                        for i in range(n_seg):
                            j = (i + 1) % n_seg
                            side_face = [[base_pts[i], base_pts[j], apex_pt]]
                            poly_side = Poly3DCollection(
                                side_face,
                                facecolor=c,
                                alpha=0.7,
                                edgecolor=c,
                                linewidth=0.8,
                            )
                            ax.add_collection3d(poly_side)

        ax.legend()
        ax.grid(False)
        ax.axis("off")
        half_range = 0.05  # 缩小视距使手占据约 2/3 显示区域
        ax.set_xlim([-half_range, half_range])
        ax.set_ylim([-half_range, half_range])
        # 以掌根为锚点，将掌根放在显示区域底部附近
        root_z = float(self.palm_pos[2])
        z_bottom_margin = 0.004
        ax.set_zlim([root_z - z_bottom_margin, root_z + 2 * half_range - z_bottom_margin])
        ax.view_init(elev=20, azim=0)
        # 让手占据 2/3 显示区域：缩小边距
        fig = ax.get_figure()
        fig.subplots_adjust(left=0.05, right=0.95, bottom=0.05, top=0.95)
