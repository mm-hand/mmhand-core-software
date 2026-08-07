"""hand_geometry.py - MMHand Desktop 模块契约。

职责：提供手部关节、手指和执行器几何关系查询。
线程：除 comm_layer 的串口收发线程外不主动创建 GUI 线程；共享状态必须按既有锁读取。
输入：P4 字节流、上层类型化命令、配置和持久化数据。
输出：线程安全的 HandModel 快照、协议帧、ACK 和后端操作结果。
状态所有权：通信对象拥有串口与解析缓存；HandModel 的写入只允许在核心同步边界内发生。
禁止依赖：禁止依赖 Qt/UI、app、ROS2 或 Replay 页面，禁止把控件和展示规则带入核心层。
"""
# hand_geometry.py - 从 config/hand_geo.json 加载人手尺寸并计算 15 段骨头端点
# 坐标原点：手腕根部 (0,0,0)。0° 时手指伸直、指尖朝上 (+Y)；屈曲角使指尖向掌心侧弯曲

import json
import os
from typing import List, Tuple, Optional
import numpy as np

from desktop.paths import CALIB_DEG_PATH as DEFAULT_CALIB_DEG_PATH
from desktop.paths import HAND_GEO_PATH as DEFAULT_HAND_GEO_PATH

from .protocol import ENCODER_COUNT

HAND_GEO_PATH = str(DEFAULT_HAND_GEO_PATH)
CALIB_DEG_PATH = str(DEFAULT_CALIB_DEG_PATH)


def load_hand_geo(path: Optional[str] = None) -> dict:
    p = path or HAND_GEO_PATH
    if not os.path.isfile(p):
        return _default_hand_geo()
    with open(p, "r", encoding="utf-8") as f:
        return json.load(f)


def _default_hand_geo() -> dict:
    return {
        "palm": {"width_mm": 85, "length_mm": 100, "thickness_mm": 28, "center_mm": [0, 50, 0]},
        "fingers": [
            {"name": "thumb", "segment_lengths_mm": [40, 32, 28], "base_offset_mm": [38, 18, 8], "angle_indices": [0, 1, 2], "flex_plane": "xz"},
            {"name": "index", "segment_lengths_mm": [48, 32, 24], "base_offset_mm": [12, 28, 0], "angle_indices": [5, 6, 7], "flex_plane": "yz"},
            {"name": "middle", "segment_lengths_mm": [52, 36, 26], "base_offset_mm": [0, 52, 0], "angle_indices": [9, 10, 11], "flex_plane": "yz"},
            {"name": "ring", "segment_lengths_mm": [46, 34, 25], "base_offset_mm": [-12, 70, 0], "angle_indices": [13, 14, 15], "flex_plane": "yz"},
            {"name": "pinky", "segment_lengths_mm": [36, 26, 20], "base_offset_mm": [-22, 82, 0], "angle_indices": [17, 18, 19], "flex_plane": "yz"},
        ],
        "scale_display": 1.0,
    }


def compute_segment_endpoints(hand_geo: dict, angles_deg: List[float]) -> List[Tuple[np.ndarray, np.ndarray]]:
    """
    根据 21 个关节角度（度）计算 15 段骨头的 3D 端点 (p_start, p_end)。
    所有坐标相对于手腕原点 (0,0,0)。0° 时四指沿 +Y（指尖朝上），拇指沿 +X；屈曲角使指尖向掌心侧弯曲。
    """
    scale = hand_geo.get("scale_display", 1.0)
    fingers = hand_geo["fingers"]
    deg = np.array(angles_deg, dtype=float)
    while len(deg) < ENCODER_COUNT:
        deg = np.append(deg, 0.0)
    deg = deg[:ENCODER_COUNT] * np.pi / 180.0

    segments = []
    for fin in fingers:
        base = np.array(fin["base_offset_mm"], dtype=float) * scale
        lengths = np.array(fin["segment_lengths_mm"], dtype=float) * scale
        idx = fin["angle_indices"]
        plane = fin.get("flex_plane", "yz")

        p = base.copy()
        angle_sum = 0.0

        for i in range(3):
            angle_sum += deg[idx[i]] if idx[i] < len(deg) else 0.0
            L = lengths[i]
            # 屈曲：0° 为伸直沿 +Y，正角度向掌心（-Z）弯
            if plane == "yz":
                # 四指：在 YZ 平面，初始方向 (0, 1, 0)，屈曲后 (0, cos(a), -sin(a))
                dy = L * np.cos(angle_sum)
                dz = -L * np.sin(angle_sum)
                delta = np.array([0.0, dy, dz])
            else:
                # 拇指：在 XZ 平面，初始 (1, 0, 0) 或略向掌心，屈曲 (cos(a), 0, -sin(a))
                dx = L * np.cos(angle_sum)
                dz = -L * np.sin(angle_sum)
                delta = np.array([dx, 0.0, dz])
            p_next = p + delta
            segments.append((p.copy(), p_next.copy()))
            p = p_next

    return segments


def get_palm_outline(hand_geo: dict) -> np.ndarray:
    """返回手掌轮廓点 (N, 3) mm，用于 3D 显示"""
    scale = hand_geo.get("scale_display", 1.0)
    palm = hand_geo.get("palm", {})
    w = palm.get("width_mm", 80) * scale * 0.5
    l = palm.get("length_mm", 100) * scale * 0.5
    center = np.array(palm.get("center_mm", [0, 50, 0]), dtype=float) * scale
    # 近似椭圆/矩形轮廓
    t = np.linspace(0, 2 * np.pi, 24, endpoint=False)
    x = w * np.cos(t) + center[0]
    y = l * np.sin(t) + center[1]
    z = np.full_like(x, center[2])
    return np.column_stack([x, y, z])
