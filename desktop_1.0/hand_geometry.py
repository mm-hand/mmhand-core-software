import json
import os
from typing import List, Optional, Tuple

import numpy as np

from protocol import ENCODER_COUNT

CONFIG_DIR = os.path.join(os.path.dirname(__file__), "config")
HAND_GEO_PATH = os.path.join(CONFIG_DIR, "hand_geo.json")
CALIB_DEG_PATH = os.path.join(CONFIG_DIR, "calib_deg.json")


def load_hand_geo(path: Optional[str] = None) -> dict:
    p = path or HAND_GEO_PATH
    if not os.path.isfile(p):
        return _default_hand_geo()
    with open(p, "r", encoding="utf-8") as f:
        return json.load(f)


def load_calib_zero_raw(path: Optional[str] = None) -> Optional[List[int]]:
    p = path or CALIB_DEG_PATH
    if not os.path.isfile(p):
        return None
    try:
        with open(p, "r", encoding="utf-8") as f:
            data = json.load(f)
    except (json.JSONDecodeError, OSError):
        return None

    raw = data.get("zero_encoder_raw", [])
    if len(raw) < ENCODER_COUNT:
        raise ValueError(
            f"校准文件 {p} 中 zero_encoder_raw 条目数为 {len(raw)}，需要 {ENCODER_COUNT} 个"
        )
    try:
        out = [int(v) for v in raw[:ENCODER_COUNT]]
    except (TypeError, ValueError):
        raise ValueError(
            f"校准文件 {p} 中 zero_encoder_raw 含有非整数或无效值"
        ) from None
    return out


def has_calib_file(path: Optional[str] = None) -> bool:
    try:
        return load_calib_zero_raw(path) is not None
    except ValueError:
        return False


def save_calib_zero_raw(values: List[int], path: Optional[str] = None) -> bool:
    if len(values) < ENCODER_COUNT:
        return False
    p = path or CALIB_DEG_PATH
    try:
        data = {
            "comment": (
                f"{ENCODER_COUNT} 个关节角编码器在手指 0° 时对应的编码器原始值"
                "（14 位 0~16383），下位机据此计算关节角度"
            ),
            "zero_encoder_raw": [int(v) for v in values[:ENCODER_COUNT]],
        }
        with open(p, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        return True
    except (OSError, TypeError):
        return False


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
    scale = hand_geo.get("scale_display", 1.0)
    fingers = hand_geo["fingers"]
    deg = np.array(angles_deg, dtype=float)
    while len(deg) < ENCODER_COUNT:
        deg = np.append(deg, 0.0)
    deg = deg[:ENCODER_COUNT] * np.pi / 180.0

    segments: List[Tuple[np.ndarray, np.ndarray]] = []
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
            if plane == "yz":
                dy = L * np.cos(angle_sum)
                dz = -L * np.sin(angle_sum)
                delta = np.array([0.0, dy, dz])
            else:
                dx = L * np.cos(angle_sum)
                dz = -L * np.sin(angle_sum)
                delta = np.array([dx, 0.0, dz])

            p_next = p + delta
            segments.append((p.copy(), p_next.copy()))
            p = p_next

    return segments


def get_palm_outline(hand_geo: dict) -> np.ndarray:
    scale = hand_geo.get("scale_display", 1.0)
    palm = hand_geo.get("palm", {})
    w = palm.get("width_mm", 80) * scale * 0.5
    l = palm.get("length_mm", 100) * scale * 0.5
    center = np.array(palm.get("center_mm", [0, 50, 0]), dtype=float) * scale

    t = np.linspace(0, 2 * np.pi, 24, endpoint=False)
    x = w * np.cos(t) + center[0]
    y = l * np.sin(t) + center[1]
    z = np.full_like(x, center[2])
    return np.column_stack([x, y, z])
