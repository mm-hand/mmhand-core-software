"""hand_geometry.py - MMHand Desktop 模块契约。

职责：加载手掌尺寸、手指段长度和关节索引等静态几何配置。
线程：无内部状态、无后台线程，可由 UI 或可视化工具直接调用。
输入：可选的 JSON 配置路径。
输出：手部几何配置字典；文件缺失时返回内置默认值。
状态所有权：不持有运行时设备状态；调用方拥有返回配置的生命周期。
禁止依赖：禁止依赖通信、控制、Qt、ROS2 或 Replay 模块。
"""
# hand_geometry.py - 从 config/hand_geo.json 加载人手尺寸

import json
import os
from typing import Optional

from desktop.paths import HAND_GEO_PATH as DEFAULT_HAND_GEO_PATH

HAND_GEO_PATH = str(DEFAULT_HAND_GEO_PATH)


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
