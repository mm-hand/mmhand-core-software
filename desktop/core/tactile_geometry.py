"""Manufacturer taxel coordinates and URDF sensor-link mapping.

The index of a FULL packet taxel maps to the manufacturer's one-based point
number. Coordinates are millimetres in the sensor-link frame; the helpers
below expose them in metres for FK and the 3D scene.

职责：提供触觉测点坐标、模块到 URDF sensor link 的映射。
线程：纯只读资源，lru_cache 后可由任意线程共享。
输入：desktop/resources/config/tactile_geometry.json。
输出：52/25 点坐标（米）与 sensor link 名称。
状态所有权：只读几何资源，不拥有采样数据。
禁止依赖：禁止设备命令或模拟测点数据。
"""
from __future__ import annotations

import json
from functools import lru_cache

import numpy as np

from desktop.paths import CONFIG_DIR

GEOMETRY_PATH = CONFIG_DIR / 'tactile_geometry.json'

# 触觉模块 (finger, segment) -> URDF sensor link。finger 顺序为
# Little/Ring/Middle/Index/Thumb；segment 0=tip(52 点)，1/2=pad(25 点)。
SENSOR_LINKS = {
    (finger, segment): f"link_{name}_{5 - finger}_skeleton_sensor"
    for finger in range(5)
    for segment, name in ((0, "finger_tip"), (1, "distal_phalanx"),
                          (2, "proximal_phalanx"))
}
SENSOR_ORDER = tuple(
    (finger, segment) for finger in range(5) for segment in range(3))


@lru_cache(maxsize=1)
def load_geometry() -> dict:
    payload = json.loads(GEOMETRY_PATH.read_text(encoding='utf-8'))
    result = {}
    for module, expected in (('tip', 52), ('pad', 25)):
        points = payload['modules'][module]['points']
        if len(points) != expected or [p['id'] for p in points] != list(range(1, expected + 1)):
            raise ValueError(f'{module} manufacturer coordinates are incomplete or out of order')
        result[module] = points
    return result


@lru_cache(maxsize=1)
def module_points_m() -> dict[int, np.ndarray]:
    """按 segment 返回模块测点坐标，单位米，顺序与 FULL taxel 一致。"""
    geometry = load_geometry()
    return {
        0: np.asarray([[p["x"], p["y"], p["z"]] for p in geometry["tip"]],
                      dtype=float) / 1000.0,
        1: np.asarray([[p["x"], p["y"], p["z"]] for p in geometry["pad"]],
                      dtype=float) / 1000.0,
        2: np.asarray([[p["x"], p["y"], p["z"]] for p in geometry["pad"]],
                      dtype=float) / 1000.0,
    }


def module_points(segment: int) -> np.ndarray:
    return module_points_m()[int(segment)]
