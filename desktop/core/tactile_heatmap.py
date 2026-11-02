"""Tactile taxel grid and smooth pressure-map rendering.

职责：把 52/25 个测点的法向力插值成可贴到传感器表面的热力图。
线程：仅由 UI 刷新线程调用；纯 numpy/PIL，无设备访问。
输入：模块类型 ('tip'|'pad') 与逐测点法向力。
输出：贴图网格顶点、RGBA PNG 字节和点云颜色。
状态所有权：只读几何缓存与有界 PNG 缓存，不拥有设备状态。
禁止依赖：禁止协议、串口和控制器。
"""
from __future__ import annotations

import io
from functools import lru_cache

import numpy as np
from PIL import Image, ImageFilter
from scipy.interpolate import griddata
from scipy.spatial import ConvexHull, cKDTree

from .tactile_geometry import load_geometry

TIP_ROWS, TIP_COLS = 24, 16
PAD_ROWS, PAD_COLS = 11, 11
# 贴片向外多铺一点，盖住测点包围盒外的传感器边缘。
_MARGIN = 1.0
_RADIUS = 1.35
# 顺序热力色表（深灰→红→橙→黄→白）：压力越大越亮，不用彩虹色以免误读。
_LUT_STOPS = ('#2b2f36', '#4a2a22', '#7a2a12', '#a83209', '#c64306', '#e2600a',
              '#f28c12', '#f9ae1a', '#fcc42d', '#fed84f', '#fee97a', '#fff2a0',
              '#fff8c4', '#fffbdd', '#fffef0', '#ffffff')
_BASE_R = 0.10
_BASE_G = 0.11
_BASE_B = 0.13
_BASE_ALPHA = 0.55
_CACHE: dict[tuple, bytes] = {}


def _rgb(stop: str) -> tuple[float, float, float]:
    return tuple(int(stop[i:i + 2], 16) / 255.0 for i in (1, 3, 5))


@lru_cache(maxsize=1)
def _lut() -> np.ndarray:
    stops = np.array([_rgb(s) for s in _LUT_STOPS], float)
    return np.stack([np.interp(np.linspace(0, 1, 256), np.linspace(0, 1, len(stops)), stops[:, c])
                     for c in range(3)], axis=1)


def _points(kind: str) -> np.ndarray:
    return np.array([[p['x'], p['y'], p['z']] for p in load_geometry()[kind]], float) / 1000.0


def _pitch(points: np.ndarray) -> float:
    distances, _ = cKDTree(points[:, :2]).query(points[:, :2], k=2)
    return float(np.median(distances[:, 1]))


def _inside(poly: np.ndarray, x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """射线法判断网格点是否落在多边形内。"""
    inside = np.zeros(x.shape, bool)
    for index in range(len(poly)):
        x1, y1 = poly[index]
        x2, y2 = poly[(index + 1) % len(poly)]
        crosses = (y1 > y) != (y2 > y)
        inside ^= crosses & (x < (x2 - x1) * (y - y1) / (y2 - y1 + 1e-12) + x1)
    return inside


def _distance_to_edges(poly: np.ndarray, x: np.ndarray, y: np.ndarray) -> np.ndarray:
    """网格点到凸包边界的最短距离。"""
    best = np.full(x.shape, np.inf)
    for index in range(len(poly)):
        ax, ay = poly[index]
        bx, by = poly[(index + 1) % len(poly)]
        dx, dy = bx - ax, by - ay
        t = np.clip(((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy + 1e-12), 0.0, 1.0)
        best = np.minimum(best, np.hypot(x - (ax + t * dx), y - (ay + t * dy)))
    return best


@lru_cache(maxsize=2)
def _grid(kind: str):
    """返回 (surface_coords, gx, gy, valid)：测点凸包向外扩半个间距，边缘用最近值补。"""
    points = _points(kind)
    rows, cols = (PAD_ROWS, PAD_COLS) if kind == 'pad' else (TIP_ROWS, TIP_COLS)
    margin = _MARGIN * _pitch(points)
    xs = np.linspace(points[:, 0].min() - margin, points[:, 0].max() + margin, cols)
    ys = np.linspace(points[:, 1].min() - margin, points[:, 1].max() + margin, rows)
    gx, gy = np.meshgrid(xs, ys)
    linear = griddata(points[:, :2], points[:, 2], (gx, gy), method='linear')
    nearest = griddata(points[:, :2], points[:, 2], (gx, gy), method='nearest')
    gz = np.where(np.isnan(linear), nearest, linear)
    hull = points[ConvexHull(points[:, :2]).vertices, :2]
    # 凸包内直接算，凸包外最多再铺 margin 宽的一圈，轮廓平滑不冒尖。
    valid = _inside(hull, gx, gy) | (_distance_to_edges(hull, gx, gy) <= margin * 1.2)
    coords = [[([gx[j, i], gy[j, i], gz[j, i]] if valid[j, i] else None)
               for i in range(cols)] for j in range(rows)]
    return coords, gx, gy, valid


def grid_coords(kind: str) -> list[list[list[float] | None]]:
    return _grid(kind)[0]


def _values_grid(kind: str, values: np.ndarray) -> np.ndarray:
    _, gx, gy, _ = _grid(kind)
    points = _points(kind)
    linear = griddata(points[:, :2], values, (gx, gy), method='linear')
    nearest = griddata(points[:, :2], values, (gx, gy), method='nearest')
    return np.where(np.isnan(linear), nearest, linear)


def colormap(values) -> list[list[float]]:
    """点云配色：零压力深灰，压力越大越接近 turbo 高端。"""
    data = np.clip(np.asarray(values, float).reshape(-1) / max(24.0, float(np.max(values, initial=0.0))), 0.0, 1.0)
    colors = _lut()[np.clip((data * 255).astype(int), 0, 255)]
    weight = data[:, None]
    return (np.array([_BASE_R, _BASE_G, _BASE_B]) * (1.0 - weight) + colors * weight).tolist()


def render_png(kind: str, values) -> bytes:
    """渲染 RGBA PNG：未接触处是半透明深灰垫片，受力处叠加 turbo 热力。"""
    data = np.asarray(values, float).reshape(-1)
    key = (kind, data.tobytes())
    if key in _CACHE:
        return _CACHE[key]
    grid = _values_grid(kind, data)
    weight = np.clip(grid / max(24.0, float(grid.max())), 0.0, 1.0)
    colors = _lut()[np.clip((weight * 255).astype(int), 0, 255)]
    base = np.array([_BASE_R, _BASE_G, _BASE_B])
    rgba = np.empty(grid.shape + (4,), float)
    rgba[..., :3] = base * (1.0 - weight[..., None]) + colors * weight[..., None]
    rgba[..., 3] = (_BASE_ALPHA + (1.0 - _BASE_ALPHA) * weight) * _grid(kind)[3]
    # 贴到网格后由 GPU 线性过滤，这里不做放大，省 CPU 也避免插值糊边。
    image = Image.fromarray((rgba * 255).astype(np.uint8), 'RGBA')
    # 轻模糊一下边缘，去掉网格台阶感。
    image.putalpha(image.getchannel('A').filter(ImageFilter.GaussianBlur(1.2)))
    buffer = io.BytesIO()
    image.save(buffer, 'PNG')
    if len(_CACHE) > 32:
        _CACHE.pop(next(iter(_CACHE)))
    _CACHE[key] = buffer.getvalue()
    return _CACHE[key]
