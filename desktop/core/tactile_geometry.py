"""Manufacturer taxel coordinates and a flat-hand SVG view model.

The index of a FULL packet taxel maps to the manufacturer's one-based point
number. A SUMMARY packet has only a resultant vector and never invents taxels.


职责：按真实测点坐标渲染手形触觉图。
线程：页面刷新线程。
输入：触觉快照及已登记测点坐标。
输出：SVG及离线/摘要/全量显示。
状态所有权：只读几何资源，不拥有采样数据。
禁止依赖：禁止设备命令或模拟测点数据。
"""
from __future__ import annotations

import json
import math
from functools import lru_cache
from html import escape
from pathlib import Path


GEOMETRY_PATH = Path(__file__).resolve().parents[1] / 'resources' / 'config' / 'tactile_geometry.json'
FINGERS = ('Little', 'Ring', 'Middle', 'Index', 'Thumb')
SEGMENTS = ('Fingertip', 'Distal segment', 'Proximal segment')
MODULE_BOUNDS = {
    (0, 0): (750, 240, 100, 90), (0, 1): (710, 345, 100, 90), (0, 2): (655, 440, 100, 90),
    (1, 0): (535, 55, 100, 90), (1, 1): (535, 160, 100, 90), (1, 2): (535, 265, 100, 90),
    (2, 0): (415, 25, 100, 90), (2, 1): (415, 130, 100, 90), (2, 2): (415, 235, 100, 90),
    (3, 0): (295, 45, 100, 90), (3, 1): (295, 150, 100, 90), (3, 2): (295, 255, 100, 90),
    (4, 0): (175, 95, 100, 90), (4, 1): (175, 200, 100, 90), (4, 2): (175, 305, 100, 90),
}
# Wire order is G0 Little through G4 Thumb; the drawing bounds were laid out
# from the thumb's right-hand side to the little finger's left.
MODULE_BOUNDS = {(4 - finger, segment): bounds for (finger, segment), bounds in MODULE_BOUNDS.items()}


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


def flatten_taxels(module: dict) -> list[tuple[float, float, float]]:
    raw = module.get('taxels')
    if raw is None:
        return []
    return [tuple(float(v) for v in vector[:3]) for row in raw for vector in row]


def component(vector: tuple[float, float, float], axis: str) -> float:
    if axis == 'Magnitude':
        return math.sqrt(sum(value * value for value in vector))
    return vector[('X', 'Y', 'Z').index(axis)]


def _point_positions(points: list[dict], bounds: tuple[int, int, int, int]):
    x, y, w, h = bounds
    xs, ys = [p['x'] for p in points], [p['y'] for p in points]
    xmid, ymid = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
    scale = min((w - 18) / max(max(xs) - min(xs), 1e-9),
                (h - 18) / max(max(ys) - min(ys), 1e-9))
    return [(x + w / 2 + (p['x'] - xmid) * scale,
             y + h / 2 - (p['y'] - ymid) * scale) for p in points]


def hand_svg(modules: list[dict], axis: str = 'Magnitude') -> str:
    if axis not in ('X', 'Y', 'Z', 'Magnitude'):
        raise ValueError('invalid tactile axis')
    geometry = load_geometry()
    known = {(int(row['finger']), int(row['module'])): row for row in modules}
    parts = [
        '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 980 610" role="img" aria-label="Flat hand tactile force map">',
        '<path d="M165 540 L165 400 L170 180 Q170 85 225 85 Q280 85 280 180 L280 125 Q280 35 345 35 Q400 35 400 125 L405 100 Q405 15 465 15 Q525 15 525 100 L525 130 Q525 45 585 45 Q645 45 645 130 L645 330 L700 245 Q755 185 810 235 Q870 285 830 360 L755 505 Q740 575 670 585 L240 585 Q185 585 165 540 Z" fill="#eff6ff" stroke="#94a3b8" stroke-width="5"/>',
    ]
    for finger, name in enumerate(FINGERS):
        for segment, title in enumerate(SEGMENTS):
            x, y, w, h = MODULE_BOUNDS[finger, segment]
            row = known.get((finger, segment), {})
            valid = bool(row.get('valid'))
            full = valid and row.get('taxels') is not None
            fill = '#dbeafe' if full else '#fef3c7' if valid else '#e2e8f0'
            parts.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="20" fill="{fill}" stroke="#94a3b8"/>')
            parts.append(f'<text x="{x + w / 2}" y="{y + h + 17}" text-anchor="middle" font-size="13" fill="#334155">{escape(name)} {segment}</text>')
            if not valid:
                continue
            if full:
                points = geometry['tip' if segment == 0 else 'pad']
                vectors = flatten_taxels(row)
                if len(vectors) != len(points):
                    continue
                for point, vector, (px, py) in zip(points, vectors, _point_positions(points, (x, y, w, h))):
                    value = component(vector, axis)
                    level = min(abs(value) / 100.0, 1.0)
                    color = '#dc2626' if value < 0 else '#2563eb' if level > 0.5 else '#60a5fa'
                    radius = 2.1 + 4.0 * level
                    label = f'{name} {title} point {point["id"]}: ({point["x"]:.2f}, {point["y"]:.2f}, {point["z"]:.2f}), {axis}={value:.1f} raw'
                    parts.append(f'<circle cx="{px:.2f}" cy="{py:.2f}" r="{radius:.2f}" fill="{color}" opacity="{0.35 + 0.65 * level:.2f}"><title>{escape(label)}</title></circle>')
            else:
                force = row.get('force')
                if force is not None:
                    value = component(tuple(float(v) for v in force[:3]), axis)
                    parts.append(f'<circle cx="{x + w / 2}" cy="{y + h / 2}" r="12" fill="#f59e0b"><title>{escape(name)} {escape(title)} SUMMARY {axis}: {value:.1f} raw</title></circle>')
                    parts.append(f'<text x="{x + w / 2}" y="{y + h / 2 + 4}" text-anchor="middle" font-size="12" fill="#0f172a">Σ</text>')
    parts.append('</svg>')
    return ''.join(parts)
