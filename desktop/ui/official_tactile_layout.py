"""official_tactile_layout.py - MMHand Desktop 模块契约。

职责：定义官方触觉模组到五指 UI 的固定布局。
线程：Qt GUI 主线程执行；耗时工作必须委托后台服务，禁止阻塞界面事件循环。
输入：用户操作、HandModel 只读快照以及 HandBackend/服务返回结果。
输出：Qt 控件状态、用户提示和经门面提交的应用命令。
状态所有权：仅拥有本视图控件和瞬时交互状态；设备真值由 core 层拥有。
禁止依赖：禁止直接导入串口通信和底层控制实现，禁止在 UI 中复制协议或安全规则。
"""
from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
import re
from typing import Dict, Optional, Tuple


_NUMBER_RE = re.compile(r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?")


@dataclass(frozen=True)
class OfficialTactilePoint:
    x: float
    y: float
    z: float
    idx: Optional[int]
    neighbor: Tuple[int, ...] = ()
    shape: Tuple[float, ...] = ()


@dataclass(frozen=True)
class OfficialTactileLayout:
    label: str
    points: Tuple[OfficialTactilePoint, ...]
    min_x: float
    max_x: float
    min_y: float
    max_y: float


_JS_ARRAY_BY_TAXEL_COUNT = {
    25: ("S1610", "Zb0"),
    52: ("S2015", "sv0"),
}


def _numbers(text: str) -> Tuple[float, ...]:
    return tuple(float(item) for item in _NUMBER_RE.findall(text))


def _candidate_bundle_paths() -> Tuple[Path, ...]:
    repo_root = Path(__file__).resolve().parents[2]
    dist_dir = repo_root / "pxsr-gen3" / "resources" / "app" / "dist"
    candidates = []
    fixed = dist_dir / "index.3bcb906d.js"
    if fixed.exists():
        candidates.append(fixed)
    if dist_dir.exists():
        for item in sorted(dist_dir.glob("index*.js")):
            if item not in candidates:
                candidates.append(item)
    return tuple(candidates)


def _read_bundle() -> Optional[str]:
    for path in _candidate_bundle_paths():
        try:
            return path.read_text(encoding="utf-8", errors="ignore")
        except OSError:
            continue
    return None


def _extract_js_array(text: str, name: str) -> Optional[str]:
    marker = f"{name}=["
    start = text.find(marker)
    if start < 0:
        return None
    left = text.find("[", start)
    if left < 0:
        return None

    depth = 0
    for index in range(left, len(text)):
        char = text[index]
        if char == "[":
            depth += 1
        elif char == "]":
            depth -= 1
            if depth == 0:
                return text[left : index + 1]
    return None


def _iter_js_objects(array_text: str):
    depth = 0
    start = None
    for index, char in enumerate(array_text):
        if char == "{":
            if depth == 0:
                start = index
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0 and start is not None:
                yield array_text[start : index + 1]
                start = None


def _parse_point(text: str) -> Optional[OfficialTactilePoint]:
    position_match = re.search(r"position:\[([^\]]+)\]", text)
    if position_match is None:
        return None
    position = _numbers(position_match.group(1))
    if len(position) < 3:
        return None

    idx_match = re.search(r"idx:(\d+)", text)
    if idx_match is not None:
        return OfficialTactilePoint(
            x=position[0],
            y=position[1],
            z=position[2],
            idx=int(idx_match.group(1)),
        )

    neighbor_match = re.search(r"neighbor:\[([^\]]+)\]", text)
    shape_match = re.search(r"shape:\[([^\]]+)\]", text)
    if neighbor_match is None or shape_match is None:
        return None
    neighbor = tuple(int(item) for item in _numbers(neighbor_match.group(1)))
    shape = _numbers(shape_match.group(1))
    if not neighbor or len(neighbor) != len(shape):
        return None
    return OfficialTactilePoint(
        x=position[0],
        y=position[1],
        z=position[2],
        idx=None,
        neighbor=neighbor,
        shape=shape,
    )


@lru_cache(maxsize=1)
def _load_layouts() -> Dict[int, OfficialTactileLayout]:
    text = _read_bundle()
    if not text:
        return {}

    layouts: Dict[int, OfficialTactileLayout] = {}
    for taxel_count, (label, array_name) in _JS_ARRAY_BY_TAXEL_COUNT.items():
        array_text = _extract_js_array(text, array_name)
        if array_text is None:
            continue
        points = tuple(
            point
            for point in (_parse_point(item) for item in _iter_js_objects(array_text))
            if point is not None
        )
        if not points:
            continue
        xs = tuple(point.x for point in points)
        ys = tuple(point.y for point in points)
        layouts[taxel_count] = OfficialTactileLayout(
            label=label,
            points=points,
            min_x=min(xs),
            max_x=max(xs),
            min_y=min(ys),
            max_y=max(ys),
        )
    return layouts


def get_official_tactile_layout(taxel_count: int) -> Optional[OfficialTactileLayout]:
    return _load_layouts().get(int(taxel_count))
