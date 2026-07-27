"""CSV 动作指令或实际关节轨迹的内存数据模型。"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

from desktop.core.protocol import ENCODER_COUNT


ALL_JOINTS_MASK = (1 << ENCODER_COUNT) - 1


def _joint_values(values: Iterable[float], field_name: str) -> List[float]:
    result = [float(value) for value in values]
    if len(result) != ENCODER_COUNT:
        raise ValueError(f"{field_name} 必须包含 {ENCODER_COUNT} 个关节角")
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{field_name} 包含 NaN 或无穷值")
    return result


@dataclass(frozen=True)
class ReplayFrame:
    """一个按上位机相对时间定位的21关节动作帧。"""

    time_s: float
    target_urdf_deg: List[float]
    frame_kind: str = "continuous"
    source_kind: str = ""
    stream_id: str = ""
    command_joint_mask: int = 0
    # 21 槽位始终保留固定 J00-J20 语义；该 mask 标识 CSV 中真正有数值的关节。
    joint_value_mask: int = ALL_JOINTS_MASK

    def validate(self) -> None:
        if not math.isfinite(float(self.time_s)) or float(self.time_s) < 0.0:
            raise ValueError("Replay帧时间必须是非负有限值")
        _joint_values(self.target_urdf_deg, "target_urdf_deg")
        if self.frame_kind not in ("continuous", "discrete"):
            raise ValueError(f"不支持的Replay帧类型：{self.frame_kind}")
        if int(self.command_joint_mask) & ~ALL_JOINTS_MASK:
            raise ValueError("Replay帧包含越界的关节mask")
        if int(self.joint_value_mask) & ~ALL_JOINTS_MASK:
            raise ValueError("Replay帧包含越界的数值有效mask")


@dataclass(frozen=True)
class ReplayPreparation:
    """一次 Replay 启动时，根据当前 P4 状态生成的降级播放方案。"""

    source_joint_mask: int
    current_active_mask: int
    workspace_ready_mask: int
    effective_joint_mask: int
    skipped_joint_mask: int
    uncalibrated_joint_mask: int
    clamped_joint_mask: int = 0
    clamped_sample_count: int = 0
    clamped_counts_by_joint: Tuple[Tuple[int, int], ...] = ()
    warnings: Tuple[str, ...] = ()

    @property
    def clamped_counts(self) -> Dict[int, int]:
        return dict(self.clamped_counts_by_joint)


@dataclass
class ReplayTrace:
    """从动作总表或关节总表解析出的完整轨迹，不再持久化为手势JSON。"""

    path: Path
    frames: List[ReplayFrame]
    source: str = ""
    start_host_time: str = ""
    active_joint_mask: int = 0
    mapping_id: str = ""
    mapping_version: int = 0
    urdf_crc: int = 0
    joint_order_crc: int = 0
    raw_row_count: int = 0
    dropped_row_count: int = 0
    max_gap_s: float = 0.0
    data_kind: str = "command"
    warnings: List[str] = field(default_factory=list)

    @property
    def name(self) -> str:
        return self.path.name

    @property
    def frame_count(self) -> int:
        return len(self.frames)

    @property
    def duration_s(self) -> float:
        return float(self.frames[-1].time_s) if self.frames else 0.0

    @property
    def average_hz(self) -> float:
        if self.duration_s <= 0.0 or len(self.frames) < 2:
            return 0.0
        return float(len(self.frames) - 1) / self.duration_s

    def validate(self) -> None:
        if not self.frames:
            raise ValueError("动作CSV没有可回放的有效帧")
        if len(self.frames) == 1 and self.frames[0].frame_kind != "discrete":
            raise ValueError("连续动作CSV至少需要两帧有效数据")
        previous = -1.0
        for frame in self.frames:
            frame.validate()
            if float(frame.time_s) <= previous:
                raise ValueError("Replay轨迹时间必须严格递增")
            previous = float(frame.time_s)
        self.active_joint_mask = int(self.active_joint_mask) & ALL_JOINTS_MASK
