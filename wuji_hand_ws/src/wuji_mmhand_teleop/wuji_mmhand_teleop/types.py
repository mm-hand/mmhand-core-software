from dataclasses import dataclass, field
from typing import Any, Dict, List


@dataclass
class WujiFrame:
    seq: int
    timestamp: float
    recv_time: float
    recv_perf_time: float
    dt: float
    glove_id: int
    side: str
    skeleton: Dict[str, Any] = field(default_factory=dict)
    joint_angles_rad: List[List[float]] = field(default_factory=list)
    joint_confidence: List[float] = field(default_factory=list)
    frame_id: str = ""
    source_addr: str = "wuji-sdk"
    valid: bool = True
    pending_packets_count: int = 0
    dropped_old_packets_count: int = 0
    latest_frame_id: int = 0
    processed_frame_id: int = 0
    timing_recv_call_ms: float = 0.0
    ergonomics_names: List[str] = field(default_factory=list)
    ergonomics_values: List[float] = field(default_factory=list)
    raw_skeleton_nodes: List[Dict[str, Any]] = field(default_factory=list)


# The copied retargeting code only relies on this structural frame interface.
ManusFrame = WujiFrame


__all__ = ["WujiFrame", "ManusFrame"]

