"""motion_sampler.py - MMHand Desktop 模块契约。

职责：分类控制输入并逐条保留连续运动轨迹。
线程：采样入口可被通信线程调用，磁盘写入由记录器串行化；不得阻塞设备控制。
输入：带双时间戳的类型化 Topic 样本、事件和会话配置。
输出：CSV 文件以及结构化写入状态。
状态所有权：记录会话拥有文件句柄、采样序号和降采样缓存，不拥有设备真值。
禁止依赖：禁止依赖 UI 或改变控制状态，禁止在 writer 内重新解释协议业务语义。
"""

from __future__ import annotations

import threading
from typing import Dict, List, Tuple


MOTION_RECORD_PERIOD_SECONDS = 0.05


def classify_motion_source(source: str, live: bool) -> Tuple[str, str, bool]:
    """把关节命令分类为来源、帧类型和是否允许 Replay。

    ``live`` 表示调用方会持续刷新目标：外部 ROS2 或滑条拖动属于 continuous；
    按钮 Send/Open/Fist/Zero 属于 discrete。测试、保持和 Replay 自身仍可保存
    用于诊断，但不能再次作为 Replay 输入。
    """
    text = str(source or "").strip()
    upper = text.upper()
    if upper.startswith("REPLAY:"):
        return "replay", "diagnostic", False
    if upper.startswith("HOLD "):
        return "system_hold", "diagnostic", False
    if "TEST" in upper:
        return "test", "diagnostic", False
    if "ROS2" in upper:
        source_kind = "ros2_external"
    else:
        source_kind = "ui_feedback"
    return source_kind, "continuous" if live else "discrete", True


class MotionCommandSampler:
    """逐条编号和分类，不在入口降采样；50 ms 仅用于 writer 刷新。"""

    def __init__(self, period_s: float = MOTION_RECORD_PERIOD_SECONDS) -> None:
        self.period_s = max(0.001, float(period_s))
        self._lock = threading.Lock()
        self._stream_counter = 0
        self._command_sequence = 0
        self._active_key: Tuple[object, ...] | None = None
        self._active_stream_id = ""

    def reset(self) -> None:
        with self._lock:
            self._stream_counter = 0
            self._command_sequence = 0
            self._active_key = None
            self._active_stream_id = ""

    def submit(
        self,
        row: Dict[str, object],
        *,
        now_monotonic: float | None = None,
    ) -> List[Dict[str, object]]:
        """接收一条backend命令记录，返回本次应立即写入的0到多行。"""
        out = dict(row)
        source_kind, frame_kind, source_replayable = classify_motion_source(
            str(out.get("source", "")),
            bool(out.get("live_enabled", False)),
        )
        sent = bool(out.get("sent_to_backend", False))
        blocked = bool(out.get("blocked", False))
        replayable = bool(source_replayable and sent and not blocked)
        if not replayable:
            frame_kind = "diagnostic"
        out["source_kind"] = source_kind
        out["frame_kind"] = frame_kind
        out["replayable"] = 1 if replayable else 0

        with self._lock:
            emitted: List[Dict[str, object]] = []
            if frame_kind != "continuous":
                self._close_stream_locked()
                self._stream_counter += 1
                out["stream_id"] = f"event-{self._stream_counter:06d}"
                emitted.append(self._stamp_locked(out))
                return emitted

            key = (
                source_kind,
                str(out.get("source", "")),
                str(out.get("command_joint_mask", "")),
            )
            if key != self._active_key:
                self._stream_counter += 1
                self._active_key = key
                self._active_stream_id = f"stream-{self._stream_counter:06d}"

            out["stream_id"] = self._active_stream_id
            emitted.append(self._stamp_locked(out))
            return emitted

    def flush_due(self, *, now_monotonic=None, force=False) -> List[Dict[str, object]]:
        with self._lock:
            if force:
                self._close_stream_locked()
        return []

    def _stamp_locked(self, row: Dict[str, object]) -> Dict[str, object]:
        stamped = dict(row)
        stamped["command_sequence"] = self._command_sequence
        self._command_sequence += 1
        return stamped

    def _close_stream_locked(self) -> None:
        self._active_key = None
        self._active_stream_id = ""


__all__ = [
    "MOTION_RECORD_PERIOD_SECONDS",
    "MotionCommandSampler",
    "classify_motion_source",
]
