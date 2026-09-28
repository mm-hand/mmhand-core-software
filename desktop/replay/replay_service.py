"""replay_service.py - MMHand Desktop 模块契约。

职责：向 Qt 页面提供轨迹导入、预览、播放和停止用例。
线程：解析在调用线程执行，播放调度在 Replay 工作线程执行。
输入：运行 CSV、模型契约、活动关节范围和播放控制事件。
输出：带时间轴的稀疏关节帧、进度、诊断和停止结果。
状态所有权：轨迹对象由 ReplayService 拥有；设备运行状态只读且归 HandBackend。
禁止依赖：禁止依赖 Qt、串口和 ROS2，禁止直接修改 HandModel 或下位机状态。
"""

from __future__ import annotations

from pathlib import Path
import threading
from typing import Callable, Optional

from desktop.core.hand_backend import HandBackend

from .replay_controller import ReplayController, ReplayStatus
from .replay_csv_loader import load_replay_csv
from .replay_models import ReplayTrace


class ReplayService:
    """集中管理当前导入的动作CSV和播放线程，UI不直接操作控制器。"""

    def __init__(self, backend: HandBackend) -> None:
        self.backend = backend
        self.controller = ReplayController(backend)
        self._trace: Optional[ReplayTrace] = None
        self._lock = threading.RLock()
        self._event_sink: Optional[Callable[[str, str], None]] = None

    def set_event_sink(self, sink: Optional[Callable[[str, str], None]]) -> None:
        self._event_sink = sink
        self.controller.set_event_sink(sink)

    def _emit(self, event_type: str, detail: str) -> None:
        if self._event_sink is not None:
            self._event_sink(event_type, detail)

    def load_csv(self, path: Path | str) -> ReplayTrace:
        if self.controller.is_running():
            raise RuntimeError("CSV cannot be changed while Replay is running")
        trace = load_replay_csv(path)
        with self._lock:
            self._trace = trace
        self._emit(
            "replay_csv_loaded",
            (
                f"path={trace.path}; frames={trace.frame_count}; "
                f"duration_s={trace.duration_s:.3f}; dropped={trace.dropped_row_count}"
            ),
        )
        return trace

    def trace(self) -> Optional[ReplayTrace]:
        with self._lock:
            return self._trace

    def send(self) -> tuple[bool, str]:
        trace = self.trace()
        if trace is None:
            return False, "Import a motion or joint-state CSV first"
        return self.controller.start(trace)

    def stop(self, reason: str = "User stopped replay") -> bool:
        return self.controller.stop(reason)

    def status(self) -> ReplayStatus:
        return self.controller.status()

    def is_running(self) -> bool:
        return self.controller.is_running()

    def shutdown(self) -> None:
        self.controller.shutdown()
