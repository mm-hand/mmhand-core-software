"""ui_services.py - MMHand Desktop 模块契约。

职责：管理 UI 使用的后台服务、线程和停止生命周期。
线程：Qt GUI 主线程执行；耗时工作必须委托后台服务，禁止阻塞界面事件循环。
输入：用户操作、HandModel 只读快照以及 HandBackend/服务返回结果。
输出：Qt 控件状态、用户提示和经门面提交的应用命令。
状态所有权：仅拥有本视图控件和瞬时交互状态；设备真值由 core 层拥有。
禁止依赖：禁止直接导入串口通信和底层控制实现，禁止在 UI 中复制协议或安全规则。
"""
import copy
import queue
import threading
from typing import Dict, Iterable, Optional

from desktop.core.data_models import HandModel
from desktop.core.run_data_logger import RunDataLogger


def clone_hand_model(model: HandModel) -> HandModel:
    snapshot = HandModel()
    snapshot.copy_state_from(model)
    snapshot.calib_status = str(getattr(model, "calib_status", "IDLE"))
    snapshot.tactile_data = list(getattr(model, "tactile_data", []) or [])
    try:
        snapshot.set_target_angles(model.target_angles)
    except Exception:
        pass
    for i, enc in enumerate(snapshot.encoders):
        if i < len(getattr(model, "encoders", [])):
            enc.zero_raw = int(model.encoders[i].zero_raw)
    return snapshot


class LatestStateBuffer:
    def __init__(self):
        self._lock = threading.Lock()
        self._state: Optional[HandModel] = None
        self._version = 0

    def set(self, state: HandModel) -> None:
        with self._lock:
            self._state = state
            self._version += 1

    def consume_latest(self, last_version: int) -> tuple[Optional[HandModel], int]:
        with self._lock:
            if self._version == last_version:
                return None, last_version
            return self._state, self._version


class AsyncRunDataLogger:
    def __init__(self, flush_every: int = 10, max_queue: int = 2000):
        self._logger = RunDataLogger(flush_every=flush_every)
        self._queue: queue.Queue = queue.Queue(maxsize=max_queue)
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._lock = threading.Lock()

    @property
    def path(self) -> str:
        return self._logger.path

    def start(self, path: str, fieldnames: Optional[Iterable[str]] = None) -> None:
        self.stop()
        with self._lock:
            self._logger.start(path, fieldnames=fieldnames)
            self._stop.clear()
            self._running = True
            self._thread = threading.Thread(target=self._worker, daemon=True)
            self._thread.start()

    def stop(self) -> None:
        thread = None
        with self._lock:
            if not self._running:
                return
            self._running = False
            self._stop.set()
            thread = self._thread
        if thread is not None:
            thread.join(timeout=2.0)
        while True:
            try:
                row = self._queue.get_nowait()
            except queue.Empty:
                break
            self._logger.write_row(row)
            self._queue.task_done()
        self._logger.stop()
        with self._lock:
            self._thread = None

    def is_running(self) -> bool:
        with self._lock:
            return self._running and self._logger.is_running()

    def write_row(self, row: Dict[str, object]) -> None:
        if not self.is_running():
            return
        try:
            self._queue.put_nowait(copy.copy(row))
        except queue.Full:
            try:
                self._queue.get_nowait()
                self._queue.task_done()
            except queue.Empty:
                pass
            try:
                self._queue.put_nowait(copy.copy(row))
            except queue.Full:
                pass

    def _worker(self) -> None:
        while not self._stop.is_set() or not self._queue.empty():
            try:
                row = self._queue.get(timeout=0.1)
            except queue.Empty:
                continue
            self._logger.write_row(row)
            self._queue.task_done()
