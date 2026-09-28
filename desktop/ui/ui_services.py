"""ui_services.py - MMHand Desktop 模块契约。

职责：管理 UI 使用的后台服务、线程和停止生命周期。
线程：Qt GUI 主线程执行；耗时工作必须委托后台服务，禁止阻塞界面事件循环。
输入：用户操作、HandModel 只读快照以及 HandBackend/服务返回结果。
输出：Qt 控件状态、用户提示和经门面提交的应用命令。
状态所有权：仅拥有本视图控件和瞬时交互状态；设备真值由 core 层拥有。
禁止依赖：禁止直接导入串口通信和底层控制实现，禁止在 UI 中复制协议或安全规则。
"""
import threading
from typing import Optional

from desktop.core.data_models import HandModel, clone_hand_model


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
