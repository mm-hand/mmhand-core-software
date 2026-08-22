"""event_channel.py - 不允许被状态快照覆盖的进程内事件通道。

职责：在线程之间按到达顺序传递 ACK、安全规则等低频离散事件。
线程：通信 RX 线程调用 ``publish``，核心状态线程调用 ``drain``。
输入：不可变事件值；调用方负责在发布前复制可变字典。
输出：自上次排空以来的全部事件，保持原始顺序。
状态所有权：本类独占事件 deque 和互斥锁。
禁止依赖：禁止依赖串口、HandModel、Qt/UI 或具体协议编号。
"""

from __future__ import annotations

from collections import deque
import threading
from typing import Deque, Generic, List, TypeVar


T = TypeVar("T")


class EventChannel(Generic[T]):
    """低频可靠事件通道；状态拥塞时也不丢弃已经接收的事件。"""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._events: Deque[T] = deque()

    def publish(self, event: T) -> None:
        """追加一个事件；事件量低，不设置会静默丢数据的容量上限。"""
        with self._lock:
            self._events.append(event)

    def drain(self) -> List[T]:
        """原子取走当前全部事件，让后续发布进入下一批。"""
        with self._lock:
            events = list(self._events)
            self._events.clear()
            return events

    def reset(self) -> None:
        """连接新设备时清空旧会话尚未消费的事件。"""
        with self._lock:
            self._events.clear()
