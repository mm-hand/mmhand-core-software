"""ack_tracker.py - P4 命令确认包的线程同步器。

职责：登记最新协议 ACK，并让业务线程按命令号和发送时间等待对应确认。
线程：RX 线程调用 ``publish``；UI/后端调用线程可并发调用 ``wait_for``。
输入：ACK 命令号、模式、状态、P4 时间和主机接收时间。
输出：匹配 ACK 元组或超时 ``None``，并提供单调递增 ACK 序号。
状态所有权：本类唯一持有 Condition、最新 ACK 与 ACK sequence。
禁止依赖：禁止发送串口、修改 HandModel、触发 UI 弹窗或解释业务状态机。
"""

from __future__ import annotations

import threading
import time
from typing import Optional

from .event_channel import EventChannel


class AckTracker:
    """ACK 的发布/等待协调器。"""

    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._last_ack: Optional[tuple] = None
        self._acks_by_command: dict[int, tuple] = {}
        self._received_monotonic: dict[int, float] = {}
        self._last_p4_time_ms: Optional[int] = None
        self._sequence = 0
        # 等待保留各命令的最近ACK，防止同批次其他命令覆盖待确认结果。
        # 协议命令字为单字节，缓存最多256项；事件记录使用独立通道。
        self._events: EventChannel[dict] = EventChannel()

    def reset(
        self,
        *,
        notify_waiters: bool = False,
        reset_sequence: bool = False,
    ) -> None:
        """清除旧 ACK；默认保留跨重连单调递增的 sequence。

        UI 使用 sequence 去重操作事件。连接短暂中断时若 sequence 回到零，可能把重连后
        第一条 ACK 误判成旧事件，因此只有创建全新会话时才允许显式清零。
        """
        with self._condition:
            self._last_ack = None
            self._acks_by_command.clear()
            self._received_monotonic.clear()
            self._last_p4_time_ms = None
            if reset_sequence:
                self._sequence = 0
            self._events.reset()
            if notify_waiters:
                self._condition.notify_all()

    def publish(
        self,
        command: int,
        mode: int,
        status: int,
        received_at: float,
        p4_time_ms: Optional[int] = None,
    ) -> int:
        """由 RX 线程发布一个 ACK，并返回发布后的 sequence。"""
        with self._condition:
            # 兼容旧调用方：(cmd, mode, status, host_received_at) 必须保持四元素。
            self._last_ack = (int(command), int(mode), int(status), float(received_at))
            self._acks_by_command[int(command)] = self._last_ack
            received_monotonic = time.monotonic()
            self._received_monotonic[int(command)] = received_monotonic
            self._last_p4_time_ms = None if p4_time_ms is None else int(p4_time_ms)
            self._sequence += 1
            sequence = int(self._sequence)
            self._events.publish({
                "sequence": sequence,
                "command": int(command),
                "mode": int(mode),
                "status": int(status),
                "received_at": float(received_at),
                "received_monotonic": received_monotonic,
                "p4_time_ms": 0 if p4_time_ms is None else int(p4_time_ms),
            })
            self._condition.notify_all()
            return sequence

    def drain_events(self) -> list[dict]:
        """返回尚未交给核心状态线程的 ACK，等待事务不受该操作影响。"""
        return self._events.drain()

    def prepare_command(self, command: int) -> None:
        """Invalidate the preceding reply before queuing a new command.

        Windows wall-clock timestamps can be equal for adjacent transactions.
        A timestamp cutoff alone cannot distinguish their replies. Commands of
        the same type must still be sent and awaited sequentially: the wire
        ACK has no transaction ID.
        """
        with self._condition:
            self._acks_by_command.pop(int(command), None)
            self._received_monotonic.pop(int(command), None)

    def wait_for(
        self,
        command: int,
        timeout_s: float = 1.0,
        received_after: Optional[float] = None,
        *, received_after_monotonic: Optional[float] = None,
    ) -> Optional[tuple]:
        """等待指定命令的新 ACK，避免把上一次操作的 ACK 误当成本次成功。"""
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        with self._condition:
            while True:
                ack = self._acks_by_command.get(int(command))
                ack_is_new = (
                    received_after is None
                    or (
                        ack is not None
                        and len(ack) >= 4
                        and float(ack[3]) >= float(received_after)
                    )
                )
                if received_after_monotonic is not None:
                    ack_is_new = self._received_monotonic.get(int(command), 0) >= received_after_monotonic
                if ack is not None and int(ack[0]) == int(command) and ack_is_new:
                    return ack
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._condition.wait(timeout=remaining)
