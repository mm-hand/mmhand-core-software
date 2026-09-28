"""packet_router.py - P4 原始包的时间戳剥离与语义路由。

职责：把 ``(packet_type, payload)`` 转成带语义名称和 P4 时间的路由对象。
线程：由 RX 线程顺序调用；对象无可变解析状态，可在测试中独立复用。
输入：FrameDecoder 输出的原始协议包。
输出：``RoutedPacket``，供 StateReducer 按语义分派状态更新。
状态所有权：只持有静态 packet-type 映射，不持有 HandModel 或串口缓存。
禁止依赖：禁止修改状态、等待 ACK、记录 UI 事件或发送任何控制命令。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Iterable, Optional


@dataclass(frozen=True)
class RoutedPacket:
    """完成公共时间戳处理后的单个 P4 业务包。"""

    packet_type: int
    kind: str
    payload: bytes
    p4_time_ms: Optional[int]


class P4PacketRouter:
    """根据 packet type 建立稳定的语义路由。"""

    def __init__(
        self,
        packet_kinds: dict[int, str],
        timestamp_splitter: Callable[[int, bytes], tuple[bytes, Optional[int]]],
    ) -> None:
        self._packet_kinds = {int(key): str(value) for key, value in packet_kinds.items()}
        self._timestamp_splitter = timestamp_splitter

    def route(self, packet_type: int, payload: bytes) -> RoutedPacket:
        """剥离协议公共时间戳，并保留未知包供诊断而不是静默改写。"""
        packet_type = int(packet_type)
        business_payload, p4_time_ms = self._timestamp_splitter(packet_type, bytes(payload))
        return RoutedPacket(
            packet_type=packet_type,
            kind=self._packet_kinds.get(packet_type, "unknown"),
            payload=business_payload,
            p4_time_ms=p4_time_ms,
        )

    def route_many(self, packets: Iterable[tuple]) -> list[RoutedPacket]:
        """保持串口接收顺序路由整批数据包。"""
        return [self.route(packet_type, payload) for packet_type, payload in packets]
