"""frame_decoder.py - P4 二进制帧与固件文本混合流解码器。

职责：从任意分片的串口字节流中提取完整 P4 帧，并把非帧字节交给日志回调。
线程：仅允许所属串口的单个 RX 线程调用；本类不自行加锁或创建线程。
输入：可能粘包、拆包并夹杂固件 ASCII 日志的原始字节。
输出：``(packet_type, payload)`` 列表以及旁路日志字节回调。
状态所有权：本类唯一持有未完成帧缓存；断线重连必须调用 ``reset``。
禁止依赖：禁止解析业务 payload，禁止修改 HandModel 或等待 ACK。
"""

from __future__ import annotations

from typing import Callable, List, Optional


class FrameDecoder:
    """长度字段协议的增量解码器。"""

    def __init__(
        self,
        header: int,
        tail: int,
        *,
        log_byte_callback: Optional[Callable[[int], None]] = None,
        flush_log_callback: Optional[Callable[[], None]] = None,
    ) -> None:
        self.header = int(header) & 0xFF
        self.tail = int(tail) & 0xFF
        self._buffer = bytearray()
        self._log_byte_callback = log_byte_callback
        self._flush_log_callback = flush_log_callback

    @property
    def buffer(self) -> bytearray:
        """兼容旧诊断代码访问接收缓存；调用方不得长期保存该引用。"""
        return self._buffer

    @buffer.setter
    def buffer(self, value: bytearray) -> None:
        self._buffer = bytearray(value)

    def reset(self) -> None:
        """丢弃断线前残留半帧，防止重连后拼出伪数据包。"""
        self._buffer.clear()

    def feed(self, chunk: bytes) -> List[tuple]:
        """追加新字节并返回当前能够提取的全部完整数据包。"""
        if chunk:
            self._buffer.extend(chunk)
        return self.extract()

    def extract(self) -> List[tuple]:
        """从已有缓存提取完整帧，半帧继续留给下一轮串口读取。"""
        packets: List[tuple] = []
        while self._buffer:
            if self._buffer[0] != self.header:
                self._emit_log_byte(self._buffer.pop(0))
                continue

            # 帧最短结构为 header + length + type + tail。
            if len(self._buffer) < 4:
                break
            frame_len = 2 + int(self._buffer[1])
            if frame_len < 4:
                self._emit_log_byte(self._buffer.pop(0))
                continue
            if len(self._buffer) < frame_len:
                break

            if self._buffer[frame_len - 1] == self.tail:
                if self._flush_log_callback is not None:
                    self._flush_log_callback()
                packet_type = int(self._buffer[2])
                payload = bytes(self._buffer[3 : frame_len - 1])
                packets.append((packet_type, payload))
                del self._buffer[:frame_len]
            else:
                # 错误 header 不能一次丢整段，否则会跳过后面真正的合法帧。
                self._emit_log_byte(self._buffer.pop(0))
        return packets

    def _emit_log_byte(self, value: int) -> None:
        if self._log_byte_callback is not None:
            self._log_byte_callback(int(value) & 0xFF)
