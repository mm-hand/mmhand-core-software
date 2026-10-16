"""serial_transport.py - P4 串口字节传输边界。

职责：封装串口打开、关闭、可用字节读取和互斥写入，不解释任何 P4 协议。
线程：RX 线程调用读取接口，TX/心跳线程调用写入接口；写操作由内部锁串行化。
输入：端口名、波特率、超时和待发送字节。
输出：原始接收字节、写入字节数和连接状态。
状态所有权：本类唯一持有 pyserial 对象与串口写锁；上层不得直接关闭底层句柄。
禁止依赖：禁止依赖协议解析、HandModel、Qt/UI、状态机或标定业务。
"""

from __future__ import annotations

import threading
from ..session_log import log_session
from typing import Any, Callable, Optional


SerialFactory = Callable[..., Any]


class SerialTransport:
    """线程安全的原始串口传输器。

    ``LowerComputerComm`` 仍保留 ``serial`` 兼容属性，但真正的句柄生命周期
    统一由本类管理。这样后续替换 USB、TCP 或测试假串口时，不需要触碰协议层。
    """

    def __init__(
        self,
        port: str,
        baudrate: int,
        timeout_s: float,
        serial_factory: Optional[SerialFactory],
    ) -> None:
        self.port = str(port or "").strip()
        self.baudrate = int(baudrate)
        self.timeout_s = float(timeout_s)
        self._serial_factory = serial_factory
        self._serial: Optional[Any] = None
        # P4 心跳与普通命令可能来自同一 TX 线程的不同调用路径，必须共用此锁。
        self._write_lock = threading.Lock()

    @property
    def serial(self) -> Optional[Any]:
        """返回底层句柄，仅供旧接口兼容和只读诊断使用。"""
        return self._serial

    def attach(self, serial_handle: Optional[Any]) -> None:
        """兼容旧测试注入假串口；生产代码应通过 :meth:`open` 创建句柄。"""
        self._serial = serial_handle

    def open(self) -> bool:
        """打开串口；已打开时保持幂等，不重复创建 OS 句柄。"""
        if self.is_open:
            return True
        if not self.port or self._serial_factory is None:
            return False

        handle = self._serial_factory(
            None,
            self.baudrate,
            timeout=self.timeout_s,
        )
        # Configure native USB Serial/JTAG before opening the OS handle so
        # an ordinary connection cannot perform the ROM DTR/RTS reset sequence.
        handle.write_timeout = 0.5
        handle.dtr = False
        handle.rts = False
        handle.port = self.port
        # Record intent before open, including failures. Never toggle these
        # lines as a connection-recovery mechanism.
        log_session('COMM', f'open port={self.port} baud={self.baudrate} dtr=False rts=False write_timeout=0.5')
        handle.open()
        self._serial = handle
        log_session('COMM', f'opened port={self.port}')
        return self.is_open

    @property
    def is_open(self) -> bool:
        """底层句柄存在且由驱动报告为打开时返回 ``True``。"""
        return bool(self._serial is not None and getattr(self._serial, "is_open", False))

    def available(self) -> int:
        """返回当前可无阻塞读取的字节数。"""
        if not self.is_open:
            return 0
        return max(0, int(getattr(self._serial, "in_waiting", 0) or 0))

    def read_available(self) -> bytes:
        """一次读完驱动当前已经缓存的字节；没有数据时返回空字节串。"""
        count = self.available()
        if count <= 0:
            return b""
        return bytes(self._serial.read(count))

    def write(self, data: bytes) -> int:
        """串行写入一帧，避免心跳帧和控制帧在字节层交叉。"""
        if not data:
            return 0
        if not self.is_open:
            raise OSError("serial transport is not open")
        with self._write_lock:
            written = self._serial.write(data)
        count = len(data) if written is None else int(written)
        if count != len(data):
            raise OSError(f"serial short write: {count}/{len(data)} bytes")
        return count

    def close(self) -> None:
        """关闭并释放串口句柄；重复调用安全。"""
        handle = self._serial
        try:
            if handle is not None and getattr(handle, "is_open", False):
                handle.close()
        finally:
            self._serial = None
