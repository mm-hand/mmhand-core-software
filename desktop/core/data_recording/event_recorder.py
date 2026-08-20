"""event_recorder.py - MMHand Desktop 模块契约。

职责：把 UI、ROS2、Replay 和标定操作写入统一事件流。
线程：采样入口可被通信线程调用，磁盘写入由记录器串行化；不得阻塞设备控制。
输入：带双时间戳的类型化 Topic 样本、事件和会话配置。
输出：CSV 文件以及结构化写入状态。
状态所有权：记录会话拥有文件句柄、采样序号和降采样缓存，不拥有设备真值。
禁止依赖：禁止依赖 UI 或改变控制状态，禁止在 writer 内重新解释协议业务语义。
"""
from __future__ import annotations

import time
from typing import Callable, Dict, Optional


class EventRecorder:
    def __init__(self, write_func: Callable[[str, Dict[str, object]], None]):
        self._write = write_func

    def event(
        self,
        event_type: str,
        *,
        source: str = "ui",
        detail: str = "",
        p4_time_ms: object = "",
        active_scope_label: str = "",
        active_motor_mask: str = "",
        active_joint_mask: str = "",
        host_time: Optional[float] = None,
    ) -> None:
        self._write(
            "event_log",
            {
                "host_time": time.time() if host_time is None else float(host_time),
                "p4_time_ms": p4_time_ms,
                "event_type": str(event_type),
                "source": str(source),
                "detail": str(detail),
                "active_scope_label": str(active_scope_label),
                "active_motor_mask": str(active_motor_mask),
                "active_joint_mask": str(active_joint_mask),
            },
        )
