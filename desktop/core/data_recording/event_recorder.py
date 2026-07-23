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
