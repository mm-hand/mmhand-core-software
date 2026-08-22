"""TASK_DIAG v4 协议回归：固定周期和事件唤醒统计不得与旧格式混用。"""

import struct

from desktop.core.protocol import parse_task_diag_packet


def _build_v4_payload() -> bytes:
    """构造一条完整 v4 任务窗口包，长度必须与 P4 发布器严格一致。"""
    return b"".join((
        struct.pack(">BBBBB", 4, 3, 1, 4, 1),
        struct.pack(">15I", 8192, 4096, 2048, 1000, 12000, 900, 42,
                    1000000, 100, 98, 12, 250, 400, 500, 600),
        struct.pack(">H", 320),
        struct.pack(">I", 2),
        struct.pack(">4H", 9, 1, 800, 750),
        struct.pack(">4I", 10, 1, 2, 123456),
        struct.pack(">24I", *range(1, 25)),
    ))


def test_task_diag_v4_decodes_window_metrics() -> None:
    payload = _build_v4_payload()
    assert len(payload) == 191
    parsed = parse_task_diag_packet(payload)
    assert parsed is not None
    assert parsed["version"] == 4
    assert parsed["task_id"] == 3
    assert parsed["window_sequence"] == 42
    assert parsed["window_duration_us"] == 1000000
    assert parsed["window_periodic_count"] == 98
    assert parsed["window_event_count"] == 12
    assert parsed["avg_exec_us"] == 250
    assert parsed["p95_exec_us"] == 400
    assert parsed["p99_exec_us"] == 500
    assert parsed["cpu_permille"] == 320
    assert parsed["io_sample_count"] == [1, 2, 3, 4]
    assert parsed["io_p99_us"] == [21, 22, 23, 24]


def test_task_diag_rejects_legacy_or_truncated_payload() -> None:
    payload = _build_v4_payload()
    assert parse_task_diag_packet(payload[:-1]) is None
    assert parse_task_diag_packet(bytes([2]) + payload[1:]) is None
