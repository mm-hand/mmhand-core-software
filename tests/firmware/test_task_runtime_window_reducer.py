"""TASK_DIAG 窗口归并回归：UI 不得混合不同秒的任务统计行。"""

import desktop.core.communication.state_reducer as state_reducer_module

from desktop.core.communication.ack_tracker import AckTracker
from desktop.core.communication.packet_router import P4PacketRouter
from desktop.core.communication.state_reducer import StateReducer


class _Owner:
    """只实现任务诊断归并需要的兼容门面，避免引入串口或 Qt 依赖。"""

    _heartbeat_enabled = False

    @staticmethod
    def _parse_protocol_payload(_name: str, payload: bytes) -> dict:
        return {
            "task_id": int(payload[0]),
            "window_sequence": int(payload[1]),
            "device_timestamp_ms": 1000 + int(payload[1]),
        }

    @staticmethod
    def _apply_connectivity_visibility(_model) -> None:
        pass


def _reducer() -> StateReducer:
    router = P4PacketRouter(
        {1: "task_diagnostic"}, lambda _packet_type, payload: (payload, None)
    )
    return StateReducer(router, AckTracker())


def _packets(sequence: int):
    return [(1, bytes((task_id, sequence))) for task_id in range(7)]


def test_task_runtime_is_published_only_after_full_window(monkeypatch) -> None:
    # 连续批次都要拿到快照：关闭发布节流（默认 20 ms 只约束高频遥测）。
    monkeypatch.setattr(state_reducer_module, "STATE_SNAPSHOT_MIN_INTERVAL_S", 0.0)
    reducer = _reducer()
    owner = _Owner()
    partial = reducer.process_packets(owner, _packets(1)[:6])
    assert partial is not None
    assert partial.task_runtime == {}
    assert partial.task_runtime_received_mask == 0x3F
    assert partial.task_runtime_expected_mask == 0x7F

    model = reducer.process_packets(owner, _packets(1)[6:])
    assert model is not None
    assert sorted(model.task_runtime) == list(range(7))
    assert {diag["window_sequence"] for diag in model.task_runtime.values()} == {1}


def test_new_window_discards_partial_old_window(monkeypatch) -> None:
    monkeypatch.setattr(state_reducer_module, "STATE_SNAPSHOT_MIN_INTERVAL_S", 0.0)
    reducer = _reducer()
    owner = _Owner()
    assert reducer.process_packets(owner, _packets(1)[:6]) is not None

    model = reducer.process_packets(owner, _packets(2))
    assert model is not None
    assert {diag["window_sequence"] for diag in model.task_runtime.values()} == {2}
    assert model.task_runtime_received_mask == 0x7F
    assert model.task_runtime_dropped_windows == 1
