"""TASK_DIAG 窗口归并回归：UI 不得混合不同秒的任务统计行。"""

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


def test_task_runtime_is_published_only_after_full_window() -> None:
    reducer = _reducer()
    owner = _Owner()
    assert reducer.process_packets(owner, _packets(1)[:6]) is None

    model = reducer.process_packets(owner, _packets(1)[6:])
    assert model is not None
    assert sorted(model.task_runtime) == list(range(7))
    assert {diag["window_sequence"] for diag in model.task_runtime.values()} == {1}


def test_new_window_discards_partial_old_window() -> None:
    reducer = _reducer()
    owner = _Owner()
    assert reducer.process_packets(owner, _packets(1)[:6]) is None

    model = reducer.process_packets(owner, _packets(2))
    assert model is not None
    assert {diag["window_sequence"] for diag in model.task_runtime.values()} == {2}
