"""Manual 单路写入 ACK 回归：不得误清其他关节的 Auto 和 ABS Limit。"""

from desktop.core.communication.ack_tracker import AckTracker
from desktop.core.communication.packet_router import P4PacketRouter
from desktop.core.communication.state_reducer import StateReducer
from desktop.core.data_models import HandModel
from desktop.core.protocol import (
    CMD_MANUAL_ENCODER_SINGLE,
    PROTO_ACK_STATUS_OK,
    SYSTEM_STATE_IDLE,
)


class _Owner:
    """只实现 ACK 归并需要的兼容门面。"""

    _heartbeat_enabled = False

    @staticmethod
    def _parse_protocol_payload(_name: str, _payload: bytes):
        return CMD_MANUAL_ENCODER_SINGLE, SYSTEM_STATE_IDLE, PROTO_ACK_STATUS_OK

    @staticmethod
    def _apply_connectivity_visibility(_model) -> None:
        pass


def test_manual_single_ack_preserves_unrelated_calibration() -> None:
    """P4 会按 changedMask 精确失效 Auto，ACK 层不能全局清空。"""
    reducer = StateReducer(
        P4PacketRouter({1: "protocol_ack"}, lambda _kind, payload: (payload, None)),
        AckTracker(),
    )
    initial = HandModel()
    initial.motor_abs_limit_valid_bitmap = (1 << 18) | (1 << 19)
    initial.joint_calib_results[14] = {"joint": 14, "success": True}
    initial.joint_calib_results[15] = {"joint": 15, "success": True}
    reducer.last_model = initial

    model = reducer.process_packets(_Owner(), [(1, b"ack")])

    assert model is not None
    assert model.motor_abs_limit_valid_bitmap == ((1 << 18) | (1 << 19))
    assert model.joint_calib_results[14] == {"joint": 14, "success": True}
    assert model.joint_calib_results[15] == {"joint": 15, "success": True}

