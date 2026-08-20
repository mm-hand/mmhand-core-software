"""触觉摘要有效位图、来源和在线状态测试。"""

import struct
import time

from desktop.core.comm_layer import LowerComputerComm, TactileSerialComm
from desktop.core.data_models import (
    HandModel,
    TACTILE_DATA_LEVEL_SUMMARY,
    TACTILE_SOURCE_P4_CAN_RELAY,
    TACTILE_SOURCE_S3_SERIAL,
)
from desktop.core.hand_backend import HandBackend
from desktop.core.protocol import (
    PACKET_TYPE_TACTILE,
    TACTILE_SUMMARY_BYTES,
    parse_tactile_packet,
)


def _summary_payload(mask: int) -> bytes:
    values = bytearray(TACTILE_SUMMARY_BYTES)
    values[0] = 0xFF  # G0S0 Fx=-1，验证有符号解析。
    values[1] = 2
    values[2] = 3
    return bytes([7]) + bytes(values) + struct.pack(">H", mask & 0x7FFF)


def test_tactile_summary_parser_requires_and_returns_module_valid_mask():
    parsed = parse_tactile_packet(_summary_payload(0x4001))

    assert parsed is not None
    seq, groups, valid_mask = parsed
    assert seq == 7
    assert groups[0][0] == [-1, 2, 3]
    assert valid_mask == 0x4001
    assert parse_tactile_packet(_summary_payload(0x4001)[:-2]) is None


def test_s3_direct_summary_uses_mask_instead_of_nonzero_force_for_online_state():
    comm = TactileSerialComm("")
    model = comm._process_tactile_packets([(PACKET_TYPE_TACTILE, _summary_payload(0x0001))])

    assert model is not None
    assert model.tactile_source == TACTILE_SOURCE_S3_SERIAL
    assert model.tactile_data_level == TACTILE_DATA_LEVEL_SUMMARY
    assert model.tactile_online is True
    assert model.tactile_module_online[0] is True
    assert model.tactile_module_online[1:] == [False] * 14


def test_p4_relay_summary_sets_source_timestamp_and_exact_module_mask():
    comm = LowerComputerComm("")
    # P4包由路由器先剥离末尾4字节设备时间戳，再交给摘要解析器。
    packet = _summary_payload(0x0005) + struct.pack(">I", 1234)
    model = comm._process_packets([(PACKET_TYPE_TACTILE, packet)])

    assert model is not None
    assert model.tactile_source == TACTILE_SOURCE_P4_CAN_RELAY
    assert model.tactile_data_level == TACTILE_DATA_LEVEL_SUMMARY
    assert model.tactile_online is True
    assert model.tactile_module_valid_mask == 0x0005
    assert model.tactile_module_online[:3] == [True, False, True]
    assert model.tactile_last_rx_ts > 0.0


def test_p4_relay_timeout_does_not_confuse_link_online_with_module_mask():
    model = HandModel()
    model.tactile_source = TACTILE_SOURCE_P4_CAN_RELAY
    model.tactile_online = True
    model.tactile_module_valid_mask = 0x7FFF
    model.tactile_module_online = [True] * 15
    model.tactile_last_rx_ts = time.time() - 1.0

    LowerComputerComm._apply_connectivity_visibility(model)

    assert model.tactile_online is False
    assert model.tactile_module_valid_mask == 0
    assert model.tactile_module_online == [False] * 15


def test_fresh_s3_overlay_is_reused_and_stale_s3_falls_back_to_p4():
    class _TactileComm:
        def is_connected(self) -> bool:
            return True

        def get_latest_model(self):
            return None

    backend = HandBackend()
    backend.tactile_comm = _TactileComm()
    p4_state = HandModel()
    p4_state.tactile_source = TACTILE_SOURCE_P4_CAN_RELAY
    p4_state.tactile_online = True
    s3_state = HandModel()
    s3_state.tactile_source = TACTILE_SOURCE_S3_SERIAL
    s3_state.tactile_online = True
    backend._tactile_state = s3_state

    has_new, merged = backend.consume_tactile_state_into(p4_state)
    assert has_new is False
    assert merged is p4_state
    assert merged.tactile_source == TACTILE_SOURCE_S3_SERIAL

    backend._tactile_state.tactile_online = False
    with backend._controller.state_lock:
        backend._controller.current_state.tactile_source = TACTILE_SOURCE_P4_CAN_RELAY
        backend._controller.current_state.tactile_online = True
    has_new, fallback = backend.consume_tactile_state_into(p4_state)
    assert has_new is False
    assert fallback is not None
    assert fallback.tactile_source == TACTILE_SOURCE_P4_CAN_RELAY
