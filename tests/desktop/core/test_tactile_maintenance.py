import struct

from desktop.core.protocol import crc16_ccitt
from desktop.core.tactile_maintenance import (
    REQUEST_TYPE, RESULT_TYPE, RENUMBER, SCAN,
    TactileMaintenanceChannel, parse_result, request_frame, plan_tactile_ids,
)


def test_request_exact_layout_and_crc():
    packet = request_frame(RENUMBER, 0x12345678, selector=4, model=2, new_id=2)
    assert packet[:3] == bytes((0xFE, 14, REQUEST_TYPE))
    assert packet[-1] == 0xFF
    assert packet[3:13] == bytes((2, 1, 0x12, 0x34, 0x56, 0x78, 4, 2, 2, 0))
    assert struct.unpack('>H', packet[13:15])[0] == crc16_ccitt(packet[3:13])
    assert len(request_frame(SCAN, 1, selector=0)) == 16


def test_verified_result_requires_valid_crc_and_exact_length():
    body = bytearray(22)
    body[0] = 2
    body[1] = RENUMBER
    body[2:6] = struct.pack('>I', 9)
    body[8:15] = bytes((4, 5, 2, 25, 0, 1, 1))
    body[16:20] = struct.pack('>I', 1234)
    body[20:22] = struct.pack('>H', crc16_ccitt(body[:20]))
    result = parse_result(bytes(body))
    assert result['token'] == 9
    assert result['status_name'] == 'VERIFIED'
    assert result['old_selector'] == 4 and result['new_selector'] == 5
    assert result['model'] == 2 and result['points'] == 25
    assert result['new_valid'] and result['write_sent'] and not result['old_valid']
    assert parse_result(bytes(body[:-1])) is None
    body[11] ^= 1
    assert parse_result(bytes(body)) is None


def test_channel_rejects_duplicate_send_when_timeout_unknown():
    channel = TactileMaintenanceChannel()
    sent = []
    try:
        channel.request(lambda packet: sent.append(packet) or len(packet), SCAN, selector=0, timeout=.001)
    except TimeoutError:
        pass
    else:
        assert False, 'expected timeout'
    assert len(sent) == 1


def test_tactile_ids_are_unique_per_finger_and_two_cycle_uses_reachable_free_slot():
    rows = [dict(bus=0,id=0),dict(bus=0,id=1),dict(bus=4,id=1)]
    moves = plan_tactile_ids(rows,{(0,0):1,(0,1):0,(4,1):2})
    assert moves == [(0,0,2),(0,1,0),(0,2,1),(4,1,2)]
    assert plan_tactile_ids(rows,{(0,0):None,(0,1):None,(4,1):None}) == []


def test_tactile_full_bus_cycle_and_duplicate_destinations_are_rejected_before_write():
    import pytest
    rows = [dict(bus=0,id=i) for i in range(3)]
    with pytest.raises(ValueError,match='free reachable ID'):
        plan_tactile_ids(rows,{(0,0):1,(0,1):2,(0,2):0})
    with pytest.raises(ValueError,match='unique'):
        plan_tactile_ids(rows,{(0,0):1,(0,1):1,(0,2):2})


def test_reconcile_route_is_separate_from_sensor_id_write():
    from desktop.core.tactile_maintenance import RECONCILE
    packet = request_frame(RECONCILE, 9, selector=2, model=2, new_id=1)
    assert packet[4] == 2
    assert packet[9:13] == bytes((2, 2, 1, 0))
