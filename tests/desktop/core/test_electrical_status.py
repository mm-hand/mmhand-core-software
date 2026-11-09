"""CAN label and per-joint encoder state; a dead CAN never blames the encoders."""
import struct

from desktop.core.electrical_status import can_state, joint_states
from desktop.core.protocol import (DISCONNECT_SENTINEL, ENCODER_OPEN_SENTINEL,
                                   ENCODER_SHORT_SENTINEL, parse_sensor_packet)


def test_can_unknown_until_p4_is_ready():
    assert can_state(p4_online=False, connected_for_s=10, has_raw=True, raw_age_s=0.1)['label'] == 'CAN: UNKNOWN'
    assert can_state(p4_online=True, connected_for_s=0.5, has_raw=True, raw_age_s=0.1)['level'] == 'unknown'


def test_can_ok_only_while_raw_keeps_arriving():
    assert can_state(p4_online=True, connected_for_s=10, has_raw=True, raw_age_s=0.1)['label'] == 'CAN: OK'
    assert can_state(p4_online=True, connected_for_s=10, has_raw=True, raw_age_s=5)['label'] == 'CAN: ERROR'
    assert can_state(p4_online=True, connected_for_s=10, has_raw=False, raw_age_s=None)['cause'] == 'can_error'


def test_can_down_marks_every_joint_as_can_error():
    assert joint_states(can_level='can_error') == ['can_error'] * 21
    assert joint_states(can_level='unknown') == ['unknown'] * 21


def test_can_ok_maps_sentinels_and_invalid_joints():
    faults = [''] * 21
    faults[0] = 'open'
    faults[4] = 'short'
    states = joint_states(can_level='ok', faults=faults, invalid_joints=[5])
    assert states[0] == 'open' and states[4] == 'short'
    assert states[5] == 'error' and states[1] == 'ok'


def test_parse_sensor_packet_recognises_the_sentinels():
    payload = bytearray(21 * 2)
    for i in range(21):
        struct.pack_into('>H', payload, i * 2, 100 + i)
    struct.pack_into('>H', payload, 0, ENCODER_OPEN_SENTINEL)
    struct.pack_into('>H', payload, 2, ENCODER_SHORT_SENTINEL)
    struct.pack_into('>H', payload, 4, DISCONNECT_SENTINEL)
    values, errors, faults = parse_sensor_packet(bytes(payload))
    assert faults[:4] == ['open', 'short', 'error', '']
    assert errors[:3] == [True, True, True]
    assert values[3] == 103 and values[1] == 0 and values[2] == 0
