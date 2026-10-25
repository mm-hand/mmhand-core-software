"""Online firmware reports, blocked motors and the sole native coordinate bundle."""
import struct
import time
from unittest.mock import Mock
import pytest
from desktop.core.comm_layer import LowerComputerComm
from desktop.core.data_models import HandModel
from desktop.core.protocol import (PACKET_TYPE_SERVO_ONLINE_STATUS, ServoOnlineCheck,
                                   parse_servo_online_status_packet)
from desktop.core.device_state_store import BundleMultiTurnStore
from desktop.core.calibration_snapshot import read_snapshot
from _clock_helpers import stamp


def test_online_packet_reaches_published_model_with_versions_and_generation():
    comm = LowerComputerComm('')
    rows = [(12, 3, 9, 2), (19, 3, 10, 1), (1, 4, 0, 1)] + [(0, 0, 0, 0)] * 24
    payload = b''.join(struct.pack('>IBBB', *row) for row in rows)
    model = comm._state_reducer.process_packets(comm, [(PACKET_TYPE_SERVO_ONLINE_STATUS, payload + struct.pack('>I', 2000))])
    assert model.has_servo_online_status
    assert model.servo_firmware_minor[:3] == [9, 10, 0]
    assert model.servo_online_generation[:3] == [12, 19, 1]
    assert model.servo_online_check[:3] == [2, 1, 1]
    assert model.received_at('servo_online_check_last_rx_ts') > 0
    assert model.p4_time_ms == 2000


@pytest.mark.parametrize('payload', [b'', bytes(188), bytes(190), bytes(6) + b'\x05' + bytes(182)])
def test_malformed_online_packet_does_not_become_a_version(payload):
    assert parse_servo_online_status_packet(payload) is None


def test_new_hand_native_bundle_has_no_fabricated_motor_calibration(tmp_path):
    identity = dict(device_id='0000123456789ABC', urdf_repo_path='models/hand.urdf',
                    product_fingerprint=dict(schema_version=2, feedback_mapping_version=4, hardware_map_crc=123))
    store = BundleMultiTurnStore(tmp_path / 'new.mmcal.json')
    saved = store.save(identity=identity, valid_motor_mask=3, logical_abs=[0] * 27)
    assert int(saved['valid_motor_mask'], 0) == 3
    payload = read_snapshot(store.path)
    assert int(payload['motor_abs_limit_valid_bitmap'], 0) == 0
    assert not any(payload['joint_calib_results'])
    assert int(payload['manual_reference_valid_bitmap'], 0) == 0
    assert 'multiturn_anchor_raw' not in payload
    assert store.load(identity=identity)['logical_abs'] == [0] * 27


def test_low_or_failed_motor_check_blocks_zero_before_hardware_command():
    from desktop.core.core_logic import HandController
    state = HandModel()
    state.has_system_status_data = state.has_servo_route_status = state.has_servo_online_status = True
    state.system_state = 1
    state.system_status_last_rx_ts = time.time()
    stamp(state, 'system_status_last_rx_ts')
    state.servo_online_bitmap = state.servo_route_controllable_bitmap = 1
    for status in (ServoOnlineCheck.PENDING, ServoOnlineCheck.UNSUPPORTED_FIRMWARE,
                   ServoOnlineCheck.VERSION_READ_FAILED, ServoOnlineCheck.CONFIGURATION_FAILED):
        state.servo_online_check[0] = status
        assert not HandController._coordinate_safe_state(state, 1)
    state.servo_online_check[0] = ServoOnlineCheck.READY
    assert HandController._coordinate_safe_state(state, 1)


def test_motor_maintenance_no_longer_returns_a_profile():
    from desktop.core.id_protocol import MaintenanceChannel
    channel = MaintenanceChannel()
    data = struct.pack('>IIBBBBBBBB', 9, 5, 3, 1, 7, 0, 0, 1, 3, 1)
    assert channel.feed(data)
    assert 'profile' not in channel.responses[9]
    assert not channel.feed(data + bytes(3))


def test_new_hand_cannot_create_coordinate_file_without_valid_identity(tmp_path):
    from desktop.core.device_state_store import DeviceStateStoreError
    store = BundleMultiTurnStore(tmp_path / 'invalid.mmcal.json')
    with pytest.raises(DeviceStateStoreError):
        store.save(identity={}, valid_motor_mask=1, logical_abs=[0] * 27)
    assert not store.path.exists()
