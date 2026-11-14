"""Motor scanning, renumbering and operation state."""

from unittest.mock import Mock

import pytest
from _clock_helpers import stamp
from _conftest_support import wait_operation as finish

from desktop.core.id_protocol import BEGIN, BUZZ, CAPABILITY, END, PROBE, RENAME
from desktop.core.protocol import HOST_PROTOCOL_VERSION

def test_motor_and_direct_s3_tactile_operations_are_available(hand):
    from desktop.core.comm_layer import TactileSerialComm
    assert set(hand.id_reset_snapshot()) == {'motor'}
    assert not hasattr(hand, 'request_tactile_renumber')
    assert not hasattr(TactileSerialComm, 'maintenance_request')

def test_calibrating_badge_follows_current_joint_and_stop_clears_it(hand):
    from desktop.core.calibration_workflow import CalibrationWorkflowEvent
    hand._calibration_operation_id='test-calibration'
    hand._application_busy='test-calibration'
    hand._application_operations.append(dict(operation_id='test-calibration',name='Selected-joint Calibration',status='RUNNING'))
    hand._motor_transactions.update({0:'PENDING',1:'PENDING'})
    hand._application_calibration_event(CalibrationWorkflowEvent(0,'joint_started',True,'starting',1,3))
    rows=hand.application_snapshot()['joints']
    assert rows[0]['motor_quality']==['CALIBRATING']
    assert rows[1]['motor_quality']==['PENDING']
    hand.stop_motion()
    assert hand.application_snapshot()['joints'][0]['motor_quality']==['FAILED']

class MotorDevice:
    def __init__(self):
        self.writes = []
        self.probes = []
        self.buzzes = []
        self.buzz_status = 0
        self.inventory={(0,i) for i in range(5)}
        self.lose_result = False
    def is_connected(self): return True
    def disconnect(self): pass
    def send_command(self, _command): return True
    def maintenance_request(self,session,operation,bus=0,old_id=0,new_id=0,**kwargs):
        reply=dict(version=3,status=0,present=0,stage=0)
        if operation==CAPABILITY: reply['present']=1
        if operation==PROBE:
            self.probes.append((bus, old_id))
            reply['present']=int((bus,old_id) in self.inventory)
        if operation==BUZZ:
            self.buzzes.append((bus, old_id, new_id))
            reply['status'] = self.buzz_status
            reply['present'] = int((bus, old_id) in self.inventory)
        if operation==RENAME:
            assert (bus,old_id) in self.inventory and (bus,new_id) not in self.inventory
            self.inventory.remove((bus,old_id)); self.inventory.add((bus,new_id))
            self.writes.append((bus,old_id,new_id))
            if self.lose_result:
                self.lose_result=False
                raise TimeoutError('lost result after EEPROM write')
        return reply

@pytest.fixture
def renumber_device(hand, monkeypatch):
    from desktop.core import calibration_snapshot

    invalidate = Mock()
    monkeypatch.setattr(calibration_snapshot, 'invalidate_calibration_snapshot_scope', invalidate)
    monkeypatch.setattr(hand._controller, '_invalidate_coordinate_anchors', Mock())
    state = hand._controller.current_state
    state.has_product_fingerprint = True
    state.product_fingerprint = dict(device_id='1234567890ABCDEF', protocol_version=HOST_PROTOCOL_VERSION)
    device = MotorDevice()
    hand._controller.comm = device
    return device, invalidate


def test_motor_scan_replaces_inventory_and_covers_both_ranges(hand, renumber_device, monkeypatch, tmp_path):
    from desktop.core import id_actions

    monkeypatch.setattr(id_actions, 'DEVICE_STATE_DIR', tmp_path, raising=False)
    old_journal = tmp_path / 'id_reset_1234567890ABCDEF.json'
    old_journal.write_text('invalid legacy record', encoding='utf-8')
    device, _ = renumber_device
    device.inventory = {(0, 0), (1, 30), (2, 31), (3, 253)}
    assert finish(hand, hand.request_id_scan())['status'] == 'SUCCESS'
    assert device.probes == [(bus, identifier) for bus in range(4) for identifier in range(31)]
    assert hand.id_reset_snapshot()['motor']['rows'] == [dict(bus=0, id=0), dict(bus=1, id=30)]

    device.inventory = {(2, 31), (3, 253)}
    device.probes.clear()
    stamp(hand._controller.current_state, 'system_status_last_rx_ts')
    assert finish(hand, hand.request_id_scan(253))['status'] == 'SUCCESS'
    assert device.probes == [(bus, identifier) for bus in range(4) for identifier in range(254)]
    assert hand.id_reset_snapshot()['motor']['rows'] == [dict(bus=2, id=31), dict(bus=3, id=253)]
    assert finish(hand, hand.request_motor_renumber({(2, 31): 31, (3, 253): 8}))['status'] == 'SUCCESS'
    assert device.writes == [(3, 253, 8)]
    assert old_journal.read_text(encoding='utf-8') == 'invalid legacy record'


@pytest.mark.parametrize('occupied', [(0, 31), (3, 31), (3, 15)])
def test_motor_renumber_rejects_an_unscanned_target(hand, renumber_device, occupied):
    device, _ = renumber_device
    device.inventory = {(0, 0)}
    finish(hand, hand.request_id_scan())
    device.inventory.add(occupied)
    result = finish(hand, hand.request_motor_renumber({(0, 0): occupied[1]}))
    assert result['status'] == 'FAILED'
    assert 'occupied' in result['message']
    assert not device.writes


def test_missing_source_stops_batch_before_any_write(hand, renumber_device):
    device, invalidate = renumber_device
    device.inventory = {(0, 1), (1, 2)}
    finish(hand, hand.request_id_scan())
    device.inventory.remove((1, 2))
    result = finish(hand, hand.request_motor_renumber({(0, 1): 3, (1, 2): 4}))
    assert result['status'] == 'FAILED'
    assert 'BUS1 ID2 did not answer' in result['message']
    assert not device.writes
    invalidate.assert_not_called()


def test_cycle_checks_only_its_bus_for_a_hidden_253(hand, renumber_device):
    device, _ = renumber_device
    device.inventory = {(0, 1), (0, 2), (0, 253), (2, 253)}
    finish(hand, hand.request_id_scan())
    targets = {(0, 1): 2, (0, 2): 1}
    result = finish(hand, hand.request_motor_renumber(targets))
    assert result['status'] == 'FAILED'
    assert 'ID 253 is occupied on BUS0' in result['message']
    assert 'Scan (0–253)' in result['message']
    assert not device.writes

    device.inventory.remove((0, 253))
    stamp(hand._controller.current_state, 'system_status_last_rx_ts')
    finish(hand, hand.request_id_scan())
    device.probes.clear()
    assert finish(hand, hand.request_motor_renumber(targets))['status'] == 'SUCCESS'
    assert device.writes == [(0, 1, 253), (0, 2, 1), (0, 253, 2)]
    assert [(bus, identifier) for bus, identifier in device.probes if identifier == 253] == [(0, 253)]


def test_lost_result_stops_batch_and_full_scan_allows_new_targets(hand, renumber_device):
    device, _ = renumber_device
    device.inventory = {(0, 1), (0, 2)}
    device.lose_result = True
    finish(hand, hand.request_id_scan())
    result = finish(hand, hand.request_motor_renumber({(0, 1): 2, (0, 2): 1}))
    assert result['status'] == 'FAILED'
    assert 'BUS0 ID1 → ID253' in result['message']
    assert 'Scan (0–253)' in result['message']
    assert device.writes == [(0, 1, 253)]
    assert not hand.id_reset_snapshot()['motor']['scanned']
    assert not hand.request_motor_renumber({(0, 1): 2, (0, 2): 1}).accepted

    device.inventory.remove((0, 2))
    stamp(hand._controller.current_state, 'system_status_last_rx_ts')
    finish(hand, hand.request_id_scan(253))
    assert hand.id_reset_snapshot()['motor']['rows'] == [dict(bus=0, id=253)]
    assert finish(hand, hand.request_motor_renumber({(0, 253): 5}))['status'] == 'SUCCESS'
    assert device.writes == [(0, 1, 253), (0, 253, 5)]
    assert device.inventory == {(0, 5)}


def test_unchanged_motor_missing_does_not_block_renumber(hand, renumber_device):
    device, invalidate = renumber_device
    device.inventory = {(2, 1), (2, 10), (2, 253)}
    finish(hand, hand.request_id_scan())
    device.probes.clear()
    device.inventory.remove((2, 10))

    result = finish(hand, hand.request_motor_renumber({(2, 1): 15, (2, 10): 10}))
    assert result['status'] == 'SUCCESS', result
    assert 'BUS2 ID10' in result['message']
    assert device.writes == [(2, 1, 15)]
    assert all(identifier != 253 for _, identifier in device.probes)
    invalidate.assert_called_once()
    assert hand.id_reset_snapshot()['motor']['rows'] == [dict(bus=2, id=15)]


def test_motor_buzz_requires_a_scan(hand):
    device = MotorDevice()
    hand._controller.comm = device
    result = hand.request_motor_buzz(0, 3)
    assert not result.accepted and 'scan all buses' in result.message
    assert not device.buzzes

def test_motor_buzz_moves_only_the_addressed_servo(hand):
    from desktop.core import id_actions
    device = MotorDevice()
    device.inventory = {(0, 3), (2, 7)}
    hand._controller.comm = device
    finish(hand, hand.request_id_scan())
    result = finish(hand, hand.request_motor_buzz(2, 7))
    assert result['status'] == 'SUCCESS', result
    assert device.buzzes == [(2, 7, id_actions.BUZZ_AMPLITUDE)]
    assert not device.writes

def test_motor_buzz_repeats_in_one_session_until_released(hand):
    device = MotorDevice()
    hand._controller.comm = device
    finish(hand, hand.request_id_scan())
    device.maintenance_request = Mock(wraps=device.maintenance_request)
    result = finish(hand, hand.request_motor_buzz(0, 3, held=lambda: len(device.buzzes) < 3))
    assert result['status'] == 'SUCCESS', result
    assert device.buzzes == [(0, 3, 128)] * 3
    calls = [call.args for call in device.maintenance_request.call_args_list]
    assert [args[1] for args in calls] == [BEGIN, BUZZ, BUZZ, BUZZ, END]
    assert len({args[0] for args in calls}) == 1


def test_motor_buzz_reports_an_unsupported_firmware(hand):
    device = MotorDevice()
    device.buzz_status = 3
    hand._controller.comm = device
    finish(hand, hand.request_id_scan())
    result = finish(hand, hand.request_motor_buzz(0, 3))
    assert result['status'] == 'FAILED'
    assert 'does not support motor buzz' in result['message']
