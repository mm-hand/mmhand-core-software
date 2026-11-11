"""Motor renumbering recovery, operation state and removed tactile ID APIs."""

from _clock_helpers import stamp
import json
import time
import pytest
from unittest.mock import Mock

from desktop.core.id_protocol import CAPABILITY, BEGIN, END, PROBE, RENAME, BUZZ
from desktop.core.protocol import HOST_PROTOCOL_VERSION

def finish(hand, result):
    assert result.accepted, result.message
    deadline=time.monotonic()+2
    while time.monotonic()<deadline:
        operation=next(op for op in hand._application_operations if op['operation_id']==result.operation_id)
        if operation['status'] in ('SUCCESS','FAILED'): return operation
        time.sleep(.005)
    pytest.fail('ID operation did not finish')

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
    def __init__(self, lose_result=False, locked=True):
        self.writes = []
        self.probes = []
        self.buzzes = []
        self.buzz_status = 0
        self.inventory={(0,i) for i in range(5)}
        self.lose_result=lose_result
        self.locked=locked
        self.profiles = {}
    def is_connected(self): return True
    def disconnect(self): pass
    def send_command(self, _command): return True
    def maintenance_request(self,session,operation,bus=0,old_id=0,new_id=0,**kwargs):
        reply=dict(version=3,status=0,present=0,stage=int(self.locked))
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

def test_motor_scan_covers_only_zero_through_thirty_on_every_bus(hand, monkeypatch, tmp_path):
    from desktop.core import id_actions
    monkeypatch.setattr(id_actions, 'DEVICE_STATE_DIR', tmp_path)
    device = MotorDevice()
    device.inventory = {(0, 0), (1, 30), (2, 31), (3, 253)}
    hand._controller.comm = device
    finish(hand, hand.request_id_scan())
    assert device.probes == [(bus, identifier) for bus in range(4) for identifier in range(31)]
    assert hand.id_reset_snapshot()['motor']['rows'] == [dict(bus=0, id=0), dict(bus=1, id=30)]
    assert not device.writes

def test_failed_id_reconciliation_preserves_scanned_addresses(hand, monkeypatch, tmp_path):
    import json
    from desktop.core import id_actions

    device_id = '1234567890ABCDEF'
    monkeypatch.setattr(id_actions, 'DEVICE_STATE_DIR', tmp_path)
    monkeypatch.setattr(hand._controller, 'servo_abs_zero_device_id', lambda _: device_id)
    path = tmp_path / f'id_reset_{device_id}.json'
    journal = {'inventory': [{'bus': 0, 'id': 0}], 'moves': [[0, 0, 2]], 'completed': 0}
    path.write_text(json.dumps(journal), encoding='utf-8')
    device = MotorDevice()
    device.inventory = {(0, 1)}
    hand._controller.comm = device

    result = finish(hand, hand.request_id_scan())
    scan = hand.id_reset_snapshot()['motor']
    assert result['status'] == 'FAILED'
    assert 'Recorded batch does not match' in scan['message']
    assert scan['rows'] == [{'bus': 0, 'id': 1}]
    assert not scan['scanned']
    assert not hand.request_motor_renumber({}).accepted
    assert not hand.request_motor_buzz(0, 1).accepted
    assert json.loads(path.read_text(encoding='utf-8')) == journal
    assert not device.writes and not device.buzzes


@pytest.mark.parametrize('occupied', [(0, 31), (3, 31), (2, 253)])
def test_motor_quick_scan_still_rejects_unscanned_target_or_temporary_conflict(hand, monkeypatch, tmp_path, occupied):
    from desktop.core import id_actions
    monkeypatch.setattr(id_actions, 'DEVICE_STATE_DIR', tmp_path)
    monkeypatch.setattr(hand._controller, 'servo_abs_zero_device_id', lambda _: '1234567890ABCDEF')
    state = hand._controller.current_state
    state.has_product_fingerprint = True
    state.product_fingerprint = dict(device_id='1234567890ABCDEF', protocol_version=HOST_PROTOCOL_VERSION)
    device = MotorDevice()
    device.inventory = {(0, 0), occupied}
    hand._controller.comm = device
    finish(hand, hand.request_id_scan())
    result = finish(hand, hand.request_motor_renumber({(0, 0): 31}))
    assert result['status'] == 'FAILED'
    assert 'occupied' in result['message']
    assert not device.writes

@pytest.mark.parametrize('locked',[True,False])
def test_motor_lost_result_resumes_after_scan_without_duplicate_write(hand,monkeypatch,tmp_path,locked):
    from desktop.core import id_actions,calibration_snapshot
    monkeypatch.setattr(id_actions,'DEVICE_STATE_DIR',tmp_path)
    monkeypatch.setattr(calibration_snapshot,'invalidate_calibration_snapshot_scope',Mock())
    monkeypatch.setattr(hand._controller,'_invalidate_coordinate_anchors',Mock())
    monkeypatch.setattr(hand._controller,'servo_abs_zero_device_id',lambda _: '1234567890ABCDEF')
    state=hand._controller.current_state
    state.has_product_fingerprint=True
    state.product_fingerprint=dict(device_id='1234567890ABCDEF',protocol_version=HOST_PROTOCOL_VERSION)
    device=MotorDevice(lose_result=True,locked=locked); hand._controller.comm=device
    finish(hand,hand.request_id_scan())
    result=finish(hand,hand.request_motor_renumber({(0,i):i+5 for i in range(5)}))
    assert result['status']=='FAILED' and device.writes==[(0,0,253)], result
    hand._controller.current_state.system_status_last_rx_ts=time.time()
    stamp(hand._controller.current_state, 'system_status_last_rx_ts')
    finish(hand,hand.request_id_scan())
    hand._controller.current_state.system_status_last_rx_ts=time.time()
    stamp(hand._controller.current_state, 'system_status_last_rx_ts')
    recovered=hand.id_reset_snapshot()['motor']
    assert recovered['recovering']
    assert next(row for row in recovered['rows'] if row['id']==253)['new_id']==5
    result=finish(hand,hand.request_motor_renumber({}))
    assert result['status']=='SUCCESS'
    assert device.writes.count((0,0,253))==1
    assert device.inventory=={(0,i) for i in range(5,10)}
    assert not device.profiles  # Profile configuration belongs to online checks.

@pytest.mark.parametrize('recovering', [False, True])
def test_unchanged_motor_missing_does_not_block_renumber(hand, monkeypatch, tmp_path, recovering):
    from desktop.core import id_actions, calibration_snapshot

    device_id = '1234567890ABCDEF'
    monkeypatch.setattr(id_actions, 'DEVICE_STATE_DIR', tmp_path)
    invalidate = Mock()
    monkeypatch.setattr(calibration_snapshot, 'invalidate_calibration_snapshot_scope', invalidate)
    monkeypatch.setattr(hand._controller, '_invalidate_coordinate_anchors', Mock())
    monkeypatch.setattr(hand._controller, 'servo_abs_zero_device_id', lambda _: device_id)
    device = MotorDevice()
    hand._controller.comm = device
    journal_path = tmp_path / f'id_reset_{device_id}.json'
    moves = [(2, 1, 253), (2, 253, 15)]
    if recovering:
        journal_path.write_text(json.dumps(dict(
            inventory=[dict(bus=2, id=1), dict(bus=2, id=10)],
            moves=moves, completed=2, status='NEEDS_RECOVERY')), encoding='utf-8')
        device.inventory = {(2, 15)}
    else:
        device.inventory = {(2, 1), (2, 10)}
    assert finish(hand, hand.request_id_scan())['status'] == 'SUCCESS'
    device.inventory.discard((2, 10))

    targets = {} if recovering else {(2, 1): 15, (2, 10): 10}
    result = finish(hand, hand.request_motor_renumber(targets))
    assert result['status'] == 'SUCCESS', result
    assert 'BUS2 ID10' in result['message']
    assert device.writes == ([] if recovering else moves)
    assert invalidate.call_count == (0 if recovering else 1)
    assert hand.id_reset_snapshot()['motor']['rows'] == [dict(bus=2, id=15)]
    assert json.loads(journal_path.read_text(encoding='utf-8'))['status'] == 'SUCCESS'


def test_motor_buzz_requires_a_scan(hand, monkeypatch, tmp_path):
    from desktop.core import id_actions
    monkeypatch.setattr(id_actions, 'DEVICE_STATE_DIR', tmp_path)
    device = MotorDevice()
    hand._controller.comm = device
    result = hand.request_motor_buzz(0, 3)
    assert not result.accepted and 'scan all buses' in result.message
    assert not device.buzzes

def test_motor_buzz_moves_only_the_addressed_servo(hand, monkeypatch, tmp_path):
    from desktop.core import id_actions
    monkeypatch.setattr(id_actions, 'DEVICE_STATE_DIR', tmp_path)
    device = MotorDevice()
    device.inventory = {(0, 3), (2, 7)}
    hand._controller.comm = device
    finish(hand, hand.request_id_scan())
    result = finish(hand, hand.request_motor_buzz(2, 7))
    assert result['status'] == 'SUCCESS', result
    assert device.buzzes == [(2, 7, id_actions.BUZZ_AMPLITUDE)]
    assert not device.writes

def test_motor_buzz_reports_an_unsupported_firmware(hand, monkeypatch, tmp_path):
    from desktop.core import id_actions
    monkeypatch.setattr(id_actions, 'DEVICE_STATE_DIR', tmp_path)
    device = MotorDevice()
    device.buzz_status = 3
    hand._controller.comm = device
    finish(hand, hand.request_id_scan())
    result = finish(hand, hand.request_motor_buzz(0, 3))
    assert result['status'] == 'FAILED'
    assert 'does not support motor buzz' in result['message']
