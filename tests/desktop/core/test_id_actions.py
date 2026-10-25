"""Motor renumbering recovery, operation state and removed tactile ID APIs."""

from _clock_helpers import stamp
import time
import pytest
from unittest.mock import Mock
from test_application_actions import hand
from desktop.core.id_protocol import CAPABILITY, BEGIN, END, PROBE, RENAME
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
    assert set(hand.id_reset_snapshot()) == {'motor', 'tactile'}
    assert hasattr(hand, 'request_tactile_renumber')
    assert hasattr(TactileSerialComm, 'maintenance_request')


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
