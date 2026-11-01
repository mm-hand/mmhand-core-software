from copy import deepcopy

from test_id_actions import finish
from desktop.core.tactile_maintenance import SCAN, RENUMBER

class SensorBus:
    port = 'COM_FAKE'
    serial_number = 'S3_TEST_DEVICE'

    def __init__(self, lose_result=False):
        self.inventory = {(0,0):(1,52,0),(3,1):(2,25,10),(4,1):(2,25,13)}
        self.writes = []
        self.scans = []
        self.lose_result = lose_result

    def is_connected(self): return True
    def disconnect(self): pass

    def maintenance_request(self, operation, selector, model=255, new_id=255, **kwargs):
        location = (selector // 3, selector % 3)
        item = self.inventory.get(location)
        response = dict(version=2,status=0 if item else 1,status_name='VERIFIED' if item else 'NONE',
                        old_selector=selector,new_selector=selector,model=item[0] if item else 255,
                        points=item[1] if item else 0,semantic=item[2] if item else 255,
                        old_valid=bool(item),new_valid=False,write_sent=False,stage=0)
        if operation == SCAN:
            self.scans.append(selector)
            return response
        assert operation == RENUMBER and item and item[0] == model
        destination = (location[0],new_id)
        assert destination not in self.inventory
        self.inventory[destination] = self.inventory.pop(location)
        self.writes.append((location[0], location[1], new_id))
        if self.lose_result:
            self.lose_result=False
            raise TimeoutError('result lost after sensor write')
        response.update(old_valid=False,new_valid=True,write_sent=True,new_selector=location[0]*3+new_id)
        return response

def setup_bus(hand,monkeypatch,tmp_path,lose_result=False):
    from desktop.core import tactile_id_actions
    monkeypatch.setattr(tactile_id_actions,'DEVICE_STATE_DIR',tmp_path)
    bus=SensorBus(lose_result)
    hand.tactile_comm=bus; hand.tactile_port=bus.port
    return bus

def test_scan_discovers_multiple_fingers_and_batch_preserves_sensor_identity(hand,monkeypatch,tmp_path):
    bus=setup_bus(hand,monkeypatch,tmp_path)
    assert finish(hand,hand.request_tactile_scan())['status']=='SUCCESS'
    assert bus.scans==list(range(15))
    rows=hand.id_reset_snapshot()['tactile']['rows']
    assert len(rows)==3
    assert finish(hand,hand.request_tactile_renumber({(0,0):None,(3,1):0,(4,1):2}))['status']=='SUCCESS'
    assert bus.writes==[(3,1,0),(4,1,2)]
    assert bus.inventory[(4,2)]==(2,25,13)
    assert len(hand.id_reset_snapshot()['tactile']['rows'])==3

def test_lost_result_is_reconciled_by_rescan_without_duplicate_write(hand,monkeypatch,tmp_path):
    bus=setup_bus(hand,monkeypatch,tmp_path,True)
    finish(hand,hand.request_tactile_scan())
    assert finish(hand,hand.request_tactile_renumber({(0,0):None,(3,1):0,(4,1):2}))['status']=='FAILED'
    assert bus.writes==[(3,1,0)]
    finish(hand,hand.request_tactile_scan())
    assert hand.id_reset_snapshot()['tactile']['recovering']
    assert finish(hand,hand.request_tactile_renumber({}))['status']=='SUCCESS'
    assert bus.writes==[(3,1,0),(4,1,2)]

def test_interrupted_move_restores_preserved_model_route_without_another_id_write(hand,monkeypatch,tmp_path):
    from desktop.core.tactile_maintenance import RECONCILE
    bus=setup_bus(hand,monkeypatch,tmp_path,True)
    finish(hand,hand.request_tactile_scan())
    assert finish(hand,hand.request_tactile_renumber({(0,0):None,(3,1):0,(4,1):None}))['status']=='FAILED'
    bus.inventory[(3,0)] = (1,52,9)  # address default differs from preserved installation
    original_request=bus.maintenance_request
    repairs=[]
    def request(operation, selector, model=255, new_id=255, **kwargs):
        if operation==RECONCILE:
            repairs.append((selector,model,new_id))
            bus.inventory[(selector//3,selector%3)] = (model,25,selector//3*3+new_id)
            return dict(status=0,new_valid=True,write_sent=False)
        return original_request(operation,selector,model,new_id,**kwargs)
    bus.maintenance_request=request
    assert finish(hand,hand.request_tactile_scan())['status']=='SUCCESS'
    assert repairs==[(9,2,1)]
    assert bus.writes==[(3,1,0)]
    row=next(row for row in hand.id_reset_snapshot()['tactile']['rows'] if row['bus']==3)
    assert (row['id'],row['model'],row['points'],row['semantic'])==(0,2,25,10)
