import json
from copy import deepcopy
import struct
from pathlib import Path
import pytest
from desktop.core.control_parameters import ControlParameterStore, DEFAULT_PATH, configuration_crc, configuration_payload
from desktop.core.comm_layer import LowerComputerComm
from desktop.core.generated_upper_protocol import CMD_CONTROL_PARAMETERS_QUERY, PACKET_TYPE_CONTROL_PARAMETERS


def test_persist_validate_restore_and_invalid_current_is_not_overwritten(tmp_path):
    path=tmp_path/'control.json'
    store=ControlParameterStore(path)
    original=store.snapshot()
    proposal=deepcopy({key:value for key,value in original.items() if key!='crc32'})
    proposal['joint_kp']['aa']=2.0
    saved=store.save(proposal)
    assert saved['revision']==original['revision']+1 and saved['crc32']!=original['crc32']
    assert ControlParameterStore(path).snapshot()==saved
    defaults=store.restore_defaults()
    assert defaults['joint_kp']['aa']==original['joint_kp']['aa']
    assert defaults['revision']==saved['revision']+1
    path.write_text('{corrupt',encoding='utf-8')
    invalid=ControlParameterStore(path)
    assert invalid.error and path.read_text()=='{corrupt'
    invalid.restore_defaults()
    assert not invalid.error and json.loads(path.read_text())['schema_version']==5


def test_bounds_match_generated_schema_order():
    from desktop.core.control_parameters import BOUNDS
    schema = json.loads((Path(__file__).resolve().parents[3] / 'desktop/resources/config/control_parameters.schema.json').read_text(encoding='utf-8'))
    assert [(field['group'], field['name']) for field in schema['fields']] == [
        (group, name) for group, fields in BOUNDS.items() for name in fields]


def test_parameter_apply_requires_separate_matching_ram_readback(monkeypatch):
    data=json.loads(DEFAULT_PATH.read_text())
    comm=LowerComputerComm('')
    monkeypatch.setattr(comm,'is_connected',lambda:True)
    commands=[]
    def send(command):
        commands.append(command[1][2])
        result=bytes((1,0))+struct.pack('>II',data['revision'],configuration_crc(data))
        comm._process_packets([(PACKET_TYPE_CONTROL_PARAMETERS,result)])
        return True
    monkeypatch.setattr(comm,'send_command',send)
    assert comm.apply_control_parameters(data)==(data['revision'],configuration_crc(data))
    assert commands[-1]==CMD_CONTROL_PARAMETERS_QUERY and len(commands)==2
    from desktop.core.generated_upper_protocol import COMMAND_PAYLOAD_LENGTHS, CMD_CONTROL_PARAMETERS_SET
    assert len(configuration_payload(data))==COMMAND_PAYLOAD_LENGTHS[CMD_CONTROL_PARAMETERS_SET]


def test_parameter_readback_mismatch_cannot_authorize_controls(monkeypatch):
    data=json.loads(DEFAULT_PATH.read_text()); comm=LowerComputerComm('')
    monkeypatch.setattr(comm,'is_connected',lambda:True)
    def send(command):
        crc=configuration_crc(data) ^ int(command[1][2]==CMD_CONTROL_PARAMETERS_QUERY)
        comm._process_packets([(PACKET_TYPE_CONTROL_PARAMETERS,bytes((1,0))+struct.pack('>II',data['revision'],crc))])
        return True
    monkeypatch.setattr(comm,'send_command',send)
    with pytest.raises(ValueError,match='readback differs'):
        comm.apply_control_parameters(data)
