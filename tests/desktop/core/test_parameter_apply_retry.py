"""P4 parameter apply: retry ACK status=8 (contract mismatch) and keep UI state coherent."""
import json
import struct
from copy import deepcopy
from unittest.mock import Mock

import pytest

from desktop.core.comm_layer import LowerComputerComm
from desktop.core.control_parameters import (
    DEFAULT_PATH,
    ControlParameterStore,
    configuration_crc,
    ramp_configuration_crc,
)
from desktop.core.generated_upper_protocol import (
    CMD_CONTROL_PARAMETERS_QUERY,
    CMD_CONTROL_PARAMETERS_SET,
    CMD_CONTROL_RAMP_PARAMETERS_QUERY,
    CMD_CONTROL_RAMP_PARAMETERS_SET,
    PACKET_TYPE_CONTROL_PARAMETERS,
    PACKET_TYPE_CONTROL_RAMP_PARAMETERS,
)


def _applied_packet(data, status=0):
    return (
        PACKET_TYPE_CONTROL_PARAMETERS,
        bytes((1, status)) + struct.pack('>II', data['revision'], configuration_crc(data)),
    )


def _ramp_applied_packet(data, status=0):
    return (
        PACKET_TYPE_CONTROL_RAMP_PARAMETERS,
        bytes((1, status)) + struct.pack('>II', data['ramp_revision'], ramp_configuration_crc(data)),
    )


def _fake_comm(monkeypatch, data, results):
    """results: list of status values per SET attempt; None means no reply (timeout)."""
    comm = LowerComputerComm('')
    monkeypatch.setattr(comm, 'is_connected', lambda: True)
    sends = []
    pending = list(results)

    def send(command):
        frame = command[1]
        sends.append(frame[2])
        if frame[2] == CMD_CONTROL_PARAMETERS_SET:
            status = pending.pop(0) if pending else 0
            if status is not None:
                comm._process_packets([_applied_packet(data, status)])
        elif frame[2] == CMD_CONTROL_PARAMETERS_QUERY:
            comm._process_packets([_applied_packet(data, 0)])
        elif frame[2] == CMD_CONTROL_RAMP_PARAMETERS_SET:
            comm._process_packets([_ramp_applied_packet(data, 0)])
        elif frame[2] == CMD_CONTROL_RAMP_PARAMETERS_QUERY:
            comm._process_packets([_ramp_applied_packet(data, 0)])
        return True

    monkeypatch.setattr(comm, 'send_command', send)
    return comm, sends


def test_contract_mismatch_is_retried_then_succeeds(monkeypatch):
    data = json.loads(DEFAULT_PATH.read_text(encoding='utf-8'))
    comm, sends = _fake_comm(monkeypatch, data, [8, 0])
    assert comm.apply_control_parameters(data) == (data['revision'], configuration_crc(data), data['ramp_revision'], ramp_configuration_crc(data))
    assert sends.count(CMD_CONTROL_PARAMETERS_SET) == 2
    assert sends.count(CMD_CONTROL_PARAMETERS_QUERY) == 1


def test_persistent_contract_mismatch_raises_after_three_attempts(monkeypatch):
    data = json.loads(DEFAULT_PATH.read_text(encoding='utf-8'))
    comm, sends = _fake_comm(monkeypatch, data, [8, 8, 8])
    with pytest.raises(ValueError) as excinfo:
        comm.apply_control_parameters(data)
    assert 'status=8' in str(excinfo.value)
    assert sends.count(CMD_CONTROL_PARAMETERS_SET) == 3
    assert sends.count(CMD_CONTROL_PARAMETERS_QUERY) == 0


def test_no_reply_retries_then_times_out(monkeypatch):
    data = json.loads(DEFAULT_PATH.read_text(encoding='utf-8'))
    comm, sends = _fake_comm(monkeypatch, data, [None, None, None])
    with pytest.raises(TimeoutError):
        comm.apply_control_parameters(data, timeout=0.03)
    assert sends.count(CMD_CONTROL_PARAMETERS_SET) == 3


def _backend(tmp_path):
    from desktop.core.hand_backend import HandBackend
    hand = HandBackend()
    hand._control_parameters = ControlParameterStore(tmp_path / 'params.json')
    hand._control_parameter_sync = 'PENDING'
    comm = Mock()
    comm._protocol_compatible = True
    hand._controller.comm = comm
    hand.is_comm_connected = lambda: True
    return hand


def _proposal(hand):
    proposed = deepcopy(hand.control_parameter_snapshot())
    proposed.pop('crc32', None)
    proposed.pop('ramp_crc32',None)
    return proposed


def test_synchronize_transitions_pending_retry_then_applied(tmp_path):
    hand = _backend(tmp_path)
    hand.comm.apply_control_parameters = Mock(side_effect=[ValueError('P4 parameter revision/CRC not applied: (8, 1, 2)'), (1, 2, 1, 2)])
    proposed = _proposal(hand)
    hand.request_save_control_parameters(proposed)
    ok, message = hand._synchronize_control_parameters(hand._application_epoch)
    assert not ok
    assert hand._control_parameter_sync.startswith('PENDING retry 1/3')
    assert hand._control_parameter_retry_count == 1

    hand._control_parameter_retry_after = 0.0
    ok, message = hand._synchronize_control_parameters(hand._application_epoch)
    assert ok
    assert hand._control_parameter_sync.startswith('APPLIED revision')
    assert hand._control_parameter_retry_count == 0


def test_synchronize_gives_up_after_three_failures(tmp_path):
    hand = _backend(tmp_path)
    hand.comm.apply_control_parameters = Mock(side_effect=ValueError('contract mismatch'))
    hand.request_save_control_parameters(_proposal(hand))
    for _ in range(3):
        hand._control_parameter_retry_after = 0.0
        ok, _message = hand._synchronize_control_parameters(hand._application_epoch)
        assert not ok
    assert hand._control_parameter_sync.startswith('FAILED')
    assert hand.comm.apply_control_parameters.call_count == 3


def test_request_apply_control_parameters_schedules_sync(tmp_path):
    hand = _backend(tmp_path)
    hand._schedule_control_parameter_sync = Mock()
    result = hand.request_apply_control_parameters(_proposal(hand))
    assert result.accepted
    hand._schedule_control_parameter_sync.assert_called_once()
    assert hand._control_parameter_sync == 'PENDING'
