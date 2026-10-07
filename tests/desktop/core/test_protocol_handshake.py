"""Handshake confirmation gates motion, without ever gating STOP or status reads."""

import struct
from types import SimpleNamespace

from desktop.core import comm_layer, protocol


def _comm():
    comm = comm_layer.LowerComputerComm("")
    comm.is_connected = lambda: True
    return comm


def _ack(comm, status):
    return comm._process_packets([(
        protocol.PACKET_TYPE_PROTO_ACK,
        bytes([protocol.CMD_PROTOCOL_HELLO, 1, status]) + struct.pack(">I", 100),
    )])


def test_unconfirmed_or_rejected_contract_blocks_all_motion_forms():
    comm = _comm()
    for cmd in ("start", "calibrate", "run_feedback",
                ("angle_live", [0.0] * 21),
                ("joint_target_run_masked", [0.0] * 21, 1),
                ("motor_pos_abs", [0] * 27)):
        assert not comm.send_command(cmd)
    assert comm.tx_queue.empty()
    _ack(comm, protocol.PROTO_ACK_STATUS_CONTRACT_MISMATCH)
    assert not comm._protocol_compatible
    assert comm.send_command("stop")
    assert list(comm.tx_queue.queue) == ["stop"]


def test_only_positive_hello_confirmation_opens_contract_gate():
    comm = _comm()
    model = _ack(comm, protocol.PROTO_ACK_STATUS_OK)
    assert comm._protocol_compatible and model.protocol_compatible
    assert comm.send_command("start")
    _ack(comm, protocol.PROTO_ACK_STATUS_CONTRACT_MISMATCH)
    assert not comm.send_command("start")


def test_hello_retry_is_two_hz_even_when_motion_heartbeat_is_disabled(monkeypatch):
    comm = _comm()
    sent = []
    comm._transport = SimpleNamespace(is_open=True, write=sent.append)
    monkeypatch.setattr(comm_layer.time, "time", lambda: 100.0)
    comm._maybe_send_host_heartbeat()
    comm._maybe_send_host_heartbeat()
    assert len(sent) == 1
    assert sent[0][2] == protocol.CMD_PROTOCOL_HELLO
    monkeypatch.setattr(comm_layer.time, "time", lambda: 100.5)
    comm._maybe_send_host_heartbeat()
    assert len(sent) == 2
