from desktop.core.communication.connection_diagnostics import describe_connection_failure
from desktop.core.comm_layer import LowerComputerComm


def test_usb_open_is_not_evidence_of_protocol_or_fresh_status():
    diag = dict(port='COM12', rx_bytes=0, rx_packets=0, protocol_compatible=False)
    assert 'no bytes' in describe_connection_failure(diag)
    diag['rx_bytes'] = 200
    assert 'no valid P4 protocol' in describe_connection_failure(diag)
    diag['rx_packets'] = 1
    assert 'handshake was not accepted' in describe_connection_failure(diag)
    diag['protocol_compatible'] = True
    assert 'no fresh system status' in describe_connection_failure(diag)
    diag['error'] = 'PermissionError: port occupied'
    assert 'port occupied' in describe_connection_failure(diag)


def test_connection_totals_are_not_reset_by_rate_reporting():
    comm = LowerComputerComm('')
    comm._connection_rx_bytes = 300
    comm._connection_rx_packets = 2
    comm._reset_rx_diag()
    diag = comm.connection_diagnostics()
    assert diag['rx_bytes'] == 300 and diag['rx_packets'] == 2
    assert not diag['rx_thread_alive'] and not diag['tx_thread_alive']
