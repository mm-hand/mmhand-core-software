"""Low-rate CAN health remains observable without logging every SPI warning."""
from desktop.core import comm_layer


def test_s3_can_health_is_visible_and_bounded(monkeypatch):
    records=[]
    monkeypatch.setattr(comm_layer,'log_session',lambda tag,text:records.append((tag,text)))
    comm=comm_layer.TactileSerialComm('COM_FAKE')
    for line in (
        '[CAN WARN] repeated warning\n',
        '[S3 CAN] state=1 txOk=7 txFail=0\n',
        '[S3 CAN FAULT] reason=tx_saturated pending=513 txFail=91\r\n',
        '[S3 CAN RECOVERED] up=8200 outageMs=3100 pending=7\r\n',
        '[S3 CAN] ' + 'x'*400 + '\n',
    ):
        for value in line.encode('ascii'): comm._append_firmware_log_byte(value)
    assert records == [
        ('S3 CAN', 'state=1 txOk=7 txFail=0'),
        ('S3 CAN FAULT', 'reason=tx_saturated pending=513 txFail=91'),
        ('S3 CAN RECOVERED', 'up=8200 outageMs=3100 pending=7'),
        ('S3 CAN', 'x'*240),
    ]
    assert not comm._firmware_text_buffer
