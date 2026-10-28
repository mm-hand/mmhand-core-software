"""会话日志必须持续落盘：完整行进文件，UI 镜像仍有界。"""

from pathlib import Path

from desktop.core.session_log import SessionLog


def test_session_log_persists_full_lines_and_keeps_ui_bounded(tmp_path):
    log = SessionLog(max_lines=100, max_line_chars=40)
    path = log.start(str(tmp_path))
    assert path and Path(path).exists()

    long_line = 'x' * 120
    log.add('HOST_CALIB', long_line)
    for index in range(120):
        log.add('APP', f'line {index}')
    log.close_and_save()

    text = Path(path).read_text(encoding='utf-8')
    assert long_line in text                     # 文件保留完整行
    assert 'line 119' in text                    # 关闭前的最后几行不会丢
    assert 'start ' in text                      # 启动前缓冲的 start 行也落盘

    ui = log.snapshot()
    assert len(ui) == 100                        # UI 环形缓冲仍有上限
    assert max(len(line) for line in ui) <= 40   # UI 行仍截断
    assert log.close_and_save() == path          # 幂等


def test_session_log_without_start_keeps_memory_only(tmp_path):
    log = SessionLog(max_lines=5, max_line_chars=20)
    log.add('APP', 'not persisted')
    assert log.snapshot()
    assert not list(Path(tmp_path).glob('session_*.log'))


def test_session_log_unwritable_directory_does_not_raise(tmp_path):
    blocker = tmp_path / 'blocker'
    blocker.write_text('not a directory', encoding='utf-8')
    log = SessionLog()
    assert log.start(str(blocker)) is None
    log.add('APP', 'still in memory')
    assert any('still in memory' in line for line in log.snapshot())


def test_close_before_start_does_not_disable_later_logging(tmp_path):
    """测试里 close_application 会调 close；没有 start 时不能把全局日志关死。"""
    log = SessionLog()
    assert log.close_and_save() is None
    path = log.start(str(tmp_path))
    log.add('APP', 'after close-before-start')
    log.close_and_save()
    assert 'after close-before-start' in Path(path).read_text(encoding='utf-8')


def test_calibration_commands_ack_and_firmware_share_one_session_log(tmp_path, monkeypatch):
    import struct

    from desktop.core import comm_layer
    from desktop.core.communication import state_reducer
    from desktop.core.protocol import CMD_CLEAR_CALIBRATION, PACKET_TYPE_PROTO_ACK

    log = SessionLog()
    path = Path(log.start(str(tmp_path)))
    monkeypatch.setattr(comm_layer, 'log_session', log.add)
    monkeypatch.setattr(state_reducer, 'log_session', log.add)
    comm = comm_layer.LowerComputerComm('')
    comm.port = 'COM_TEST'
    comm._protocol_compatible = True
    monkeypatch.setattr(comm, 'is_connected', lambda: True)
    assert comm.send_command('clear_calibration')
    assert comm.send_command(('active_scope', 0x3E0, 0xF0, False))

    lines = ('[AUTO CALIB] progress=20', '[BOOT CALIB] restored',
             '[CALIB_TX_APPLY] accepted',
             '[SERVO READ FAIL] bus=2 id=5 reason=checksum status_valid=0')
    ack_payload = bytes((CMD_CLEAR_CALIBRATION, 0, 0)) + struct.pack('>I', 123)
    frame = bytes((0xFE, len(ack_payload) + 2, PACKET_TYPE_PROTO_ACK)) + ack_payload + b'\xff'
    stream = ('\n'.join(lines) + '\n').encode('ascii') + frame
    # Exercise mixed text/binary decoding with arbitrary serial chunk boundaries.
    for offset in range(0, len(stream), 7):
        comm._process_packets(comm._frame_decoder.feed(stream[offset:offset + 7]))

    # Read before closing: each record must already have been flushed to disk.
    text = path.read_text(encoding='utf-8')
    for line in lines:
        assert text.count('[FW] ' + line) == 1
    assert '[CALIB_TX] port=COM_TEST command=clear_calibration' in text
    assert "[CALIB_TX] port=COM_TEST command=('active_scope', 992, 240, False)" in text
    assert text.count(f'[PROTO] ACK cmd=0x{CMD_CLEAR_CALIBRATION:02X}') == 1
    assert comm._state_reducer.last_model.proto_ack_cmd == CMD_CLEAR_CALIBRATION
    log.close_and_save()
    assert list(tmp_path.rglob('*.log')) == [path]
    assert not (tmp_path / 'auto_calib').exists()
