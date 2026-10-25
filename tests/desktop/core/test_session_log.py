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
