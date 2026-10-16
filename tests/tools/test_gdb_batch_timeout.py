"""Debug timeout must reach rescue even when a launcher spawns a child."""
import importlib.util
from pathlib import Path
import subprocess
import sys
import time

import pytest


def load_debug_module():
    path = Path(__file__).resolve().parents[2] / 'tools/debug/gdb_batch.py'
    spec = importlib.util.spec_from_file_location('mmhand_gdb_checks', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_debug_requires_uploaded_matching_elf_and_board_identity(tmp_path):
    module = load_debug_module()
    elf = tmp_path / 'firmware.elf'
    elf.write_bytes(b'build-one')
    with pytest.raises(ValueError, match='No flashed-firmware marker'):
        module.validate_flashed_elf(elf)
    marker = elf.with_name(elf.name + '.flashed.sha256')
    marker.write_text(module.sha256_of(elf))
    with pytest.raises(ValueError, match='No USB-JTAG identity'):
        module.validate_flashed_elf(elf)
    elf.with_name(elf.name + '.jtag_serial').write_text('physical-board-A')
    assert module.validate_flashed_elf(elf) == 'physical-board-A'
    elf.write_bytes(b'build-two')
    with pytest.raises(ValueError, match='ELF differs'):
        module.validate_flashed_elf(elf)


def test_live_attach_disables_vendor_flash_reset_before_board_config(monkeypatch):
    module = load_debug_module()
    commands = []

    def stop_at_launch(argv, **kwargs):
        commands.extend(argv)
        raise RuntimeError('launch intercepted')

    monkeypatch.setattr(module.subprocess, 'Popen', stop_at_launch)
    with pytest.raises(RuntimeError, match='launch intercepted'):
        module.spawn_openocd(Path('openocd'), Path('scripts'), 'board/p4.cfg', 3333, 1, 'board-A')
    assert commands.index('set ESP_FLASH_SIZE 0') < commands.index('board/p4.cfg')
    assert 'adapter serial board-A' in commands
    assert all(target['hw_breakpoints'] for target in module.TARGETS.values())


def test_rejected_existing_openocd_is_never_cleaned_up(monkeypatch, tmp_path):
    module = load_debug_module()
    elf = tmp_path / 'firmware.elf'; elf.touch()
    monkeypatch.setattr(sys, 'argv', ['gdb_batch', '--elf', str(elf)])
    monkeypatch.setattr(module, 'core_tools_dir', lambda: tmp_path)
    monkeypatch.setattr(module, 'latest_tool', lambda *args: tmp_path / 'bin/tool.exe')
    monkeypatch.setattr(module, 'validate_flashed_elf', lambda _: 'board-A')
    monkeypatch.setattr(module, 'openocd_is_running', lambda: True)
    monkeypatch.setattr(module, 'openocd_rpc', lambda _: 'esp32s3.cpu0')
    def forbidden():
        pytest.fail('must not resume or clear triggers on a rejected board')
    monkeypatch.setattr(module, 'verify_target_resumed', forbidden)
    with pytest.raises(SystemExit, match='do not match'):
        module.main()


def test_windows_launcher_child_cannot_keep_timeout_cleanup_blocked():
    if sys.platform != 'win32':
        pytest.skip('Windows GDB launcher process-tree behavior')
    path = Path(__file__).resolve().parents[2] / 'tools/debug/gdb_batch.py'
    spec = importlib.util.spec_from_file_location('mmhand_gdb_batch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    program = ('import subprocess,sys,time; '
               'subprocess.Popen([sys.executable,"-c","import time; time.sleep(15)"]); '
               'time.sleep(15)')
    start = time.monotonic()
    with pytest.raises(subprocess.TimeoutExpired):
        module.bounded_gdb([sys.executable, '-c', program], .5)
    assert time.monotonic() - start < 8


def test_openocd_output_is_drained_after_listening_marker(tmp_path):
    path = Path(__file__).resolve().parents[2] / 'tools/debug/gdb_batch.py'
    spec = importlib.util.spec_from_file_location('mmhand_gdb_drain', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    program = ('print("Listening on port 3333 for gdb connections",flush=True); '
               'print("diagnostic line\\n" * 50000,flush=True)')
    process = subprocess.Popen([sys.executable, '-c', program], stdout=subprocess.PIPE,
                               text=True, errors='replace')
    log = tmp_path / 'openocd.log'
    lines = module.drain_openocd_output(process, log)
    try:
        assert 'Listening on port' in lines.get(timeout=5)
        assert process.wait(timeout=5) == 0
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and log.stat().st_size < 700000:
            time.sleep(.01)
        assert log.stat().st_size >= 700000
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()


def test_hardware_cleanup_requires_both_cores_running():
    path = Path(__file__).resolve().parents[2] / 'tools/debug/gdb_batch.py'
    spec = importlib.util.spec_from_file_location('mmhand_gdb_resume', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    commands = []

    def rpc(command):
        commands.append(command)
        return 'esp32p4.hp.cpu0=running,esp32p4.hp.cpu1=running'

    assert 'cpu1=running' in module.verify_target_resumed(rpc)
    assert commands[0].index('mmhand_debug_release') < commands[0].index('arp_poll')
    cleanup = (path.parent / 'esp_debug_cleanup.tcl').read_text()
    assert cleanup.index('rbp all') < cleanup.index('reg tdata1 0') < cleanup.index('{ resume }')
    with pytest.raises(RuntimeError, match='not confirmed'):
        module.verify_target_resumed(lambda _: 'esp32p4.hp.cpu0=running,esp32p4.hp.cpu1=halted')
    with pytest.raises(RuntimeError, match='not confirmed'):
        module.verify_target_resumed(lambda _: 'invalid command name')
