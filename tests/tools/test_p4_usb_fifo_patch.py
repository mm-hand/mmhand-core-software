"""The SDK backport must be narrow, reproducible and preserve the original."""
from pathlib import Path
import importlib.util

import pytest

spec = importlib.util.spec_from_file_location('p4_fifo_patch', Path(__file__).resolve().parents[2] / 'tools/debug/patch_p4_usb_fifo.py')
patch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(patch)


def test_backport_preserves_unrelated_bytes_and_original(tmp_path):
    header = tmp_path / 'usb_serial_jtag_ll.h'
    original = b'// vendor\r\n    ' + patch.OLD.encode() + b'\r\n// end\r\n'
    header.write_bytes(original)
    with pytest.raises(ValueError, match='required'):
        patch.patch_header(header, check=True)
    assert header.read_bytes() == original
    assert patch.patch_header(header)
    assert header.read_bytes() == original.replace(patch.OLD.encode(), patch.NEW.encode())
    backup = header.with_name(header.name + '.mmhand-before-fifo-fix.bak')
    assert backup.read_bytes() == original
    assert not patch.patch_header(header)
    assert not patch.patch_header(header, check=True)
    assert backup.read_bytes() == original


def test_unknown_sdk_is_not_modified(tmp_path):
    header = tmp_path / 'usb_serial_jtag_ll.h'
    header.write_bytes(b'unknown implementation')
    with pytest.raises(ValueError, match='Unrecognized'):
        patch.patch_header(header)
    assert header.read_bytes() == b'unknown implementation'
    assert len(list(tmp_path.iterdir())) == 1
