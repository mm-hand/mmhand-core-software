"""HWCDC backport must reject unknown baselines and remain reproducible."""
import hashlib
import importlib.util
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location('hwcdc_patch', Path(__file__).resolve().parents[2] / 'tools/debug/patch_p4_hwcdc.py')
patch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(patch)


def test_patch_preserves_original_and_rejects_modified_installed_source(tmp_path, monkeypatch):
    source = tmp_path / 'HWCDC.cpp'
    original = b'// audited fixture\n' + patch.ANCHOR + b'\n  portEXIT_CRITICAL_SAFE(&hw_cdc_tx_mux);\n}\n'
    monkeypatch.setattr(patch, 'OFFICIAL_SHA256', hashlib.sha256(original).hexdigest())
    source.write_bytes(original)
    with pytest.raises(ValueError, match='required'):
        patch.patch_source(source, check=True)
    assert source.read_bytes() == original
    assert patch.patch_source(source)
    assert not patch.patch_source(source, check=True)
    assert not patch.patch_source(source)
    assert source.with_name('HWCDC.cpp.mmhand-before-empty-in-fix.bak').read_bytes() == original
    assert b'#if CONFIG_IDF_TARGET_ESP32P4' in source.read_bytes()
    assert patch.REPLACEMENT.index(b'txfifo_flush') < patch.REPLACEMENT.index(b'ena_intr_mask')
    source.write_bytes(source.read_bytes() + b'// unknown edit\n')
    with pytest.raises(ValueError, match='Unknown'):
        patch.patch_source(source)


def test_unknown_vendor_file_is_not_modified(tmp_path):
    source = tmp_path / 'HWCDC.cpp'
    source.write_bytes(b'unknown SDK source')
    with pytest.raises(ValueError, match='Unknown'):
        patch.patch_source(source)
    assert source.read_bytes() == b'unknown SDK source'
    assert len(list(tmp_path.iterdir())) == 1
