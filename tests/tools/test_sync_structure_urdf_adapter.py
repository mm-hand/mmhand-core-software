"""验证 structure URDF 源文件选择规则，防止更新时静默选错模型。"""

from pathlib import Path

import pytest

from tools import sync_structure_urdf_adapter


def test_prefers_named_mmhand_urdf(tmp_path: Path) -> None:
    urdf_root = tmp_path / "urdf"
    urdf_root.mkdir()
    preferred = urdf_root / "mmhand.urdf"
    preferred.write_text("<robot/>", encoding="utf-8")
    (urdf_root / "other.urdf").write_text("<robot/>", encoding="utf-8")

    assert sync_structure_urdf_adapter._resolve_source_urdf(tmp_path) == preferred


def test_accepts_only_urdf_as_explicit_fallback(tmp_path: Path) -> None:
    urdf_root = tmp_path / "urdf"
    urdf_root.mkdir()
    fallback = urdf_root / "(Unsaved).urdf"
    fallback.write_text("<robot/>", encoding="utf-8")

    assert sync_structure_urdf_adapter._resolve_source_urdf(tmp_path) == fallback


def test_rejects_ambiguous_urdf_fallback(tmp_path: Path) -> None:
    urdf_root = tmp_path / "urdf"
    urdf_root.mkdir()
    (urdf_root / "one.urdf").write_text("<robot/>", encoding="utf-8")
    (urdf_root / "two.urdf").write_text("<robot/>", encoding="utf-8")

    with pytest.raises(FileNotFoundError, match="fallback is ambiguous"):
        sync_structure_urdf_adapter._resolve_source_urdf(tmp_path)
