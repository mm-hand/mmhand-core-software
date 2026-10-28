"""Safety catalog generation and firmware-only output regression tests."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

from tools import generate_p4_safety_catalog as catalog


def test_firmware_only_safety_catalog_preserves_desktop(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    desktop = tmp_path / "desktop_rules.json"
    header = tmp_path / "SafetyRuleCatalog.generated.h"
    index = tmp_path / "SafetyRuleIndex.generated.json"
    desktop.write_text("desktop retained", encoding="utf-8")
    monkeypatch.setattr(catalog, "OUTPUT", desktop)
    monkeypatch.setattr(catalog, "FIRMWARE_HEADER", header)
    monkeypatch.setattr(catalog, "FIRMWARE_INDEX_JSON", index)
    monkeypatch.setattr(catalog, "OBSOLETE_FIRMWARE_INDEX_FILES", ())
    monkeypatch.setattr(sys, "argv", ["generate_p4_safety_catalog.py", "--firmware-only"])
    assert catalog.main() == 0
    assert desktop.read_text(encoding="utf-8") == "desktop retained"
    monkeypatch.setattr(sys, "argv", ["generate_p4_safety_catalog.py", "--firmware-only", "--check"])
    assert catalog.main() == 0
    index.write_text("outdated runtime bindings", encoding="utf-8")
    assert catalog.main() == 1
    index.write_text(catalog.render_firmware_index_json(), encoding="utf-8")
    monkeypatch.setattr(sys, "argv", ["generate_p4_safety_catalog.py", "--check"])
    assert catalog.main() == 1
