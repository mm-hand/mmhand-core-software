"""Safety catalog generation and firmware-only output regression tests."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

from tools import generate_p4_safety_catalog as catalog


def test_firmware_only_safety_catalog_preserves_desktop(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    desktop = tmp_path / "desktop_rules.json"
    header = tmp_path / "SafetyRuleCatalog.generated.h"
    desktop.write_text("desktop retained", encoding="utf-8")
    monkeypatch.setattr(catalog, "OUTPUT", desktop)
    monkeypatch.setattr(catalog, "FIRMWARE_HEADER", header)
    monkeypatch.setattr(catalog, "OBSOLETE_FIRMWARE_INDEX_FILES", ())
    monkeypatch.setattr(sys, "argv", ["generate_p4_safety_catalog.py", "--firmware-only"])
    assert catalog.main() == 0
    assert desktop.read_text(encoding="utf-8") == "desktop retained"
    monkeypatch.setattr(sys, "argv", ["generate_p4_safety_catalog.py", "--firmware-only", "--check"])
    assert catalog.main() == 0
    header.write_text("outdated catalog", encoding="utf-8")
    assert catalog.main() == 1
    monkeypatch.setattr(sys, "argv", ["generate_p4_safety_catalog.py", "--check"])
    assert catalog.main() == 1


def test_audit_reports_a_broken_rule_chain(monkeypatch: pytest.MonkeyPatch) -> None:
    """闭环审计必须能定位到具体规则，而不是只返回一个布尔值。"""

    real = catalog.load_rule_definitions()

    def broken() -> list[dict]:
        rules = [dict(rule) for rule in real]
        rules[0]["name"] = "NotARuleInRegistry"
        return rules

    monkeypatch.setattr(catalog, "load_rule_definitions", broken)
    failures = catalog.audit_rules()
    assert any("NotARuleInRegistry" in message for message in failures)
