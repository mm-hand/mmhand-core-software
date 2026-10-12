"""Safety catalog retirement and firmware-only output regression tests."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

from tools import generate_p4_safety_catalog as catalog
from tools.p4_architecture import check_safety_rule_closure as closure


def test_retired_calibration_ids_keep_history_without_runtime_bindings() -> None:
    rules = catalog.load_rule_definitions()
    index = json.loads(catalog.render_firmware_index_json())
    entries = {item["name"]: item for item in index["rules"]}
    retired = [rule for rule in rules if catalog.is_retired_rule(rule)]
    assert {rule["rule_id"] for rule in retired}.issuperset({3005, 3504, 4016})
    for rule in retired:
        assert catalog.resolve_rule_binding(rule) is None
        entry = entries[rule["name"]]
        assert entry["rule_id"] == rule["rule_id"]
        assert entry["active"] is False and entry["retired"] is True
        assert not any(key in entry for key in ("policy", "caller", "event", "recovery", "registry_function"))
    warnings = [rule for rule in rules if rule["code"].startswith("CAW-")]
    assert len(warnings) == 3
    for rule in warnings:
        assert catalog.resolve_rule_binding(rule) is not None
        assert entries[rule["name"]]["active"] is True


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


def test_closure_rejects_resurrected_retired_runtime_mapping(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    source = tmp_path / "ESP32-P4/src"
    (source / "generated").mkdir(parents=True)
    (source / "safety").mkdir()
    (source / "generated/SafetyRuleCatalog.generated.h").write_text(
        catalog.render_firmware_header(), encoding="utf-8",
    )
    index = source / "generated/SafetyRuleIndex.generated.json"
    index.write_text(catalog.render_firmware_index_json(), encoding="utf-8")
    desktop = tmp_path / "desktop/resources/config"
    desktop.mkdir(parents=True)
    (desktop / "p4_safety_rules.json").write_text(catalog.render(), encoding="utf-8")
    monkeypatch.setattr(closure, "FIRMWARE_INDEX_JSON", index)
    retired = next(rule for rule in catalog.load_rule_definitions() if rule["name"] == "CalibrationTargetPublishFailed")
    monkeypatch.setattr(closure, "load_rule_definitions", lambda: [retired])
    clean = closure.audit(tmp_path)
    assert clean["retired_rule_count"] == 1
    assert clean["rules"][0]["mapping"] is True
    (source / "safety/SafetyRuleRegistry.cpp").write_text(
        "return SafetyRuleId::CalibrationTargetPublishFailed;", encoding="utf-8",
    )
    revived = closure.audit(tmp_path)
    assert revived["rules"][0]["mapping"] is False
    assert revived["rules"][0]["runtime_references"] == [str(Path("safety/SafetyRuleRegistry.cpp"))]
