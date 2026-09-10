"""Verify unique English safety-rule explanations and compact reason mappings."""

import re

from desktop.core.safety_rules import (
    explain_direct_reject,
    explain_feedback_reject,
    explain_rope_init_reason,
    explain_safety_rule,
    safety_rule_table,
)


def test_safety_rule_ids_are_unique_and_stable() -> None:
    table = safety_rule_table()
    assert len(table) == 103
    assert table[1009]["code"] == "ACT-1009"
    assert table[3001]["code"] == "SYS-3001"
    assert table[8001]["code"] == "RUN-8001"
    assert table[6005]["code"] == "COM-6005"
    assert table[8006]["code"] == "RUN-8006"
    assert all({"title", "action", "recovery"} <= set(item) for item in table.values())
    assert not any(re.search(r"[\u4e00-\u9fff]", explain_safety_rule(rule_id)) for rule_id in table)


def test_reject_reasons_use_central_explanations() -> None:
    for reason in range(1, 7):
        assert f"DIR-{3100 + reason}" in explain_direct_reject(reason)
    for reason in range(1, 8):
        assert f"FBK-{3200 + reason}" in explain_feedback_reject(reason)
    for reason in range(1, 10):
        assert f"ROP-{3400 + reason}" in explain_rope_init_reason(reason)
    assert "Unknown Rope Init reason 10" == explain_rope_init_reason(10)
