"""验证上位机安全规则解释表的唯一性和关键拒绝映射。"""

from desktop.core.safety_rules import (
    explain_direct_reject,
    explain_feedback_reject,
    safety_rule_table,
)


def test_safety_rule_ids_are_unique_and_stable() -> None:
    table = safety_rule_table()
    assert len(table) == 102
    assert table[1009]["code"] == "ACT-1009"
    assert table[3001]["code"] == "SYS-3001"
    assert table[8001]["code"] == "RUN-8001"
    assert table[6005]["code"] == "COM-6005"
    assert table[8006]["code"] == "RUN-8006"


def test_reject_reasons_use_central_explanations() -> None:
    assert "DIR-3103" in explain_direct_reject(3)
    assert "FBK-3202" in explain_feedback_reject(2)
