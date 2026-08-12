"""验证 P4 每条安全规则均具备生成索引、Policy、调用、上报和恢复探针。"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
CHECKER_PATH = ROOT / "tools" / "p4_architecture" / "check_safety_rule_closure.py"


def _load_checker():
    """从提交门禁脚本加载同一套检查逻辑，避免测试与脚本各维护一份规则。"""

    spec = importlib.util.spec_from_file_location("p4_safety_rule_closure", CHECKER_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_every_registered_rule_has_a_closed_safety_path() -> None:
    checker = _load_checker()
    report = checker.audit(ROOT)
    failures = {
        item["code"]: [
            key
            for key in ("registration", "policy", "caller", "mapping", "event", "recovery")
            if not item[key]
        ]
        for item in report["rules"]
        if not all(
            item[key]
            for key in ("registration", "policy", "caller", "mapping", "event", "recovery")
        )
    }
    assert report["rule_count"] == 101
    assert failures == {}
    assert report["registry_mapped_rule_count"] > 0
    assert report["direct_rule_count"] > 0


def test_collision_and_lut_rules_use_dedicated_state_machine_contracts() -> None:
    """碰撞保持和 LUT 钳位不能被 CTL 公共探针误判为已经闭环。"""

    checker = _load_checker()
    collision_rule = next(
        rule for rule in checker.load_rule_definitions()
        if rule["name"] == "ControlCollisionHold"
    )
    lut_rule = next(
        rule for rule in checker.load_rule_definitions()
        if rule["name"] == "ControlLutBoundaryClamp"
    )
    collision = checker.resolve_rule_binding(collision_rule)
    lut_clamp = checker.resolve_rule_binding(lut_rule)

    assert collision is not None
    assert lut_clamp is not None
    assert "CollisionSafetyPolicy.cpp" in collision.policy.path
    assert "holdReleased" in collision.recovery.token
    assert "ControlLutBoundaryClamp" in lut_clamp.event.token
    assert "g_lutClampEventLatchedBitmap" in lut_clamp.recovery.token


def test_generated_index_contains_every_rule() -> None:
    """生成 C++ 索引必须与规则目录一一对应，不允许只生成部分规则。"""

    checker = _load_checker()
    generated = checker.FIRMWARE_INDEX_SOURCE.read_text(encoding="utf-8")
    rules = checker.load_rule_definitions()

    assert generated.count("{SafetyRuleId::") == len(rules) == 101
    for rule in rules:
        assert f"SafetyRuleId::{rule['name']}" in generated
