"""验证 P4 每条安全规则均具备生成索引、Policy、调用、上报和恢复探针。"""

from __future__ import annotations

import importlib.util
import json
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
    assert report["rule_count"] == len(checker.load_rule_definitions())
    assert failures == {}
    assert report["registry_mapped_rule_count"] > 0
    assert report["direct_rule_count"] > 0


def test_collision_rule_uses_dedicated_state_machine_contract() -> None:
    """碰撞保持不能被 CTL 公共探针误判为已经闭环。"""

    checker = _load_checker()
    collision_rule = next(
        rule for rule in checker.load_rule_definitions()
        if rule["name"] == "ControlCollisionHold"
    )
    collision = checker.resolve_rule_binding(collision_rule)

    assert collision is not None
    assert "CollisionSafetyPolicy.cpp" in collision.policy.path
    assert "holdReleased" in collision.recovery.token


def test_generated_index_contains_every_rule() -> None:
    """生成 JSON 索引必须与规则目录一一对应，不允许只生成部分规则。"""

    checker = _load_checker()
    generated = json.loads(
        checker.FIRMWARE_INDEX_JSON.read_text(encoding="utf-8")
    )
    rules = checker.load_rule_definitions()

    generated_names = {item["name"] for item in generated["rules"]}
    assert len(generated["rules"]) == len(rules)
    for rule in rules:
        assert rule["name"] in generated_names
