"""验证 P4 每条安全规则均具备 Policy、调用、上报和恢复探针。"""

from __future__ import annotations

from tools import generate_p4_safety_catalog as catalog


def test_every_registered_rule_has_a_closed_safety_path() -> None:
    rules = catalog.load_rule_definitions()
    assert len(rules) == 73
    assert catalog.audit_rules() == []


def test_collision_rule_uses_dedicated_state_machine_contract() -> None:
    """碰撞保持不能被 CTL 公共探针误判为已经闭环。"""

    collision_rule = next(
        rule for rule in catalog.load_rule_definitions()
        if rule["name"] == "ControlCollisionHold"
    )
    collision = catalog.resolve_rule_binding(collision_rule)

    assert collision is not None
    assert "CollisionSafetyPolicy.cpp" in collision.policy.path
    assert "holdReleased" in collision.recovery.token


def test_every_rule_has_a_user_explanation() -> None:
    """生成器已强制 guidance 与规则一一对应；这里从消费端再确认一次。"""

    from desktop.core.safety_rules import safety_rule_table

    table = safety_rule_table()
    assert set(table) == {int(rule["rule_id"]) for rule in catalog.load_rule_definitions()}
