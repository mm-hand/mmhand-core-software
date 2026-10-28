"""safety_rules.py - P4 安全规则的上位机只读解释服务。

职责：加载稳定 rule_id 对应的英文标题、动作和恢复建议，并为现有门禁枚举提供统一文本。
线程：首次导入时只读 JSON；之后仅访问内存字典，可在通信线程和 GUI 主线程读取。
输入：P4 rule_id，或 SYSTEM_STATUS 中的 direct/feedback reject reason。
输出：面向日志、状态栏和弹窗的英文解释，不参与任何控制裁决。
状态所有权：规则真值属于 P4；本模块仅拥有展示副本。
禁止依赖：不得导入 GUI、串口或 HandBackend，不得在 UI 文件复制规则文本。
"""

from __future__ import annotations

import json
from functools import lru_cache
from pathlib import Path
from typing import Dict, Mapping


_RULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "resources"
    / "config"
    / "p4_safety_rules.json"
)


@lru_cache(maxsize=1)
def safety_rule_table() -> Mapping[int, dict]:
    """加载并按 rule_id 建索引；重复 ID 会立即报错，避免展示错误规则。"""
    payload = json.loads(_RULE_PATH.read_text(encoding="utf-8"))
    table: Dict[int, dict] = {}
    for item in payload.get("rules", []):
        rule_id = int(item["rule_id"])
        if rule_id in table:
            raise ValueError(f"duplicate P4 safety rule_id: {rule_id}")
        table[rule_id] = dict(item)
    return table


@lru_cache(maxsize=1)
def safety_rule_archive() -> Mapping[int, dict]:
    """Historical decode data is separate from the active firmware contract."""
    data = json.loads((_RULE_PATH.parent / 'safety_rules.archive.json').read_text(encoding='utf-8'))
    return {int(item['rule_id']): item for item in data['rules']}


def explain_safety_rule(rule_id: int) -> str:
    """Return the rule code, action, and recovery guidance."""
    item = safety_rule_table().get(int(rule_id))
    if item is None:
        item = safety_rule_archive().get(int(rule_id))
        if item is None:
            return f"Unknown safety rule {int(rule_id)}"
    return (
        f"{item['code']} {item['title']}; "
        f"Action: {item['action']}; Recovery: {item['recovery']}"
    )


# SYSTEM_STATUS 仍上报紧凑拒绝枚举；这里把每一个值映射到注册表中的专用规则，
# 不再退化成含义相近但不准确的舵机层规则。
_DIRECT_REASON_TO_RULE = {
    1: 3101,
    2: 3102,
    3: 3103,
    4: 3104,
    5: 3105,
    6: 3106,
}
_FEEDBACK_REASON_TO_RULE = {
    1: 3201,
    2: 3202,
    3: 3203,
    4: 3204,
    5: 3205,
    6: 3206,
    7: 3207,
}
_ROPE_INIT_REASON_TO_RULE = {reason: 3400 + reason for reason in range(1, 10)}


def explain_direct_reject(reason: int) -> str:
    """解释 SYSTEM_STATUS.direct_reject_reason。"""
    reason = int(reason)
    rule_id = _DIRECT_REASON_TO_RULE.get(reason)
    if rule_id is not None:
        return explain_safety_rule(rule_id)
    return f"Unknown direct-control rejection reason {reason}"


def explain_feedback_reject(reason: int) -> str:
    """解释 SYSTEM_STATUS.feedback_reject_reason。"""
    reason = int(reason)
    rule_id = _FEEDBACK_REASON_TO_RULE.get(reason)
    if rule_id is not None:
        return explain_safety_rule(rule_id)
    return f"Unknown feedback-control rejection reason {reason}"


def explain_rope_init_reason(reason: int) -> str:
    """Explain SYSTEM_STATUS.rope_init_reason using the same rule catalog."""
    reason = int(reason)
    if reason == 0:
        return "Rope Init completed without an error"
    rule_id = _ROPE_INIT_REASON_TO_RULE.get(reason)
    if rule_id is not None:
        return explain_safety_rule(rule_id)
    return f"Unknown Rope Init reason {reason}"
