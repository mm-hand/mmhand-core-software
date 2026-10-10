#!/usr/bin/env python3
"""审计生成安全索引中的 Policy、业务调用、事件上报和恢复入口是否闭环。"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
TOOLS_ROOT = REPO_ROOT / "tools"
if str(TOOLS_ROOT) not in sys.path:
    sys.path.insert(0, str(TOOLS_ROOT))

from generate_p4_safety_catalog import (  # noqa: E402
    FIRMWARE_INDEX_JSON,
    Probe,
    load_rule_definitions,
    is_retired_rule,
    resolve_rule_binding,
)


BANNED_SCAFFOLD = (
    "safety/facades",
    "SafetySupervisor.cpp",
    "SafetySupervisor.h",
    "SafetyActionExecutor.cpp",
    "SafetyActionExecutor.h",
)


def read(path: Path) -> str:
    """统一按 UTF-8 读取源码和生成文件。"""

    return path.read_text(encoding="utf-8")


def probe_ok(source_root: Path, probe: Probe) -> bool:
    """探针文件必须存在，并且包含登记的函数或状态符号。"""

    path = source_root / probe.path
    return path.is_file() and probe.token in read(path)


def generated_entry_ok(rule: dict, contract, entry: dict | None) -> bool:
    """确认离线 JSON 索引包含本规则的完整调用链，而非仅存在生成文件。"""

    if entry is None:
        return False
    return (
        entry.get("name") == rule["name"]
        and int(entry.get("rule_id", -1)) == int(rule["rule_id"])
        and entry.get("code") == rule["code"]
        and entry.get("active") is True
        and entry.get("retired") is False
        and entry.get("policy") == {
            "file": contract.policy.path,
            "symbol": contract.policy.token,
        }
        and entry.get("caller") == {
            "file": contract.caller.path,
            "symbol": contract.caller.token,
        }
        and entry.get("event") == {
            "file": contract.event.path,
            "symbol": contract.event.token,
        }
        and entry.get("recovery") == {
            "file": contract.recovery.path,
            "symbol": contract.recovery.token,
        }
    )


def audit(repo_root: Path) -> dict[str, object]:
    """逐条验证生成索引、Policy、业务调用、事件入口和恢复入口。"""

    source_root = repo_root / "ESP32-P4" / "ServoBoardMain" / "src"
    catalog_header = read(source_root / "generated" / "SafetyRuleCatalog.generated.h")
    generated_items = (
        json.loads(read(FIRMWARE_INDEX_JSON)).get("rules", [])
        if FIRMWARE_INDEX_JSON.is_file()
        else []
    )
    generated_by_name = {str(item.get("name", "")): item for item in generated_items}
    desktop_items = json.loads(read(
        repo_root / "desktop" / "resources" / "config" / "p4_safety_rules.json"
    ))["rules"]
    desktop_by_id = {int(item["rule_id"]): item for item in desktop_items}
    runtime_sources = {
        str(path.relative_to(source_root)): read(path)
        for path in source_root.rglob("*")
        if path.suffix in {".c", ".cpp", ".h"}
        and "generated" not in path.relative_to(source_root).parts
    }

    results: list[dict[str, object]] = []
    mapped_rule_count = 0
    for rule in load_rule_definitions():
        contract = resolve_rule_binding(rule)
        code = str(rule["code"])
        if is_retired_rule(rule):
            entry = generated_by_name.get(str(rule["name"]), {})
            runtime_pattern = re.compile(
                rf"\bSafetyRuleId\s*::\s*{re.escape(str(rule['name']))}\b"
            )
            runtime_references = [
                path for path, text in runtime_sources.items()
                if runtime_pattern.search(text)
            ]
            has_no_binding = contract is None and not any(
                name in entry for name in
                ("policy", "caller", "event", "recovery", "registry_function")
            )
            inactive = has_no_binding and not runtime_references
            results.append({
                "name": rule["name"],
                "code": code,
                "retired": True,
                "registration": (
                    str(rule["name"]) in catalog_header
                    and int(rule["rule_id"]) in desktop_by_id
                    and desktop_by_id[int(rule["rule_id"])]["code"] == code
                    and entry.get("name") == rule["name"]
                    and entry.get("rule_id") == rule["rule_id"]
                    and entry.get("code") == code
                    and entry.get("active") is False
                    and entry.get("retired") is True
                ),
                "policy": has_no_binding,
                "caller": inactive,
                "mapping": inactive,
                "event": inactive,
                "recovery": inactive,
                "runtime_references": runtime_references,
            })
            continue
        if contract is None:
            results.append({
                "name": rule["name"],
                "code": code,
                "registration": False,
                "policy": False,
                "caller": False,
                "mapping": False,
                "event": False,
                "recovery": False,
            })
            continue

        if contract.mapping is not None:
            mapped_rule_count += 1
        mapping_ok = contract.mapping is None or (
            probe_ok(source_root, contract.mapping)
            and f"SafetyRuleId::{rule['name']}" in read(source_root / contract.mapping.path)
        )
        registration = (
            str(rule["name"]) in catalog_header
            and int(rule["rule_id"]) in desktop_by_id
            and desktop_by_id[int(rule["rule_id"])]["code"] == code
            and FIRMWARE_INDEX_JSON.is_file()
            and generated_entry_ok(
                rule,
                contract,
                generated_by_name.get(str(rule["name"])),
            )
        )
        results.append({
            "name": rule["name"],
            "code": code,
            "registration": registration,
            "policy": probe_ok(source_root, contract.policy),
            "caller": probe_ok(source_root, contract.caller),
            "mapping": mapping_ok,
            "event": probe_ok(source_root, contract.event),
            "recovery": (
                bool(str(rule["recovery"]).strip())
                and probe_ok(source_root, contract.recovery)
            ),
            "policy_ref": f"{contract.policy.path}::{contract.policy.token}",
            "caller_ref": f"{contract.caller.path}::{contract.caller.token}",
            "event_ref": f"{contract.event.path}::{contract.event.token}",
            "registry_ref": contract.mapping.token if contract.mapping else "",
        })

    stale_scaffold = [
        item for item in BANNED_SCAFFOLD
        if (source_root / item).exists()
    ]
    direct_store_writers: list[str] = []
    for path in source_root.rglob("*"):
        if path.suffix not in {".c", ".cpp", ".h"}:
            continue
        if source_root / "safety" in path.parents:
            continue
        text = read(path)
        if "recordSafetyRuleEvent(" in text or "recordSafetyEvent(" in text:
            direct_store_writers.append(str(path.relative_to(source_root)))

    keys = ("registration", "policy", "caller", "mapping", "event", "recovery")
    active_results = [item for item in results if not item.get("retired", False)]
    return {
        "rule_count": len(active_results),
        "retired_rule_count": len(results) - len(active_results),
        "closed_count": sum(all(item[key] for key in keys) for item in active_results),
        "registry_mapped_rule_count": mapped_rule_count,
        "direct_rule_count": len(active_results) - mapped_rule_count,
        "rules": results,
        "stale_scaffold": stale_scaffold,
        "direct_store_writers": direct_store_writers,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json", action="store_true", help="输出机器可读 JSON")
    args = parser.parse_args()
    report = audit(REPO_ROOT)

    keys = ("registration", "policy", "caller", "mapping", "event", "recovery")
    failures = [
        item for item in report["rules"]
        if not all(item[key] for key in keys)
    ]
    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=2))
    elif failures or report["stale_scaffold"] or report["direct_store_writers"]:
        print("P4 safety rule closure check FAILED:")
        for item in failures:
            missing = [key for key in keys if not item[key]]
            print(f" - {item['code']} {item['name']}: {', '.join(missing)}")
            for path in item.get("runtime_references", []):
                print(f"   retired rule still referenced at runtime: {path}")
        for path in report["stale_scaffold"]:
            print(f" - stale safety scaffold: {path}")
        for path in report["direct_store_writers"]:
            print(f" - business writes SafetyEventStore directly: {path}")
    else:
        print(
            "P4 safety rule closure check passed: "
            f"{report['closed_count']}/{report['rule_count']} rules closed; "
            f"Registry maps {report['registry_mapped_rule_count']}, "
            f"Policy direct IDs {report['direct_rule_count']}; "
            f"{report['retired_rule_count']} retired rules have no runtime bindings."
        )
    return 1 if failures or report["stale_scaffold"] or report["direct_store_writers"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
