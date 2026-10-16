"""Evidence-only AI explanations for every graph element and documented command."""

from __future__ import annotations

import json
import re

from source import Sources, range_changed


FIELDS = ("plain", "condition", "effect", "failure", "recovery", "unknown", "difference", "change")
COMMANDS = [
    ("install", "python -m pip install -r desktop/requirements.txt", "mmhand"),
    ("monitor", "python -m desktop --no-ros2", "mmhand"),
    ("teleop", "python track.py --ros", "teleop"),
    ("ros_ui", "python -m desktop", "mmhand"),
    ("ros_echo", "ros2 topic echo --once /mmhand/joint_angle_deg", "mmhand"),
    ("arduino_p4", ". tools/debug/debug_env.ps1 -Target p4", "mmhand"),
    ("arduino_s3", ". tools/debug/debug_env.ps1 -Target s3", "mmhand"),
    ("build_p4", "tools/debug/build_debug.ps1 -Target p4", "mmhand"),
    ("build_s3", "tools/debug/build_debug.ps1 -Target s3", "mmhand"),
    ("upload_p4", "tools/debug/upload_debug.ps1 -Target p4 -Port COMx", "mmhand"),
    ("upload_s3", "tools/debug/upload_debug.ps1 -Target s3 -Port COMx", "mmhand"),
    ("breakpoint_p4", 'python tools/debug/gdb_batch.py --target p4 --break motionControlTask --bt', "mmhand"),
    ("breakpoint_s3", 'python tools/debug/gdb_batch.py --target s3 --break pollTactileMaintenance --bt', "mmhand"),
    ("urdf", "tools/update_structure_urdf.ps1 -StructureRepo <path> -NoPull -SkipP4Build", "mmhand"),
    ("robot_generate", "python tools/generate_robot_model.py", "mmhand"),
    ("robot_check", "python tools/generate_robot_model.py --check", "mmhand"),
    ("protocol_generate", "python tools/generate_upper_protocol.py", "mmhand"),
    ("protocol_check", "python tools/generate_upper_protocol.py --check", "mmhand"),
    ("safety_generate", "python tools/generate_p4_safety_catalog.py", "mmhand"),
    ("safety_check", "python tools/generate_p4_safety_catalog.py --check", "mmhand"),
    ("safety_audit", "python tools/p4_architecture/check_safety_rule_closure.py --json", "mmhand"),
]


class ExplanationError(RuntimeError):
    def __init__(self, message: str, missing: list[str], report: dict | None = None):
        super().__init__(message)
        self.missing = missing
        self.report = report or {}


def ref_id(ref: dict) -> str:
    return f"{ref['repo']}:{ref['path']}:L{ref['start']}-L{ref['end']}"


def _source_context(sources: Sources, ref: dict) -> str:
    rows = sources.lines(ref["repo"], ref["path"])
    start = max(1, ref["start"] - 5)
    end = min(len(rows), ref["end"] + 8)
    return "\n".join(f"{i}: {rows[i - 1]}" for i in range(start, end + 1))[:1100]


def _refs_for(item: dict, graph: dict, *, edge: bool) -> list[dict]:
    refs = list(item.get("evidence", []))
    def nested_refs(value: object) -> list[dict]:
        if isinstance(value, dict):
            if {"repo", "path", "start", "end", "text"}.issubset(value):
                return [value]
            return [ref for part in value.values() for ref in nested_refs(part)]
        if isinstance(value, list):
            return [ref for part in value for ref in nested_refs(part)]
        return []
    refs.extend(nested_refs(item.get("details", {})))
    for fact in item.get("details", {}).get("conditions", []):
        if isinstance(fact, dict) and fact.get("evidence"):
            refs.append(fact["evidence"])
    for fact in item.get("details", {}).get("chain", []):
        if fact.get("evidence"):
            refs.append(fact["evidence"])
    for fact in item.get("details", {}).get("period", []):
        if isinstance(fact, dict) and fact.get("evidence"):
            refs.append(fact["evidence"])
    if edge:
        nodes = {node["id"]: node for node in graph["nodes"]}
        for endpoint in (item["source"], item["target"]):
            refs.extend(nodes[endpoint].get("evidence", [])[:2])
    if not refs:
        refs.extend(next((n["evidence"] for n in graph["nodes"] if n["evidence"]), []))
    return list({ref_id(ref): ref for ref in refs}.values())


def _variable_definitions(sources: Sources, expression: str, refs: list[dict]) -> list[dict]:
    if not expression:
        return []
    excluded = {"if", "else", "return", "true", "false", "and", "or", "not", "self",
                "state", "input", "sharedData", "const", "float", "int", "bool", "isfinite"}
    symbols = [name for name in dict.fromkeys(re.findall(r"\b[A-Za-z_][A-Za-z_0-9]*\b", expression))
               if name not in excluded and len(name) > 2][:14]
    found = []
    for name in symbols:
        for repo, path in dict.fromkeys((ref["repo"], ref["path"]) for ref in refs[:5]):
            rows = sources.lines(repo, path)
            for line_no, row in enumerate(rows, 1):
                if re.search(r"\b" + re.escape(name) + r"\b\s*(?:=|:)", row) or re.search(
                        r"\b(?:const|constexpr|bool|float|int|uint\d+_t)\s+" + re.escape(name) + r"\b", row):
                    found.append({"name": name, "evidence": sources.evidence(repo, path, line_no)})
                    break
            if found and found[-1]["name"] == name:
                break
        if len(found) >= 8:
            break
    return found


def cards(sources: Sources, graphs: list[dict], changes: dict, baseline: str | None) -> list[dict]:
    result = []
    diff_cache: dict[str, str] = {}
    for graph in graphs:
        for kind, items in (("node", graph["nodes"]), ("edge", graph["edges"])):
            for index, item in enumerate(items):
                item_id = (f"graph:{graph['id']}:node:{item['id']}" if kind == "node"
                           else f"graph:{graph['id']}:edge:{index}:{item['source']}:{item['target']}")
                refs = _refs_for(item, graph, edge=kind == "edge")
                expression = item.get("condition", "") + " " + " ".join(
                    fact.get("expr", "") for field in ("conditions", "period")
                    for fact in item.get("details", {}).get(field, []) if isinstance(fact, dict))
                definitions = _variable_definitions(sources, expression, refs)
                evidence = [{"id": ref_id(ref), "ref": ref,
                             "context": _source_context(sources, ref) if position < 6 else "",
                             "repo": ref["repo"], "path": ref["path"]}
                            for position, ref in enumerate(refs)]
                is_changed = any(range_changed(ref, changes) for ref in refs)
                diffs = []
                if is_changed and baseline:
                    for ref in refs:
                        path = ref["path"]
                        if ref["repo"] != "mmhand" or path not in changes:
                            continue
                        if path not in diff_cache:
                            from source import git
                            diff_cache[path] = git(sources.roots["mmhand"], "diff", "--unified=3", baseline,
                                                   sources.commits["mmhand"], "--", path)[:4000]
                        diffs.append(diff_cache[path][:2200])
                        break
                result.append({"id": item_id, "kind": kind, "graph": graph["id"],
                               "title": item["label"], "condition": item.get("condition", ""),
                               "source": item.get("source", ""), "target": item.get("target", ""),
                               "drilldown": item.get("drilldown", ""),
                               "details": item.get("details", {}), "evidence": evidence,
                               "variable_definitions": [{"name": entry["name"],
                                                         "source": ref_id(entry["evidence"]),
                                                         "ref": entry["evidence"],
                                                         "text": entry["evidence"]["text"]} for entry in definitions],
                               "changed": is_changed, "diff": diffs})
    for name, command, repo in COMMANDS:
        scope = sources.scope["command_sources"][name]
        path, needle = scope["path"], scope["needle"]
        if scope["repo"] != repo or path not in sources.paths[repo]:
            raise ValueError(f"Command source is not committed: {name}:{repo}:{path}")
        rows = sources.lines(repo, path)
        line = next((i for i, row in enumerate(rows, 1) if needle in row), None)
        if line is None:
            raise ValueError(f"Missing command evidence: {repo}:{path}:{needle}")
        ref = sources.evidence(repo, path, line)
        refs = [ref]
        for extra in scope.get("extra", []):
            extra_repo, extra_path, extra_needle = extra["repo"], extra["path"], extra["needle"]
            if extra_path not in sources.paths[extra_repo]:
                raise ValueError(f"Command extra source is not committed: {name}:{extra_repo}:{extra_path}")
            for extra_line, row in enumerate(sources.lines(extra_repo, extra_path), 1):
                if extra_needle in row:
                    refs.append(sources.evidence(extra_repo, extra_path, extra_line))
                    break
        result.append({"id": "command:" + name, "kind": "command", "graph": "readme",
                       "title": {"zh": command, "en": command}, "condition": "",
                       "details": {"command": command, "working_dir": "mmhand_teleop" if repo == "teleop" else "mmhand",
                                   "required_terms": scope.get("required_terms", [])},
                       "evidence": [{"id": ref_id(entry), "ref": entry,
                                     "context": _source_context(sources, entry),
                                     "repo": entry["repo"], "path": entry["path"]} for entry in refs],
                       "variable_definitions": [], "changed": False, "diff": []})
    return result


def attach(graphs: list[dict], annotations: dict[str, dict], cards_: list[dict] | None = None) -> None:
    card_map = {card["id"]: card for card in (cards_ or [])}
    for graph in graphs:
        for node in graph["nodes"]:
            item_id = f"graph:{graph['id']}:node:{node['id']}"
            node["explanation"] = annotations[item_id]
            if item_id in card_map:
                refs = {ref_id(ref): ref for ref in node["evidence"]}
                for evidence in card_map[item_id]["evidence"]:
                    refs[evidence["id"]] = evidence["ref"]
                node["evidence"] = list(refs.values())
        for index, edge in enumerate(graph["edges"]):
            item_id = f"graph:{graph['id']}:edge:{index}:{edge['source']}:{edge['target']}"
            edge["explanation"] = annotations[item_id]
            if item_id in card_map:
                refs = {ref_id(ref): ref for ref in edge["evidence"]}
                for evidence in card_map[item_id]["evidence"]:
                    refs[evidence["id"]] = evidence["ref"]
                edge["evidence"] = list(refs.values())
