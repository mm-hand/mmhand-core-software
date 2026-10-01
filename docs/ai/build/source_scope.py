"""Versioned source scope, evidence-backed graph extensions, and review candidates.

The module reads committed source through ``Sources``. It never invokes an AI
service: a coding harness reviews the candidate list and edits source_scope.json.
"""
from __future__ import annotations

import ast
import hashlib
import json
from pathlib import Path
import re

from graph_context import _brace_functions

SCHEMA_VERSION = 1
RELATIONS = {"calls", "reads", "writes", "publishes", "subscribes", "data", "state", "context"}
WHOLE_FILE_SUFFIXES = {".json", ".yaml", ".yml", ".def", ".ps1", ".sh", ".cmd", ".bat", ".toml", ".ini", ".cfg", ".urdf", ".xml", ".proto", ".msg"}
SOURCE_SUFFIXES = {".py", ".c", ".cc", ".cpp", ".h", ".hpp", ".ino", ".js", ".ts", *WHOLE_FILE_SUFFIXES}


def _digest(value: str) -> str:
    return hashlib.sha256(value.encode("utf-8")).hexdigest()


def _source_key(repo: str, path: str) -> str:
    return f"{repo}:{path}"


def _valid_path(repo: str, path: str) -> bool:
    return (isinstance(repo, str) and repo in {"mmhand", "teleop"}
            and isinstance(path, str) and bool(path)
            and not path.startswith(("/", "\\")) and "\\" not in path
            and all(part not in {"", ".", ".."} for part in path.split("/")))


def load_scope(path: Path) -> dict:
    if not path.is_file():
        raise ValueError(f"Missing source scope: {path}; prepare this branch's docs/ai/source_scope.json")
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or data.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("Invalid source scope schema version")
    for key in ("paths", "entrypoints", "p4_tasks", "s3_tasks", "command_sources",
                "extensions", "exclusions", "audit"):
        if key not in data:
            raise ValueError("Missing source scope field: " + key)
    if (not isinstance(data["paths"], dict) or not isinstance(data["entrypoints"], dict)
            or not isinstance(data["command_sources"], dict)
            or any(not isinstance(data[key], list) for key in ("p4_tasks", "s3_tasks", "extensions", "exclusions"))
            or not isinstance(data["audit"], dict)):
        raise ValueError("Invalid source scope collections")
    for entry in data["paths"].values():
        if not isinstance(entry, dict) or not _valid_path(entry.get("repo"), entry.get("path")):
            raise ValueError("Invalid configured source path")
    for graph_id, entries in data["entrypoints"].items():
        if not re.fullmatch(r"[a-z][a-z0-9_]*", graph_id) or not isinstance(entries, list):
            raise ValueError("Invalid configured entrypoint graph")
        for entry in entries:
            if (not isinstance(entry, dict) or not _valid_path(entry.get("repo"), entry.get("path"))
                    or not isinstance(entry.get("symbol"), str) or not entry["symbol"]):
                raise ValueError("Invalid configured entrypoint")
            if "node_id" in entry and (not isinstance(entry["node_id"], str)
                                       or not re.fullmatch(r"[A-Za-z_][A-Za-z_0-9]*", entry["node_id"])):
                raise ValueError("Invalid configured entrypoint node ID")
    for group in ("p4_tasks", "s3_tasks"):
        for entry in data[group]:
            if (not isinstance(entry, dict) or not _valid_path("mmhand", entry.get("path"))
                    or not isinstance(entry.get("name"), str) or not entry["name"]):
                raise ValueError(f"Invalid configured {group} entry")
            if group == "s3_tasks" and not isinstance(entry.get("config_prefix"), str):
                raise ValueError("Invalid S3 task config prefix")
    for entry in data["command_sources"].values():
        if (not isinstance(entry, dict) or not _valid_path(entry.get("repo"), entry.get("path"))
                or not isinstance(entry.get("needle"), str) or not entry["needle"]):
            raise ValueError("Invalid configured command source")
        for extra in entry.get("extra", []):
            if (not isinstance(extra, dict) or not _valid_path(extra.get("repo"), extra.get("path"))
                    or not isinstance(extra.get("needle"), str) or not extra["needle"]):
                raise ValueError("Invalid configured command extra source")
    for group in ("extensions", "exclusions"):
        if any(not isinstance(entry, dict) for entry in data[group]):
            raise ValueError(f"Invalid {group} entry")
    return data


def configured_path(sources, key: str, repo: str = "mmhand") -> str:
    entry = sources.scope["paths"].get(key)
    if not entry or entry["repo"] != repo:
        raise ValueError(f"Missing {repo} source scope path: {key}")
    path = entry["path"]
    if path not in sources.paths[repo]:
        raise ValueError(f"Configured source is not committed: {repo}:{path}")
    return path


def entrypoint_id(graph_id: str, entry: dict) -> str:
    """Keep existing explicit IDs; derive new IDs from path and symbol, never line."""
    return entry.get("node_id") or "source_" + _digest(
        "|".join((graph_id, entry["repo"], entry["path"], entry["symbol"])))[:16]


def validate_configured_scope(sources) -> None:
    """Reject paths that no longer exist in either selected committed revision."""
    entries = list(sources.scope["paths"].values())
    entries += [entry for group in sources.scope["entrypoints"].values() for entry in group]
    entries += [{"repo": "mmhand", "path": entry["path"]}
                for group in ("p4_tasks", "s3_tasks") for entry in sources.scope[group]]
    for command in sources.scope["command_sources"].values():
        entries.append(command)
        entries.extend(command.get("extra", []))
    entries += sources.scope["extensions"] + sources.scope["exclusions"]
    for entry in sources.scope["extensions"]:
        if isinstance(entry.get("anchor"), dict):
            entries.append(entry["anchor"])
    for entry in entries:
        repo, path = entry.get("repo"), entry.get("path")
        if not _valid_path(repo, path) or path not in sources.paths[repo]:
            raise ValueError(f"Scope path is missing from committed source: {repo}:{path}")
    for graph_id, group in sources.scope["entrypoints"].items():
        ids = [entrypoint_id(graph_id, entry) for entry in group]
        if len(ids) != len(set(ids)):
            raise ValueError(f"Duplicate configured entrypoint IDs: {graph_id}")


def _python_spans(rows: list[str], symbol: str) -> list[tuple[int, int]]:
    try:
        tree = ast.parse("\n".join(rows))
    except SyntaxError:
        return []
    found = []

    def visit(body: list, parents: list[str]) -> None:
        for node in body:
            if isinstance(node, (ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)):
                name = ".".join([*parents, node.name])
                if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and symbol in {node.name, name}:
                    found.append((min([node.lineno] + [x.lineno for x in node.decorator_list]),
                                  node.end_lineno or node.lineno))
                visit(node.body, [*parents, node.name])

    visit(tree.body, [])
    return found


def resolve_selector(sources, repo: str, path: str, selector: dict) -> dict:
    if not _valid_path(repo, path) or path not in sources.paths[repo]:
        raise ValueError(f"Scope path is not in committed source: {repo}:{path}")
    if not isinstance(selector, dict) or selector.get("kind") not in {"function", "definition", "file"}:
        raise ValueError(f"Invalid selector for {repo}:{path}")
    rows = sources.lines(repo, path)
    kind = selector["kind"]
    symbol = selector.get("symbol", "")
    if kind == "file":
        if Path(path).suffix.lower() not in WHOLE_FILE_SUFFIXES:
            raise ValueError(f"Whole-file selection is restricted to configs/scripts: {repo}:{path}")
        return sources.evidence(repo, path, 1, len(rows))
    if not isinstance(symbol, str) or not re.fullmatch(r"[A-Za-z_][A-Za-z_0-9:.]*", symbol):
        raise ValueError(f"Invalid selector symbol: {repo}:{path}")
    if kind == "definition":
        matching = [i for i, row in enumerate(rows, 1) if re.search(r"\b" + re.escape(symbol) + r"\b\s*(?:=|:)", row)]
        if len(matching) != 1:
            raise ValueError(f"Definition must resolve uniquely: {repo}:{path}:{symbol} ({len(matching)})")
        return sources.evidence(repo, path, matching[0])
    if Path(path).suffix.lower() == ".py":
        spans = _python_spans(rows, symbol)
    else:
        spans = [span for span in _brace_functions(rows)
                 if re.search(r"\b" + re.escape(symbol.split("::")[-1]) + r"\s*\(",
                              "\n".join(rows[span[0] - 1:span[1]]).split("{", 1)[0])]
    if len(spans) != 1:
        raise ValueError(f"Function must resolve uniquely: {repo}:{path}:{symbol} ({len(spans)})")
    return sources.evidence(repo, path, spans[0][0], spans[0][1])


def _anchor(sources, entry: dict) -> dict:
    anchor = entry.get("anchor")
    if not isinstance(anchor, dict) or not _valid_path(anchor.get("repo"), anchor.get("path")):
        raise ValueError("Scope extension requires a valid source anchor")
    repo, path, needle = anchor["repo"], anchor["path"], anchor.get("needle")
    if path not in sources.paths[repo] or not isinstance(needle, str) or not needle.strip():
        raise ValueError(f"Invalid source anchor: {repo}:{path}")
    matching = [i for i, row in enumerate(sources.lines(repo, path), 1) if needle in row]
    occurrence = anchor.get("occurrence", 1)
    if not isinstance(occurrence, int) or occurrence < 1 or occurrence > len(matching):
        raise ValueError(f"Source anchor does not resolve: {repo}:{path}:{needle}")
    if len(matching) != 1 and "occurrence" not in anchor:
        raise ValueError(f"Ambiguous source anchor; specify occurrence: {repo}:{path}:{needle}")
    return sources.evidence(repo, path, matching[occurrence - 1])


def extension_id(entry: dict) -> str:
    selector = entry["selector"]
    identity = "|".join((entry["graph_id"], entry["repo"], entry["path"],
                         selector["kind"], selector.get("symbol", "")))
    return "scope_" + _digest(identity)[:16]


def _direct_call_evidence(anchor: dict, parent: dict, symbol: str) -> bool:
    """A call edge needs executable call syntax within the parent's cited range."""
    line = anchor["text"].strip()
    if (not line or line.startswith(("#", "//", "/*", "*", "def ", "async def "))
            or line.endswith("{")):
        return False
    bare = symbol.split(".")[-1].split("::")[-1]
    if not re.search(r"\b" + re.escape(bare) + r"\s*\(", line):
        return False
    return any(ref["repo"] == anchor["repo"] and ref["path"] == anchor["path"]
               and ref["start"] <= anchor["start"] <= ref["end"]
               for ref in parent.get("evidence", []))


def apply_extensions(sources, graphs: list[dict]) -> dict:
    by_id = {graph["id"]: graph for graph in graphs}
    seen = set()
    unverified = []
    for entry in sources.scope["extensions"]:
        if not isinstance(entry, dict) or not _valid_path(entry.get("repo"), entry.get("path")):
            raise ValueError("Invalid scope extension path")
        graph = by_id.get(entry.get("graph_id"))
        if graph is None:
            raise ValueError(f"Unknown scope extension graph: {entry.get('graph_id')}")
        parent = entry.get("parent_id")
        parent_node = next((node for node in graph["nodes"] if node["id"] == parent), None)
        if parent_node is None:
            raise ValueError(f"Unknown scope extension parent: {graph['id']}:{parent}")
        relation = entry.get("relation")
        if relation not in RELATIONS:
            raise ValueError(f"Invalid scope relation: {relation}")
        node_id = extension_id(entry)
        if node_id in seen or node_id in {node["id"] for node in graph["nodes"]}:
            raise ValueError(f"Duplicate scope extension: {node_id}")
        seen.add(node_id)
        selected = resolve_selector(sources, entry["repo"], entry["path"], entry.get("selector"))
        anchor = _anchor(sources, entry)
        symbol = entry["selector"].get("symbol", Path(entry["path"]).name)
        proven_call = relation == "calls" and _direct_call_evidence(anchor, parent_node, symbol)
        uncertain = not proven_call
        if uncertain:
            unverified.append(f"{graph['id']}:{node_id}")
        graph["nodes"].append({"id": node_id, "label": {"zh": symbol, "en": symbol},
                               "kind": "process", "evidence": [selected, anchor],
                               "details": {"source_scope": {"repo": entry["repo"], "path": entry["path"],
                                                             "selector": entry["selector"], "relation": relation},
                                           "text": {"zh": "此源码范围由路径索引补入；关系见锚点源码。",
                                                    "en": "The source scope adds this range; inspect the anchor for the relationship."}},
                               "drilldown": "", "unverified": uncertain})
        graph["edges"].append({"source": parent, "target": node_id,
                               "label": {"zh": relation, "en": relation},
                               "evidence": [anchor], "condition": "", "unverified": uncertain})
    return {"extensions": len(seen), "unverified_links": unverified}


def _hints(rows: list[str]) -> list[dict]:
    patterns = (
        ("python_definition", re.compile(r"^\s*(?:async\s+)?def\s+([A-Za-z_]\w*)\s*\(")),
        ("ui_signal", re.compile(r"\.(?:clicked|toggled|triggered|currentChanged)\.connect\s*\(")),
        ("ros_topic", re.compile(r"create_(?:publisher|subscription)\s*\(|/raw_ik_target")),
        ("freertos_task", re.compile(r"xTaskCreate(?:PinnedToCore)?\s*\(|pdMS_TO_TICKS\s*\(")),
        ("protocol", re.compile(r"\b(?:CAN_ID_[A-Z0-9_]+|CMD_[A-Z0-9_]+)\b")),
        ("configuration", re.compile(r"\b(?:CONFIG_|TASK_[A-Z_]+|ROBOT_TOPIC)\b")),
    )
    found = []
    for i, row in enumerate(rows, 1):
        for kind, pattern in patterns:
            match = pattern.search(row)
            if match:
                found.append({"kind": kind, "line": i, "symbol": match.group(1) if match.lastindex else match.group(0)[:80]})
                break
        if len(found) >= 12:
            break
    return found


def _eligible(repo: str, path: str, kind: str | None = None) -> bool:
    if Path(path).suffix.lower() not in SOURCE_SUFFIXES or path.startswith("docs/ai/"):
        return False
    if repo == "mmhand":
        return kind in {"source", "generated"}
    return not (path.startswith((".github/", "tests/", "test/")) or "/vendor/" in path)


def _related_paths(sources, paths: list[tuple[str, str]], changes: dict,
                   teleop_changes: dict | None) -> dict[str, str]:
    """Find callers, callees, topics, constants and UI bindings near changed code."""
    changed_tokens: set[str] = set()
    for repo, path in paths:
        spans = (changes if repo == "mmhand" else (teleop_changes or {})).get(path, [])
        if not spans:
            continue
        rows = sources.lines(repo, path)
        for start, end in spans:
            for row in rows[max(0, start - 3):min(len(rows), end + 2)]:
                changed_tokens.update(token for token in re.findall(r"\b[A-Za-z_][A-Za-z_0-9]{7,}\b", row)
                                      if ("_" in token or any(char.isupper() for char in token[1:]))
                                      and token not in {"sharedData", "const_cast", "static_cast"})
    if not changed_tokens:
        return {}
    # Common tokens create noise; require a plausible symbol with limited fan-out.
    references: dict[str, set[str]] = {}
    for repo, path in paths:
        content = sources.read(repo, path)
        for token in changed_tokens:
            if token in content and re.search(r"\b" + re.escape(token) + r"\b", content):
                references.setdefault(token, set()).add(_source_key(repo, path))
    related: dict[str, str] = {}
    for token, keys in references.items():
        if 1 < len(keys) <= 24:
            for key in keys:
                related.setdefault(key, f"references changed symbol {token}")
    return related


def discover_candidates(sources, graphs: list[dict], inventory: list[dict], changes: dict,
                        branch: str, baseline: str | None, force_full: bool = False,
                        teleop_changes: dict | None = None) -> dict:
    """Inventory the committed source; provide paths and symbols, never full code."""
    mode = "full" if branch == "main" or force_full or not baseline else "incremental"
    mapped: dict[str, list[tuple[int, int]]] = {}
    for graph in graphs:
        for item in [*graph["nodes"], *graph["edges"]]:
            for ref in item.get("evidence", []):
                mapped.setdefault(_source_key(ref["repo"], ref["path"]), []).append((ref["start"], ref["end"]))
    inventory_kind = {item["path"]: item["kind"] for item in inventory}
    exclusions = {}
    for item in sources.scope["exclusions"]:
        if (not isinstance(item, dict) or not _valid_path(item.get("repo"), item.get("path"))
                or not isinstance(item.get("source_hash"), str)
                or not isinstance(item.get("reason"), str)
                or len(item["reason"].strip()) < 20):
            raise ValueError("Invalid source scope exclusion; cite a path, source_hash, and reason")
        key = _source_key(item["repo"], item["path"])
        if key in exclusions:
            raise ValueError("Duplicate scope exclusion: " + key)
        exclusions[key] = item
    paths = [("mmhand", path) for path in sources.paths["mmhand"]
             if _eligible("mmhand", path, inventory_kind.get(path))]
    paths += [("teleop", path) for path in sources.paths["teleop"] if _eligible("teleop", path)]
    related = _related_paths(sources, paths, changes, teleop_changes) if mode == "incremental" else {}
    candidates = []
    for repo, path in sorted(paths):
        key = _source_key(repo, path)
        spans = (changes if repo == "mmhand" else (teleop_changes or {})).get(path, [])
        refs = mapped.get(key, [])
        changed_uncovered = bool(spans and any(not any(a <= end and b >= start for a, b in refs)
                                               for start, end in spans))
        if (mode == "incremental" and not spans and key not in related
                and not (repo == "teleop" and teleop_changes is None)):
            continue
        if mode == "incremental" and refs and not changed_uncovered and not spans and key not in related:
            continue
        rows = sources.lines(repo, path)
        # Blank-line movement changes citations, not the reviewed source content.
        source_hash = _digest("\n".join(row for row in rows if row.strip()))
        excluded = exclusions.get(key)
        status = ("mapped" if refs and not changed_uncovered else
                  "excluded" if excluded and excluded["source_hash"] == source_hash else "unresolved")
        candidates.append({"id": "path_" + _digest(key)[:16], "repo": repo, "path": path,
                           "kind": inventory_kind.get(path, "source") if repo == "mmhand" else "source",
                           "source_hash": source_hash, "changed_ranges": spans,
                           "hints": _hints(rows), "status": status,
                           "reason": (related[key] if key in related and not spans else
                                      ("changed code covered by graph evidence" if spans else
                                       "source cited by a graph") if status == "mapped" else
                                      "changed code outside graph evidence" if changed_uncovered else
                                      related.get(key, "source path not mapped to a graph")),
                           "exclusion_reason": excluded["reason"] if status == "excluded" else ""})
    audit = sources.scope["audit"]
    expected = {"branch": branch, "mmhand": sources.commits["mmhand"],
                "teleop": sources.commits["teleop"], "mode": mode}
    audit_current = all(audit.get(key) == value for key, value in expected.items())
    unresolved = [item["id"] for item in candidates if item["status"] == "unresolved"]
    if not audit_current:
        unresolved.insert(0, "scope:audit")
    return {"schema_version": SCHEMA_VERSION, "mode": mode, "commits": expected,
            "mapped_paths": len(mapped), "candidates": candidates,
            "unresolved_ids": unresolved, "audit_current": audit_current}
