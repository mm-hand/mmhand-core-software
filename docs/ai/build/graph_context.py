"""Deduplicated, committed-source context for one graph or the README commands."""

from __future__ import annotations

import ast
import hashlib
import json
from pathlib import Path
import re

from source import Sources


FUNCTION_SUFFIXES = {".c", ".cc", ".cpp", ".h", ".hpp", ".ino", ".js", ".ts"}
CONTROL_WORDS = {"if", "for", "while", "switch", "catch", "sizeof"}


def _python_functions(rows: list[str]) -> list[tuple[int, int]]:
    try:
        tree = ast.parse("\n".join(rows))
    except SyntaxError:
        return []
    return [(min([node.lineno] + [part.lineno for part in node.decorator_list]),
             node.end_lineno or node.lineno)
            for node in ast.walk(tree) if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))]


def _masked_c(text: str) -> str:
    """Keep offsets and newlines while hiding comments and quoted braces."""
    pattern = re.compile(r'//[^\n]*|/\*[\s\S]*?\*/|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'')
    return pattern.sub(lambda match: re.sub(r"[^\n]", " ", match.group()), text)


def _brace_functions(rows: list[str]) -> list[tuple[int, int]]:
    text = "\n".join(rows)
    masked = _masked_c(text)
    starts = [0]
    starts.extend(i + 1 for i, char in enumerate(masked) if char == "\n")
    stack: list[int] = []
    candidates: list[tuple[int, int]] = []
    for index, char in enumerate(masked):
        if char == "{":
            stack.append(index)
        elif char == "}" and stack:
            opening = stack.pop()
            start_line = _line_number(starts, opening)
            end_line = _line_number(starts, index)
            prefix = masked[max(0, opening - 500):opening].rstrip()
            match = re.search(r"([A-Za-z_~][\w:~]*)\s*\([^;{}]*\)\s*(?:const\s*)?(?:noexcept\s*)?$", prefix)
            if match and match[1].split("::")[-1] not in CONTROL_WORDS:
                signature_start = max(1, start_line - prefix[match.start():].count("\n"))
                candidates.append((signature_start, end_line))
    return candidates


def _line_number(starts: list[int], position: int) -> int:
    import bisect
    return bisect.bisect_right(starts, position)


def _source_range(rows: list[str], path: str, line: int,
                  spans: list[tuple[int, int]]) -> tuple[int, int, str]:
    suffix = Path(path).suffix.lower()
    if suffix in {".ps1", ".sh", ".cmd", ".bat"} or (path == "track.py" and len(rows) <= 150):
        return 1, len(rows), "script"
    enclosing = [span for span in spans if span[0] <= line <= span[1]]
    if enclosing:
        span = min(enclosing, key=lambda item: item[1] - item[0])
        return (*span, "function")
    # A configuration entry or definition without a function body needs its local context.
    return max(1, line - 12), min(len(rows), line + 16), "fragment"


def _stable_ref(sources: Sources, ref: dict) -> str:
    rows = sources.lines(ref["repo"], ref["path"])
    needle = tuple(rows[ref["start"] - 1:ref["end"]])
    width = len(needle)
    occurrence = sum(tuple(rows[i:i + width]) == needle for i in range(ref["start"] - 1))
    raw = json.dumps([ref["repo"], ref["path"], needle, occurrence], ensure_ascii=False)
    return "src_" + hashlib.sha256(raw.encode("utf-8")).hexdigest()[:24]


def _strip_refs(value: object, sources: Sources, refs: dict[str, dict]) -> object:
    if isinstance(value, dict):
        if {"repo", "path", "start", "end", "text"}.issubset(value):
            stable = _stable_ref(sources, value)
            refs[stable] = value
            return stable
        return {key: _strip_refs(part, sources, refs) for key, part in value.items()}
    if isinstance(value, list):
        return [_strip_refs(part, sources, refs) for part in value]
    return value


def _semantic_diff(diff: str) -> str:
    return re.sub(r"(?m)^@@[^\n]*@@", "@@ @@", diff)


def _source_ids(value: object) -> list[str]:
    if isinstance(value, str) and value.startswith("src_"):
        return [value]
    if isinstance(value, dict):
        return [item for part in value.values() for item in _source_ids(part)]
    if isinstance(value, list):
        return [item for part in value for item in _source_ids(part)]
    return []


def build_bundle(sources: Sources, graph: dict | None, cards: list[dict]) -> dict:
    """One graph manifest plus complete functions containing its direct evidence."""
    refs: dict[str, dict] = {}
    manifest = []
    for card in cards:
        evidence_ids = []
        for entry in card["evidence"]:
            ref = entry["ref"]
            stable = _stable_ref(sources, ref)
            refs[stable] = ref
            evidence_ids.append(stable)
        definitions = []
        for definition in card.get("variable_definitions", []):
            original = definition.get("ref") or next((entry["ref"] for entry in card["evidence"]
                                                     if entry["id"] == definition["source"]), None)
            if original:
                stable = _stable_ref(sources, original)
                refs[stable] = original
            else:
                stable = ""
            definitions.append({"name": definition["name"], "source_id": stable,
                                "source_text": definition["text"]})
        details = _strip_refs(card["details"], sources, refs)
        manifest.append({"id": card["id"], "kind": card["kind"], "title": card["title"],
                         "source": card.get("source", ""), "target": card.get("target", ""),
                         "drilldown": card.get("drilldown", ""), "condition": card["condition"],
                         "details": details,
                         "evidence_ids": list(dict.fromkeys(evidence_ids + _source_ids(details))),
                         "variable_definitions": definitions,
                         "changed": card["changed"],
                         "diff": [_semantic_diff(part) for part in card["diff"]]})
    ranges: dict[tuple[str, str, int, int], str] = {}
    file_spans: dict[tuple[str, str], list[tuple[int, int]]] = {}
    for ref in refs.values():
        rows = sources.lines(ref["repo"], ref["path"])
        file_key = (ref["repo"], ref["path"])
        if file_key not in file_spans:
            suffix = Path(ref["path"]).suffix.lower()
            file_spans[file_key] = (_python_functions(rows) if suffix == ".py" else
                                    _brace_functions(rows) if suffix in FUNCTION_SUFFIXES else [])
        start, end, kind = _source_range(rows, ref["path"], ref["start"], file_spans[file_key])
        ranges[(ref["repo"], ref["path"], start, end)] = kind
    code = []
    merged: list[tuple[str, str, int, int, str]] = []
    for repo, path, start, end in sorted(ranges):
        kind = ranges[(repo, path, start, end)]
        if merged and merged[-1][0:2] == (repo, path) and start <= merged[-1][3] + 1:
            old_repo, old_path, old_start, old_end, old_kind = merged[-1]
            merged[-1] = (old_repo, old_path, old_start, max(old_end, end),
                          old_kind if old_kind == kind else "mixed")
        else:
            merged.append((repo, path, start, end, kind))
    for repo, path, start, end, kind in merged:
        rows = sources.lines(repo, path)
        content = "\n".join(rows[start - 1:end])
        code.append({"repo": repo, "path": path, "start": start, "end": end,
                     "kind": kind, "code": content})
    semantic = {"graph_id": graph["id"] if graph else "readme_commands",
                "title": graph["title"] if graph else {"zh": "操作命令", "en": "Operation commands"},
                "parent": graph.get("parent", "") if graph else "",
                "items": manifest,
                "source": [{key: part[key] for key in ("repo", "path", "kind", "code")} for part in code]}
    digest = hashlib.sha256(json.dumps(semantic, ensure_ascii=False, sort_keys=True).encode("utf-8")).hexdigest()
    return {"id": semantic["graph_id"], "hash": digest, "manifest": manifest,
            "code": code, "refs": refs, "title": semantic["title"], "parent": semantic["parent"]}


def bundle_estimate(bundle: dict, explanation_ids: list[str]) -> dict:
    source_chars = sum(len(part["code"]) for part in bundle["code"])
    return {"graph_id": bundle["id"], "items": len(bundle["manifest"]),
            "required_explanations": len(explanation_ids),
            "source_files": len({(part["repo"], part["path"]) for part in bundle["code"]}),
            "source_functions_or_fragments": len(bundle["code"]), "source_chars": source_chars,
            "source_fragments": sum(part["kind"] == "fragment" for part in bundle["code"])}
