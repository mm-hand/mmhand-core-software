"""Prepare source-backed explanation tasks and validate externally written annotations."""

from __future__ import annotations

import json
from pathlib import Path
import re

from explain import FIELDS, ExplanationError, ref_id
from graph_context import build_bundle, bundle_estimate
from source import Sources


ANNOTATION_SCHEMA_VERSION = 1
INSTRUCTION_VERSION = 2

# These phrases are deliberately rejected: they describe a diagram generically
# without explaining the source-backed responsibility of the individual item.
GENERIC_TEXT = {
    "zh": (
        "图中的一个节点", "对应的代码职责或状态", "未显示写入、调用或硬件结果",
        "以源码片段作为定位依据", "同一图中其他节点相比",
    ),
    "en": (
        "is a node in the", "locating the corresponding code responsibility",
        "the source excerpts associated with this node", "writes, calls, or hardware outcomes not shown",
        "compared with other graph items",
    ),
}


def _condition_expressions(card: dict) -> list[str]:
    expressions = []
    if card.get("condition", "").strip():
        expressions.append(card["condition"].strip())
    expressions.extend(part["expr"].strip() for part in card.get("details", {}).get("conditions", [])
                       if isinstance(part, dict) and part.get("expr", "").strip())
    return list(dict.fromkeys(expressions))


def _contains_generic_text(lang: str, values: dict) -> bool:
    content = " ".join(values.values()).casefold()
    return any(phrase.casefold() in content for phrase in GENERIC_TEXT[lang])

LANG_SCHEMA = {"type": "object", "additionalProperties": False,
               "properties": {field: {"type": "string"} for field in FIELDS},
               "required": list(FIELDS)}
SUMMARY_SCHEMA = {"type": "object", "additionalProperties": False,
                  "properties": {"zh": {"type": "string"}, "en": {"type": "string"},
                                 "evidence_ids": {"type": "array", "items": {"type": "string"}}},
                  "required": ["zh", "en", "evidence_ids"]}
SCHEMA = {"type": "object", "additionalProperties": False,
          "properties": {
              "schema_version": {"type": "integer"},
              "instruction_version": {"type": "integer"},
              "graph_id": {"type": "string"},
              "bundle_hash": {"type": "string"},
              "overview": SUMMARY_SCHEMA,
              "glossary": {"type": "array", "items": {"type": "object", "additionalProperties": False,
                           "properties": {"name": {"type": "string"}, "zh": {"type": "string"},
                                          "en": {"type": "string"},
                                          "evidence_ids": {"type": "array", "items": {"type": "string"}}},
                           "required": ["name", "zh", "en", "evidence_ids"]}},
              "items": {"type": "array", "items": {"type": "object", "additionalProperties": False,
                        "properties": {"id": {"type": "string"}, "zh": LANG_SCHEMA,
                                       "en": LANG_SCHEMA,
                                       "evidence_ids": {"type": "array", "items": {"type": "string"}}},
                        "required": ["id", "zh", "en", "evidence_ids"]}}},
          "required": ["schema_version", "instruction_version", "graph_id", "bundle_hash",
                       "overview", "glossary", "items"]}


def is_conditional(card: dict) -> bool:
    return bool(card["condition"].strip() or any(
        isinstance(part, dict) and part.get("expr", "").strip()
        for part in card["details"].get("conditions", [])))


def is_branch_predicate(card: dict) -> bool:
    """Separate boolean branches from edges that name a data or signal channel."""
    if card["kind"] != "edge" or not is_conditional(card):
        return False
    title = card.get("title", {})
    title_text = " ".join(title.values()) if isinstance(title, dict) else str(title)
    if "predicate" in title_text.casefold() or "condition" in title_text.casefold():
        return True
    return any(re.search(
        r"(?:==|!=|<=|>=|\b(?:not|and|or|is|in)\b)", expr)
        for expr in _condition_expressions(card))


def needs_explanation(card: dict) -> bool:
    return card["kind"] != "edge" or is_conditional(card)


def plans(sources: Sources, graphs: list[dict], cards: list[dict]) -> list[tuple[dict, list[dict]]]:
    grouped: dict[str, list[dict]] = {}
    for card in cards:
        grouped.setdefault(card["graph"], []).append(card)
    result = [(build_bundle(sources, graph, grouped[graph["id"]]), grouped[graph["id"]])
              for graph in graphs]
    if grouped.get("readme"):
        result.append((build_bundle(sources, None, grouped["readme"]), grouped["readme"]))
    return result


def estimate(sources: Sources, graphs: list[dict], cards: list[dict]) -> dict:
    sections = [bundle_estimate(bundle, [card["id"] for card in items if needs_explanation(card)])
                for bundle, items in plans(sources, graphs, cards)]
    return {"graphs": len(graphs), "items": len(cards), "task_files": len(sections),
            "required_explanations": sum(item["required_explanations"] for item in sections),
            "structural_edges_from_template": sum(card["kind"] == "edge" and not needs_explanation(card)
                                                  for card in cards),
            "source_chars": sum(item["source_chars"] for item in sections), "sections": sections}


def _packet(bundle: dict, group: list[dict], commits: dict) -> dict:
    return {"schema_version": ANNOTATION_SCHEMA_VERSION,
            "instruction_version": INSTRUCTION_VERSION,
            "graph_id": bundle["id"], "bundle_hash": bundle["hash"],
            "commits": commits, "title": bundle["title"], "parent": bundle["parent"],
            "manifest": bundle["manifest"], "evidence": [
                {"id": key, **ref} for key, ref in bundle["refs"].items()],
            "source": [{"repo": part["repo"], "path": part["path"],
                        "start": part["start"], "end": part["end"], "kind": part["kind"],
                        "numbered_code": "\n".join(
                f"{number}: {row}" for number, row in enumerate(part["code"].splitlines(), part["start"]))}
                for part in bundle["code"]],
            "required_ids": [card["id"] for card in group if needs_explanation(card)],
            "templated_ids": [card["id"] for card in group if not needs_explanation(card)],
            "annotation_path": f"docs/ai/annotations/{bundle['id']}.json",
            "response_schema": SCHEMA}


def prepare_tasks(sources: Sources, graphs: list[dict], cards: list[dict],
                  task_dir: Path, commits: dict, candidate_report: dict | None = None) -> dict:
    """Write local task packets; no model process or network call is involved."""
    task_dir.mkdir(parents=True, exist_ok=True)
    entries = []
    for bundle, group in plans(sources, graphs, cards):
        if not re.fullmatch(r"[A-Za-z0-9_]+", bundle["id"]):
            raise ValueError("Unsafe graph ID: " + bundle["id"])
        name = bundle["id"] + ".json"
        packet = _packet(bundle, group, commits)
        target = task_dir / name
        content = json.dumps(packet, ensure_ascii=False, indent=2) + "\n"
        if not target.is_file() or target.read_text(encoding="utf-8") != content:
            target.write_text(content, encoding="utf-8")
        entries.append({"graph_id": bundle["id"], "bundle_hash": bundle["hash"],
                        "task_file": name, "annotation_path": packet["annotation_path"],
                        "required_ids": packet["required_ids"]})
    index = {"schema_version": ANNOTATION_SCHEMA_VERSION,
             "instruction_version": INSTRUCTION_VERSION, "commits": commits,
             "instructions": "docs/ai/GENERATE.md", "tasks": entries}
    if candidate_report is not None:
        (task_dir / "scope_candidates.json").write_text(
            json.dumps(candidate_report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        index["scope_candidates"] = "scope_candidates.json"
    (task_dir / "index.json").write_text(json.dumps(index, ensure_ascii=False, indent=2) + "\n",
                                         encoding="utf-8")
    return index


def _validate_item(item: dict, card: dict, allowed: set[str], own: set[str],
                   context_text: str) -> None:
    if set(item) != {"id", "zh", "en", "evidence_ids"} or item.get("id") != card["id"]:
        raise ValueError("Invalid item or ID: " + card["id"])
    if (not isinstance(item["evidence_ids"], list) or not item["evidence_ids"]
            or not set(item["evidence_ids"]).issubset(allowed)
            or not set(item["evidence_ids"]).intersection(own)):
        raise ValueError("Invalid evidence IDs: " + card["id"])
    for lang in ("zh", "en"):
        values = item[lang]
        if not isinstance(values, dict) or set(values) != set(FIELDS):
            raise ValueError("Missing bilingual fields: " + card["id"])
        min_plain_length = 22 if lang == "zh" else 35
        if (not all(isinstance(value, str) for value in values.values())
                or len(values["plain"].strip()) < min_plain_length):
            raise ValueError("Empty explanation: " + card["id"])
        for field, value in values.items():
            if len(value.strip()) < 12:
                raise ValueError(f"Explanation field too short ({lang}.{field}): {card['id']}")
        if _contains_generic_text(lang, values):
            raise ValueError(f"Generic template explanation ({lang}): {card['id']}")
        text = ' '.join(values.values())
        for term in card.get('details', {}).get('required_terms', []):
            if term not in text:
                raise ValueError(f"Required installation/use content omitted ({lang}): {card['id']}: {term}")
        if is_conditional(card):
            condition = values["condition"]
            if len(condition.strip()) < 24:
                raise ValueError("Missing condition meaning: " + card["id"])
            for expression in _condition_expressions(card):
                if expression not in condition:
                    raise ValueError(f"Raw condition omitted ({lang}): {card['id']}: {expression}")
        if is_branch_predicate(card):
            failure = values["failure"].casefold()
            if lang == "en":
                has_true = re.search(r"\btrue\b", failure) is not None
                has_false = re.search(r"\bfalse\b", failure) is not None
            else:
                has_true = any(marker in failure for marker in ("为真", "真时", "条件成立", "真分支", "成立时", "真：", "真:"))
                has_false = any(marker in failure for marker in ("为假", "假时", "条件不成立", "假分支", "不成立时", "假：", "假:"))
            if not (has_true and has_false):
                raise ValueError(f"Conditional edge must explain both true and false outcomes ({lang}): {card['id']}")
        if card["details"].get("chain") and not values["unknown"].strip():
            raise ValueError("Safety uncertainty omitted: " + card["id"])
        if card["changed"] and not values["change"].strip():
            raise ValueError("Change impact omitted: " + card["id"])
        for identifier in re.findall(r"`([A-Za-z_][A-Za-z_0-9:]*)`", " ".join(values.values())):
            if identifier not in context_text:
                raise ValueError("Unsubstantiated identifier: " + identifier)


def _validated_response(response: dict, bundle: dict, requested: list[dict]) -> tuple[dict[str, dict], dict | None]:
    if not isinstance(response, dict) or response.get("graph_id") != bundle["id"]:
        raise ValueError("Wrong graph response")
    if set(response) != set(SCHEMA["required"]):
        raise ValueError("Wrong annotation fields")
    if response.get("schema_version") != ANNOTATION_SCHEMA_VERSION:
        raise ValueError("Wrong annotation schema version")
    if response.get("instruction_version") != INSTRUCTION_VERSION:
        raise ValueError("Stale annotation instruction version")
    if response.get("bundle_hash") != bundle["hash"]:
        raise ValueError("Stale source bundle hash")
    expected = {card["id"]: card for card in requested}
    if not isinstance(response.get("items"), list):
        raise ValueError("Missing items")
    manifest = {item["id"]: item for item in bundle["manifest"]}
    context_text = json.dumps(bundle["manifest"], ensure_ascii=False) + "\n" + "\n".join(
        part["code"] for part in bundle["code"])
    found = {}
    for item in response["items"]:
        if not isinstance(item, dict) or item.get("id") not in expected or item["id"] in found:
            raise ValueError("Unexpected or duplicate response item")
        own = set(manifest[item["id"]]["evidence_ids"])
        _validate_item(item, expected[item["id"]], set(bundle["refs"]), own, context_text)
        found[item["id"]] = item
    if set(found) != set(expected):
        raise ValueError("Omitted IDs: " + ", ".join(sorted(set(expected) - set(found))))
    meta = None
    if bundle["id"] != "readme_commands":
        overview = response.get("overview")
        if not isinstance(overview, dict) or not all(len(overview.get(lang, "").strip()) >= 15
                                                      for lang in ("zh", "en")):
            raise ValueError("Missing bilingual graph overview")
        if (not isinstance(overview.get("evidence_ids"), list) or not overview["evidence_ids"]
                or not set(overview["evidence_ids"]).issubset(bundle["refs"])):
            raise ValueError("Invalid graph overview evidence")
        glossary = response.get("glossary")
        if not isinstance(glossary, list) or len(glossary) > 20:
            raise ValueError("Invalid graph glossary")
        term_refs: dict[str, set[str]] = {}
        for entry in bundle["manifest"]:
            for definition in entry.get("variable_definitions", []):
                term_refs.setdefault(definition["name"], set()).add(definition["source_id"])
        for term in glossary:
            if not isinstance(term, dict) or set(term) != {"name", "zh", "en", "evidence_ids"}:
                raise ValueError("Invalid glossary item")
            if term["name"] not in term_refs or not term["zh"].strip() or not term["en"].strip():
                raise ValueError("Unverified glossary term")
            if (not isinstance(term["evidence_ids"], list) or not term["evidence_ids"]
                    or not set(term["evidence_ids"]).issubset(bundle["refs"])):
                raise ValueError("Invalid glossary evidence")
            if not set(term["evidence_ids"]).intersection(term_refs[term["name"]]):
                raise ValueError("Glossary definition not cited")
        meta = {"overview": overview, "glossary": glossary}
    return found, meta


def _template(card: dict, manifest: dict, node_titles: dict) -> dict:
    label = card["title"]
    source = node_titles.get(card["source"], {"zh": card["source"], "en": card["source"]})
    target = node_titles.get(card["target"], {"zh": card["target"], "en": card["target"]})
    texts = {}
    for lang in ("zh", "en"):
        a = source[lang] if isinstance(source, dict) else source
        b = target[lang] if isinstance(target, dict) else target
        relation = label[lang] if isinstance(label, dict) else label
        plain = (f"图中以「{relation}」连接「{a}」与「{b}」。它标记两节点在源码中的关联；具体触发时机请查看两端节点及引用源码。"
                 if lang == "zh" else
                 f"This graph links '{a}' to '{b}' as '{relation}'. The link marks a source-backed relationship; inspect both nodes and the cited code for its actual timing.")
        texts[lang] = {field: (plain if field == "plain" else "") for field in FIELDS}
    return {"id": card["id"], "zh": texts["zh"], "en": texts["en"],
            "evidence_ids": manifest[card["id"]]["evidence_ids"][:1]}


def _materialize(item: dict, bundle: dict) -> dict:
    return {**item, "evidence_ids": [ref_id(bundle["refs"][key]) for key in item["evidence_ids"]]}


def _materialize_meta(meta: dict, bundle: dict) -> dict:
    overview = {**meta["overview"], "evidence": [bundle["refs"][key]
                                                    for key in meta["overview"]["evidence_ids"]]}
    glossary = [{**term, "evidence": [bundle["refs"][key] for key in term["evidence_ids"]]}
                for term in meta["glossary"]]
    return {"overview": overview, "glossary": glossary}


def load_annotations(sources: Sources, graphs: list[dict], cards: list[dict],
                     annotations_dir: Path) -> tuple[dict[str, dict], dict[str, dict], dict]:
    """Validate every graph before rendering; return source-line-refreshed explanations."""
    results: dict[str, dict] = {}
    graph_meta: dict[str, dict] = {}
    issues: list[dict] = []
    missing: list[str] = []
    valid_graphs = 0
    work = plans(sources, graphs, cards)
    for bundle, group in work:
        graph_id = bundle["id"]
        path = annotations_dir / f"{graph_id}.json"
        required = [card for card in group if needs_explanation(card)]
        if not path.is_file():
            issues.append({"graph_id": graph_id, "file": str(path), "reason": "Missing annotation file"})
            missing.extend(card["id"] for card in required)
            continue
        response = None
        try:
            response = json.loads(path.read_text(encoding="utf-8"))
            found, meta = _validated_response(response, bundle, required)
        except (OSError, ValueError, TypeError, KeyError, AttributeError) as exc:
            issues.append({"graph_id": graph_id, "file": str(path), "reason": str(exc)})
            present = ({item["id"] for item in response["items"]
                        if isinstance(item, dict) and isinstance(item.get("id"), str)}
                       if isinstance(response, dict) and isinstance(response.get("items"), list)
                       else set())
            missing.extend(card["id"] for card in required if card["id"] not in present)
            if not any(card["id"] not in present for card in required):
                missing.extend(card["id"] for card in required)
            continue
        manifest = {item["id"]: item for item in bundle["manifest"]}
        node_titles = {card["id"].split(":node:", 1)[1]: card["title"] for card in group
                       if card["kind"] == "node"}
        for card in group:
            item = (found[card["id"]] if needs_explanation(card) else
                    _template(card, manifest, node_titles))
            results[card["id"]] = _materialize(item, bundle)
        if meta:
            graph_meta[graph_id] = _materialize_meta(meta, bundle)
        valid_graphs += 1
    all_ids = {card["id"] for card in cards}
    missing = sorted(set(missing) | (all_ids - set(results)))
    report = {"annotation_files": valid_graphs, "expected_files": len(work),
              "completed_items": len(results), "required_items": len(cards),
              "missing_ids": missing, "issues": issues}
    if issues or missing:
        raise ExplanationError("Annotations are incomplete or stale", missing, report)
    return results, graph_meta, report
