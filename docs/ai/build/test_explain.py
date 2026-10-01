"""External annotation contract and graph-source packet checks."""

from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest

from explain import FIELDS, ExplanationError
from graph_context import build_bundle
from graph_explain import (ANNOTATION_SCHEMA_VERSION, INSTRUCTION_VERSION,
                           _validate_item, estimate, load_annotations, prepare_tasks)


class FakeSources:
    def __init__(self, pad: int = 0):
        self.rows = ["# shift"] * pad + ["def transition(value):", "    allowed = value > 0",
                                          "    return allowed"]

    def lines(self, repo: str, path: str) -> list[str]:
        assert (repo, path) == ("mmhand", "sample.py")
        return self.rows


def fixture(pad: int = 0):
    sources = FakeSources(pad)
    line = pad + 2
    ref = {"repo": "mmhand", "path": "sample.py", "start": line, "end": line,
           "text": sources.rows[line - 1]}
    graph = {"id": "sample", "title": {"zh": "状态图", "en": "State graph"}, "parent": ""}
    cards = []
    for index in range(2):
        cards.append({"id": f"graph:sample:node:{index}", "kind": "node", "graph": "sample",
                      "title": {"zh": f"节点{index}", "en": f"Node {index}"}, "condition": "",
                      "details": {}, "evidence": [{"id": f"mmhand:sample.py:L{line}-L{line}", "ref": ref}],
                      "variable_definitions": [], "changed": False, "diff": [],
                      "source": "", "target": "", "drilldown": ""})
    for index, condition in enumerate(("", "allowed == true")):
        cards.append({"id": f"graph:sample:edge:{index}:0:1", "kind": "edge", "graph": "sample",
                      "title": {"zh": "继续", "en": "Next"}, "condition": condition, "details": {},
                      "evidence": [{"id": f"mmhand:sample.py:L{line}-L{line}", "ref": ref}],
                      "variable_definitions": [], "changed": False, "diff": [],
                      "source": "0", "target": "1", "drilldown": ""})
    return sources, graph, cards


def response(bundle: dict) -> dict:
    ref = next(iter(bundle["refs"]))
    items = []
    for card in bundle["manifest"]:
        if card["kind"] == "edge" and not card["condition"]:
            continue
        bilingual = {}
        for lang in ("zh", "en"):
            values = {}
            for field in FIELDS:
                if field == "plain":
                    value = (f"节点 {card['title'][lang]} 调用状态处理代码；该结果决定是否沿此控制路径继续。"
                             if lang == "zh" else
                             f"Node {card['title'][lang]} invokes the state handler; its result determines whether this control path continues.")
                elif field == "condition" and card["kind"] == "edge" and card["condition"]:
                    value = ("原始条件 `allowed == true`：`allowed` 表示前置判断结果；条件成立才沿此边继续。"
                             if lang == "zh" else
                             "Raw condition `allowed == true`: `allowed` is the preceding check result; this edge is followed only when true.")
                elif field == "failure" and card["kind"] == "edge" and card["condition"]:
                    value = ("条件成立时沿此边继续；条件不成立时不经过此边。" if lang == "zh" else
                             "When the condition is true, follow this edge; when false, do not take this edge.")
                else:
                    value = (f"源码中的 `{card['title'][lang]}` 是本项的标识；本字段说明它在流程中的作用。"
                             if lang == "zh" else
                             f"The source identifier `{card['title'][lang]}` names this item; this field explains its role in the flow.")
                values[field] = value
            bilingual[lang] = values
        items.append({"id": card["id"], **bilingual, "evidence_ids": [ref]})
    return {"schema_version": ANNOTATION_SCHEMA_VERSION,
            "instruction_version": INSTRUCTION_VERSION,
            "graph_id": bundle["id"], "bundle_hash": bundle["hash"],
            "overview": {"zh": "本图展示状态判断和后续路径之间的关系。",
                         "en": "This graph shows how the state check determines the following path.",
                         "evidence_ids": [ref]}, "glossary": [], "items": items}


class GraphExplanationTest(unittest.TestCase):
    def test_prepare_exports_complete_source_and_stable_ids(self) -> None:
        sources, graph, cards = fixture()
        with tempfile.TemporaryDirectory() as temp:
            task_dir = Path(temp) / "tasks"
            index = prepare_tasks(sources, [graph], cards, task_dir,
                                  {"mmhand": "abc", "teleop": "def", "branch": "main"})
            packet = json.loads((task_dir / "sample.json").read_text(encoding="utf-8"))
            self.assertEqual(index["tasks"][0]["bundle_hash"], packet["bundle_hash"])
            self.assertEqual(packet["commits"]["teleop"], "def")
            self.assertEqual(len(packet["required_ids"]), 3)
            self.assertEqual(len(packet["templated_ids"]), 1)
            self.assertIn("return allowed", packet["source"][0]["numbered_code"])
            self.assertIn("allowed == true", json.dumps(packet["manifest"]))
            self.assertEqual(packet["annotation_path"], "docs/ai/annotations/sample.json")

    def test_complete_annotation_loads_and_templates_structural_edge(self) -> None:
        sources, graph, cards = fixture()
        bundle = build_bundle(sources, graph, cards)
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            (directory / "sample.json").write_text(json.dumps(response(bundle)), encoding="utf-8")
            result, meta, report = load_annotations(sources, [graph], cards, directory)
            self.assertEqual(set(result), {card["id"] for card in cards})
            self.assertEqual(report["annotation_files"], 1)
            self.assertIn("sample", meta)
            self.assertIn("links", result[cards[2]["id"]]["en"]["plain"])

    def test_line_shift_reuses_annotation_and_refreshes_citation(self) -> None:
        sources, graph, cards = fixture()
        original = build_bundle(sources, graph, cards)
        shifted, graph2, cards2 = fixture(pad=2)
        self.assertEqual(original["hash"], build_bundle(shifted, graph2, cards2)["hash"])
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            (directory / "sample.json").write_text(json.dumps(response(original)), encoding="utf-8")
            result, _, _ = load_annotations(shifted, [graph2], cards2, directory)
            self.assertEqual(result[cards2[0]["id"]]["evidence_ids"],
                             ["mmhand:sample.py:L4-L4"])

    def test_missing_invalid_and_stale_annotations_keep_old_page(self) -> None:
        sources, graph, cards = fixture()
        bundle = build_bundle(sources, graph, cards)
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            published = directory / "behavior.zh.html"
            published.write_bytes(b"previous output")
            path = directory / "sample.json"
            cases = []
            cases.append(("missing", None))
            omitted = response(bundle)
            omitted["items"].pop()
            cases.append(("omitted", omitted))
            bad_evidence = response(bundle)
            bad_evidence["items"][0]["evidence_ids"] = ["not-a-source"]
            cases.append(("evidence", bad_evidence))
            one_language = response(bundle)
            del one_language["items"][0]["en"]
            cases.append(("bilingual", one_language))
            stale = response(bundle)
            stale["bundle_hash"] = "old-source"
            cases.append(("stale", stale))
            for name, value in cases:
                with self.subTest(name=name):
                    if value is None:
                        path.unlink(missing_ok=True)
                    else:
                        path.write_text(json.dumps(value), encoding="utf-8")
                    with self.assertRaises(ExplanationError) as caught:
                        load_annotations(sources, [graph], cards, directory)
                    self.assertTrue(caught.exception.missing)
                    self.assertEqual(caught.exception.report["issues"][0]["graph_id"], "sample")
                    self.assertEqual(published.read_bytes(), b"previous output")

    def test_relevant_source_change_invalidates_annotation(self) -> None:
        sources, graph, cards = fixture()
        original = build_bundle(sources, graph, cards)
        sources.rows[1] = "    allowed = value >= 0"
        for card in cards:
            card["evidence"][0]["ref"]["text"] = sources.rows[1]
        self.assertNotEqual(original["hash"], build_bundle(sources, graph, cards)["hash"])

    def test_item_must_cite_its_own_source(self) -> None:
        _, _, cards = fixture()
        item = response(build_bundle(*fixture()))["items"][0]
        item["evidence_ids"] = ["own", "related"]
        _validate_item(item, cards[0], {"own", "related"}, {"own"}, "allowed == true Node")
        item["evidence_ids"] = ["related"]
        with self.assertRaisesRegex(ValueError, "Invalid evidence IDs"):
            _validate_item(item, cards[0], {"own", "related"}, {"own"}, "allowed == true Node")

    def test_generic_template_explanation_is_rejected(self) -> None:
        sources, graph, cards = fixture()
        bundle = build_bundle(sources, graph, cards)
        item = response(bundle)["items"][0]
        item["zh"]["plain"] = "节点 0 是图中的一个节点，帮助读者定位对应代码职责或状态。"
        with self.assertRaisesRegex(ValueError, "Generic template explanation"):
            _validate_item(item, cards[0], set(bundle["refs"]),
                           set(bundle["manifest"][0]["evidence_ids"]),
                           "allowed == true Node")

    def test_conditional_edge_requires_raw_condition_and_both_outcomes(self) -> None:
        sources, graph, cards = fixture()
        bundle = build_bundle(sources, graph, cards)
        card = cards[-1]
        item = next(x for x in response(bundle)["items"] if x["id"] == card["id"])
        item["en"]["condition"] = "The check determines whether this edge is used."
        with self.assertRaisesRegex(ValueError, "Raw condition omitted"):
            _validate_item(item, card, set(bundle["refs"]),
                           set(bundle["manifest"][-1]["evidence_ids"]),
                           "allowed == true Node Next")
        item = next(x for x in response(bundle)["items"] if x["id"] == card["id"])
        item["en"]["failure"] = "When true, continue to the next step."
        with self.assertRaisesRegex(ValueError, "both true and false outcomes"):
            _validate_item(item, card, set(bundle["refs"]),
                           set(bundle["manifest"][-1]["evidence_ids"]),
                           "allowed == true Node Next")

    def test_estimate_does_not_need_model(self) -> None:
        sources, graph, cards = fixture()
        report = estimate(sources, [graph], cards)
        self.assertEqual(report["structural_edges_from_template"], 1)
        self.assertEqual(report["task_files"], 1)


if __name__ == "__main__":
    unittest.main()
