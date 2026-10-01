"""Full rendering smoke test with deterministic external annotations; publishes nothing."""

from __future__ import annotations

import json
from pathlib import Path
import unittest

from explain import COMMANDS, FIELDS, attach, cards
from extract import architecture_graphs, behavior_graphs
from render import all_evidence, html_page, markdown
from source import Sources, extend_registry, git, load_registry, source_inventory


ROOT = Path(__file__).resolve().parents[3]
AI = ROOT / "docs/ai"
TELEOP = ROOT.parent / "mmhand_teleop"


class PipelineTest(unittest.TestCase):
    def test_bilingual_render_with_complete_explanations(self) -> None:
        sources = Sources(ROOT, TELEOP, git(ROOT, "rev-parse", "HEAD"), git(TELEOP, "rev-parse", "main"))
        behavior, counts_b = behavior_graphs(sources)
        architecture, counts_a = architecture_graphs(sources)
        graphs = behavior + architecture
        inventory = source_inventory(sources)
        registry = load_registry(AI / "source_refs.json")
        registry = extend_registry(registry, {Sources.key("mmhand", item["path"]) for item in inventory}
                                   | {Sources.key(ref["repo"], ref["path"]) for ref in all_evidence(graphs)}
                                   | {Sources.key("teleop", path) for path in ("config.py", "track.py", "ros.py")})
        facts = cards(sources, graphs, {}, None)
        explanations = {}
        for fact in facts:
            texts = {}
            for lang in ("zh", "en"):
                plain = ("这段源码说明此节点或连线的用途、输入及后续执行路径。" if lang == "zh" else
                         "This source explains the role, input and subsequent path of this node or edge.")
                texts[lang] = {field: (plain if field == "plain" else
                                       "Condition and outcome follow the cited source." if field in ("condition", "failure") else
                                       "The exact trigger is unverified." if field == "unknown" else "")
                               for field in FIELDS}
            explanations[fact["id"]] = {"id": fact["id"], "zh": texts["zh"], "en": texts["en"],
                                        "evidence_ids": [fact["evidence"][0]["id"]]}
        attach(graphs, explanations, facts)
        overview_ref = facts[0]["evidence"][0]["ref"]
        for graph in graphs:
            graph["summary"] = {"overview": {"zh": "本图展示节点和连线的源码关系。",
                                             "en": "This graph shows source relationships among its nodes and edges.",
                                             "evidence_ids": [], "evidence": [overview_ref]},
                                "glossary": []}
        protocol = json.loads(sources.read("mmhand", "desktop/resources/config/upper_protocol_schema.json"))
        translated = {item["name"]: "Source-backed English protocol explanation"
                      for item in protocol["commands"] + protocol["packets"]}
        info = {"branch": "main", "overview": {"zh": "源码概览。", "en": "Source overview."},
                "protocol_en": translated, "output_dir": AI,
                "overview_refs": [facts[0]["evidence"][0]["ref"]],
                "command_explanations": {name: explanations["command:" + name] for name, *_ in COMMANDS},
                "command_evidence": {card["id"].split(":", 1)[1]: [x["ref"] for x in card["evidence"]]
                                     for card in facts if card["kind"] == "command"}}
        coverage = {**counts_b, **counts_a, "unverified_nodes": 103,
                    "unmapped_changes": [], "generator_checks": {}}
        for lang in ("zh", "en"):
            readme = markdown(sources, lang, graphs, inventory, registry, {}, None, coverage, info)
            self.assertEqual(len([line for line in readme.splitlines() if line.startswith("## ")]), 8)
            self.assertIn("Safety policy locations" if lang == "en" else "安全策略位置", readme)
            self.assertIn("Command-by-command usage" if lang == "en" else "逐条命令使用说明", readme)
            self.assertIn("check_safety_rule_closure.py --json", readme)
            self.assertIn("--mode replay", readme)
            for kind in ("behavior", "architecture"):
                page = html_page(sources, lang, kind, graphs, registry, {}, coverage, AI)
                self.assertIn('"explanation"', page)
                self.assertIn('"summary"', page)
                self.assertIn("guide.js", page)


if __name__ == "__main__":
    unittest.main()
