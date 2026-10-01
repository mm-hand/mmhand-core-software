"""Regression checks for the generated bilingual, source-backed guide."""

from __future__ import annotations

import json
from pathlib import Path
import re
import unittest

from source import Sources, git


ROOT = Path(__file__).resolve().parents[3]
AI = ROOT / "docs/ai"
TELEOP = ROOT.parent / "mmhand_teleop"


def payload(path: Path) -> dict:
    content = path.read_text(encoding="utf-8")
    match = re.search(r'<script type="application/json" id="guide-data">(.*?)</script>', content, re.S)
    if not match:
        raise AssertionError(f"Missing graph payload in {path}")
    return json.loads(match[1].replace("<\\/", "</"))


class GuideTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.snapshot = json.loads((AI / "snapshot.json").read_text(encoding="utf-8"))
        cls.sources = Sources(ROOT, TELEOP, cls.snapshot["mmhand_commit"], cls.snapshot["teleop_commit"])
        cls.registry = json.loads((AI / "source_refs.json").read_text(encoding="utf-8"))["paths"]

    def test_guide_uses_committed_revisions(self) -> None:
        self.assertEqual(self.snapshot["mmhand_commit"], git(ROOT, "rev-parse", "main"))
        self.assertEqual(self.snapshot["teleop_commit"], git(TELEOP, "rev-parse", "main"))
        self.assertFalse((AI / "AI_README.md").exists())
        report = json.loads((AI / "coverage_report.json").read_text(encoding="utf-8"))
        self.assertEqual(report["mmhand_commit"], self.snapshot["mmhand_commit"])
        self.assertEqual(report["coverage"]["unverified_nodes"],
                         len(report["coverage"]["unverified_items"]))

    def test_eight_sections_and_links(self) -> None:
        for language in ("zh", "en"):
            content = (AI / f"AI_README.{language}.md").read_text(encoding="utf-8")
            titles = re.findall(r"^## (?:[一二三四五六七八]、|[1-8]\. )", content, re.M)
            self.assertEqual(len(titles), 8)
            self.assertIn(f"](behavior.{language}.html)", content)
            self.assertIn(f"](architecture.{language}.html)", content)
            self.assertRegex(content, r"\[\d+\]\(#ref-\d+\)")
            self.assertIn("/raw_ik_target", content)
            self.assertIn("--ros2-input raw_ik", content)

    def test_bilingual_graph_identity_and_evidence(self) -> None:
        for kind in ("behavior", "architecture"):
            zh = payload(AI / f"{kind}.zh.html")
            en = payload(AI / f"{kind}.en.html")
            self.assertEqual([g["id"] for g in zh["graphs"]], [g["id"] for g in en["graphs"]])
            for left, right in zip(zh["graphs"], en["graphs"]):
                self.assertEqual([n["id"] for n in left["nodes"]], [n["id"] for n in right["nodes"]])
                self.assertEqual([(e["source"], e["target"], e["condition"]) for e in left["edges"]],
                                 [(e["source"], e["target"], e["condition"]) for e in right["edges"]])
                for item in [*left["nodes"], *left["edges"]]:
                    for ref in item["evidence"]:
                        self.assertEqual(ref["text"], self.sources.evidence(
                            ref["repo"], ref["path"], ref["start"], ref["end"])["text"])
                        self.assertEqual(ref["number"], self.registry[Sources.key(ref["repo"], ref["path"])])
                        self.assertIn(self.sources.commits[ref["repo"]], ref["github"])
                        self.assertIn("#L", ref["github"])
        self.assertIn("collision", [g["id"] for g in payload(AI / "behavior.zh.html")["graphs"]])


if __name__ == "__main__":
    unittest.main()
