"""Source-scope discovery and graph-extension regression tests."""
from __future__ import annotations

import copy
from pathlib import Path
import tempfile
import unittest

from graph_explain import prepare_tasks
from source_scope import (_digest, apply_extensions, discover_candidates,
                          extension_id, resolve_selector)


class FakeSources:
    def __init__(self, pad: int = 0):
        self.pad = pad
        self.files = {
            ("mmhand", "desktop/ui/caller.py"): ["def request():", "    return new_handler(1)"],
            ("mmhand", "desktop/ui/new_panel.py"): ["def new_handler(value):", "    return value + 1"],
            ("mmhand", "ESP32-P4/ServoBoardMain/src/tasks/NewTask.cpp"):
                ["void newTask() {", "  xTaskCreate(newTask, \"new\", 2048, nullptr, 1, nullptr);", "}",
                 "void boot() {", "  newTask();", "}"],
            ("mmhand", "desktop/ros2/new_topic.py"):
                ["def subscribe_once(self):", "    self.create_subscription(Any, '/new_topic', callback, 1)",
                 "    return True"],
        }
        if pad:
            self.files = {key: [""] * pad + value for key, value in self.files.items()}
        self.paths = {"mmhand": {path for repo, path in self.files if repo == "mmhand"}, "teleop": set()}
        self.commits = {"mmhand": "a" * 40, "teleop": "b" * 40}
        self.scope = {"extensions": [], "exclusions": [],
                      "audit": {"branch": "main", "mmhand": self.commits["mmhand"],
                                "teleop": self.commits["teleop"], "mode": "full"}}

    def read(self, repo: str, path: str) -> str:
        return "\n".join(self.files[(repo, path)])

    def lines(self, repo: str, path: str) -> list[str]:
        return self.files[(repo, path)]

    def evidence(self, repo: str, path: str, start: int, end: int | None = None) -> dict:
        end = start if end is None else end
        rows = self.lines(repo, path)
        return {"repo": repo, "path": path, "start": start, "end": end,
                "text": "\n".join(rows[start - 1:end])}


def graphs(sources: FakeSources) -> list[dict]:
    result = []
    for graph_id, path, line in (
        ("ui_flow", "desktop/ui/caller.py", 2),
        ("arch_p4", "ESP32-P4/ServoBoardMain/src/tasks/NewTask.cpp", 5),
        ("arch_ros", "desktop/ros2/new_topic.py", 2),
    ):
        result.append({"id": graph_id, "title": {"zh": graph_id, "en": graph_id}, "parent": "",
                       "nodes": [{"id": "root", "label": {"zh": "入口", "en": "Entry"},
                        "evidence": [sources.evidence("mmhand", path, line + sources.pad)],
                        "details": {}}],
                       "edges": []})
    return result


def extensions() -> list[dict]:
    return [
        {"graph_id": "ui_flow", "repo": "mmhand", "path": "desktop/ui/new_panel.py",
         "selector": {"kind": "function", "symbol": "new_handler"}, "parent_id": "root",
         "relation": "calls", "anchor": {"repo": "mmhand", "path": "desktop/ui/caller.py",
                                        "needle": "return new_handler(1)"}},
        {"graph_id": "arch_p4", "repo": "mmhand", "path": "ESP32-P4/ServoBoardMain/src/tasks/NewTask.cpp",
         "selector": {"kind": "function", "symbol": "newTask"}, "parent_id": "root",
         "relation": "calls", "anchor": {"repo": "mmhand", "path": "ESP32-P4/ServoBoardMain/src/tasks/NewTask.cpp",
                                        "needle": "newTask();"}},
        {"graph_id": "arch_ros", "repo": "mmhand", "path": "desktop/ros2/new_topic.py",
         "selector": {"kind": "function", "symbol": "subscribe_once"}, "parent_id": "root",
         "relation": "subscribes", "anchor": {"repo": "mmhand", "path": "desktop/ros2/new_topic.py",
                                             "needle": "create_subscription("}},
    ]


def inventory(sources: FakeSources) -> list[dict]:
    return [{"path": path, "kind": "source"} for path in sources.paths["mmhand"]]


class SourceScopeTest(unittest.TestCase):
    def test_new_ui_p4_and_ros_paths_extend_graphs_and_scope(self) -> None:
        sources = FakeSources()
        before = discover_candidates(sources, graphs(sources), inventory(sources), {}, "main", None)
        self.assertEqual({item["path"] for item in before["candidates"]},
                         set(sources.paths["mmhand"]))
        self.assertEqual(len(before["unresolved_ids"]), 1)
        self.assertTrue({"python_definition", "freertos_task", "ros_topic"}.issubset(
            {hint["kind"] for item in before["candidates"] for hint in item["hints"]}))
        sources.scope["extensions"] = extensions()
        result = graphs(sources)
        report = apply_extensions(sources, result)
        self.assertEqual(report["extensions"], 3)
        self.assertEqual(len(report["unverified_links"]), 1)
        self.assertEqual([len(graph["nodes"]) for graph in result], [2, 2, 2])
        self.assertTrue(result[2]["edges"][0]["unverified"])
        after = discover_candidates(sources, result, inventory(sources), {}, "main", None)
        self.assertEqual(after["unresolved_ids"], [])
        node = result[0]["nodes"][1]
        ref = node["evidence"][0]
        card = {"id": f"graph:ui_flow:node:{node['id']}", "kind": "node", "graph": "ui_flow",
                "title": node["label"], "condition": "", "details": node["details"],
                "evidence": [{"id": "test-source", "ref": ref}], "variable_definitions": [],
                "changed": False, "diff": [], "source": "", "target": "", "drilldown": ""}
        with tempfile.TemporaryDirectory() as temp:
            prepare_tasks(sources, [result[0]], [card], Path(temp),
                          {"branch": "main", "mmhand": sources.commits["mmhand"],
                           "teleop": sources.commits["teleop"]})
            import json
            packet = json.loads((Path(temp) / "ui_flow.json").read_text(encoding="utf-8"))
            self.assertIn(card["id"], packet["required_ids"])
            self.assertIn("return value + 1", "\n".join(x["numbered_code"] for x in packet["source"]))

    def test_stable_id_and_citation_after_blank_line_shift(self) -> None:
        entry = extensions()[0]
        first = resolve_selector(FakeSources(), entry["repo"], entry["path"], entry["selector"])
        shifted = resolve_selector(FakeSources(3), entry["repo"], entry["path"], entry["selector"])
        self.assertEqual(extension_id(entry), extension_id(copy.deepcopy(entry)))
        self.assertEqual(first["text"], shifted["text"])
        self.assertEqual(shifted["start"], first["start"] + 3)

    def test_changed_symbol_lists_its_caller_on_incremental_branch(self) -> None:
        sources = FakeSources()
        sources.scope["audit"] = {"branch": "feature", "mmhand": sources.commits["mmhand"],
                                  "teleop": sources.commits["teleop"], "mode": "incremental"}
        report = discover_candidates(sources, graphs(sources), inventory(sources),
                                     {"desktop/ui/new_panel.py": [(1, 1)]},
                                     "feature", "base-commit")
        paths = {item["path"]: item for item in report["candidates"]}
        self.assertEqual(set(paths), {"desktop/ui/new_panel.py", "desktop/ui/caller.py"})
        self.assertIn("new_handler", paths["desktop/ui/caller.py"]["reason"])

    def test_forced_full_review_covers_unchanged_files(self) -> None:
        sources = FakeSources()
        report = discover_candidates(sources, graphs(sources), inventory(sources), {},
                                     "feature", "base-commit", force_full=True)
        self.assertEqual(report["mode"], "full")
        self.assertEqual({item["path"] for item in report["candidates"]},
                         sources.paths["mmhand"])
        self.assertIn("scope:audit", report["unresolved_ids"])

    def test_a_symbol_mention_does_not_prove_a_call_edge(self) -> None:
        sources = FakeSources()
        sources.files[("mmhand", "desktop/ui/caller.py")][1] = "    # new_handler(1)"
        sources.scope["extensions"] = [extensions()[0]]
        sources.scope["extensions"][0]["anchor"]["needle"] = "# new_handler(1)"
        report = apply_extensions(sources, graphs(sources))
        self.assertEqual(len(report["unverified_links"]), 1)

    def test_missing_symbol_and_moved_path_are_rejected(self) -> None:
        sources = FakeSources()
        with self.assertRaisesRegex(ValueError, "uniquely"):
            resolve_selector(sources, "mmhand", "desktop/ui/new_panel.py",
                             {"kind": "function", "symbol": "renamed_handler"})
        sources.scope["extensions"] = extensions()
        sources.scope["extensions"][0]["path"] = "desktop/ui/renamed.py"
        with self.assertRaisesRegex(ValueError, "not in committed source"):
            apply_extensions(sources, graphs(sources))

    def test_exclusion_expires_on_content_change_not_line_shift(self) -> None:
        sources = FakeSources()
        path = "desktop/ui/new_panel.py"
        hashed = _digest("\n".join(sources.lines("mmhand", path)))
        sources.scope["exclusions"] = [{"repo": "mmhand", "path": path,
            "source_hash": hashed, "reason": "This helper is unrelated to the current control graphs."}]
        initial = discover_candidates(sources, graphs(sources), inventory(sources), {}, "main", None)
        self.assertEqual(next(item["status"] for item in initial["candidates"] if item["path"] == path), "excluded")
        shifted = FakeSources(2)
        shifted.scope["exclusions"] = sources.scope["exclusions"]
        shifted_report = discover_candidates(shifted, graphs(shifted), inventory(shifted), {}, "main", None)
        self.assertEqual(next(item["status"] for item in shifted_report["candidates"] if item["path"] == path), "excluded")
        sources.files[("mmhand", path)][1] = "    return value + 2"
        stale = discover_candidates(sources, graphs(sources), inventory(sources), {}, "main", None)
        self.assertIn(next(item["id"] for item in stale["candidates"] if item["path"] == path),
                      stale["unresolved_ids"])

    def test_audit_and_decisions_are_both_required(self) -> None:
        sources = FakeSources()
        selected_inventory = [item for item in inventory(sources)
                              if item["path"] in {"desktop/ui/caller.py", "desktop/ui/new_panel.py"}]
        path = "desktop/ui/new_panel.py"
        sources.scope["exclusions"] = [{"repo": "mmhand", "path": path,
            "source_hash": _digest("\n".join(sources.lines("mmhand", path))),
            "reason": "This panel helper has no link to the documented control graph."}]
        report = discover_candidates(sources, graphs(sources)[:1], selected_inventory, {}, "main", None)
        self.assertEqual(report["unresolved_ids"], [])
        sources.scope["audit"]["mmhand"] = "old-commit"
        stale = discover_candidates(sources, graphs(sources)[:1], selected_inventory, {}, "main", None)
        self.assertEqual(stale["unresolved_ids"], ["scope:audit"])


if __name__ == "__main__":
    unittest.main()
