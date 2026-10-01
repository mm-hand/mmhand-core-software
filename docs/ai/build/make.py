"""Prepare graph tasks, validate external annotations, and publish bilingual guides."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from datetime import datetime, timezone

from extract import architecture_graphs, behavior_graphs
from explain import ExplanationError, attach, cards, COMMANDS
from graph_explain import estimate, load_annotations, prepare_tasks
from render import all_evidence, html_page, markdown
from source import Sources, change_ranges, extend_registry, find_branch_worktree, git, load_registry, source_inventory
from source_scope import apply_extensions, configured_path, discover_candidates, validate_configured_scope

HERE = Path(__file__).resolve().parent
AI = HERE.parent
ROOT = AI.parent.parent
TELEOP = ROOT.parent / "mmhand_teleop"


def narrative(branch: str) -> dict:
    """Assemble only after all source-backed explanations have passed validation."""
    return {"branch": branch, "overview": {"zh": "", "en": ""},
            "protocol_en": {}, "untranslated_protocol": [], "ai_status": "pending"}


def baseline_for(worktree: Path, branch: str, explicit: str | None) -> str | None:
    if explicit:
        return git(worktree, "rev-parse", "--verify", explicit)
    if branch != "main":
        return git(worktree, "merge-base", "main", branch)
    snapshot = worktree / "docs/ai/snapshot.json"
    if snapshot.is_file():
        try:
            old = json.loads(snapshot.read_text(encoding="utf-8"))["mmhand_commit"]
            git(worktree, "cat-file", "-e", old + "^{commit}")
            return old
        except (OSError, ValueError, KeyError, RuntimeError):
            pass
    return None


def validate(graphs: list[dict], sources: Sources, registry: dict[str, int], coverage: dict) -> None:
    graph_ids = [g["id"] for g in graphs]
    if len(graph_ids) != len(set(graph_ids)):
        raise ValueError("Duplicate graph ID")
    for graph in graphs:
        ids = [node["id"] for node in graph["nodes"]]
        if len(ids) != len(set(ids)):
            raise ValueError("Duplicate node ID: " + graph["id"])
        if graph["parent"] and graph["parent"] not in graph_ids:
            raise ValueError("Missing parent graph: " + graph["id"])
        for node in graph["nodes"]:
            if node["drilldown"] and node["drilldown"] not in graph_ids:
                raise ValueError("Missing drilldown graph: " + node["drilldown"])
        for edge in graph["edges"]:
            if edge["source"] not in ids or edge["target"] not in ids:
                raise ValueError("Broken graph edge: " + graph["id"])
    for ref in all_evidence(graphs):
        key = Sources.key(ref["repo"], ref["path"])
        if key not in registry or sources.evidence(ref["repo"], ref["path"],
                                                   ref["start"], ref["end"])["text"] != ref["text"]:
            raise ValueError("Unverifiable source evidence: " + key)
    if coverage["p4_states"] != 9 or coverage["safety_rules"] < 100:
        raise ValueError("P4 state or safety catalog coverage incomplete")
    if coverage["commands"] < 20 or coverage["packets"] < 15:
        raise ValueError("Protocol coverage incomplete")


def generator_checks(worktree: Path, sources: Sources) -> dict[str, dict]:
    checks = {}
    for name, key in (("upper_protocol", "tools.protocol_generator"),
                      ("robot_model", "tools.robot_generator"),
                      ("p4_safety_catalog", "tools.safety_generator")):
        result = subprocess.run([sys.executable, str(worktree / configured_path(sources, key)), "--check"],
                                cwd=worktree, capture_output=True, text=True,
                                encoding="utf-8", errors="replace", timeout=90)
        checks[name] = {"ok": result.returncode == 0,
                        "output": (result.stdout + result.stderr).strip()[:1200]}
    return checks


def publish(directory: Path, files: dict[str, str]) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    staged: dict[Path, Path] = {}
    backups: dict[Path, bytes | None] = {}
    replaced = []
    try:
        for name, content in files.items():
            target = directory / name
            with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=directory,
                                             prefix=".guide_stage_", delete=False) as handle:
                handle.write(content)
                staged[target] = Path(handle.name)
        for target, temporary in staged.items():
            backups[target] = target.read_bytes() if target.is_file() else None
            os.replace(temporary, target)
            replaced.append(target)
    except OSError:
        for target in reversed(replaced):
            old = backups[target]
            if old is None:
                target.unlink(missing_ok=True)
            else:
                target.write_bytes(old)
        raise
    finally:
        for temporary in staged.values():
            temporary.unlink(missing_ok=True)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--branch", required=True, help="Existing local branch")
    parser.add_argument("--base", help="Optional baseline commit or ref")
    parser.add_argument("--full-scope-audit", action="store_true",
                        help="Force a full committed-source path review on a non-main branch")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--prepare", action="store_true", help="Write graph source packets (the default)")
    mode.add_argument("--validate", action="store_true", help="Check annotation JSON without publishing")
    mode.add_argument("--publish", action="store_true", help="Validate annotations and publish six guides")
    mode.add_argument("--estimate", action="store_true", help="Read-only graph source size estimate")
    args = parser.parse_args(argv)
    try:
        if not TELEOP.is_dir():
            raise RuntimeError(f"Missing sibling mmhand_teleop repository: {TELEOP}")
        worktree = find_branch_worktree(ROOT, args.branch, create=not args.estimate)
        commit = git(worktree, "rev-parse", "HEAD")
        dirty = [p for p in git(worktree, "diff", "--name-only", commit).splitlines()
                 if p and not p.startswith("docs/ai/") and p != ".gitignore"]
        if dirty:
            raise RuntimeError("Commit source changes before generating: " + ", ".join(dirty[:12]))
        teleop_commit = git(TELEOP, "rev-parse", "main")
        sources = Sources(worktree, TELEOP, commit, teleop_commit)
        validate_configured_scope(sources)
        output = worktree / "docs/ai"
        baseline = baseline_for(worktree, args.branch, args.base)
        changes = change_ranges(worktree, baseline, commit)
        inventory = source_inventory(sources)
        behavior, behavior_counts = behavior_graphs(sources)
        architecture, architecture_counts = architecture_graphs(sources)
        graphs = behavior + architecture
        extension_report = apply_extensions(sources, graphs)
        registry = load_registry(output / "source_refs.json")
        keys = {Sources.key("mmhand", item["path"]) for item in inventory}
        keys.update(Sources.key(ref["repo"], ref["path"]) for ref in all_evidence(graphs))
        keys.update(Sources.key("teleop", configured_path(sources, name, "teleop"))
                    for name in ("teleop.config", "teleop.publisher", "teleop.track"))
        registry = extend_registry(registry, keys)
        refs_by_path: dict[str, list[dict]] = {}
        for ref in all_evidence(graphs):
            if ref["repo"] == "mmhand":
                refs_by_path.setdefault(ref["path"], []).append(ref)
        unmapped = []
        for path, spans in changes.items():
            for start, end in spans:
                if not any(ref["start"] <= end and ref["end"] >= start
                           for ref in refs_by_path.get(path, [])):
                    unmapped.append(f"{path}:L{start}-L{end}")
        coverage = {**behavior_counts, **architecture_counts,
                    "unverified_nodes": sum(1 for g in graphs for n in g["nodes"] if n.get("unverified")),
                    "unverified_items": [f"{g['id']}:{n['id']}" for g in graphs
                                         for n in g["nodes"] if n.get("unverified")],
                    "unmapped_changes": sorted(unmapped)}
        prior_teleop = sources.scope["audit"].get("teleop")
        teleop_changes = None
        if prior_teleop:
            try:
                git(TELEOP, "cat-file", "-e", prior_teleop + "^{commit}")
                teleop_changes = change_ranges(TELEOP, prior_teleop, teleop_commit)
            except RuntimeError:
                pass
        scope_report = discover_candidates(sources, graphs, inventory, changes, args.branch,
                                           baseline, args.full_scope_audit, teleop_changes)
        coverage["source_scope"] = {
            "mode": scope_report["mode"], "mapped_paths": scope_report["mapped_paths"],
            "extensions": extension_report["extensions"],
            "unverified_links": extension_report["unverified_links"],
            "candidates": len(scope_report["candidates"]),
            "excluded": sum(item["status"] == "excluded" for item in scope_report["candidates"]),
            "unresolved_ids": scope_report["unresolved_ids"],
            "decisions": [{"id": item["id"], "repo": item["repo"], "path": item["path"],
                           "status": item["status"], "reason": item["exclusion_reason"]}
                          for item in scope_report["candidates"]],
        }
        checks = generator_checks(worktree, sources)
        coverage["generator_checks"] = checks
        if not checks["p4_safety_catalog"]["ok"] and (args.validate or args.publish):
            failure = {"schema_version": 1, "branch": args.branch, "mmhand_commit": commit,
                       "teleop_commit": teleop_commit, "baseline": baseline,
                       "reason": "Safety catalog export is stale: " + checks["p4_safety_catalog"]["output"],
                       "missing_ids": [], "coverage": coverage}
            publish(output, {"coverage_report.failed.json": json.dumps(failure, ensure_ascii=False, indent=2) + "\n"})
            print("AI guide not published; safety export is stale. See coverage_report.failed.json")
            return 1
        validate(graphs, sources, registry, coverage)
        fact_cards = cards(sources, graphs, changes, baseline)
        if args.estimate:
            result = estimate(sources, graphs, fact_cards)
            result.update({"branch": args.branch, "mmhand_commit": commit,
                           "teleop_commit": teleop_commit, "baseline": baseline,
                           "source_scope": {key: value for key, value in coverage["source_scope"].items()
                                            if key not in {"decisions", "unresolved_ids"}},
                           "unresolved_scope_count": len(scope_report["unresolved_ids"])})
            print(json.dumps(result, ensure_ascii=False, indent=2))
            return 0
        if not (args.validate or args.publish):
            task_dir = output / ".explain_cache" / "tasks" / f"{commit[:12]}_{teleop_commit[:12]}"
            index = prepare_tasks(sources, graphs, fact_cards, task_dir,
                                  {"branch": args.branch, "mmhand": commit,
                                   "teleop": teleop_commit, "baseline": baseline}, scope_report)
            print(f"Prepared {len(index['tasks'])} source packets at {task_dir}")
            print("Read docs/ai/GENERATE.md; write annotations JSON, then run --validate and --publish.")
            return 0
        if scope_report["unresolved_ids"]:
            failure = {"schema_version": 1, "branch": args.branch, "mmhand_commit": commit,
                       "teleop_commit": teleop_commit, "baseline": baseline,
                       "reason": "Source scope review is incomplete or stale",
                       "missing_ids": scope_report["unresolved_ids"], "coverage": coverage}
            publish(output, {"coverage_report.failed.json": json.dumps(failure, ensure_ascii=False, indent=2) + "\n"})
            print(f"Guide not published; {len(scope_report['unresolved_ids'])} source-scope items need review. "
                  "Run --prepare and read scope_candidates.json.")
            return 1
        required_ids = [card["id"] for card in fact_cards]
        try:
            explanations, graph_meta, annotation_report = load_annotations(
                sources, graphs, fact_cards, output / "annotations")
        except (ExplanationError, OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as raw_error:
            exc = (raw_error if isinstance(raw_error, ExplanationError) else
                   ExplanationError(str(raw_error), required_ids))
            failure = {"schema_version": 1, "branch": args.branch, "mmhand_commit": commit,
                       "teleop_commit": teleop_commit, "baseline": baseline,
                       "reason": str(exc), "required": len(required_ids),
                       "completed": len(required_ids) - len(exc.missing),
                       "missing_ids": exc.missing, "coverage": coverage,
                       "annotation_validation": exc.report}
            publish(output, {"coverage_report.failed.json": json.dumps(failure, ensure_ascii=False, indent=2) + "\n"})
            print(f"Guide not published; {len(exc.missing)} explanations missing or invalid. "
                  f"See {output / 'coverage_report.failed.json'}")
            return 1
        if args.validate:
            print(json.dumps({"branch": args.branch, "mmhand_commit": commit,
                              "teleop_commit": teleop_commit, **annotation_report},
                             ensure_ascii=False, indent=2))
            return 0
        attach(graphs, explanations, fact_cards)
        for graph in graphs:
            graph["summary"] = graph_meta[graph["id"]]
        coverage["ai_explained_nodes"] = sum(len(g["nodes"]) for g in graphs)
        coverage["ai_explained_edges"] = sum(len(g["edges"]) for g in graphs)
        coverage["ai_explained_commands"] = len(COMMANDS)
        info = narrative(args.branch)
        info["ai_status"] = (f"complete: {len(explanations)} externally authored bilingual "
                             "explanations with validated source IDs")
        info["overview"] = {
            lang: (explanations["graph:architecture_overview:node:desktop"][lang]["plain"] + " " +
                   explanations["graph:behavior_overview:node:operator"][lang]["plain"])
            for lang in ("zh", "en")}
        info["overview_refs"] = [evidence["ref"] for card in fact_cards
                                 if card["id"] in {"graph:architecture_overview:node:desktop",
                                                   "graph:behavior_overview:node:operator"}
                                 for evidence in card["evidence"]]
        info["command_explanations"] = {name: explanations["command:" + name]
                                        for name, *_ in COMMANDS}
        info["command_evidence"] = {name: [evidence["ref"] for evidence in card["evidence"]]
                                    for card in fact_cards if card["kind"] == "command"
                                    for name in [card["id"].split(":", 1)[1]]}
        coverage["untranslated_protocol"] = info["untranslated_protocol"]
        protocol_schema = json.loads(sources.read("mmhand", configured_path(sources, "protocol.schema")))
        for section in ("commands", "packets"):
            graph = next(g for g in graphs if g["id"] == "arch_serial_" + section)
            for index, item in enumerate(protocol_schema[section]):
                info["protocol_en"][item["name"]] = explanations[
                    f"graph:arch_serial_{section}:node:item_{index}"]["en"]["plain"]
                graph["nodes"][index + 1]["details"]["description"] = {
                    "zh": item["description"], "en": info["protocol_en"][item["name"]]}
        coverage["untranslated_protocol"] = []
        info["output_dir"] = output
        files = {}
        for lang in ("zh", "en"):
            files[f"AI_README.{lang}.md"] = markdown(sources, lang, graphs, inventory,
                                                       registry, changes, baseline, coverage, info)
            for kind in ("behavior", "architecture"):
                files[f"{kind}.{lang}.html"] = html_page(sources, lang, kind, graphs,
                                                          registry, changes, coverage, output)
        files["source_refs.json"] = json.dumps({"version": 2, "paths": dict(sorted(registry.items(), key=lambda x: x[1]))},
                                                 ensure_ascii=False, indent=2) + "\n"
        files["annotation_report.json"] = json.dumps({"schema_version": 1, "branch": args.branch,
            "mmhand_commit": commit, "teleop_commit": teleop_commit, "baseline": baseline,
            **annotation_report}, ensure_ascii=False, indent=2) + "\n"
        files["coverage_report.json"] = json.dumps({
            "schema_version": 1, "mmhand_commit": commit, "teleop_commit": teleop_commit,
            "baseline": baseline, "coverage": coverage,
            "known_gaps": [
                {"zh": f"{coverage['unverified_nodes']} 条安全目录规则的触发谓词尚未自动证明；图中列出规则及调用点。",
                 "en": f"Trigger predicates for {coverage['unverified_nodes']} safety catalog rules are not automatically proven; graph nodes cite rules and call sites."},
                {"zh": "P4 状态图仅展示可静态定位的事件分支和显式 setSystemState 调用，间接转换需核对源码。",
                 "en": "The P4 state graph shows statically found event branches and explicit setSystemState calls; indirect transitions need source review."},
                {"zh": "UI 控件接线已枚举；除显式命令链外，其余控件到固件反馈的完整路径尚未自动证明。",
                 "en": "UI bindings are enumerated; full firmware and feedback paths beyond explicit command chains are not automatically proven."},
                {"zh": "配置索引列出文件和字段，未穷举每个字段的所有读取方。",
                 "en": "The configuration index lists files and keys without proving every reader of each field."},
            ],
            "note": {"zh": "未核实节点表示规则目录和调用点已定位，但触发条件尚不能自动证明。",
                     "en": "Unverified nodes have catalog and call-site evidence, but their trigger predicates are not automatically proven."}},
            ensure_ascii=False, indent=2) + "\n"
        files["snapshot.json"] = json.dumps({"schema_version": 1, "branch": args.branch,
                                             "mmhand_commit": commit, "teleop_commit": teleop_commit,
                                             "baseline": baseline, "generated_at_utc": datetime.now(timezone.utc).isoformat(),
                                             "ai_status": info["ai_status"], "coverage": coverage},
                                            ensure_ascii=False, indent=2) + "\n"
        publish(output, files)
        (output / "coverage_report.failed.json").unlink(missing_ok=True)
        assets = output / "assets"
        assets.mkdir(exist_ok=True)
        for name in ("guide.css", "guide.js", "cytoscape.min.js", "CYTOSCAPE_LICENSE.txt"):
            source = AI / "assets" / name
            target = assets / name
            if source.resolve() != target.resolve():
                shutil.copy2(source, target)
        for old in ("AI_README.md", "view.html"):
            (output / old).unlink(missing_ok=True)
        (output / "ai_usage_report.json").unlink(missing_ok=True)
        print(f"Generated 6 bilingual files for {args.branch} at {output}")
        print(f"mmhand {commit}; teleop main {teleop_commit}; baseline {baseline or 'initial'}")
        summary = {key: value for key, value in coverage.items() if isinstance(value, int)}
        print(f"Graphs {len(graphs)}; coverage {json.dumps(summary, ensure_ascii=False)}; "
              f"unmapped {len(unmapped)}; generator checks "
              f"{sum(x['ok'] for x in checks.values())}/{len(checks)}")
        print(f"AI: {info['ai_status']}")
        return 0
    except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as exc:
        if args.validate or args.publish:
            target = locals().get("output", ROOT / "docs/ai")
            failure = {"schema_version": 1, "branch": args.branch,
                       "mmhand_commit": locals().get("commit"),
                       "teleop_commit": locals().get("teleop_commit"),
                       "reason": str(exc), "missing_ids": [],
                       "coverage": locals().get("coverage", {})}
            try:
                publish(target, {"coverage_report.failed.json":
                                 json.dumps(failure, ensure_ascii=False, indent=2) + "\n"})
            except OSError:
                pass
        print("Guide generation failed; previous output retained: " + str(exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
