"""Committed-source inventory, provenance, branch worktrees, and change ranges."""

from __future__ import annotations

import json
import os
from pathlib import Path
import re
import subprocess
from urllib.parse import quote


CODE_SUFFIXES = {
    ".py", ".c", ".cc", ".cpp", ".h", ".hpp", ".ino", ".ps1",
    ".sh", ".cmd", ".bat", ".json", ".yaml", ".yml", ".def", ".urdf",
    ".toml", ".ini", ".cfg", ".xml", ".proto", ".msg", ".js", ".ts", ".cmake",
}


def git(repo: Path, *args: str, check: bool = True) -> str:
    result = subprocess.run(
        ["git", "-C", str(repo), *args], capture_output=True, check=False
    )
    if check and result.returncode:
        raise RuntimeError(
            f"git {' '.join(args)} failed: "
            + result.stderr.decode("utf-8", "replace")[-1500:]
        )
    return result.stdout.decode("utf-8", "replace").strip()


def github_base(repo: Path) -> str:
    remote = git(repo, "remote", "get-url", "origin").removesuffix(".git")
    if remote.startswith("git@github.com:"):
        remote = "https://github.com/" + remote.split(":", 1)[1]
    if not remote.startswith("https://github.com/"):
        raise RuntimeError(f"Cannot build GitHub source links from {remote}")
    return remote


def find_branch_worktree(repo: Path, branch: str, *, create: bool = True) -> Path:
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._/-]*", branch) or ".." in branch:
        raise ValueError("Invalid branch name")
    git(repo, "rev-parse", "--verify", f"refs/heads/{branch}")
    rows = git(repo, "worktree", "list", "--porcelain").splitlines()
    path: Path | None = None
    for row in rows + [""]:
        if row.startswith("worktree "):
            path = Path(row[9:]).resolve()
        elif row == f"branch refs/heads/{branch}" and path is not None:
            return path
    if not create:
        raise RuntimeError(f"No existing worktree for branch: {branch}")
    target = repo / ".ai-worktrees" / branch.replace("/", "__")
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists():
        raise RuntimeError(f"Worktree path already exists: {target}")
    git(repo, "worktree", "add", str(target), branch)
    return target


def committed_files(repo: Path, commit: str) -> list[str]:
    result = subprocess.run(
        ["git", "-C", str(repo), "ls-tree", "-r", "-z", "--name-only", commit],
        capture_output=True, check=True,
    )
    return sorted(x.decode("utf-8") for x in result.stdout.split(b"\0") if x)


class Sources:
    def __init__(self, root: Path, teleop: Path, commit: str, teleop_commit: str):
        from source_scope import load_scope
        self.roots = {"mmhand": root, "teleop": teleop}
        self.scope = load_scope(root / "docs/ai/source_scope.json")
        self.commits = {"mmhand": commit, "teleop": teleop_commit}
        self.remotes = {name: github_base(path) for name, path in self.roots.items()}
        self.paths = {
            name: set(committed_files(path, self.commits[name]))
            for name, path in self.roots.items()
        }
        self.cache: dict[str, str] = {}

    @staticmethod
    def key(repo: str, path: str) -> str:
        return f"{repo}:{path.replace(chr(92), '/')}"

    def read(self, repo: str, path: str) -> str:
        key = self.key(repo, path)
        if path not in self.paths[repo]:
            raise ValueError(f"Source is not committed: {key}")
        if key not in self.cache:
            result = subprocess.run(
                ["git", "-C", str(self.roots[repo]), "show", f"{self.commits[repo]}:{path}"],
                capture_output=True, check=True,
            )
            self.cache[key] = result.stdout.decode("utf-8", "replace")
        return self.cache[key]

    def lines(self, repo: str, path: str) -> list[str]:
        return self.read(repo, path).splitlines()

    def evidence(self, repo: str, path: str, start: int, end: int | None = None) -> dict:
        rows = self.lines(repo, path)
        end = start if end is None else end
        if start < 1 or end < start or end > len(rows):
            raise ValueError(f"Invalid source range: {repo}:{path}:{start}-{end}")
        return {"repo": repo, "path": path, "start": start, "end": end,
                "text": "\n".join(rows[start - 1:end])}

    def github_url(self, evidence: dict) -> str:
        repo, path = evidence["repo"], evidence["path"]
        url = f"{self.remotes[repo]}/blob/{self.commits[repo]}/{quote(path, safe='/._-')}"
        return url + f"#L{evidence['start']}-L{evidence['end']}"

    def local_url(self, output_dir: Path, evidence: dict) -> str:
        path = self.roots[evidence["repo"]] / evidence["path"]
        relative = os.path.relpath(path, output_dir).replace("\\", "/")
        return quote(relative, safe="/._-")


def source_inventory(sources: Sources) -> list[dict]:
    items = []
    for path in sorted(sources.paths["mmhand"]):
        if path.startswith("docs/ai/") or (Path(path).suffix.lower() not in CODE_SUFFIXES
                                               and Path(path).name != "CMakeLists.txt"
                                               and path != "desktop/requirements.txt"):
            continue
        raw = sources.read("mmhand", path)
        if "\0" in raw:
            continue
        kind = ("tooling" if any(part in {".idea", ".vscode"} for part in Path(path).parts)
                else "generated" if "/generated/" in f"/{path}/" or
                Path(path).name.startswith("generated_") or
                path in {"desktop/resources/config/urdf_joint_mapping.json",
                         "desktop/resources/config/product_fingerprint.json",
                         "desktop/resources/config/p4_safety_rules.json"}
                else "third_party" if any(x in path.lower() for x in
                                          ("third_party/", "/libraries/", "vendor/"))
                else "source")
        items.append({"path": path, "lines": len(raw.splitlines()), "kind": kind})
    return items


def change_ranges(repo: Path, base: str | None, head: str) -> dict[str, list[tuple[int, int]]]:
    if not base:
        return {}
    diff = git(repo, "diff", "--unified=0", "--no-ext-diff", base, head, "--", ".")
    changes: dict[str, list[tuple[int, int]]] = {}
    path = ""
    for line in diff.splitlines():
        if line.startswith("+++ b/"):
            path = line[6:]
            changes.setdefault(path, [])
        elif line.startswith("@@") and path:
            match = re.search(r"\+(\d+)(?:,(\d+))?", line)
            if match:
                start = int(match[1]); count = int(match[2] or 1)
                changes[path].append((start, max(start, start + count - 1)))
    return changes


def range_changed(evidence: dict, changes: dict[str, list[tuple[int, int]]]) -> bool:
    if evidence["repo"] != "mmhand":
        return False
    return any(a <= evidence["end"] and b >= evidence["start"]
               for a, b in changes.get(evidence["path"], []))


def load_registry(path: Path) -> dict[str, int]:
    if not path.is_file():
        return {}
    data = json.loads(path.read_text(encoding="utf-8"))
    old = data.get("paths", {})
    return {(key if ":" in key else "mmhand:" + key): int(value)
            for key, value in old.items()}


def extend_registry(registry: dict[str, int], keys: set[str]) -> dict[str, int]:
    next_number = max(registry.values(), default=0) + 1
    for key in sorted(keys):
        if key not in registry:
            registry[key] = next_number
            next_number += 1
    if len(set(registry.values())) != len(registry):
        raise ValueError("Duplicate source reference number")
    return registry
