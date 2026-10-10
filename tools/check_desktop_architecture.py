"""检查 Desktop 上位机文件头与静态依赖边界，不导入业务模块。"""

from __future__ import annotations

import argparse
import ast
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DESKTOP_ROOT = PROJECT_ROOT / "desktop"

# Check every module in the supported Web GUI runtime.
TARGET_TOP_LEVEL_FILES = {
    "__init__.py",
    "__main__.py",
    "main.py",
    "paths.py",
    "serial_log_filter.py",
}
TARGET_PACKAGES = (
    "app",
    "core",
    "replay",
    "ros2",
    "ui",
)
REQUIRED_HEADER_FIELDS = (
    "职责：",
    "线程：",
    "输入：",
    "输出：",
    "状态所有权：",
    "禁止依赖：",
)


@dataclass(frozen=True)
class DependencyRule:
    """描述一个源目录不得依赖的 Python 模块前缀集合。"""

    source_prefix: str
    forbidden_imports: tuple[str, ...]


DEPENDENCY_RULES = (
    DependencyRule(
        "desktop/core/communication/",
        ("desktop.core.comm_layer",),
    ),
    DependencyRule(
        "desktop/core/",
        ("desktop.ui", "desktop.app", "desktop.ros2", "desktop.replay"),
    ),
    DependencyRule(
        "desktop/replay/",
        ("desktop.ui", "desktop.app", "desktop.ros2", "desktop.recording"),
    ),
    DependencyRule(
        "desktop/recording/",
        ("desktop.ui", "desktop.app"),
    ),
    DependencyRule(
        "desktop/ros2/",
        ("desktop.ui", "desktop.app"),
    ),
    DependencyRule(
        "desktop/teleop/",
        ("desktop.ui", "desktop.app", "desktop.replay", "desktop.recording"),
    ),
    DependencyRule(
        "desktop/ui/",
        (
            "desktop.core.calibration_snapshot",
            "desktop.core.comm_layer",
            "desktop.core.core_logic",
        ),
    ),
)

# example_teleop 是人工示例入口，需要组装 UI；正式 teleop 模块仍受反向依赖门禁约束。
DEPENDENCY_EXCEPTIONS = {
    ("desktop/teleop/example_teleop.py", "desktop.ui"),
}
QT_IMPORT_PREFIXES = ("PySide", "PyQt", "pyqtgraph")


def iter_target_files(root: Path = PROJECT_ROOT) -> list[Path]:
    """返回稳定排序的正式 Desktop Python 文件清单。"""
    desktop_root = root / "desktop"
    files = [desktop_root / name for name in TARGET_TOP_LEVEL_FILES]
    for package in TARGET_PACKAGES:
        files.extend((desktop_root / package).rglob("*.py"))
    return sorted({path.resolve() for path in files if path.is_file()})


def _module_name_for(path: Path, root: Path) -> str:
    relative = path.relative_to(root).with_suffix("")
    # 保留 __init__ 这一层，使相对导入解析时仍能以包目录作为父级。
    return ".".join(relative.parts)


def _resolve_import(module_name: str, level: int, imported: str | None) -> str:
    """把 AST 中的相对导入还原为可参与边界判断的绝对模块名。"""
    if level <= 0:
        return imported or ""
    package_parts = module_name.split(".")[:-1]
    keep = max(0, len(package_parts) - level + 1)
    base = package_parts[:keep]
    if imported:
        base.extend(imported.split("."))
    return ".".join(base)


def _iter_imports(tree: ast.AST, module_name: str) -> Iterable[str]:
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            yield from (alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            yield _resolve_import(module_name, node.level, node.module)


def _matches_prefix(module_name: str, prefix: str) -> bool:
    return module_name == prefix or module_name.startswith(prefix + ".")


def collect_failures(root: Path = PROJECT_ROOT) -> list[str]:
    """执行纯静态检查并返回全部问题，便于 CI 一次给出完整修复清单。"""
    failures: list[str] = []
    for path in iter_target_files(root):
        relative = path.relative_to(root).as_posix()
        try:
            source = path.read_text(encoding="utf-8")
            tree = ast.parse(source, filename=str(path))
        except (OSError, SyntaxError, UnicodeError) as exc:
            failures.append(f"无法解析 {relative}: {exc}")
            continue

        module_doc = ast.get_docstring(tree, clean=False) or ""
        missing = [field for field in REQUIRED_HEADER_FIELDS if field not in module_doc]
        if missing:
            failures.append(f"{relative} 缺少文件头字段: {', '.join(missing)}")

        module_name = _module_name_for(path, root)
        imports = tuple(_iter_imports(tree, module_name))
        normalized = relative.replace("\\", "/")

        # Qt/OpenGL 只能停留在 UI 适配层，避免核心逻辑无法在 CLI/Linux 测试环境导入。
        if not normalized.startswith("desktop/ui/"):
            for imported in imports:
                if any(imported.startswith(prefix) for prefix in QT_IMPORT_PREFIXES):
                    failures.append(f"{relative} 非 UI 层直接依赖 {imported}")

        for rule in DEPENDENCY_RULES:
            if not normalized.startswith(rule.source_prefix):
                continue
            for imported in imports:
                for forbidden in rule.forbidden_imports:
                    if not _matches_prefix(imported, forbidden):
                        continue
                    if (normalized, forbidden) in DEPENDENCY_EXCEPTIONS:
                        continue
                    failures.append(f"{relative} 禁止依赖 {imported}")

    return failures


def main() -> int:
    """提供本地和 CI 共用的非零退出码接口。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=PROJECT_ROOT)
    args = parser.parse_args()
    root = args.root.resolve()
    failures = collect_failures(root)
    if failures:
        print("Desktop architecture check FAILED:")
        for failure in failures:
            print(f" - {failure}")
        return 1
    print(f"Desktop architecture check passed: {len(iter_target_files(root))} files.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
