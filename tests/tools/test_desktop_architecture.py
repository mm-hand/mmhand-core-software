"""Desktop 文件头和静态依赖边界回归测试。"""

from pathlib import Path

from tools import check_desktop_architecture


def test_desktop_architecture_contract() -> None:
    """正式模块必须保留两项中文契约头，且不得反向穿透分层。"""
    failures = check_desktop_architecture.collect_failures()
    assert not failures, "\n" + "\n".join(failures)


def _fake_tree(tmp_path: Path, body: str) -> Path:
    """在临时目录里铺一个最小的 desktop 结构，放一个 core 模块。"""
    root = tmp_path / "repo"
    for package in ("app", "core", "replay", "ros2", "ui"):
        (root / "desktop" / package).mkdir(parents=True)
    (root / "desktop" / "core" / "fake.py").write_text(body, encoding="utf-8")
    return root


def test_qt_import_in_core_is_rejected(tmp_path: Path) -> None:
    """Qt 已从仓库移除；任何人把 PyQt/PySide 引回核心层都必须被拦住。"""
    root = _fake_tree(tmp_path, '"""fake.\n\n职责：测试。\n禁止依赖：无。\n"""\nimport PyQt6.QtWidgets\n')
    failures = check_desktop_architecture.collect_failures(root)
    assert failures == ["desktop/core/fake.py 非 UI 层直接依赖 PyQt6.QtWidgets"]


def test_pyside_and_pyqtgraph_are_rejected_too(tmp_path: Path) -> None:
    root = _fake_tree(tmp_path, '"""fake.\n\n职责：测试。\n禁止依赖：无。\n"""\nfrom PySide6 import QtCore\nimport pyqtgraph\n')
    failures = check_desktop_architecture.collect_failures(root)
    assert [item.split(" 非 UI 层直接依赖 ")[1] for item in failures] == ["PySide6", "pyqtgraph"]


def test_missing_header_field_is_reported(tmp_path: Path) -> None:
    root = _fake_tree(tmp_path, '"""fake without the required contract fields."""\nvalue = 1\n')
    failures = check_desktop_architecture.collect_failures(root)
    assert any("缺少文件头字段" in item for item in failures)
