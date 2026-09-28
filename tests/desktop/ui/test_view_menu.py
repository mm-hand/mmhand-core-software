"""验证发布版 View 菜单仅通过独立入口打开 Task Runtime。"""

from desktop.ui.ui_qt_compat import QtWidgets, ensure_app
from desktop.ui.ui_app import HandGUI
from desktop.ui.ui_windows import DisplayWindowManager


class _ViewMenuOwner(QtWidgets.QMainWindow):
    """为 View 菜单提供最小宿主，避免测试中创建完整控制界面。"""

    def __init__(self):
        super().__init__()
        self.plot_open_count = 0
        self.task_runtime_open_count = 0

    def _open_joint_rt_window(self) -> None:
        self.plot_open_count += 1

    def show_task_runtime_window(self) -> None:
        self.task_runtime_open_count += 1


def test_view_menu_opens_task_runtime_outside_display_window_manager() -> None:
    """Task Runtime 应位于 View 中，且不与可勾选显示窗口混用生命周期。"""
    ensure_app()
    owner = _ViewMenuOwner()
    manager = DisplayWindowManager(owner)
    manager.attach_menu(owner.menuBar())

    # PySide6 中需保留 QAction 包装对象；直接在生成器内取 menu() 会让临时包装对象
    # 提前析构，导致其关联的 QMenu 被误判为已删除。
    view_action = next(
        action
        for action in owner.menuBar().actions()
        if action.menu() is not None and action.menu().title() == "View"
    )
    view_menu = view_action.menu()
    actions = {
        action.text(): action
        for action in view_menu.actions()
        if not action.isSeparator()
    }

    assert "Task Runtime" in actions
    assert not actions["Task Runtime"].isCheckable()
    assert "task_runtime" not in manager.actions
    assert "Cameras" not in actions
    assert "cameras" not in manager.actions

    actions["Task Runtime"].trigger()
    assert owner.task_runtime_open_count == 1


def test_release_gui_no_longer_defines_device_menu_factory() -> None:
    """发布版不得重新引入仅用于调试的 Device 菜单。"""
    assert not hasattr(HandGUI, "_attach_device_menu")
