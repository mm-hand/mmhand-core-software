"""跨平台、可滚动的模态消息对话框。"""

from typing import Optional

from .ui_qt_compat import QtCore, QtGui, QtWidgets


_ICON_NAMES = {
    "information": "SP_MessageBoxInformation",
    "warning": "SP_MessageBoxWarning",
    "critical": "SP_MessageBoxCritical",
    "question": "SP_MessageBoxQuestion",
}


def _standard_pixmap(name: str):
    """兼容 PySide6/PyQt5 的 QStyle.StandardPixmap 枚举差异。"""
    value = getattr(QtWidgets.QStyle, name, None)
    if value is not None:
        return value
    return getattr(QtWidgets.QStyle.StandardPixmap, name)


class ScrollableMessageDialog(QtWidgets.QDialog):
    """正文可滚动、按钮固定可见，并受当前屏幕可用区域约束的弹窗。"""

    def __init__(
        self,
        parent,
        title: str,
        text: str,
        *,
        severity: str = "information",
        accept_text: str = "确定",
        reject_text: Optional[str] = None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle(str(title))
        self.setModal(True)
        self.setSizeGripEnabled(True)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 12)
        layout.setSpacing(12)

        content_layout = QtWidgets.QHBoxLayout()
        content_layout.setSpacing(14)
        icon_name = _ICON_NAMES.get(str(severity).lower(), _ICON_NAMES["information"])
        icon_label = QtWidgets.QLabel(self)
        icon = self.style().standardIcon(_standard_pixmap(icon_name))
        icon_label.setPixmap(icon.pixmap(48, 48))
        icon_label.setAlignment(QtCore.Qt.AlignTop | QtCore.Qt.AlignHCenter)
        content_layout.addWidget(icon_label, 0)

        # 长故障说明必须放入滚动控件，避免 Linux 窗口管理器把按钮挤出屏幕。
        self.text_view = QtWidgets.QPlainTextEdit(self)
        self.text_view.setReadOnly(True)
        self.text_view.setPlainText(str(text))
        try:
            self.text_view.setLineWrapMode(QtWidgets.QPlainTextEdit.WidgetWidth)
        except AttributeError:
            self.text_view.setLineWrapMode(
                QtWidgets.QPlainTextEdit.LineWrapMode.WidgetWidth
            )
        self.text_view.setTextInteractionFlags(
            QtCore.Qt.TextSelectableByMouse | QtCore.Qt.TextSelectableByKeyboard
        )
        content_layout.addWidget(self.text_view, 1)
        layout.addLayout(content_layout, 1)

        self.button_box = QtWidgets.QDialogButtonBox(self)
        self.accept_button = self.button_box.addButton(
            str(accept_text), QtWidgets.QDialogButtonBox.AcceptRole
        )
        self.accept_button.setDefault(True)
        self.accept_button.setAutoDefault(True)
        self.reject_button = None
        if reject_text is not None:
            self.reject_button = self.button_box.addButton(
                str(reject_text), QtWidgets.QDialogButtonBox.RejectRole
            )
        self.button_box.accepted.connect(self.accept)
        self.button_box.rejected.connect(self.reject)
        layout.addWidget(self.button_box, 0)

        self._fit_to_available_screen(parent)

    def _fit_to_available_screen(self, parent) -> None:
        """按当前显示器可用区域设置初始尺寸，保留任务栏和窗口装饰空间。"""
        screen = None
        if parent is not None:
            screen_getter = getattr(parent, "screen", None)
            if callable(screen_getter):
                screen = screen_getter()
        if screen is None:
            screen = QtGui.QGuiApplication.primaryScreen()
        if screen is None:
            self.resize(760, 520)
            return

        area = screen.availableGeometry()
        max_width = max(320, int(area.width()) - 40)
        max_height = max(240, int(area.height()) - 40)
        width = min(max_width, max(560, int(area.width() * 0.62)))
        height = min(max_height, max(360, int(area.height() * 0.62)))
        self.setMinimumSize(min(width, 480), min(height, 300))
        self.resize(width, height)
        self.move(area.center() - self.rect().center())


def show_scrollable_message(
    parent,
    title: str,
    text: str,
    *,
    severity: str = "information",
    accept_text: str = "确定",
    reject_text: Optional[str] = None,
) -> bool:
    """显示统一长文本弹窗；返回 True 表示用户点击了确认按钮。"""
    dialog = ScrollableMessageDialog(
        parent,
        title,
        text,
        severity=severity,
        accept_text=accept_text,
        reject_text=reject_text,
    )
    return dialog.exec() == QtWidgets.QDialog.Accepted


__all__ = ["ScrollableMessageDialog", "show_scrollable_message"]
