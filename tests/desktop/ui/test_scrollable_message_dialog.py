import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from desktop.ui.ui_dialogs import ScrollableMessageDialog
from desktop.ui.ui_qt_compat import QtWidgets, ensure_app


_APP = ensure_app()


def test_long_message_uses_scrollable_read_only_body_and_visible_buttons():
    dialog = ScrollableMessageDialog(
        None,
        "系统故障保护",
        "\n".join(f"故障详情 {index}" for index in range(300)),
        severity="warning",
        accept_text="清除故障",
        reject_text="取消",
    )

    text_view = dialog.findChild(QtWidgets.QPlainTextEdit)
    button_box = dialog.findChild(QtWidgets.QDialogButtonBox)

    assert text_view is not None
    assert text_view.isReadOnly()
    assert "故障详情 299" in text_view.toPlainText()
    assert button_box is not None
    assert dialog.accept_button.text() == "清除故障"
    assert dialog.reject_button is not None
    assert dialog.reject_button.text() == "取消"
    assert dialog.isSizeGripEnabled()


def test_dialog_initial_size_stays_inside_available_screen():
    dialog = ScrollableMessageDialog(
        None,
        "标定装配警告",
        "warning\n" * 1000,
        severity="warning",
    )
    screen = dialog.screen()
    area = screen.availableGeometry()

    assert dialog.width() <= area.width()
    assert dialog.height() <= area.height()
