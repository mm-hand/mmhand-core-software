from .ui_qt_compat import QtWidgets


def configure_qt_application(app: QtWidgets.QApplication) -> None:
    app.setStyle("Fusion")
    font = app.font()
    if font.pointSize() < 9:
        font.setPointSize(9)
    app.setFont(font)


def apply_main_window_defaults(window: QtWidgets.QWidget) -> None:
    screen = QtWidgets.QApplication.primaryScreen()
    if screen is None:
        window.resize(1180, 760)
        return
    area = screen.availableGeometry()
    w = min(1280, max(980, int(area.width() * 0.78)))
    h = min(820, max(680, int(area.height() * 0.82)))
    x = area.x() + max(24, (area.width() - w) // 2)
    y = area.y() + max(24, (area.height() - h) // 2)
    window.setGeometry(x, y, w, h)
