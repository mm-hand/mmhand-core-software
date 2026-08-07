"""ui_style.py - MMHand Desktop 模块契约。

职责：集中定义上位机颜色、字体和控件样式。
线程：Qt GUI 主线程执行；耗时工作必须委托后台服务，禁止阻塞界面事件循环。
输入：用户操作、HandModel 只读快照以及 HandBackend/服务返回结果。
输出：Qt 控件状态、用户提示和经门面提交的应用命令。
状态所有权：仅拥有本视图控件和瞬时交互状态；设备真值由 core 层拥有。
禁止依赖：禁止直接导入串口通信和底层控制实现，禁止在 UI 中复制协议或安全规则。
"""
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
