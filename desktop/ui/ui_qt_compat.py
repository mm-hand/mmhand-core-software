"""ui_qt_compat.py - MMHand Desktop 模块契约。

职责：隔离 PySide6 与 PyQt5 差异并提供统一 Qt 符号。
线程：Qt GUI 主线程执行；耗时工作必须委托后台服务，禁止阻塞界面事件循环。
输入：用户操作、HandModel 只读快照以及 HandBackend/服务返回结果。
输出：Qt 控件状态、用户提示和经门面提交的应用命令。
状态所有权：仅拥有本视图控件和瞬时交互状态；设备真值由 core 层拥有。
禁止依赖：禁止直接导入串口通信和底层控制实现，禁止在 UI 中复制协议或安全规则。
"""
import os
import sys


def _try_pyside6():
    os.environ.setdefault("PYQTGRAPH_QT_LIB", "PySide6")
    from PySide6 import QtCore, QtGui, QtWidgets

    return QtCore, QtGui, QtWidgets, "PySide6"


def _try_pyqt5():
    os.environ["PYQTGRAPH_QT_LIB"] = "PyQt5"
    from PyQt5 import QtCore, QtGui, QtWidgets

    return QtCore, QtGui, QtWidgets, "PyQt5"


try:
    QtCore, QtGui, QtWidgets, QT_LIB = _try_pyside6()
except Exception:
    QtCore, QtGui, QtWidgets, QT_LIB = _try_pyqt5()

Signal = QtCore.Signal if QT_LIB == "PySide6" else QtCore.pyqtSignal
Slot = QtCore.Slot if QT_LIB == "PySide6" else QtCore.pyqtSlot


def configure_opengl_before_app() -> str:
    """在 QApplication 创建前选择 OpenGL 后端。

    少数仍使用 OpenGL 的窗口可通过 ``MMHAND_QT_OPENGL`` 选择后端。
    默认保持桌面 OpenGL，与改动前的行为一致；只有定位显卡驱动问题时，
    才设置 ``MMHAND_QT_OPENGL=software`` 使用软件实现。
    """
    requested = os.environ.get("MMHAND_QT_OPENGL", "desktop").strip().lower()
    mode = requested if requested in {"software", "desktop", "angle"} else "desktop"
    os.environ["QT_OPENGL"] = mode
    if QtWidgets.QApplication.instance() is None:
        attributes = getattr(QtCore.Qt, "ApplicationAttribute", QtCore.Qt)
        attr_name = {
            "software": "AA_UseSoftwareOpenGL",
            "desktop": "AA_UseDesktopOpenGL",
            "angle": "AA_UseOpenGLES",
        }[mode]
        attribute = getattr(attributes, attr_name, None)
        if attribute is not None:
            QtCore.QCoreApplication.setAttribute(attribute, True)
    return mode


def ensure_app():
    app = QtWidgets.QApplication.instance()
    if app is None:
        configure_opengl_before_app()
        app = QtWidgets.QApplication(sys.argv[:1])
    return app


__all__ = [
    "QtCore",
    "QtGui",
    "QtWidgets",
    "Signal",
    "Slot",
    "QT_LIB",
    "configure_opengl_before_app",
    "ensure_app",
]
