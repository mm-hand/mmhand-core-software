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


def ensure_app():
    app = QtWidgets.QApplication.instance()
    if app is None:
        app = QtWidgets.QApplication(sys.argv[:1])
    return app


__all__ = ["QtCore", "QtGui", "QtWidgets", "Signal", "Slot", "QT_LIB", "ensure_app"]
