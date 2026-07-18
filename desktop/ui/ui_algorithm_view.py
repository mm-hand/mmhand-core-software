from .ui_qt_compat import QtCore, QtWidgets


class AlgorithmView(QtWidgets.QWidget):
    def __init__(self, context, parent=None):
        super().__init__(parent)
        self.context = context
        self._build()

    def _build(self) -> None:
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)

        panel = QtWidgets.QGroupBox("Algorithm")
        panel_layout = QtWidgets.QVBoxLayout(panel)
        self.algorithm_enabled = QtWidgets.QCheckBox("Algorithm Enabled")
        self.algorithm_enabled.toggled.connect(self._set_algorithm_enabled)
        self.algorithm_status = QtWidgets.QLabel("Idle: select Algorithm Enabled to start")
        self.algorithm_status.setStyleSheet("color:#57606a;")
        self.algorithm_output = QtWidgets.QPlainTextEdit()
        self.algorithm_output.setReadOnly(True)
        panel_layout.addWidget(self.algorithm_enabled)
        panel_layout.addWidget(self.algorithm_status)
        panel_layout.addWidget(self.algorithm_output, 1)
        layout.addWidget(panel, 1)

        self._status_timer = QtCore.QTimer(self)
        self._status_timer.timeout.connect(self._refresh_running_state)
        self._status_timer.start(250)

    def _set_algorithm_enabled(self, checked: bool) -> None:
        accepted = self.context.set_algorithm_enabled(bool(checked))
        if checked and not accepted:
            blocker = QtCore.QSignalBlocker(self.algorithm_enabled)
            self.algorithm_enabled.setChecked(False)
            del blocker
        self._refresh_running_state()

    def _refresh_running_state(self) -> None:
        running = self.context.is_algorithm_running()
        if running:
            self.algorithm_status.setText("Running: Open/Fist pose cycle is active")
            return
        if self.algorithm_enabled.isChecked():
            blocker = QtCore.QSignalBlocker(self.algorithm_enabled)
            self.algorithm_enabled.setChecked(False)
            del blocker
        self.algorithm_status.setText("Idle: select Algorithm Enabled to start")

    def append_output(self, text: str) -> None:
        self.algorithm_output.appendPlainText(str(text))
        if self.algorithm_output.blockCount() > 300:
            cursor = self.algorithm_output.textCursor()
            cursor.movePosition(cursor.Start)
            cursor.select(cursor.LineUnderCursor)
            cursor.removeSelectedText()
            cursor.deleteChar()


__all__ = ["AlgorithmView"]
