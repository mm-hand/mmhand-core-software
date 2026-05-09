from .ui_qt_compat import QtWidgets


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
        self.algorithm_enabled.toggled.connect(
            lambda checked: self.context.emit_action(f"Algorithm {'ON' if checked else 'OFF'}")
        )
        self.algorithm_output = QtWidgets.QPlainTextEdit()
        self.algorithm_output.setReadOnly(True)
        panel_layout.addWidget(self.algorithm_enabled)
        panel_layout.addWidget(self.algorithm_output, 1)
        layout.addWidget(panel, 1)

    def append_output(self, text: str) -> None:
        self.algorithm_output.appendPlainText(str(text))
        if self.algorithm_output.blockCount() > 300:
            cursor = self.algorithm_output.textCursor()
            cursor.movePosition(cursor.Start)
            cursor.select(cursor.LineUnderCursor)
            cursor.removeSelectedText()
            cursor.deleteChar()


__all__ = ["AlgorithmView"]
