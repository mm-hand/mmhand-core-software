from .ui_qt_compat import QtWidgets


class TeleopView(QtWidgets.QWidget):
    def __init__(self, context, parent=None):
        super().__init__(parent)
        self.context = context
        self._build()

    def _build(self) -> None:
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)

        panel = QtWidgets.QGroupBox("Teleop")
        form = QtWidgets.QFormLayout(panel)
        self.teleop_mode_combo = QtWidgets.QComboBox()
        self.teleop_mode_combo.addItems(["Mode 1: Visual Retargeting", "Mode 2: Rokoko Glove"])
        self.teleop_enabled = QtWidgets.QCheckBox("Teleop Enabled")
        self.teleop_mode_combo.currentTextChanged.connect(
            lambda text: self.context.emit_action(f"Teleop mode selected: {text}")
        )
        self.teleop_enabled.toggled.connect(
            lambda checked: self.context.emit_action(f"Teleop {'ON' if checked else 'OFF'}")
        )
        form.addRow("Teleop Mode:", self.teleop_mode_combo)
        form.addRow("", self.teleop_enabled)
        layout.addWidget(panel)
        layout.addStretch(1)


__all__ = ["TeleopView"]
