from typing import Callable, Optional

from .ui_qt_compat import QtWidgets
from .ui_visual_retargeting_view import VisualRetargetingView


class TeleopView(QtWidgets.QWidget):
    def __init__(self, context, selected_joint_mask_getter: Optional[Callable[[], int]] = None, parent=None):
        super().__init__(parent)
        self.context = context
        self.selected_joint_mask_getter = selected_joint_mask_getter
        self._body_widget: Optional[QtWidgets.QWidget] = None
        self._build()

    def _build(self) -> None:
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)

        panel = QtWidgets.QGroupBox("Teleop")
        form = QtWidgets.QFormLayout(panel)
        self.teleop_mode_combo = QtWidgets.QComboBox()
        self.teleop_mode_combo.addItem("Select teleop source...", "")
        self.teleop_mode_combo.addItem("Mode 1: Visual Retargeting", "visual")
        self.teleop_mode_combo.addItem("Mode 2: Rokoko Glove", "rokoko")
        self.teleop_mode_combo.currentIndexChanged.connect(lambda _idx: self._on_mode_changed())
        form.addRow("Teleop Mode:", self.teleop_mode_combo)
        layout.addWidget(panel)

        self.body_container = QtWidgets.QWidget()
        self.body_layout = QtWidgets.QVBoxLayout(self.body_container)
        self.body_layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.body_container, 1)

    def _on_mode_changed(self) -> None:
        mode = self.teleop_mode_combo.currentData()
        self.context.emit_action(f"Teleop mode selected: {self.teleop_mode_combo.currentText()}")
        self._clear_body()
        if mode == "visual":
            self._body_widget = VisualRetargetingView(
                self.context,
                selected_joint_mask_getter=self.selected_joint_mask_getter,
            )
            self.body_layout.addWidget(self._body_widget)

    def _clear_body(self) -> None:
        if self._body_widget is None:
            return
        closer = getattr(self._body_widget, "close", None)
        if callable(closer):
            try:
                closer()
            except RuntimeError:
                pass
        self.body_layout.removeWidget(self._body_widget)
        self._body_widget.deleteLater()
        self._body_widget = None

    def update_state(self, state) -> None:
        updater = getattr(self._body_widget, "update_state", None)
        if callable(updater):
            updater(state)

    def closeEvent(self, event):  # noqa: N802 - Qt override
        self._clear_body()
        super().closeEvent(event)


__all__ = ["TeleopView"]
