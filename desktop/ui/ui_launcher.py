from comm_layer import list_ports
from core_logic import HandController
from modes import Mode, run
from .ui_qt_compat import QtWidgets, ensure_app
from .ui_style import configure_qt_application


class LauncherWindow(QtWidgets.QDialog):
    def __init__(self):
        self._qt_app = ensure_app()
        configure_qt_application(self._qt_app)
        super().__init__()
        self.setWindowTitle("Dexterous Hand - Connection & Mode")
        self.resize(420, 190)
        self.ports = []
        self._build()
        self._refresh_ports()

    def _build(self) -> None:
        layout = QtWidgets.QVBoxLayout(self)
        form = QtWidgets.QFormLayout()
        self.port_combo = QtWidgets.QComboBox()
        self.mode_combo = QtWidgets.QComboBox()
        self.mode_combo.addItems(["Manual", "Teleop", "Algorithm"])
        form.addRow("Port:", self.port_combo)
        form.addRow("Mode:", self.mode_combo)
        layout.addLayout(form)
        layout.addWidget(QtWidgets.QLabel('(Select "No Connection" to open UI only)'))
        buttons = QtWidgets.QHBoxLayout()
        refresh = QtWidgets.QPushButton("Refresh")
        launch = QtWidgets.QPushButton("Connect & Start")
        close = QtWidgets.QPushButton("Exit")
        buttons.addWidget(refresh)
        buttons.addStretch(1)
        buttons.addWidget(launch)
        buttons.addWidget(close)
        layout.addLayout(buttons)
        refresh.clicked.connect(self._refresh_ports)
        launch.clicked.connect(self._on_launch)
        close.clicked.connect(self.close)

    def _refresh_ports(self) -> None:
        self.ports = list_ports()
        self.port_combo.clear()
        self.port_combo.addItem("No Connection", None)
        for device, desc in self.ports:
            self.port_combo.addItem(f"{device} ({desc})", device)

    def _on_launch(self) -> None:
        port = self.port_combo.currentData()
        mode_text = self.mode_combo.currentText()
        mode = Mode.MONITOR
        if mode_text == "Teleop":
            mode = Mode.TELEOP
        elif mode_text == "Algorithm":
            mode = Mode.ALGORITHM
        controller = HandController()
        if port and not controller.initialize(port):
            QtWidgets.QMessageBox.critical(self, "Connection Failed", f"Cannot open port: {port}")
            return
        self.close()
        run(mode, controller, port=port, gui=True)

    def run(self) -> None:
        self.show()
        exec_fn = getattr(self._qt_app, "exec", None) or getattr(self._qt_app, "exec_")
        exec_fn()


__all__ = ["LauncherWindow"]
