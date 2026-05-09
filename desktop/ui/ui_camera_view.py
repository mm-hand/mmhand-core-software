from typing import Optional
import threading
import time

import numpy as np

from .ui_qt_compat import QtCore, QtGui, QtWidgets


CAMERA_CAPTURE_SLEEP_S = 0.05


class CameraView(QtWidgets.QWidget):
    def __init__(self, context, parent=None):
        super().__init__(parent)
        self.context = context
        self._caps = [None, None]
        self._frames = [None, None]
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._build_ui()

    def _build_ui(self) -> None:
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        self.combos = []
        self.labels = []
        for idx in range(2):
            group = QtWidgets.QGroupBox(f"Camera {idx + 1}")
            box = QtWidgets.QVBoxLayout(group)
            row = QtWidgets.QHBoxLayout()
            combo = QtWidgets.QComboBox()
            combo.addItem("None", None)
            for cam in self._list_available_cameras():
                combo.addItem(f"Camera {cam}", cam)
            combo.currentIndexChanged.connect(lambda _i, slot=idx: self._on_selected(slot))
            row.addWidget(QtWidgets.QLabel("Device:"))
            row.addWidget(combo)
            row.addStretch(1)
            label = QtWidgets.QLabel("None")
            label.setAlignment(QtCore.Qt.AlignCenter)
            label.setMinimumHeight(160)
            label.setStyleSheet("background: black; color: white;")
            box.addLayout(row)
            box.addWidget(label, 1)
            layout.addWidget(group, 1)
            self.combos.append(combo)
            self.labels.append(label)

    def build(self, _parent) -> None:
        pass

    def update(self, _state=None) -> None:
        with self._lock:
            frames = list(self._frames)
        for idx, frame in enumerate(frames):
            self._show_frame(idx, frame)

    def clear(self) -> None:
        with self._lock:
            self._frames = [None, None]
        for label in self.labels:
            label.setText("None")
            label.setPixmap(QtGui.QPixmap())

    def close(self) -> None:
        self._stop_capture()

    def _list_available_cameras(self):
        try:
            import cv2
        except Exception:
            return []
        available = []
        for idx in range(5):
            cap = None
            try:
                cap = cv2.VideoCapture(idx)
                if cap.isOpened():
                    ok, _ = cap.read()
                    if ok:
                        available.append(idx)
            except Exception:
                pass
            finally:
                if cap is not None:
                    try:
                        cap.release()
                    except Exception:
                        pass
        return available

    def _on_selected(self, slot: int) -> None:
        index = self.combos[slot].currentData()
        self._set_capture(slot, index)

    def _set_capture(self, slot: int, index) -> None:
        try:
            import cv2
        except Exception:
            return
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        self._stop.clear()
        cap = self._caps[slot]
        if cap is not None:
            try:
                cap.release()
            except Exception:
                pass
        self._caps[slot] = None
        with self._lock:
            self._frames[slot] = None
        if index is not None:
            cap = cv2.VideoCapture(int(index))
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                self._caps[slot] = cap
        if any(cap is not None for cap in self._caps):
            self._thread = threading.Thread(target=self._capture_loop, daemon=True)
            self._thread.start()

    def _capture_loop(self) -> None:
        try:
            import cv2
        except Exception:
            return
        while not self._stop.is_set():
            frames = [None, None]
            for idx, cap in enumerate(self._caps):
                if cap is None or not cap.isOpened():
                    continue
                try:
                    ok, frame = cap.read()
                    if ok and frame is not None:
                        frames[idx] = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                except Exception:
                    pass
            with self._lock:
                for idx, frame in enumerate(frames):
                    if frame is not None:
                        self._frames[idx] = frame
            time.sleep(CAMERA_CAPTURE_SLEEP_S)

    def _stop_capture(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        for idx, cap in enumerate(self._caps):
            if cap is not None:
                try:
                    cap.release()
                except Exception:
                    pass
            self._caps[idx] = None
        self._stop.clear()
        self.clear()

    def _show_frame(self, idx: int, frame) -> None:
        label = self.labels[idx]
        if frame is None:
            label.setText("None")
            label.setPixmap(QtGui.QPixmap())
            return
        img = np.asarray(frame, dtype=np.uint8)
        if img.ndim == 2:
            qimg = QtGui.QImage(img.data, img.shape[1], img.shape[0], img.strides[0], QtGui.QImage.Format_Grayscale8)
        else:
            qimg = QtGui.QImage(img.data, img.shape[1], img.shape[0], img.strides[0], QtGui.QImage.Format_RGB888)
        pix = QtGui.QPixmap.fromImage(qimg.copy()).scaled(
            label.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation
        )
        label.setText("")
        label.setPixmap(pix)
