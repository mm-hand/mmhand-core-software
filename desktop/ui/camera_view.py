from typing import List, Optional
import threading
import time
import tkinter as tk
from tkinter import ttk

import numpy as np

from data_models import HandModel

CAMERA_CAPTURE_SLEEP_S = 0.05
CAMERA_UI_POLL_MS = 50

class CameraViewMixin:
    def _list_available_cameras(self) -> List[int]:
        """Probe indices 0-4 for available cameras (avoids OpenCV out-of-bound on some systems)."""
        try:
            import cv2
        except ImportError:
            return []
        available = []
        for idx in range(5):
            try:
                cap = cv2.VideoCapture(idx)
                if cap.isOpened():
                    ret, _ = cap.read()
                    cap.release()
                    if ret:
                        available.append(idx)
            except Exception:
                pass
        return available

    def _refresh_camera_menus(self) -> None:
        """Refresh camera combo dropdown values (Device > Refresh)."""
        self._update_camera_combo_values()

    def _on_camera_selected(self, index: Optional[int], slot: int) -> None:
        """Slot 1=Left widget, 2=Right widget."""
        label = "None" if index is None else f"Camera {index}"
        if slot == 1:
            self._camera1_var.set(label)
        else:
            self._camera2_var.set(label)
        self._emit_action(f"Camera {slot} selected: {index if index is not None else 'None'}")
        threading.Thread(target=lambda: self._set_camera_capture(slot, index), daemon=True).start()

    def _set_camera_capture(self, slot: int, index: Optional[int]) -> None:
        """Open or release camera for slot (1=Left, 2=Right)."""
        try:
            import cv2
        except ImportError:
            return
        self._camera_stop.set()
        if self._camera_thread is not None:
            self._camera_thread.join(timeout=1.0)
            self._camera_thread = None
        self._camera_stop.clear()

        cap_attr = "_camera1_cap" if slot == 1 else "_camera2_cap"
        old_cap = getattr(self, cap_attr, None)
        if old_cap is not None:
            try:
                old_cap.release()
            except Exception:
                pass
            setattr(self, cap_attr, None)
        if slot == 1:
            self._camera_frame_left = None
            with self._camera_buf_lock:
                self._camera_frame_left_buf = None
        else:
            self._camera_frame_right = None
            with self._camera_buf_lock:
                self._camera_frame_right_buf = None

        if index is not None:
            try:
                cap = cv2.VideoCapture(index)
                if cap.isOpened():
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                    setattr(self, cap_attr, cap)
            except Exception:
                pass

        if self._camera1_cap is not None or self._camera2_cap is not None:
            self._camera_thread = threading.Thread(target=self._camera_capture_loop, daemon=True)
            self._camera_thread.start()

    def _camera_capture_loop(self) -> None:
        """Background thread: read frames from cameras, store in buffers."""
        try:
            import cv2
        except ImportError:
            return
        while True:
            if self._camera_stop.is_set():
                break
            left_frame = None
            right_frame = None
            if self._camera1_cap is not None and self._camera1_cap.isOpened():
                try:
                    ret, frame = self._camera1_cap.read()
                    if ret and frame is not None:
                        left_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                except Exception:
                    pass
            if self._camera2_cap is not None and self._camera2_cap.isOpened():
                try:
                    ret, frame = self._camera2_cap.read()
                    if ret and frame is not None:
                        right_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                except Exception:
                    pass
            with self._camera_buf_lock:
                self._camera_frame_left_buf = left_frame
                self._camera_frame_right_buf = right_frame
            time.sleep(CAMERA_CAPTURE_SLEEP_S)

    def _stop_camera_capture(self) -> None:
        stop = getattr(self, "_camera_stop", None)
        if stop is not None:
            stop.set()
        thr = getattr(self, "_camera_thread", None)
        if thr is not None:
            thr.join(timeout=1.0)
            self._camera_thread = None
        for cap_attr in ("_camera1_cap", "_camera2_cap"):
            cap = getattr(self, cap_attr, None)
            if cap is not None:
                try:
                    cap.release()
                except Exception:
                    pass
            setattr(self, cap_attr, None)
        if stop is not None:
            stop.clear()
        with self._camera_buf_lock:
            self._camera_frame_left_buf = None
            self._camera_frame_right_buf = None
        self._camera_frame_left = None
        self._camera_frame_right = None
        if hasattr(self, "_camera1_var"):
            self._camera1_var.set("None")
        if hasattr(self, "_camera2_var"):
            self._camera2_var.set("None")

    def _start_camera_poll(self) -> None:
        """Main-thread poll: copy buffers to display (non-blocking)."""

        def poll() -> None:
            try:
                with self._camera_buf_lock:
                    left = self._camera_frame_left_buf
                    right = self._camera_frame_right_buf
                self._camera_frame_left = left
                self._camera_frame_right = right
                if hasattr(self, "_display_windows"):
                    self._display_windows.update_cameras()
                elif getattr(self, "_cam_left_canvas", None) is not None:
                    self._update_camera_widget()
            except Exception:
                pass
            try:
                self.root.after(CAMERA_UI_POLL_MS, poll)
            except tk.TclError:
                pass

        self.root.after(CAMERA_UI_POLL_MS, poll)

    def _setup_tactile_and_camera_widgets(self, parent):
        """将下方区域拆分为左右两栏：触觉矩阵与相机各占左侧主区一半宽度。"""
        split = ttk.PanedWindow(parent, orient=tk.HORIZONTAL)
        split.pack(fill=tk.BOTH, expand=True, padx=4, pady=2)
        self._tactile_cam_paned = split

        left_frame = ttk.Frame(split)
        split.add(left_frame, weight=1)
        self._setup_tactile_matrix_widget(left_frame)

        cam_frame = ttk.LabelFrame(split, text="")
        split.add(cam_frame, weight=1)
        try:
            split.pane(left_frame, minsize=100)
            split.pane(cam_frame, minsize=120)
        except tk.TclError:
            pass
        self._setup_camera_widget(cam_frame)

    def _setup_camera_widget(self, parent):
        """相机图像窗格：上下两个，每个含摄像头下拉选择 + 画面全填充"""
        self._camera_frame_left = None
        self._camera_frame_right = None
        self._cam_left_image_item = None
        self._cam_left_text_item = None
        self._cam_right_image_item = None
        self._cam_right_text_item = None

        split = ttk.PanedWindow(parent, orient=tk.VERTICAL)
        split.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)

        left_box = ttk.Frame(split)
        right_box = ttk.Frame(split)
        split.add(left_box, weight=1)
        split.add(right_box, weight=1)
        try:
            split.pane(left_box, minsize=72)
            split.pane(right_box, minsize=72)
        except tk.TclError:
            pass

        def _make_camera_slot(box, var, slot):
            row = ttk.Frame(box)
            row.pack(fill=tk.X, pady=(0, 2))
            ttk.Label(row, text="Camera:").pack(side=tk.LEFT, padx=(0, 4))
            combo = ttk.Combobox(row, textvariable=var, width=12, state="readonly")
            combo.pack(side=tk.LEFT)
            combo.bind("<<ComboboxSelected>>", lambda e, c=combo, s=slot: self._on_camera_combo_selected(s, c))
            canvas_frame = ttk.Frame(box)
            canvas_frame.pack(fill=tk.BOTH, expand=True)
            canvas = tk.Canvas(canvas_frame, bg="black", highlightthickness=0)
            canvas.pack(fill=tk.BOTH, expand=True)
            return combo, canvas

        self._cam_left_combo, self._cam_left_canvas = _make_camera_slot(left_box, self._camera1_var, 1)
        self._cam_right_combo, self._cam_right_canvas = _make_camera_slot(right_box, self._camera2_var, 2)

        self._update_camera_combo_values()

    def set_camera_frame_left(self, frame: np.ndarray) -> None:
        """外部可调用：设置左相机帧（RGB/BGR/灰度均可）"""
        self._camera_frame_left = frame

    def set_camera_frame_right(self, frame: np.ndarray) -> None:
        """外部可调用：设置右相机帧（RGB/BGR/灰度均可）"""
        self._camera_frame_right = frame

    def set_camera_frame(self, frame: np.ndarray) -> None:
        """兼容旧接口：默认写入左相机帧"""
        self._camera_frame_left = frame

    def _update_camera_combo_values(self) -> None:
        """Set combo values: None + available cameras."""
        choices = ["None"] + [f"Camera {i}" for i in self._list_available_cameras()]
        if hasattr(self, "_cam_left_combo"):
            self._cam_left_combo["values"] = choices
        if hasattr(self, "_cam_right_combo"):
            self._cam_right_combo["values"] = choices

    def _on_camera_combo_selected(self, slot: int, combo: ttk.Combobox) -> None:
        """Handle camera selection from combo (slot 1 or 2). Defer to ensure value is updated."""
        def do():
            val = combo.get()
            if val == "None" or not val:
                self._on_camera_selected(None, slot)
            else:
                try:
                    idx = int(val.replace("Camera ", ""))
                    self._on_camera_selected(idx, slot)
                except ValueError:
                    pass
        self.root.after(0, do)

    def _update_single_camera_widget(
        self,
        frame,
        canvas,
        photo_attr: str,
        image_item_attr: str,
        text_item_attr: str,
    ):
        """Update canvas with frame using PIL/PhotoImage for real-time video."""
        try:
            from PIL import Image
            from PIL import ImageTk
        except ImportError:
            return
        cw = max(canvas.winfo_width() or 320, 320)
        ch = max(canvas.winfo_height() or 240, 240)
        image_item = getattr(self, image_item_attr, None)
        if image_item is None:
            image_item = canvas.create_image(cw // 2, ch // 2)
            setattr(self, image_item_attr, image_item)
        else:
            canvas.coords(image_item, cw // 2, ch // 2)

        text_item = getattr(self, text_item_attr, None)
        if text_item is None:
            text_item = canvas.create_text(cw // 2, ch // 2, text="None", fill="white", font=("", 14))
            setattr(self, text_item_attr, text_item)
        else:
            canvas.coords(text_item, cw // 2, ch // 2)

        if frame is None:
            setattr(self, photo_attr, None)
            canvas.itemconfigure(image_item, image="", state="hidden")
            canvas.itemconfigure(text_item, text="None", state="normal")
            return
        img = np.asarray(frame, dtype=np.uint8)
        if img.ndim == 2:
            pil_img = Image.fromarray(img, mode="L")
        elif img.ndim == 3 and img.shape[2] >= 3:
            pil_img = Image.fromarray(img[:, :, :3], mode="RGB")
        else:
            return
        resample = getattr(Image, "Resampling", Image).LANCZOS
        pil_img.thumbnail((max(cw, 1), max(ch, 1)), resample)
        photo = ImageTk.PhotoImage(pil_img)
        setattr(self, photo_attr, photo)
        canvas.itemconfigure(image_item, image=photo, state="normal")
        canvas.itemconfigure(text_item, state="hidden")

    def _update_camera_widget(self):
        """相机与串口 Start/Stop 无关：在 Device 中选摄像头后即采集显示，由 _start_camera_poll 刷新。"""
        if not hasattr(self, "_cam_left_canvas"):
            return
        self._update_single_camera_widget(
            self._camera_frame_left,
            self._cam_left_canvas,
            "_cam_left_photo",
            "_cam_left_image_item",
            "_cam_left_text_item",
        )
        self._update_single_camera_widget(
            self._camera_frame_right,
            self._cam_right_canvas,
            "_cam_right_photo",
            "_cam_right_image_item",
            "_cam_right_text_item",
        )



class _ComponentProxy:
    def __init__(self, owner):
        self.owner = owner
        self.root = owner.root

    def __getattr__(self, name):
        return getattr(self.owner, name)


class CameraView(_ComponentProxy, CameraViewMixin):
    def build(self, parent) -> None:
        self._setup_camera_widget(parent)

    def update(self, _state: Optional[HandModel] = None) -> None:
        self._camera_frame_left = getattr(self.owner, "_camera_frame_left", None)
        self._camera_frame_right = getattr(self.owner, "_camera_frame_right", None)
        self._update_camera_widget()

    def clear(self) -> None:
        self._camera_frame_left = None
        self._camera_frame_right = None
        self._update_camera_widget()

    def close(self) -> None:
        self.owner.stop_camera_capture()

    def _on_camera_selected(self, index: Optional[int], slot: int) -> None:
        self.owner.select_camera(index, slot)

    def _list_available_cameras(self) -> List[int]:
        return self.owner.list_available_cameras()
