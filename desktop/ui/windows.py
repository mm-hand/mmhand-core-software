import os
import tkinter as tk
from tkinter import ttk
import time
from typing import Dict, Optional, Tuple

from data_models import HandModel
from .camera_view import CameraView
from .pose_view import Pose3DView
from .tactile_view import TactileView


GAP = 16
MARGIN = 24
POSE_REAL_INTERVAL_S = 0.10
POSE_PREVIEW_INTERVAL_S = 0.25


def _work_area(root: tk.Tk) -> Tuple[int, int, int, int]:
    if os.name == "nt":
        try:
            import ctypes

            class RECT(ctypes.Structure):
                _fields_ = [
                    ("left", ctypes.c_long),
                    ("top", ctypes.c_long),
                    ("right", ctypes.c_long),
                    ("bottom", ctypes.c_long),
                ]

            rect = RECT()
            ok = ctypes.windll.user32.SystemParametersInfoW(0x0030, 0, ctypes.byref(rect), 0)
            if ok:
                return (
                    int(rect.left),
                    int(rect.top),
                    int(rect.right - rect.left),
                    int(rect.bottom - rect.top),
                )
        except Exception:
            pass
    return 0, 0, int(root.winfo_screenwidth()), int(root.winfo_screenheight())


def _clamp(value: int, low: int, high: int) -> int:
    return max(low, min(high, value))


class DisplayWindowManager:
    def __init__(self, owner):
        self.owner = owner
        self.root = owner.root
        self.windows: Dict[str, tk.Toplevel] = {}
        self.components: Dict[str, object] = {}
        self.vars: Dict[str, tk.BooleanVar] = {
            "pose": tk.BooleanVar(value=False),
            "tactile": tk.BooleanVar(value=False),
            "cameras": tk.BooleanVar(value=False),
        }
        self._last_pose_update_ts = 0.0

    def attach_menu(self, menubar: tk.Menu) -> None:
        view_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="View", menu=view_menu)
        view_menu.add_checkbutton(
            label="Hand Pose",
            variable=self.vars["pose"],
            command=lambda: self._toggle("pose"),
        )
        view_menu.add_checkbutton(
            label="Tactile",
            variable=self.vars["tactile"],
            command=lambda: self._toggle("tactile"),
        )
        view_menu.add_checkbutton(
            label="Cameras",
            variable=self.vars["cameras"],
            command=lambda: self._toggle("cameras"),
        )

    def apply_main_geometry(self) -> None:
        x, y, w, h = _work_area(self.root)
        main_w = _clamp(int(w * 0.42), 640, min(880, max(640, w - MARGIN * 2)))
        main_h = _clamp(h - 80, 620, min(760, max(620, h - MARGIN * 2)))
        self.root.geometry(f"{main_w}x{main_h}+{x + MARGIN}+{y + MARGIN}")
        self.root.minsize(640, 620)

    def open_pose(self) -> None:
        self.open("pose")

    def open_tactile(self) -> None:
        self.open("tactile")

    def open_cameras(self) -> None:
        self.open("cameras")

    def is_open(self, key: str) -> bool:
        win = self.windows.get(key)
        if win is None:
            return False
        try:
            return bool(win.winfo_exists() and win.state() != "withdrawn")
        except tk.TclError:
            return False

    def open(self, key: str) -> None:
        if self.is_open(key):
            win = self.windows[key]
            self.vars[key].set(True)
            win.deiconify()
            win.lift()
            win.focus_set()
            return

        title, component, min_size = self._make_component(key)
        win = tk.Toplevel(self.root)
        win.title(title)
        win.minsize(*min_size)
        win.protocol("WM_DELETE_WINDOW", lambda k=key: self.close(k))
        frame = ttk.Frame(win)
        frame.pack(fill=tk.BOTH, expand=True)
        component.build(frame)
        self.windows[key] = win
        self.components[key] = component
        self.vars[key].set(True)
        self.layout_open_windows()
        win.lift()

    def close(self, key: str) -> None:
        comp = self.components.pop(key, None)
        if comp is not None:
            try:
                comp.close()
            except Exception:
                pass
        win = self.windows.pop(key, None)
        if win is not None:
            try:
                win.destroy()
            except tk.TclError:
                pass
        if key in self.vars:
            self.vars[key].set(False)
        self.layout_open_windows()

    def close_all(self) -> None:
        for key in list(self.windows.keys()):
            self.close(key)

    def update_views(self, state: HandModel) -> None:
        self.update_pose_if_due(state)
        self.update_tactile_if_due(state)

    def update_pose_if_due(self, state: HandModel, *, force: bool = False) -> None:
        if not self.is_open("pose"):
            return
        comp = self.components.get("pose")
        if comp is None:
            return
        interval = POSE_REAL_INTERVAL_S if self.owner._is_comm_connected() else POSE_PREVIEW_INTERVAL_S
        now = time.time()
        if not force and (now - self._last_pose_update_ts) < interval:
            return
        self._last_pose_update_ts = now
        try:
            comp.update(state)
        except tk.TclError:
            self.close("pose")
        except Exception:
            pass

    def update_tactile_if_due(self, state: HandModel, *, force: bool = False) -> None:
        if not self.is_open("tactile"):
            return
        comp = self.components.get("tactile")
        if comp is None:
            return
        try:
            if hasattr(comp, "update_if_due"):
                comp.update_if_due(state, force=force)
            else:
                comp.update(state)
        except tk.TclError:
            self.close("tactile")
        except Exception:
            pass

    def update_cameras(self) -> None:
        if not self.is_open("cameras"):
            return
        comp = self.components.get("cameras")
        if comp is None:
            return
        try:
            comp.update(None)
        except tk.TclError:
            self.close("cameras")
        except Exception:
            pass

    def layout_open_windows(self) -> None:
        open_keys = [k for k in ("pose", "tactile", "cameras") if self.is_open(k)]
        if not open_keys:
            return
        self.root.update_idletasks()
        wx, wy, ww, wh = _work_area(self.root)
        main_x = self.root.winfo_x()
        main_y = self.root.winfo_y()
        main_w = max(self.root.winfo_width(), 640)
        main_h = max(self.root.winfo_height(), 620)
        right_x = main_x + main_w + GAP
        right_y = main_y
        right_w = wx + ww - right_x - MARGIN
        right_h = min(main_h, wy + wh - right_y - MARGIN)
        if right_w < 420:
            right_x = wx + MARGIN
            right_y = main_y + main_h + GAP
            right_w = ww - MARGIN * 2
            right_h = wy + wh - right_y - MARGIN
        if right_h < 260:
            right_x = wx + MARGIN
            right_y = wy + MARGIN
            right_w = max(420, ww - MARGIN * 2)
            right_h = max(300, wh - MARGIN * 2)

        if len(open_keys) == 1:
            key = open_keys[0]
            w = min(760 if key != "cameras" else 860, max(420, right_w))
            h = min(520 if key != "cameras" else 460, max(300, right_h))
            self._set_geometry(key, w, h, right_x, right_y)
            return

        if len(open_keys) >= 3 and right_w >= 920 and right_h >= 560:
            left_w = max(420, (right_w - GAP) // 2)
            right_col_w = max(420, right_w - left_w - GAP)
            row_h = max(260, (right_h - GAP) // 2)
            self._set_geometry("pose", left_w, row_h, right_x, right_y)
            self._set_geometry("tactile", left_w, row_h, right_x, right_y + row_h + GAP)
            self._set_geometry("cameras", right_col_w, row_h * 2 + GAP, right_x + left_w + GAP, right_y)
            return

        each_h = max(260, (right_h - GAP * (len(open_keys) - 1)) // len(open_keys))
        for idx, key in enumerate(open_keys):
            self._set_geometry(key, max(420, right_w), each_h, right_x, right_y + idx * (each_h + GAP))

    def _set_geometry(self, key: str, w: int, h: int, x: int, y: int) -> None:
        win = self.windows.get(key)
        if win is None:
            return
        try:
            win.geometry(f"{int(w)}x{int(h)}+{int(x)}+{int(y)}")
        except tk.TclError:
            pass

    def _toggle(self, key: str) -> None:
        if self.vars[key].get():
            self.open(key)
        else:
            self.close(key)

    def _make_component(self, key: str):
        component_owner = getattr(self.owner, "context", self.owner)
        if key == "pose":
            return "Hand Pose", Pose3DView(component_owner), (420, 300)
        if key == "tactile":
            return "Tactile", TactileView(component_owner), (420, 300)
        if key == "cameras":
            return "Cameras", CameraView(component_owner), (520, 320)
        raise ValueError(f"Unknown display window: {key}")
