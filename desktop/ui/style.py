import os
import tkinter as tk
from tkinter import ttk


def configure_windows_process_dpi() -> None:
    if os.name != "nt":
        return
    try:
        import ctypes
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except Exception:
        try:
            ctypes.windll.user32.SetProcessDPIAware()
        except Exception:
            pass


def configure_windows_root(root: tk.Tk) -> None:
    if os.name != "nt":
        return
    tk_font = "{Segoe UI} 9"
    root.option_add("*Font", tk_font)
    root.option_add("*Menu.Font", tk_font)
    root.option_add("*TCombobox*Listbox.Font", tk_font)
    root.option_add("*tearOff", False)
    try:
        root.tk.call("tk", "scaling", max(1.0, root.winfo_fpixels("1i") / 96.0))
    except tk.TclError:
        pass


def configure_windows_ttk_style(root: tk.Tk) -> ttk.Style:
    style = ttk.Style(root)
    if os.name != "nt":
        return style
    for theme in ("vista", "xpnative", "winnative"):
        if theme in style.theme_names():
            try:
                style.theme_use(theme)
                break
            except tk.TclError:
                pass
    style.configure(".", font=("Segoe UI", 9))
    style.configure("TButton", padding=(8, 4))
    style.configure("TCheckbutton", padding=(4, 2))
    style.configure("TRadiobutton", padding=(4, 2))
    style.configure("TLabelFrame.Label", font=("Segoe UI", 9, "bold"))
    style.configure("TNotebook.Tab", padding=(10, 4))
    return style
