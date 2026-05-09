"""Compatibility facade for the desktop Tk UI."""

try:
    from ui.ui_app import HandGUI
    from ui.ui_launcher import LauncherWindow
except ImportError:  # pragma: no cover - package import fallback
    from .ui.ui_app import HandGUI
    from .ui.ui_launcher import LauncherWindow

__all__ = ["HandGUI", "LauncherWindow"]
