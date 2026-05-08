"""Compatibility facade for the desktop Tk UI."""

try:
    from ui.app import HandGUI
    from ui.launcher import LauncherWindow
except ImportError:  # pragma: no cover - package import fallback
    from .ui.app import HandGUI
    from .ui.launcher import LauncherWindow

__all__ = ["HandGUI", "LauncherWindow"]
