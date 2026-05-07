"""Console encoding setup for desktop Python entrypoints."""

from __future__ import annotations

import os
import sys


UTF8_CODE_PAGE = 65001


def _reconfigure_stream(stream) -> None:
    reconfigure = getattr(stream, "reconfigure", None)
    if not callable(reconfigure):
        return
    try:
        reconfigure(encoding="utf-8", errors="replace")
    except (OSError, TypeError, ValueError):
        pass


def configure_utf8_console() -> None:
    """Prefer UTF-8 for Windows console I/O without changing app logic."""
    if os.name == "nt":
        try:
            import ctypes

            kernel32 = ctypes.windll.kernel32
            kernel32.SetConsoleOutputCP(UTF8_CODE_PAGE)
            kernel32.SetConsoleCP(UTF8_CODE_PAGE)
        except Exception:
            pass

    _reconfigure_stream(getattr(sys, "stdout", None))
    _reconfigure_stream(getattr(sys, "stderr", None))
