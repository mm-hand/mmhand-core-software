"""Compatibility wrapper for the package entrypoint.

Prefer: python -m desktop ui|cli
"""

import os
import sys

if __package__ in (None, ""):
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if project_root not in sys.path:
        sys.path.insert(0, project_root)

from desktop.app.entry import main


if __name__ == "__main__":
    raise SystemExit(main())
