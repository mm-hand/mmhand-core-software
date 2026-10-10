"""Compatibility alias for the single full software check script."""
import sys
from pathlib import Path
if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.precommit_checks import main
if __name__ == '__main__':
    raise SystemExit(main())
