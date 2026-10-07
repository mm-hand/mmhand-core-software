"""Read-only entry point for repository checks and regression tests."""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fast", action="store_true", help="Run the local commit gate")
    args = parser.parse_args()
    shell = shutil.which("pwsh") or shutil.which("powershell.exe")
    if shell is None:
        parser.error("PowerShell is required for the existing P4 architecture check")
    commands = [
        [shell, "-NoProfile", "-ExecutionPolicy", "Bypass", "-File",
         str(ROOT / "tools/p4_architecture/check_p4_architecture.ps1")],
        [sys.executable, "tools/check_desktop_architecture.py"],
        [sys.executable, "-m", "pytest", "-q", "--tb=short"],
    ]
    if args.fast:
        commands[-1].extend([
            "tests/desktop/app/test_entry.py",
            "tests/desktop/ros2/test_joint_targets.py",
            "tests/desktop/core/test_manual_reference_protocol.py",
            "tests/tools/test_generate_robot_model.py",
        ])
    scratch = ROOT / "tmp"
    scratch.mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="checks-", dir=scratch) as test_temp:
        commands[-1].append(f"--basetemp={test_temp}/pytest")
        for command in commands:
            result = subprocess.run(command, cwd=ROOT)
            if result.returncode:
                return result.returncode
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
