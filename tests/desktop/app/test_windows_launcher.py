import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest


pytestmark = pytest.mark.skipif(sys.platform != "win32", reason="Windows launcher")
ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture
def launcher(tmp_path):
    root = tmp_path / "中文 space & launcher"
    (root / "desktop").mkdir(parents=True)
    (root / "tools").mkdir()
    shutil.copy(ROOT / "start_mmhand.cmd", root)
    shutil.copy(ROOT / "tools/create_desktop_shortcut.ps1", root / "tools")
    (root / "desktop/__init__.py").write_text("", encoding="utf-8")
    (root / "desktop/__main__.py").write_text(
        "import json, os, sys\n"
        "print(json.dumps({'args': sys.argv[1:], 'cwd': os.getcwd(), "
        "'setup': os.environ.get('LAUNCHER_SETUP')}))\n",
        encoding="utf-8",
    )
    env = dict(os.environ, MMHAND_PYTHON=sys.executable, MMHAND_NO_PAUSE="1")
    env.pop("MMHAND_ROS_SETUP", None)
    env.pop("PYTHONPATH", None)
    return root, env


def run_cmd(command, root, env):
    return subprocess.run(
        f'"{os.environ["ComSpec"]}" /d /v:off /s /c "{command}"',
        cwd=root.parent, env=env, capture_output=True, timeout=30,
    )


def test_launcher_paths_setup_and_argument_forwarding(launcher):
    root, env = launcher
    setup = root / "ros setup.cmd"
    setup.write_text("@set LAUNCHER_SETUP=ready\n@exit /b 0\n", encoding="ascii")
    env["MMHAND_ROS_SETUP"] = str(setup)
    result = run_cmd(f'"{root / "start_mmhand.cmd"}" --no-ros2 --port "COM 7"', root, env)
    assert result.returncode == 0, result.stdout + result.stderr
    data = json.loads(result.stdout)
    assert data == {"args": ["--no-ros2", "--port", "COM 7"],
                    "cwd": str(root), "setup": "ready"}


@pytest.mark.parametrize("source", ["py", "python", "none"])
def test_double_click_without_virtualenv_or_configuration(launcher, source):
    root, env = launcher
    env.pop("MMHAND_PYTHON", None)
    env.pop("VIRTUAL_ENV", None)
    # Isolate discovery from installed Python versions and Windows Store aliases.
    bin_dir = root / "bin"
    bin_dir.mkdir()
    env["PATH"] = str(bin_dir)
    if source != "none":
        (bin_dir / f"{source}.cmd").write_text(
            f'@echo {sys.executable}\n', encoding="ascii",
        )
    result = run_cmd(f'"{root / "start_mmhand.cmd"}"', root, env)
    if source == "none":
        assert result.returncode != 0
        assert b"Python not found" in result.stdout
    else:
        assert result.returncode == 0, result.stdout + result.stderr
        assert json.loads(result.stdout)["args"] == []


@pytest.mark.parametrize("failure", ["python", "setup_missing", "setup_failed", "app_failed"])
def test_launcher_reports_failure(launcher, failure):
    root, env = launcher
    if failure == "python":
        env["MMHAND_PYTHON"] = str(root / "missing.exe")
    elif failure == "setup_missing":
        env["MMHAND_ROS_SETUP"] = str(root / "missing.cmd")
    elif failure == "setup_failed":
        setup = root / "bad.cmd"
        setup.write_text("@exit /b 2\n", encoding="ascii")
        env["MMHAND_ROS_SETUP"] = str(setup)
    else:
        (root / "desktop/__main__.py").write_text("raise RuntimeError('test failure')\n")
    result = run_cmd(f'"{root / "start_mmhand.cmd"}"', root, env)
    assert result.returncode != 0
    assert b"MMHand startup or execution failed" in result.stdout


@pytest.mark.parametrize("with_setup", [False, True])
def test_generated_shortcut_runs_same_entry(launcher, with_setup):
    root, env = launcher
    link = root / "MMHand.lnk"
    setup = root / "ros setup.cmd"
    setup.write_text("@set LAUNCHER_SETUP=ready\n@exit /b 0\n", encoding="ascii")
    result = subprocess.run([
        "powershell.exe", "-NoProfile", "-ExecutionPolicy", "Bypass", "-File",
        str(root / "tools/create_desktop_shortcut.ps1"), "-PythonPath", sys.executable,
        "-ShortcutPath", str(link),
    ] + (["-RosSetup", str(setup)] if with_setup else []),
        env=env, capture_output=True, timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
    inspect = subprocess.run([
        "powershell.exe", "-NoProfile", "-Command",
        "[Console]::OutputEncoding = [Text.UTF8Encoding]::new(); "
        "$s=(New-Object -ComObject WScript.Shell).CreateShortcut($env:TEST_LINK); "
        "$s | Select-Object TargetPath,Arguments,WindowStyle | ConvertTo-Json -Compress",
    ], env=dict(env, TEST_LINK=str(link)), capture_output=True, timeout=30)
    shortcut = json.loads(inspect.stdout.decode("utf-8-sig"))
    assert shortcut["WindowStyle"] == 7
    launched = subprocess.run(
        f'"{shortcut["TargetPath"]}" {shortcut["Arguments"]}',
        env=env, cwd=root.parent, capture_output=True, timeout=30,
    )
    assert launched.returncode == 0, launched.stdout + launched.stderr
    assert json.loads(launched.stdout) == {
        "args": [], "cwd": str(root), "setup": "ready" if with_setup else None,
    }
