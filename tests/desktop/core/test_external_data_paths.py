"""Runtime data stays outside the repository; default calibration stays in repo config."""
import os
from pathlib import Path
import subprocess
import sys


def test_single_environment_override_controls_runtime_paths(tmp_path):
    env = {**os.environ, 'MMHAND_DATA_DIR': str(tmp_path)}
    code = "from desktop.paths import DATA_DIR, RUN_DATA_DIR, LOG_DIR, MULTITURN_RUNTIME_PATH; print(*[str(p) for p in (DATA_DIR, RUN_DATA_DIR, LOG_DIR, MULTITURN_RUNTIME_PATH)], sep='\\n')"
    paths = subprocess.check_output([sys.executable, '-c', code], env=env, text=True).splitlines()
    assert len(paths) == 4
    assert all(Path(p).is_relative_to(tmp_path) for p in paths)
    assert not (tmp_path / 'calibration/calibration_bundle.mmcal.json').exists()


def test_default_calibration_snapshot_is_repo_config_and_display_is_relative():
    from desktop.paths import CALIB_LOAD_SNAPSHOT_PATH, CONFIG_DIR, relative_display
    assert CALIB_LOAD_SNAPSHOT_PATH == CONFIG_DIR / 'calibration_bundle.mmcal.json'
    assert relative_display(CONFIG_DIR) == 'desktop/resources/config'
    assert not Path(relative_display(CONFIG_DIR)).is_absolute()


def test_default_directory_is_independent_of_checkout():
    env = dict(os.environ); env.pop('MMHAND_DATA_DIR', None)
    output = subprocess.check_output([sys.executable, '-c', 'from desktop.paths import DATA_DIR; print(DATA_DIR)'], env=env, text=True)
    assert Path(output.strip()) == Path.home() / 'mmhand_data'
