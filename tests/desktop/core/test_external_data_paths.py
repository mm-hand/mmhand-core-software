"""Runtime data stays outside the repository; default calibration stays in repo config."""
import os
from pathlib import Path
import subprocess
import sys


def test_single_environment_override_controls_runtime_paths(tmp_path):
    env = {**os.environ, 'MMHAND_DATA_DIR': str(tmp_path), 'MMHAND_LOG_DIR': str(tmp_path / 'logs')}
    code = "from desktop.paths import DATA_DIR, RUN_DATA_DIR, LOG_DIR, CALIB_ACTIVE_SNAPSHOT_PATH; print(*[str(p) for p in (DATA_DIR, RUN_DATA_DIR, LOG_DIR, CALIB_ACTIVE_SNAPSHOT_PATH)], sep='\\n')"
    paths = subprocess.check_output([sys.executable, '-c', code], env=env, text=True).splitlines()
    assert len(paths) == 4
    assert all(Path(p).is_relative_to(tmp_path) for p in paths)
    assert Path(paths[1]) == Path(paths[2]) == tmp_path / 'logs'
    assert Path(paths[3]) == tmp_path / 'calibration_bundle.mmcal.json'


def test_active_calibration_snapshot_lives_in_data_dir_and_display_is_relative():
    from desktop.paths import CALIB_ACTIVE_SNAPSHOT_PATH, CONFIG_DIR, DATA_DIR, relative_display
    assert CALIB_ACTIVE_SNAPSHOT_PATH == DATA_DIR / 'calibration_bundle.mmcal.json'
    assert relative_display(CONFIG_DIR) == 'desktop/resources/config'
    assert not Path(relative_display(CONFIG_DIR)).is_absolute()


def test_default_data_dir_is_repo_config():
    env = dict(os.environ); env.pop('MMHAND_DATA_DIR', None)
    output = subprocess.check_output([sys.executable, '-c', 'from desktop.paths import DATA_DIR; print(DATA_DIR)'], env=env, text=True)
    from desktop.paths import CONFIG_DIR
    assert Path(output.strip()) == CONFIG_DIR
