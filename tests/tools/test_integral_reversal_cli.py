from pathlib import Path
from types import SimpleNamespace

import pytest

from tools.debug.integral_reversal_autotune import main, run_dry


def test_dry_run_produces_parameters_and_report(tmp_path):
    repo = Path(__file__).resolve().parents[2]
    output = tmp_path / 'integral-reversal'
    args = SimpleNamespace(
        dry_run=True,
        hardware=False,
        replay=None,
        calibration=repo / 'tests/fixtures/calibration_bundle.mmcal.json',
        output=output,
        port='COM12',
        max_minutes=60.0,
        base_parameters=None,
    )
    assert run_dry(args) == 0
    assert (output / 'report.md').is_file()
    assert (output / 'final_parameters.json').is_file()
    assert (output / 'trial_index.json').is_file()
    report = (output / 'report.md').read_text(encoding='utf-8')
    assert '## Integral trials' in report
    assert '## Detector (offline)' in report
    assert '## Takeup/exit trials' in report


def test_main_requires_exactly_one_mode(tmp_path):
    with pytest.raises(SystemExit):
        main(['--dry-run', '--hardware', '--output', str(tmp_path / 'x')])
    with pytest.raises(SystemExit):
        main(['--output', str(tmp_path / 'x')])
    with pytest.raises(SystemExit):
        main(['--dry-run', '--calibration', 'missing.json',
              '--output', str(tmp_path / 'x'), '--max-minutes', '0'])
