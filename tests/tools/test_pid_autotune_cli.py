from pathlib import Path
from types import SimpleNamespace

from tools.debug.pid_autotune import run_dry


def test_dry_run_produces_report_and_scores(tmp_path):
    repo = Path(__file__).resolve().parents[2]
    args = SimpleNamespace(
        calibration=repo / 'tests/fixtures/calibration_bundle.mmcal.json',
        output=tmp_path / 'autotune',
        max_trials=2,
    )
    assert run_dry(args) == 0
    report = args.output / 'report.md'
    assert report.is_file()
    assert '| type |' in report.read_text(encoding='utf-8')
    rows = __import__('json').loads((args.output / 'trial_index.json').read_text(encoding='utf-8'))
    assert len(rows) == 8
    assert all('score' in row and 'valid' in row for row in rows)
