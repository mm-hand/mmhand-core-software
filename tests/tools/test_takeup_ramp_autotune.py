import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from desktop.core.control_parameters import DEFAULT_PATH, validate
from tools.debug import takeup_ramp_autotune as ramp
from tools.debug import ramp_finalize as finalize
from tools.debug.integral_reversal_autotune import index_aa_clear


def _defaults():
    return validate(json.loads(DEFAULT_PATH.read_text(encoding='utf-8')))


def test_shipped_defaults_match_validated_reversal_config():
    defaults = _defaults()
    assert defaults['dual_tendon']['pair_recovery_gain'] == 0.3
    assert defaults['joint_kp']['cmc'] == 1.3
    assert defaults['joint_kd']['cmc'] == 0.08
    for name, b0, vb in (('aa', 2, 50), ('mcp', 2, 100),
                         ('pip', 2, 100), ('dip', 5, 100),
                         ('cmc', 2, 50)):
        assert defaults['reversal'][f'takeup_max_{name}'] == 40
        assert defaults['reversal_ramp'][f'b0_{name}'] == b0
        assert defaults['reversal_ramp'][f'vb_{name}'] == vb
        assert defaults['reversal_ramp'][f'load_limit_{name}'] == 900
    assert defaults['reversal']['confirm_deg_cmc'] == 0.6


def test_feasible_accepts_validated_aa_and_rejects_bad_boundaries():
    aa = dict(valid=True, exit_rate=1.0, engage=0.1012, peak_load=744)
    assert ramp._feasible(aa) is True
    assert ramp._feasible(dict(aa, engage=0.049)) is False
    assert ramp._feasible(dict(aa, engage=0.05)) is True
    assert ramp._feasible(dict(aa, exit_rate=0.79)) is False
    assert ramp._feasible(dict(aa, peak_load=900)) is False
    assert ramp._feasible(dict(aa, valid=False)) is False


def test_make_safe_freezes_validated_gains_and_zeroes_compensation():
    safe = ramp._make_safe(_defaults())
    validate(safe)
    assert safe['joint_kp'] == ramp.EXPECTED_KP
    assert safe['joint_kd'] == ramp.EXPECTED_KD
    assert all(safe['joint_ki'][name] == 0 for name in ramp.ALL_TYPES)
    assert safe['dual_tendon']['pair_recovery_gain'] == 0
    for name in ramp.ALL_TYPES:
        assert safe['reversal'][f'takeup_max_{name}'] == 0
        assert safe['reversal'][f'exit_travel_deg_{name}'] == ramp.EXIT_TRAVEL_DEG
        assert safe['reversal'][f'confirm_deg_{name}'] == ramp.EXPECTED_CONFIRM_DEG[name]
        assert safe['reversal_ramp'][f'b0_{name}'] == 0
        assert safe['reversal_ramp'][f'vb_{name}'] == 0
        assert safe['reversal_ramp'][f'load_limit_{name}'] == ramp.DEFAULT_LOAD_LIMIT
    assert safe['reversal']['confirm_ms'] == ramp.CONFIRM_MS
    assert safe['reversal_ramp']['takeup_timeout_ms'] == ramp.TIMEOUT_MS
    assert safe['joint_ki']['integral_limit_counts_mcp'] == 100


def test_make_safe_confirm_override_applies_only_to_requested_types():
    safe = ramp._make_safe(_defaults(), confirm_deg=0.3, types=('mcp',))
    assert safe['reversal']['confirm_deg_mcp'] == 0.3
    assert safe['reversal']['confirm_deg_aa'] == ramp.EXPECTED_CONFIRM_DEG['aa']
    assert safe['reversal']['confirm_deg_dip'] == ramp.EXPECTED_CONFIRM_DEG['dip']


def test_index_aa_clear_enforces_angle_only_when_readable():
    def state(angle, error):
        encoders = [SimpleNamespace(current_angle_deg=0.0, angle_error=True, raw=0)
                    for _ in range(21)]
        encoders[12] = SimpleNamespace(current_angle_deg=angle, angle_error=error,
                                       raw=0 if error else 1500)
        return SimpleNamespace(encoders=encoders)

    assert index_aa_clear(state(25.0, False)) is True
    assert index_aa_clear(state(10.0, False)) is False
    # Released/invalidated index root: unreadable means "operator must confirm".
    assert index_aa_clear(state(0.0, True)) is False
    assert index_aa_clear(state(0.0, True), assume=True) is True


def test_metrics_peak_load_ignores_held_proximal_motors():
    defaults = _defaults()
    proposal = ramp._proposal(ramp._make_safe(defaults), 'mcp', 80.0, 5.0, 100.0, 900.0, 600.0)
    params = ramp._analysis_params(proposal, 'mcp', 0.0)
    samples = []
    for index in range(4):
        samples.append(dict(
            t=(index + 1) * 10.0, encoder_ms=(index + 1) * 10,
            target=10.0, actual=10.0, slow=10.0,
            i_counts=0.0, reversal_counts=5 if index == 1 else 0, saturated=False,
            segment='lead_in', load={'12': 300, '13': 999},
            motor_command_abs=[0] * 27, motor_actual_abs=[0] * 27))
    metrics = ramp._metrics(samples, [], [12], params)
    assert metrics['peak_load'] == 300


def test_compose_final_merges_only_deployed_types_and_can_exclude():
    base = _defaults()
    results = {
        'mcp': dict(deployed=True, takeup_max=80, b0=5, vb=100,
                    load_limit=900, confirm_deg=1.0),
        'pip': dict(deployed=False, takeup_max=0, reason='load'),
    }
    final = finalize.compose_final(base, results)
    validate(final)
    assert final['reversal']['takeup_max_mcp'] == 80
    assert final['reversal_ramp']['vb_mcp'] == 100
    assert final['reversal']['takeup_max_pip'] == 0
    assert final['reversal_ramp']['b0_pip'] == 0
    # Untested types keep the shipped defaults (AA is validated at 40/2/50).
    assert final['reversal']['takeup_max_aa'] == 40
    assert final['reversal_ramp']['b0_aa'] == 2
    excluded = finalize.compose_final(base, results, exclude=('mcp',))
    assert excluded['reversal']['takeup_max_mcp'] == 0
    assert excluded['reversal_ramp']['vb_mcp'] == 0


def test_compose_final_rejects_inconsistent_bias():
    base = _defaults()
    results = {'mcp': dict(deployed=True, takeup_max=40, b0=50, vb=100,
                           load_limit=900, confirm_deg=1.0)}
    with pytest.raises(RuntimeError):
        finalize.compose_final(base, results)


def test_cli_requires_one_mode_and_rejects_bad_ladder():
    with pytest.raises(SystemExit):
        ramp.main(['--hardware', '--verify', '--calibration', 'missing.json',
                   '--output', 'unused'])
    with pytest.raises(SystemExit):
        ramp.main(['--hardware', '--calibration', 'missing.json',
                   '--output', 'unused', '--ladder', 'abc'])
    with pytest.raises(SystemExit):
        ramp.main(['--verify', '--calibration', 'missing.json',
                   '--output', 'unused', '--ladder', '-5'])


def test_results_loader_rejects_duplicate_types(tmp_path):
    one = tmp_path / 'one.json'
    two = tmp_path / 'two.json'
    payload = {'final': {'mcp': {'deployed': True}}}
    one.write_text(json.dumps(payload), encoding='utf-8')
    two.write_text(json.dumps(payload), encoding='utf-8')
    with pytest.raises(RuntimeError):
        finalize.load_results([one, two])
    missing = tmp_path / 'missing.json'
    with pytest.raises(RuntimeError):
        finalize.load_results([missing])
