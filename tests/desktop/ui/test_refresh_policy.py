"""Refresh cadence, change detection, backpressure and reconnect policy."""
from desktop.ui import refresh_policy as policy


def test_interval_split_keeps_fast_ui_and_slow_charts_apart():
    assert policy.FAST_INTERVAL_S == 0.1
    assert policy.HEAVY_INTERVAL_S == 0.5
    assert policy.PLOTS_INTERVAL_S == 1.0
    assert policy.select_intervals('Joint Calibration') == (0.1, 0.5, None)
    assert policy.select_intervals('Replay') == (0.1, 0.5, None)
    assert policy.select_intervals('Curves & Recordings') == (0.1, 0.5, 1.0)
    assert policy.select_intervals('Tactile') == (0.1, 0.5, 1.0)


def test_only_curve_page_fetches_curve_history():
    assert policy.needs_curves('Curves & Recordings')
    assert not policy.needs_curves('Tactile')
    assert not policy.needs_curves('Joint Calibration')


def test_leaf_resolution_follows_main_and_sub_tabs():
    groups = ['Calibration', 'Replay', 'Settings']
    children = {'Calibration': ['Joint Calibration', 'Motor Calibration'],
                'Settings': ['Renumber Motor', 'Diagnostics']}
    assert policy.active_leaf('Calibration', 'Motor Calibration', groups, children) == 'Motor Calibration'
    assert policy.active_leaf('Calibration', None, groups, children) == 'Joint Calibration'
    assert policy.active_leaf('Calibration', 'Unknown', groups, children) == 'Joint Calibration'
    assert policy.active_leaf('Replay', None, groups, children) == 'Replay'
    assert policy.active_leaf('Settings', 'Diagnostics', groups, children) == 'Diagnostics'
    assert policy.active_leaf(None, None, groups, children) == 'Joint Calibration'
    assert policy.active_leaf('Missing', 'Diagnostics', groups, children) == 'Joint Calibration'


def test_plots_refresh_only_on_chart_leaves_and_at_most_once_per_second():
    assert not policy.should_refresh_plots('Joint Calibration', 'Joint Calibration', 10.0, 100.0)
    assert not policy.should_refresh_plots('Replay', 'Replay', None, 100.0)
    assert policy.should_refresh_plots('Curves & Recordings', None, None, 100.0)
    assert not policy.should_refresh_plots('Curves & Recordings', 'Curves & Recordings', 10.0, 10.99)
    assert policy.should_refresh_plots('Curves & Recordings', 'Curves & Recordings', 10.0, 11.0)
    assert not policy.should_refresh_plots('Tactile', 'Tactile', 10.0, 10.5)
    assert policy.should_refresh_plots('Tactile', 'Tactile', 10.0, 11.0)
    # Switching into a chart leaf refreshes immediately even inside the interval.
    assert policy.should_refresh_plots('Curves & Recordings', 'Tactile', 10.0, 10.2)


def test_change_cache_only_reports_real_changes():
    cache = {}
    assert policy.changed(cache, 'save', 'Save Data: ON')
    assert not policy.changed(cache, 'save', 'Save Data: ON')
    assert policy.changed(cache, 'save', 'Save Data: OFF')
    assert policy.changed(cache, 'age', None)
    assert not policy.changed(cache, 'age', None)
    assert policy.changed(cache, 'badge', 1)
    assert not policy.changed(cache, 'badge', 1)
    assert policy.text_changed('a', 'b')
    assert not policy.text_changed('a', 'a')
    assert not policy.text_changed(None, None)


def test_backpressure_skips_a_tick_while_the_previous_one_runs():
    running = {'fast': False}
    accepted = 0
    assert policy.backpressure_ok(running['fast'])
    running['fast'] = True
    accepted += 1
    for _ in range(4):
        assert not policy.backpressure_ok(running['fast'])
    running['fast'] = False
    assert policy.backpressure_ok(running['fast'])
    assert accepted == 1


def test_slow_tick_threshold_is_fifty_milliseconds():
    assert policy.SLOW_TICK_LIMIT_MS == 50.0
    assert not policy.slow_tick(0.0)
    assert not policy.slow_tick(50.0)
    assert policy.slow_tick(50.1)
    assert policy.slow_tick(400.0)
    assert not policy.slow_tick(20.0, limit_ms=30.0)
    assert policy.slow_tick(31.0, limit_ms=30.0)


def test_heartbeat_writes_are_throttled_below_the_stale_window():
    assert policy.HEARTBEAT_INTERVAL_S == 0.5
    assert policy.HEARTBEAT_POLL_MS == 500
    assert policy.HEARTBEAT_STALE_S == 3.0
    assert policy.heartbeat_due(None, 100.0)
    assert not policy.heartbeat_due(100.0, 100.4)
    assert policy.heartbeat_due(100.0, 100.5)
    assert not policy.heartbeat_stale(0.0)
    assert not policy.heartbeat_stale(3.0)
    assert policy.heartbeat_stale(3.1)
    assert policy.heartbeat_stale(30.0)


def test_auto_reload_needs_visible_stale_page_and_budget():
    assert policy.AUTO_RELOAD_DELAY_MS == 1000
    assert policy.MAX_AUTO_RELOADS == 2
    assert not policy.should_auto_reload(True, 1.0, 0)
    assert policy.should_auto_reload(True, 3.5, 0)
    assert policy.should_auto_reload(True, 3.5, 1)
    assert not policy.should_auto_reload(True, 3.5, 2)
    assert not policy.should_auto_reload(False, 10.0, 0)
    assert not policy.should_auto_reload(False, 10.0, 1)
    assert not policy.should_auto_reload(False, 10.0, 5)
    assert policy.reload_budget_exhausted(2)
    assert not policy.reload_budget_exhausted(1)
    assert policy.reload_budget_exhausted(3, max_reloads=3)


def test_injected_script_uses_policy_thresholds_and_heartbeat_class():
    from desktop.ui.browser_view import self_heal_script
    script = self_heal_script()
    assert '__' not in script
    assert policy.HEARTBEAT_LABEL_CLASS in script
    assert 'document.hidden' in script
    assert 'visibilitychange' in script
    assert 'location.reload' in script
    assert str(int(policy.HEARTBEAT_STALE_S * 1000)) in script
    assert str(policy.MAX_AUTO_RELOADS) in script
    assert str(policy.AUTO_RELOAD_DELAY_MS) in script
    assert str(policy.HEARTBEAT_POLL_MS) in script


def test_reconnect_script_is_executed_not_rendered_as_body_text():
    from pathlib import Path
    source = Path(__file__).resolve().parents[3] / 'desktop/ui/browser_view.py'
    text = source.read_text(encoding='utf-8')
    assert 'ui.run_javascript(self_heal_script())' in text
    assert 'ui.add_body_html(self_heal_script' not in text
