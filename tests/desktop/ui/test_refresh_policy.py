"""Refresh cadence, change detection, backpressure and reconnect policy."""
from desktop.ui import refresh_policy as policy
from desktop.ui.web import common


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


def test_quality_pending_reads_need_calibration():
    assert common.quality_name('PENDING') == 'NEED CALIBRATION'
    assert common.quality_name('READY') == 'READY'
    assert common.quality_name('OFFSET') == 'Magnetic Encoder or Joint Installation Warning'


def test_json_status_prefers_snapshot_field_and_falls_back_to_persistence():
    assert common.json_status({'json_status': 'Saved'}) == 'Saved'
    assert common.json_status({'json_status': 'Unknown', 'persistence': 'Saved'}) == 'Unknown'
    assert common.json_status({'persistence': 'Saved'}) == 'Saved'
    assert common.json_status({'persistence': 'Saved in JSON; not loaded in P4'}) == 'Saved'
    assert common.json_status({'persistence': 'RAM differs from saved JSON'}) == 'Saved'
    assert common.json_status({'persistence': 'RAM / JSON unknown'}) == 'Unknown'
    assert common.json_status({}) == 'Unknown'


def test_auto_span_prefers_snapshot_field_and_falls_back_to_result():
    assert common.auto_span({'auto_span': 123}) == 123
    assert common.auto_span({'result': {'autoSpanCounts': 456}}) == 456
    assert common.auto_span({'result': {'assemblyAutoSpanCounts': 789}}) == 789
    assert common.auto_span({'auto_span': None, 'result': {'autoSpanCounts': 5}}) == 5
    assert common.auto_span({}) is None


def test_control_activity_and_maintenance_gate_follow_snapshot_fields():
    idle = dict(communication='Online', busy=False, control_mode='IDLE', stop_latched=False)
    assert common.maintenance_allowed(idle)
    assert not common.control_active(idle)
    assert common.control_active(dict(idle, control_mode='FEEDBACK'))
    assert common.control_active(dict(idle, control_mode='DIRECT'))
    assert not common.maintenance_allowed(dict(idle, control_mode='FEEDBACK'))
    assert not common.maintenance_allowed(dict(idle, control_mode='DIRECT'))
    assert not common.maintenance_allowed(dict(idle, control_mode='UNKNOWN'))
    assert not common.maintenance_allowed(dict(idle, stop_latched=True))
    assert not common.maintenance_allowed(dict(idle, communication='Stale data'))
    assert not common.maintenance_allowed(dict(idle, busy=True))
    # Legacy fallbacks used when the new snapshot keys are absent.
    assert common.control_active({'device': 'Controlling'})
    assert common.control_active({'joints': [{'feedback_control_status': 'ENABLED'}]})
    assert common.control_active({'motors': [{'direct_control_status': 'ENABLED'}]})
    assert not common.control_active({'device': 'Ready', 'joints': [], 'motors': []})
    assert common.stop_latched({'stop_latched': True})
    assert common.stop_latched({'device': 'Stopped'})
    assert not common.stop_latched({'device': 'Ready', 'stop_latched': False})
    assert common.maintenance_allowed({'communication': 'Online', 'busy': False})
    assert not common.maintenance_allowed({'communication': 'Online', 'busy': False, 'device': 'Controlling'})
    assert not common.maintenance_allowed({'communication': 'Online', 'busy': False, 'device': 'Stopped'})


def test_calibration_alert_visible_only_on_its_origin_page():
    alert = {'title': 'Motor Calibration Failed', 'message': 'J03 failed', 'severity': 'negative'}
    operation = dict(operation_id='op-1', origin='Motor Calibration', calibration_alert=alert,
                     alert_acknowledged=False, finished_at=100.0)
    assert policy.ALERT_MAX_AGE_S == 15 * 60
    assert policy.calibration_alert_visible(operation, 'Motor Calibration', 110.0)
    assert not policy.calibration_alert_visible(operation, 'Joint Calibration', 110.0)


def test_calibration_alert_hidden_when_acknowledged_or_without_origin_or_alert():
    alert = {'title': 'Motor Calibration Failed', 'message': 'J03 failed'}
    base = dict(origin='Motor Calibration', calibration_alert=alert, finished_at=100.0)
    assert not policy.calibration_alert_visible(dict(base, alert_acknowledged=True), 'Motor Calibration', 110.0)
    assert not policy.calibration_alert_visible(dict(base, origin=None), 'Motor Calibration', 110.0)
    assert not policy.calibration_alert_visible(dict(base, origin=''), 'Motor Calibration', 110.0)
    assert not policy.calibration_alert_visible(dict(base, calibration_alert=None), 'Motor Calibration', 110.0)
    assert not policy.calibration_alert_visible({'origin': 'Motor Calibration'}, 'Motor Calibration', 110.0)
    assert not policy.calibration_alert_visible(None, 'Motor Calibration', 110.0)


def test_calibration_alert_expires_after_fifteen_minutes():
    alert = {'title': 'Motor Calibration Failed', 'message': 'J03 failed'}
    operation = dict(origin='Motor Calibration', calibration_alert=alert, finished_at=100.0)
    assert policy.calibration_alert_visible(operation, 'Motor Calibration', 100.0 + policy.ALERT_MAX_AGE_S)
    assert not policy.calibration_alert_visible(operation, 'Motor Calibration', 100.1 + policy.ALERT_MAX_AGE_S)
    assert not policy.calibration_alert_visible(operation, 'Motor Calibration', 110.0, max_age_s=5.0)
    # A missing finished_at timestamp keeps a current alert visible.
    assert policy.calibration_alert_visible(dict(operation, finished_at=None), 'Motor Calibration', 10 ** 6)


def test_calibration_request_passes_origin_only_when_backend_accepts_it():
    from unittest.mock import Mock

    class Backend:
        def request_calibration(self, origin='Motor Calibration'):
            return origin

    class Legacy:
        def request_calibration(self):
            return 'legacy'

    assert common.request_calibration(Backend(), 'Motor Calibration') == 'Motor Calibration'
    assert common.request_calibration(Legacy(), 'Motor Calibration') == 'legacy'
    mocked = Mock()
    result = common.request_calibration(mocked, 'Motor Calibration')
    assert result is mocked.request_calibration.return_value
    mocked.request_calibration.assert_called_once_with(origin='Motor Calibration')
