"""Native component and interaction regression; no serial devices or Qt runtime."""
from unittest.mock import Mock
import pytest
from nicegui import ui, Client
from desktop.core.hand_backend import HandBackend
from desktop.ui.web import joint_calibration, motor_calibration, feedback_control, direct_control
from desktop.ui.web import replay, plots, tactile, diagnostics
from desktop.ui.browser_view import build_page, TABS

@pytest.fixture
def page():
    hand = HandBackend()
    client = Client(ui.page('/test-console'))
    with client:
        yield hand, client
    client.delete()
    hand.close_application()

@pytest.mark.parametrize('name', TABS)
def test_every_native_page_builds_and_refreshes_unknown_feedback(page, name):
    hand, client = page
    act, load, confirm = Mock(), Mock(), Mock()
    builders = {
        TABS[0]: lambda: joint_calibration.build(hand, act, load, confirm),
        TABS[1]: lambda: motor_calibration.build(hand, act, load),
        TABS[2]: lambda: feedback_control.build(hand, act),
        TABS[3]: lambda: direct_control.build(hand, act, confirm),
        TABS[4]: lambda: replay.build(hand, act), TABS[5]: lambda: plots.build(hand, act),
        TABS[6]: lambda: tactile.build(hand),
        TABS[7]: lambda: diagnostics.build(hand, act)}
    refresh = builders[name]()
    snapshot = hand.application_snapshot()
    for _ in range(3): refresh(snapshot)
    act.assert_not_called()
    assert client.elements

def test_feedback_refresh_keeps_native_number_and_slider_draft_without_send(page):
    hand, client = page
    act = Mock()
    refresh = feedback_control.build(hand, act)
    number = next(e for e in client.elements.values() if isinstance(e, ui.number))
    number.set_value(15)
    for _ in range(3): refresh(hand.application_snapshot())
    assert number.value == 15
    act.assert_not_called()
    send = next(e for e in client.elements.values() if isinstance(e, ui.button) and e.text == 'Send')
    click = next(e for e in send._event_listeners.values() if e.type == 'click')
    click.handler(None)
    assert act.call_count == 1

def test_joint_page_has_no_manual_save_buttons_or_progress_panels(page):
    hand, client = page
    joint_calibration.build(hand, Mock(), Mock(), Mock())
    labels = [e.text for e in client.elements.values() if isinstance(e, ui.button)]
    assert labels.count('Set Min') == labels.count('Set Max') == 21
    assert 'Load Joint & Motor Calibration' in labels
    assert all(word not in ' '.join(labels) for word in ('Retry', 'Write Modified', 'Validate', 'Install', 'Read back'))

def test_shell_has_one_stop_default_recording_and_exact_navigation(page):
    hand, client = page
    build_page(hand)
    tabs = [e._props['name'] for e in client.elements.values() if isinstance(e, ui.tab)]
    buttons = [e for e in client.elements.values() if isinstance(e, ui.button)]
    assert tabs == TABS == ["Joint Calibration", "Motor Calibration", "Feedback Control", "Direct Control", "Replay", "Plots & Recordings", "Tactile", "Diagnostics"]
    assert sum('STOP' in e.text for e in buttons) == 1
    assert next(e for e in buttons if 'STOP' in e.text).enabled
    assert 'Save Data: ON' in [e.text for e in buttons]
    assert len([e for e in client.elements.values() if isinstance(e, ui.log)]) == 1

def test_tactile_summary_is_one_force_cell_and_full_uses_real_taxels(page):
    import numpy as np
    import time
    from desktop.core.data_models import FingerTactile, TactileSensor
    hand, client = page
    state = hand._controller.current_state
    state.tactile_data = [FingerTactile(sensors=[TactileSensor(np.zeros((5, 5, 3)), np.zeros(3))])]
    state.tactile_module_last_rx_ts[0] = time.time()
    state.tactile_module_valid_mask = 1
    state.tactile_data_level = 'SUMMARY'
    refresh = tactile.build(hand)
    refresh(hand.application_snapshot())
    chart = next(e for e in client.elements.values() if isinstance(e, ui.echart))
    assert chart.options['series'][0]['data'] == [[0, 0, 0]]
    state.tactile_data_level = 'FULL'
    refresh(hand.application_snapshot())
    assert len(chart.options['series'][0]['data']) == 25
    state.tactile_module_last_rx_ts[0] -= 1
    refresh(hand.application_snapshot())
    assert chart.options['series'][0]['data'] == []
