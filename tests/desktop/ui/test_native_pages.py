"""Native component and interaction regression; no serial devices or Qt runtime."""
from unittest.mock import Mock
import pytest
from nicegui import ui, Client
from nicegui.events import GenericEventArguments
from desktop.core.hand_backend import HandBackend
from desktop.ui.web import joint_calibration, motor_calibration, feedback_control, direct_control
from desktop.ui.web import replay, plots, tactile, diagnostics
from desktop.ui.web.common import FINGERS, selection
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


def test_group_selection_refresh_updates_partial_state_without_selection_commands(page, monkeypatch):
    hand, client = page
    refresh = selection(hand, 'selected_motors', [0, 1], 'Motors')
    box = next(e for e in client.elements.values() if isinstance(e, ui.checkbox))
    update = Mock()
    selected = Mock()
    monkeypatch.setattr(box, 'update', update)
    monkeypatch.setattr(hand, 'select_motors', selected)
    refresh({'selected_motors': []})
    update.reset_mock()
    refresh({'selected_motors': [0]})
    assert box.value is False and box._props['indeterminate'] is True
    update.assert_called_once()
    refresh({'selected_motors': [0, 1]})
    assert box.value is True and box._props['indeterminate'] is False
    refresh({'selected_motors': []})
    assert box.value is False and box._props['indeterminate'] is False
    selected.assert_not_called()


def slider_controls(page, direct):
    hand, client = page
    request = Mock()
    setattr(hand, 'request_motors' if direct else 'request_targets', request)
    act = Mock(side_effect=lambda callback: callback())
    refresh = direct_control.build(hand, act, Mock()) if direct else feedback_control.build(hand, act)
    snapshot = hand.application_snapshot()
    index = next(row['motor'] for row in snapshot['motors'] if row['joint'] in FINGERS[0][1]) if direct else FINGERS[0][1][0]
    if direct:
        snapshot['motors'][index].update(abs=125, limits=(-30719, 30719), online=True)
    refresh(snapshot)
    number = next(e for e in client.elements.values() if isinstance(e, ui.number))
    slider = next(e for e in client.elements.values() if isinstance(e, ui.slider))
    return refresh, snapshot, number, slider, index, request, act


def slider_event(slider, event_type, value):
    event = next(e for e in slider._event_listeners.values() if e.type == event_type)
    event.handler(GenericEventArguments(sender=slider, client=slider.client, args=value))


@pytest.mark.parametrize('direct', [False, True])
def test_slider_drag_only_updates_draft_and_release_sends_final_value_once(page, direct):
    refresh, snapshot, number, slider, index, request, act = slider_controls(page, direct)
    lo, hi = slider._props['min'], slider._props['max']
    for value in (lo, (lo + hi) / 2):
        slider_event(slider, 'update:modelValue', value)
        assert number.value == value
    request.assert_not_called()
    act.assert_not_called()
    # Release must use its payload even when a throttled drag value is older.
    slider_event(slider, 'change', hi)
    request.assert_called_once_with({index: hi})
    assert number.value == slider.value == hi
    refresh(snapshot)
    request.assert_called_once()


@pytest.mark.parametrize('direct', [False, True])
def test_slider_drag_has_no_server_echo_or_polling_binding(page, direct, monkeypatch):
    from nicegui import binding
    refresh, snapshot, number, slider, index, request, act = slider_controls(page, direct)
    assert not slider.LOOPBACK and not slider._props['loopback']
    assert not any(source is number or source is slider or target is number or target is slider
                   for source, _, target, _, _ in binding.active_links)
    update = Mock()
    monkeypatch.setattr(slider, 'update', update)
    for value in range(100):
        slider_event(slider, 'update:modelValue', value)
    assert number.value == slider.value == 99
    update.assert_not_called()
    act.assert_not_called()


@pytest.mark.parametrize('direct', [False, True])
def test_number_edit_and_clear_never_submit_slider_motion(page, direct):
    refresh, snapshot, number, slider, index, request, act = slider_controls(page, direct)
    value = (slider._props['min'] + slider._props['max']) / 2
    number.set_value(value)
    assert slider.value == value
    number.set_value(None)
    assert slider.value == value
    refresh(snapshot)
    assert number.value is None
    request.assert_not_called()
    act.assert_not_called()


def direct_controls(page):
    hand, client = page
    action = Mock()
    hand.request_direct_zero = action
    act = Mock(side_effect=lambda callback: callback())
    refresh = direct_control.build(hand, act, Mock())
    snapshot = hand.application_snapshot()
    order = [row['motor'] for _, joints in FINGERS for row in snapshot['motors'] if row['joint'] in joints]
    numbers = [e for e in client.elements.values() if isinstance(e, ui.number)]
    selected = next(e for e in client.elements.values() if isinstance(e, ui.button) and e.text == 'Reset Motor Multi-turn Position')
    return refresh, snapshot, dict(zip(order, numbers)), selected, action, act


def click(button):
    next(event for event in button._event_listeners.values() if event.type == 'click').handler(None)


def test_direct_zero_uses_backend_motor_selection_without_per_motor_buttons(page):
    refresh, snapshot, numbers, selected, action, act = direct_controls(page)
    hand, client = page
    assert not selected.enabled
    assert not any(isinstance(e, ui.button) and e.text == 'Set ABS Zero' for e in client.elements.values())
    snapshot.update(communication='Online', busy=False, selected_motors=[9, 10])
    for motor in (9, 10):
        snapshot['motors'][motor]['online'] = True
    refresh(snapshot)
    assert selected.enabled
    click(selected)
    action.assert_called_once_with()


@pytest.mark.parametrize('communication,busy,online', [('Disconnected', False, True), ('Stale data', False, True), ('Online', True, True), ('Online', False, False)])
def test_direct_zero_is_disabled_without_idle_fresh_online_motor(page, communication, busy, online):
    refresh, snapshot, numbers, selected, action, act = direct_controls(page)
    snapshot.update(communication=communication, busy=busy, selected_motors=[9])
    snapshot['motors'][9]['online'] = online
    refresh(snapshot)
    assert not selected.enabled
    action.assert_not_called()


def test_direct_zero_requires_nonempty_selection_and_every_selected_motor_online(page):
    refresh, snapshot, numbers, selected, action, act = direct_controls(page)
    snapshot.update(communication='Online', busy=False, selected_motors=[])
    refresh(snapshot)
    assert not selected.enabled
    snapshot['selected_motors'] = [9, 10]
    snapshot['motors'][9]['online'] = True
    snapshot['motors'][10]['online'] = False
    refresh(snapshot)
    assert not selected.enabled
    action.assert_not_called()


def test_direct_draft_resets_only_after_confirmed_zero_generation(page):
    refresh, snapshot, numbers, selected, action, act = direct_controls(page)
    snapshot.update(communication='Online', busy=False)
    row = snapshot['motors'][9]
    row.update(online=True, abs=125, limits=(-30719, 30719), abs_zero_generation=0)
    refresh(snapshot)
    numbers[9].set_value(777)
    row['abs'] = 0
    refresh(snapshot)
    assert numbers[9].value == 777
    row['abs_zero_generation'] = 1
    refresh(snapshot)
    assert numbers[9].value == 0
    numbers[9].set_value(66)
    refresh(snapshot)
    assert numbers[9].value == 66
    action.assert_not_called()

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
    navigation = next(e for e in client.elements.values() if isinstance(e, ui.tabs))
    panels = next(e for e in client.elements.values() if isinstance(e, ui.tab_panels))
    toolbar_area = next(e for e in client.elements.values() if 'toolbar-area' in e.classes)
    toolbars = [e for e in client.elements.values() if 'page-toolbar' in e.classes]
    assert len(toolbars) == len(TABS)
    assert toolbar_area.parent_slot is panels.parent_slot
    assert all(toolbar.parent_slot.parent is toolbar_area for toolbar in toolbars)
    for index, name in enumerate(TABS):
        navigation.set_value(name)
        assert [toolbar.visible for toolbar in toolbars] == [i == index for i in range(len(TABS))]

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
