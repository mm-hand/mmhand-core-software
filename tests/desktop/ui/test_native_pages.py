"""Native component and interaction regression; no serial devices or GUI runtime."""
from unittest.mock import Mock
import pytest
from nicegui import ui, Client
from nicegui.events import GenericEventArguments
from desktop.core.hand_backend import HandBackend
from desktop.ui.web import renumber_motors, renumber_tactile, joint_calibration, motor_calibration, feedback_control, direct_control
from desktop.ui.web import replay, plots, tactile, diagnostics, control_parameters
from desktop.ui.web.common import FINGERS, selection
from desktop.ui.browser_view import build_page, TABS, SUBTABS


def test_unplugged_serial_choice_is_removed_but_active_identity_is_retained():
    from desktop.ui.browser_view import device_selector_state
    stopped = dict(connected=False, pending=False, port='COM12')
    assert device_selector_state({}, 'COM12', stopped) == ({}, None)
    assert device_selector_state({'COM14': 'S3'}, 'COM12', stopped) == ({'COM14': 'S3'}, None)
    active = dict(connected=True, pending=False, port='COM12')
    assert device_selector_state({}, 'COM12', active) == ({'COM12': 'COM12'}, 'COM12')

@pytest.fixture
def page():
    hand = HandBackend()
    client = Client(ui.page('/test-console'))
    with client:
        yield hand, client
    client.delete()
    hand.close_application()

@pytest.mark.parametrize('name', [*SUBTABS['Calibration'], *SUBTABS['Control'], 'Replay', *SUBTABS['Plot'], *SUBTABS['Settings']])
def test_every_native_page_builds_and_refreshes_unknown_feedback(page, name):
    hand, client = page
    act, load, confirm = Mock(), Mock(), Mock()
    builders = {
        'Renumber Motor': lambda: renumber_motors.build(hand, act),
        'Renumber Tactile': lambda: renumber_tactile.build(hand, act),
        'Joint Calibration': lambda: joint_calibration.build(hand, act, load, confirm),
        'Motor Calibration': lambda: motor_calibration.build(hand, act, load),
        'Feedback Control': lambda: feedback_control.build(hand, act),
        'Direct Control': lambda: direct_control.build(hand, act, confirm),
        'Replay': lambda: replay.build(hand, act), 'Curves & Recordings': lambda: plots.build(hand, act),
        'Tactile': lambda: tactile.build(hand),
        'Diagnostics': lambda: diagnostics.build(hand, act),
        'Control Parameters': lambda: control_parameters.build(hand, act)}
    refresh = builders[name]()
    snapshot = hand.application_snapshot()
    for _ in range(3): refresh(snapshot)
    act.assert_not_called()
    confirm.assert_not_called()
    assert client.elements

def test_adaptive_kp_is_one_expansion_after_kd_with_one_enable_button(page):
    hand, client = page
    hand.request_apply_control_parameters = Mock()
    act = Mock(side_effect=lambda callback: callback())
    refresh = control_parameters.build(hand, act)
    snapshot = idle_snapshot(hand)
    refresh(snapshot)
    expansions = [e.text for e in client.elements.values() if isinstance(e, ui.expansion)]
    assert expansions[expansions.index('Kd config') + 1] == 'Adaptive Kp (runtime multiplier)'
    assert sum('Adaptive' in text for text in expansions) == 1
    adaptive = button(client, 'Enable Adaptive Kp Control')
    snapshot['adaptive_kp'].update(valid=False)
    refresh(snapshot)
    assert any(text.startswith('Adaptive: waiting for P4 1 Hz packet')
               for text in label_texts(client))
    click(adaptive)
    proposed = hand.request_apply_control_parameters.call_args.args[0]
    assert proposed['adaptive_kp']['enabled'] == 1 and 'crc32' not in proposed
    snapshot['control_parameters']['adaptive_kp']['enabled'] = 1
    refresh(snapshot)
    assert button(client, 'Disable Adaptive Kp Control') is not None
    snapshot['busy'] = True
    refresh(snapshot)
    assert not button(client, 'Disable Adaptive Kp Control').enabled


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


def test_tactile_batch_editor_groups_all_fingers_and_submits_each_new_id(page):
    hand, client = page
    act = Mock()
    hand.request_tactile_renumber = Mock()
    refresh = renumber_tactile.build(hand, act)
    snapshot = hand.application_snapshot()
    snapshot['id_reset']['tactile'].update(available=True, scanned=True, generation=1,
        rows=[dict(bus=4,id=1,model=2,points=25),dict(bus=3,id=1,model=2,points=25)])
    refresh(snapshot)
    numbers = [element for element in client.elements.values() if isinstance(element, ui.number)]
    assert len(numbers) == 2 and all(number.value is None for number in numbers)
    numbers[0].set_value(2)
    numbers[1].set_value(0)
    labels = [element.text for element in client.elements.values() if isinstance(element, ui.label)]
    assert all(finger in labels for finger in ('Thumb','Index','Middle','Ring','Little'))
    assert any('same ID on one bus' in text for text in labels)
    rename = next(element for element in client.elements.values()
                  if isinstance(element, ui.button) and element.text == 'Renumber All')
    next(event for event in rename._event_listeners.values() if event.type == 'click').handler(None)
    act.call_args.args[0]()
    hand.request_tactile_renumber.assert_called_once_with({(4,1):2,(3,1):0})
    snapshot['id_reset']['tactile']['available'] = False
    refresh(snapshot)
    assert not rename.enabled and all(not number.enabled for number in numbers)


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


def click(button):
    next(event for event in button._event_listeners.values() if event.type == 'click').handler(None)


def test_direct_draft_resets_only_after_confirmed_zero_generation(page):
    hand, client = page
    refresh = direct_control.build(hand, Mock(), Mock())
    snapshot = hand.application_snapshot()
    motors = [row['motor'] for _, joints in FINGERS for row in snapshot['motors']
              if row['joint'] in joints]
    numbers = dict(zip(motors, [e for e in client.elements.values() if isinstance(e, ui.number)]))
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
    assert TABS == ['Calibration', 'Control', 'Replay', 'Plot', 'Settings']
    assert tabs == [*TABS, *SUBTABS['Calibration'], *SUBTABS['Control'], *SUBTABS['Plot'], *SUBTABS['Settings']]
    assert sum('STOP' in e.text for e in buttons) == 1
    # The top button turns into Clear Faults and only enables with fresh P4 status.
    assert not next(e for e in buttons if 'STOP' in e.text).enabled
    assert 'Save Data: ON' in [e.text for e in buttons]
    assert len([e for e in client.elements.values() if isinstance(e, ui.log)]) == 1
    navigation = next(e for e in client.elements.values() if isinstance(e, ui.tabs))
    assert navigation.value == 'Calibration'
    panels = next(e for e in client.elements.values() if isinstance(e, ui.tab_panels))
    toolbars = [e for e in client.elements.values() if 'page-toolbar' in e.classes]
    assert len(toolbars) == 11
    workspace = next(e for e in client.elements.values() if isinstance(e, ui.splitter))
    assert panels.parent_slot is workspace.before
    assert all(isinstance(toolbar.parent_slot.parent, ui.tab_panel) for toolbar in toolbars)
    assert {toolbar.parent_slot.parent._props['name'] for toolbar in toolbars} == {
        'Replay', *(name for names in SUBTABS.values() for name in names)}
    assert all(next(child for child in toolbar.parent_slot.children
                    if 'hidden' not in child.classes) is toolbar for toolbar in toolbars)
    settings_panel = next(e for e in client.elements.values()
                          if isinstance(e, ui.tab_panel) and e._props['name'] == 'Settings')
    settings_children = settings_panel.default_slot.children
    assert isinstance(settings_children[0], ui.tabs)
    assert isinstance(settings_children[1], ui.tab_panels)
    navigation.set_value('Control')
    navigation.set_value('Settings')
    assert all(isinstance(toolbar.parent_slot.parent, ui.tab_panel) for toolbar in toolbars)

def test_tactile_summary_is_one_force_cell_and_full_uses_real_taxels(page):
    import numpy as np
    import time
    from desktop.core.data_models import FingerTactile, TactileSensor
    from desktop.core.tactile_geometry import hand_svg
    hand, client = page
    state = hand._controller.current_state
    state.tactile_data = [FingerTactile(sensors=[TactileSensor(np.zeros((4, 13, 3)), np.zeros(3))])]
    state.tactile_module_last_rx_ts[0] = time.time()
    state.tactile_module_valid_mask = 1
    state.tactile_data_level = 'SUMMARY'
    refresh = tactile.build(hand)
    refresh(hand.application_snapshot())
    summary = hand_svg(hand.application_snapshot()['tactile']['modules'])
    assert summary.count('<circle ') == 1
    assert 'SUMMARY' in summary
    state.tactile_data_level = 'FULL'
    refresh(hand.application_snapshot())
    full = hand_svg(hand.application_snapshot()['tactile']['modules'])
    assert full.count('<circle ') == 52
    assert 'point 52' in full
    state.tactile_module_last_rx_ts[0] -= 1
    refresh(hand.application_snapshot())
    assert hand_svg(hand.application_snapshot()['tactile']['modules']).count('<circle ') == 0


def label_texts(client):
    return [element.text for element in client.elements.values()
            if isinstance(element, ui.label) and isinstance(element.text, str)]


def button(client, text):
    return next(element for element in client.elements.values()
                if isinstance(element, ui.button) and element.text == text)


def idle_snapshot(hand):
    """Snapshot with a fresh, idle P4 so maintenance buttons are allowed."""
    snapshot = hand.application_snapshot()
    snapshot.update(communication='Online', busy=False, control_mode='IDLE', stop_latched=False, device='Ready')
    return snapshot


def direct_zero_dialog(client):
    dialog = next(element for element in client.elements.values() if isinstance(element, ui.dialog))
    card = next(child for child in dialog.default_slot.children if isinstance(child, ui.card))
    row = next(child for child in card.default_slot.children if isinstance(child, ui.row))
    return dialog, {child.text: child for child in row.default_slot.children
                    if isinstance(child, ui.button)}


def click(button):
    next(listener for listener in button._event_listeners.values()
         if listener.type == 'click').handler(None)


def test_direct_control_prompt_blocks_the_page_until_zeroing(page):
    hand, client = page
    hand.request_direct_zero_all = Mock()
    leave = Mock()
    act = Mock(side_effect=lambda callback: callback())
    refresh = direct_control.build(hand, act, Mock(), leave)
    dialog, buttons = direct_zero_dialog(client)
    assert dialog._props.get('persistent')  # ESC / 遮罩都不能绕过
    assert not dialog.value
    snapshot = idle_snapshot(hand)
    snapshot['direct_zero_required'] = True
    refresh(snapshot)
    assert dialog.value
    # Cancel：关窗 + 回原页面，并且下次进来仍然先拦。
    click(buttons['Cancel'])
    assert not dialog.value
    leave.assert_called_once_with()
    refresh(snapshot)
    assert dialog.value
    # Confirm：发起全部归零，留在页面，不再重复弹。
    click(buttons['Set All Motors to ABS 0'])
    hand.request_direct_zero_all.assert_called_once_with()
    assert not dialog.value
    refresh(snapshot)
    assert not dialog.value


def zero_prompt_dialog(client):
    for element in client.elements.values():
        if not isinstance(element, ui.dialog):
            continue
        card = next((child for child in element.default_slot.children if isinstance(child, ui.card)), None)
        if card is None or not any(getattr(child, 'text', '') == direct_control.ZERO_PROMPT
                                  for child in card.default_slot.children):
            continue
        row = next(child for child in card.default_slot.children if isinstance(child, ui.row))
        return element, {child.text: child for child in row.default_slot.children
                         if isinstance(child, ui.button)}
    raise AssertionError('direct zero prompt was not built')


def test_cancelling_the_zero_prompt_returns_to_joint_calibration(page):
    from desktop.ui.browser_view import build_page
    hand, client = page
    build_page(hand)
    tabs = [element for element in client.elements.values() if isinstance(element, ui.tabs)]
    main = next(element for element in tabs if element.value == 'Calibration')
    calibration = next(element for element in tabs if element.value == 'Joint Calibration')
    control = next(element for element in tabs if element.value == 'Feedback Control')
    calibration.set_value('Motor Calibration')
    main.set_value('Control')            # 进入 Control 组后停在 Feedback Control
    control.set_value('Direct Control')
    snapshot = hand.application_snapshot(include_curves=False)
    snapshot['direct_zero_required'] = True
    hand.application_snapshot = lambda **_: snapshot
    heavy = next(element for element in client.elements.values() if isinstance(element, ui.timer)
                 and getattr(element.callback, '__qualname__', '') == 'build_page.<locals>.refresh_heavy')
    heavy.callback()
    dialog, buttons = zero_prompt_dialog(client)
    assert dialog.value
    click(buttons['Cancel'])
    assert not dialog.value
    # Cancel 固定回到 Joint Calibration，不能留在 Direct Control。
    assert main.value == 'Calibration' and calibration.value == 'Joint Calibration'


def test_direct_control_does_not_prompt_without_fresh_motor_state(page):
    hand, client = page
    refresh = direct_control.build(hand, Mock(), Mock())
    dialog, _buttons = direct_zero_dialog(client)
    for _ in range(3):
        refresh(hand.application_snapshot())
    assert not dialog.value


def test_joint_page_detail_is_manual_urdf_json_only_in_order(page):
    hand, client = page
    refresh = joint_calibration.build(hand, Mock(), Mock(), Mock())
    snapshot = idle_snapshot(hand)
    for row in snapshot['joints']:
        row.update(manual_min=-100, manual_max=200, span=300, urdf_span=310,
                   min=-5.5, max=10.5, json_status='Saved', raw=123)
    refresh(snapshot)
    details = [text for text in label_texts(client) if text.startswith('Manual Span:')]
    assert len(details) == 21
    lines = details[0].splitlines()
    assert len(lines) == 1
    assert lines[0].startswith('Manual Span:')
    for marker in ('URDF Span:', 'URDF Min (deg):', 'URDF Max (deg):', 'JSON: Saved'):
        assert marker in lines[0]
    assert 'Raw:' not in details[0]
    raw_labels = [element for element in client.elements.values()
                  if isinstance(element, ui.label) and (element.text or '').startswith('Raw:')]
    assert len(raw_labels) == 21
    assert raw_labels[0].text == 'Raw: 123'
    assert 'font-bold' in raw_labels[0].classes
    manual_inputs = [element for element in client.elements.values()
                     if isinstance(element, ui.input) and element.props.get('label') in ('Manual Min', 'Manual Max')]
    assert len(manual_inputs) == 42
    assert manual_inputs[0].value == '-100' and manual_inputs[1].value == '200'
    assert 'Auto' not in details[0] and 'Current' not in details[0] and 'RAM' not in details[0]
    maintenance = [element for element in client.elements.values()
                   if isinstance(element, ui.button) and element.text in (
                       'Set Min', 'Set Max', 'Load Joint & Motor Calibration',
                       'Clear Manual Selected', 'Release Selected Tendons')]
    assert len(maintenance) == 45 and all(element.enabled for element in maintenance)


def test_joint_page_maintenance_disabled_while_controlling_or_stop_latched(page):
    hand, client = page
    refresh = joint_calibration.build(hand, Mock(), Mock(), Mock())
    snapshot = idle_snapshot(hand)
    refresh(snapshot)
    assert button(client, 'Load Joint & Motor Calibration').enabled
    snapshot['control_mode'] = 'FEEDBACK'
    refresh(snapshot)
    assert not button(client, 'Load Joint & Motor Calibration').enabled
    assert not button(client, 'Set Min').enabled and not button(client, 'Set Max').enabled
    snapshot['control_mode'] = 'IDLE'
    snapshot['stop_latched'] = True
    refresh(snapshot)
    assert not button(client, 'Set Min').enabled
    assert not button(client, 'Release Selected Tendons').enabled


def test_pending_quality_badges_read_need_calibration(page):
    hand, client = page
    refresh = joint_calibration.build(hand, Mock(), Mock(), Mock())
    snapshot = idle_snapshot(hand)
    for row in snapshot['joints']:
        row['joint_quality'] = ['PENDING']
    refresh(snapshot)
    badges = [element.text for element in client.elements.values() if isinstance(element, ui.badge)]
    assert badges.count('NEED CALIBRATION') == 21
    assert 'PENDING' not in badges


def test_motor_page_has_compact_motor_columns_and_auto_fields(page):
    hand, client = page
    refresh = motor_calibration.build(hand, Mock(), Mock())
    snapshot = idle_snapshot(hand)
    for row in snapshot['joints']:
        row.update(auto_min=111, auto_max=222, auto_span=105, angle=1.5)
        row['result'] = dict(row.get('result') or {}, autoMinOffsetCounts=10, autoMaxShortfallCounts=20)
        row['motor_quality'] = ['READY']
    refresh(snapshot)
    tables = [element for element in client.elements.values() if isinstance(element, ui.table)]
    assert len(tables) == 21
    assert [column['label'] for column in tables[0].columns] == ['Motor', 'RawNow', 'AbsNow', 'Load']
    details = [text for text in label_texts(client) if text.startswith('Auto Min:')]
    assert len(details) == 21
    lines = details[0].splitlines()
    assert len(lines) == 2
    assert lines[0].startswith('Auto Min: 111') and 'Auto Max: 222' in lines[0]
    assert 'Auto Span: 105' in lines[0] and 'Joint Angle:' in lines[0]
    assert 'Return offset: 10' in lines[1] and 'Travel shortfall: 20 counts' in lines[1]
    assert lines[1].startswith('ABS Limits:')
    texts = ' '.join(label_texts(client))
    assert 'Route' not in texts and 'SavedRaw' not in texts and 'BUS' not in texts


def test_motor_calibrate_selected_passes_page_origin_and_disables_while_controlling(page):
    hand, client = page
    hand.request_calibration = Mock()
    act = Mock(side_effect=lambda callback: callback())
    refresh = motor_calibration.build(hand, act, Mock())
    snapshot = idle_snapshot(hand)
    refresh(snapshot)
    calibrate = button(client, 'Calibrate Selected')
    assert calibrate.enabled
    click(calibrate)
    hand.request_calibration.assert_called_once_with(origin='Motor Calibration')
    snapshot['control_mode'] = 'DIRECT'
    refresh(snapshot)
    assert not calibrate.enabled


def test_control_pages_drop_duplicate_clear_faults_and_explain_handover(page):
    hand, client = page
    feedback_control.build(hand, Mock())
    direct_control.build(hand, Mock(), Mock())
    labels = [element.text for element in client.elements.values() if isinstance(element, ui.button)]
    assert 'Clear Faults' not in labels
    notes = [text for text in label_texts(client) if 'Tabs only change the view' in text]
    assert len(notes) == 2
    assert all('Clear Faults must be pressed before enabling again' in text for text in notes)

