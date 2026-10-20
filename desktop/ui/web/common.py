"""Small display helpers, without control state or a page framework.

职责：展示快照并提交后端操作。
线程：NiceGUI 页面事件循环。
输入：后端快照及用户交互。
输出：原生控件和操作请求。
状态所有权：仅目标草稿和显示缓存；设备真值归后端。
禁止依赖：禁止直接操作协议、串口和控制器。
"""
from __future__ import annotations

import inspect

from nicegui import ui

FINGERS = [('Thumb', range(16, 21)), ('Index', range(12, 16)), ('Middle', range(8, 12)), ('Ring', range(4, 8)), ('Pinky', range(0, 4))]

# Shown on maintenance buttons that are only safe while P4 is idle.
MAINTENANCE_TOOLTIP = 'Press STOP first; Clear Faults before re-enable.'
CONTROL_ACTIVE_MODES = ('FEEDBACK', 'DIRECT')

def show(value, suffix=''):
    return '—' if value is None else (f'{value:.2f}' if isinstance(value, float) else str(value)) + suffix

def label():
    return ui.label().classes('text-xs text-slate-600 whitespace-pre-wrap')

def quality_name(value):
    if value == 'OFFSET':
        return 'Magnetic Encoder or Joint Installation Error'
    if value == 'PENDING':
        return 'NEED CALIBRATION'
    return value

def control_active(snapshot):
    """True while P4 owns a joint or direct control mode (new snapshot field first)."""
    mode = snapshot.get('control_mode')
    if mode is not None:
        return str(mode).upper() in CONTROL_ACTIVE_MODES
    if snapshot.get('device') in ('Controlling', 'Calibrating'):
        return True
    return (any(row.get('feedback_control_status') == 'ENABLED' for row in snapshot.get('joints', ()))
            or any(row.get('direct_control_status') == 'ENABLED' for row in snapshot.get('motors', ())))

def stop_latched(snapshot):
    """True when the last STOP is still latched and Clear Faults has not run."""
    value = snapshot.get('stop_latched')
    if value is not None:
        return bool(value)
    return snapshot.get('device') == 'Stopped'

def maintenance_allowed(snapshot):
    """Maintenance actions need fresh P4 status, no operation, and an idle controller."""
    if snapshot.get('communication') != 'Online' or snapshot.get('busy'):
        return False
    if stop_latched(snapshot) or control_active(snapshot):
        return False
    mode = snapshot.get('control_mode')
    if mode is None:
        # Legacy snapshot without the explicit mode field: control_active's
        # fallbacks already checked for active ownership.
        return True
    return str(mode).upper() == 'IDLE'

def maintenance_button(button):
    """Attach the shared STOP/Clear Faults hint and return the button."""
    with button:
        ui.tooltip(MAINTENANCE_TOOLTIP)
    return button

def control_note():
    """Explain tab versus handover semantics on both control pages."""
    return ui.label('Tabs only change the view; ENABLE Selected requests a P4 handover to this page. '
                    'After STOP latches, Clear Faults must be pressed before enabling again.').classes(
                        'text-sm text-slate-600 whitespace-pre-wrap')

def json_status(row):
    """Saved|Unknown manual JSON status; falls back to the legacy persistence text."""
    value = row.get('json_status')
    if value is not None:
        return 'Saved' if str(value).strip().lower() in ('saved', 'true', '1') else 'Unknown'
    persistence = row.get('persistence') or ''
    if isinstance(persistence, str) and (persistence == 'Saved'
                                        or persistence.startswith('Saved in JSON')
                                        or persistence == 'RAM differs from saved JSON'):
        return 'Saved'
    return 'Unknown'

def auto_span(row):
    """Direction-corrected Auto span in counts; falls back to the calibration result."""
    value = row.get('auto_span')
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return int(value)
    result = row.get('result') or {}
    for key in ('autoSpanCounts', 'assemblyAutoSpanCounts'):
        value = result.get(key)
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            return int(value)
    return None

def request_calibration(hand, origin):
    """Call the calibration entry point with the page origin when the backend accepts it."""
    entry = hand.request_calibration
    try:
        parameters = inspect.signature(entry).parameters
    except (TypeError, ValueError):
        parameters = {}
    accepts_origin = 'origin' in parameters or any(
        parameter.kind is inspect.Parameter.VAR_KEYWORD for parameter in parameters.values())
    return entry(origin=origin) if accepts_origin else entry()

def quality():
    container, previous = ui.row().classes('gap-1 items-center'), [None]
    def refresh(values, tooltip=''):
        if previous[0] == (values, tooltip):
            return
        previous[0] = (list(values), tooltip)
        container.clear()
        with container:
            for value in values:
                color = 'primary' if value == 'CALIBRATING' else 'positive' if value in ('READY', 'ENABLED') else 'negative' if value in ('FAILED', 'COLLISION DETECTED', 'FAULT') else 'warning' if value in ('OFFSET', 'WEAK RETURN', 'HIGH FRICTION') else 'grey'
                with ui.badge(quality_name(value), color=color):
                    if tooltip:
                        ui.tooltip(tooltip)
    return refresh

def selection(hand, key, indices, text):
    syncing = [False]
    def changed(event):
        if syncing[0]:
            return
        selected = set(hand.application_snapshot(include_curves=False)[key])
        selected.update(indices) if event.value else selected.difference_update(indices)
        result = hand.select_motors(selected) if key == 'selected_motors' else hand.select_joints(selected)
        if not result.accepted:
            hand.log_operation(result.message)
            ui.notify(result.message, type='warning')
    box = ui.checkbox(text, on_change=changed).props('dense')
    def refresh(snapshot):
        syncing[0] = True
        selected = set(snapshot[key])
        box.set_value(set(indices) <= selected)
        indeterminate = bool(selected.intersection(indices) and not set(indices) <= selected)
        if box._props.get('indeterminate') is not indeterminate:
            box._props['indeterminate'] = indeterminate
        syncing[0] = False
    return refresh

def finger_cards():
    return ui.element('div').classes('finger-grid w-full')

def target_editor(initial, minimum, maximum, integer=False, on_commit=None):
    draft, syncing = {'value': initial}, [False]
    with ui.row().classes('w-full items-center gap-2'):
        number = ui.number('Target ABS' if integer else 'Target (°)', value=initial, min=minimum, max=maximum, step=1 if integer else 0.1).props('dense outlined').classes('w-32')
        slider = ui.slider(min=minimum, max=maximum, value=initial, step=1 if integer else 0.1).classes('flex-1')
    # Keep drag updates on the client; echoing each value to the slider can
    # overwrite a newer pointer position while the UI also polls feedback.
    slider.LOOPBACK = False
    slider._props['loopback'] = False
    def changed(event):
        if syncing[0]:
            return
        draft['value'] = event.value
        syncing[0] = True
        try:
            if event.sender is not number:
                number.set_value(event.value)
            if event.sender is not slider and event.value is not None:
                slider.set_value(event.value)
        finally:
            syncing[0] = False
    number.on_value_change(changed)
    slider.on_value_change(changed)
    if on_commit is not None:
        def commit(event):
            if not slider.enabled or event.args is None:
                return
            # Quasar's change event carries the final value. The throttled
            # update:modelValue event may still contain an earlier drag value.
            slider.set_value(event.args)
            on_commit(event.args)
        slider.on('change', commit, [None])
    return draft, number, slider

def table(columns, key='joint'):
    return ui.table(columns=[dict(name=name, label=title, field=name, align='left') for name, title in columns], rows=[], row_key=key).props('dense flat').classes('w-full')

def set_rows(widget, rows):
    if widget.rows != rows:
        widget.rows = rows
        widget.update()
