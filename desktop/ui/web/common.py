"""Small display helpers, without control state or a page framework.

职责：展示快照并提交后端操作。
线程：NiceGUI 页面事件循环。
输入：后端快照及用户交互。
输出：原生控件和操作请求。
状态所有权：仅目标草稿和显示缓存；设备真值归后端。
禁止依赖：禁止直接操作协议、串口和控制器。
"""
from nicegui import ui

FINGERS = [('Thumb', range(16, 21)), ('Index', range(12, 16)), ('Middle', range(8, 12)), ('Ring', range(4, 8)), ('Pinky', range(0, 4))]

def show(value, suffix=''):
    return '—' if value is None else (f'{value:.2f}' if isinstance(value, float) else str(value)) + suffix

def label():
    return ui.label().classes('text-xs text-slate-600 whitespace-pre-wrap')

def quality_name(value):
    return 'Magnetic Encoder or Joint Installation Error' if value == 'OFFSET' else value

def quality():
    container, previous = ui.row().classes('gap-1 items-center'), [None]
    def refresh(values, tooltip=''):
        if previous[0] == (values, tooltip):
            return
        previous[0] = (list(values), tooltip)
        container.clear()
        with container:
            for value in values:
                color = 'positive' if value in ('READY', 'ENABLED') else 'negative' if value == 'FAILED' else 'warning' if value in ('OFFSET', 'WEAK RETURN', 'HIGH FRICTION') else 'grey'
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
