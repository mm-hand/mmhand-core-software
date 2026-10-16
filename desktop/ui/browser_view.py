"""Native console shell. UI owns drafts and display caches; HandBackend owns operations.

职责：展示快照并提交后端操作。
线程：NiceGUI 页面事件循环。
输入：后端快照及用户交互。
输出：原生控件和操作请求。
状态所有权：仅目标草稿和显示缓存；设备真值归后端。
禁止依赖：禁止直接操作协议、串口和控制器。
"""
import time
from nicegui import ui
from .web import renumber_motors, renumber_tactile, joint_calibration, motor_calibration, feedback_control, direct_control
from .web import replay, plots, tactile, diagnostics, control_parameters
from .web.common import show

TABS = ['Calibration', 'Control', 'Replay', 'Plot', 'Settings']
SUBTABS = {
    'Calibration': ['Joint Calibration', 'Motor Calibration'],
    'Control': ['Feedback Control', 'Direct Control'],
    'Plot': ['Curves & Recordings', 'Tactile'],
    'Settings': ['Renumber Motor', 'Renumber Tactile', 'Control Parameters', 'Diagnostics'],
}
def status_background(value):
    status = str(value).upper()
    if status in ('FAILED', 'FAULT', 'ERROR', 'ACTIVE', 'HOLD', 'MISSING', 'STOPPED'):
        return '#fecaca'
    if status in ('READY', 'KNOWN', 'CLEAR', 'SUCCESS', 'OFF', 'ARMED'):
        return '#bbf7d0'
    if status in ('RUNNING', 'CALIBRATING', 'CONTROLLING'):
        return '#bfdbfe'
    if status in ('WARN', 'WARNING', 'PENDING', 'SYNCING'):
        return '#fde68a'
    return '#e2e8f0'


def device_selector_state(choices, selected, connection):
    """Retain an active connection's identity, never resurrect an unplugged port."""
    options = dict(choices)
    active = connection['connected'] or connection['pending']
    port = connection['port'] if active else None
    if port and port not in options:
        options[port] = port
    value = port or (selected if selected in options else None)
    return options, value


CSS = '''
body {background:#F5F7FA;color:#1e293b}
.nicegui-content {padding:0;gap:0;height:100dvh;overflow:hidden}
.console-shell {display:flex;flex-direction:column;height:100dvh;width:100%;overflow:hidden}
.connection-bar {min-height:44px;padding:4px 12px;background:white;gap:8px;flex-shrink:0}
.connection-bar .q-field {font-size:12px}
.connection-bar .q-field__control,.connection-bar .q-field__marginal {height:32px;min-height:32px}
.status-bar {min-height:32px;padding:4px 12px;background:#eef2f7;gap:14px;flex-shrink:0}
.status-chip {padding:4px 8px;border-radius:5px;color:#1e293b;font-weight:500;white-space:nowrap}
.console-tabs {height:36px;min-height:36px;background:white;flex-shrink:0}
.console-tabs .q-tab {min-height:36px;padding:0 12px;font-size:12px}
.console-subtabs {height:36px;min-height:36px;background:white;flex-shrink:0}
.console-subtabs .q-tab {min-height:36px;padding:0 12px;font-size:12px}
.page-panels {flex:1;min-height:0;overflow:auto;width:100%;background:transparent}
.page-panels .q-tab-panel {padding:12px;display:flex;flex-direction:column;gap:12px}
.page-toolbar {width:100%;display:flex;align-items:center;justify-content:flex-start;flex-wrap:nowrap;gap:8px;overflow-x:auto;flex-shrink:0}
.page-toolbar > * {flex-shrink:0}
.page-toolbar .q-btn {min-height:32px;padding:4px 10px;white-space:nowrap}
.page-toolbar .q-field__control,.page-toolbar .q-field__marginal {height:32px;min-height:32px}
.finger-grid {display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px}
.q-card {border-radius:10px;box-shadow:0 1px 4px #182b4410;padding:12px;gap:8px}
.joint-row {border-top:1px solid #e8edf3;padding-top:8px}
.console-workspace {flex:1;min-height:0;width:100%}
.console-workspace > .q-splitter__before {display:flex;flex-direction:column;min-height:0}
.console-workspace > .q-splitter__separator {height:8px;background:#e2e8f0;cursor:row-resize}
.log-resize-grip {width:40px;height:3px;border-radius:2px;background:#94a3b8}
.system-log {height:100%;min-height:0;background:white;padding:6px 12px;gap:4px;overflow:hidden}
.system-log .nicegui-log {flex:1;min-height:0;width:100%;font:11px ui-monospace,monospace;color:#475569}
.q-btn {text-transform:none;border-radius:6px;font-size:12px}
.tactile-grid {display:grid;grid-template-columns:repeat(5,minmax(0,1fr));gap:12px}
.renumber-motors-layout {display:grid;grid-template-columns:minmax(260px,1fr) minmax(0,2fr);gap:16px}
.renumber-motors-diagram {width:100%;max-width:720px;height:clamp(180px,calc(100dvh - 430px),380px);align-self:center}
@media(max-width:1000px) {.renumber-motors-layout {grid-template-columns:1fr}}
@media(max-width:1439px) {.finger-grid {grid-template-columns:1fr}}
@media(max-width:1000px) {.connection-bar {flex-wrap:wrap}.tactile-layout,.diagnostic-layout {flex-wrap:wrap!important}}
'''

def build_page(hand):
    ui.colors(primary='#2563eb', positive='#16a34a', warning='#ea880c', negative='#dc2626')
    ui.add_css(CSS)
    preview_token, log_sequence = [None], [0]
    displayed_calibration_alerts = set()
    pending_calibration_alerts = []
    with ui.dialog().props('persistent') as calibration_dialog, ui.card().classes('w-[600px] max-w-full'):
        calibration_title = ui.label().classes('font-semibold text-lg')
        calibration_message = ui.label().classes('whitespace-pre-wrap text-sm')
        ui.button('Close', on_click=calibration_dialog.close).props('dense')
    def refresh_calibration_alerts(snapshot):
        for operation in snapshot['operations']:
            operation_id = operation['operation_id']
            alert = operation.get('calibration_alert')
            if alert and operation_id not in displayed_calibration_alerts:
                displayed_calibration_alerts.add(operation_id)
                pending_calibration_alerts.append(alert)
        if pending_calibration_alerts and not calibration_dialog.value:
            alert = pending_calibration_alerts.pop(0)
            calibration_title.set_text(alert['title'])
            calibration_title.style('color:#dc2626' if alert['severity'] == 'negative' else 'color:#ea880c')
            calibration_message.set_text(alert['message'])
            calibration_dialog.open()
    def act(action):
        try:
            result = action()
            if not result.accepted:
                hand.log_operation('Rejected: ' + result.message)
                ui.notify(result.message, type='warning')
        except Exception as exc:
            hand.log_operation(f'Operation failed: {exc}')
            ui.notify(str(exc), type='negative')
    def confirm(message, action):
        with ui.dialog() as dialog, ui.card().classes('max-w-lg'):
            ui.label(message)
            with ui.row():
                ui.button('Cancel', on_click=dialog.close).props('flat')
                ui.button('Continue', on_click=lambda: (dialog.close(), act(action)))
        dialog.open()
    with ui.dialog() as importer, ui.card().classes('w-[600px] max-w-full'):
        ui.label('Load Joint & Motor Calibration').classes('font-semibold')
        storage = ui.label().classes('text-sm whitespace-pre-wrap break-all')
        ui.label('Joint calibration endpoints are saved automatically after verification. Motor calibration results and multi-turn anchors are saved after each successful joint. Failed joints do not replace successful results.').classes('text-sm')
        async def upload(event):
            preview_token[0] = None
            import_info.set_text('Validating…'); load.set_enabled(False)
            content = await event.file.read()
            act(lambda: hand.request_calibration_preview(event.file.name, content))
        ui.upload(label='Choose .mmcal.json / .json', on_upload=upload, auto_upload=True, max_file_size=10_000_000).props('accept=.json').classes('w-full')
        import_info = ui.label('Choose a calibration file to validate.').classes('whitespace-pre-wrap text-sm')
        ui.label('Restore scope comes from this file. Confirm that the mechanism, servo IDs and mapping have not changed.').classes('text-sm')
        with ui.row().classes('w-full justify-end'):
            ui.button('Cancel', on_click=importer.close).props('flat')
            load = ui.button('Load to P4', on_click=lambda: (act(lambda: hand.request_calibration_import(preview_token[0])), importer.close()))
            load.set_enabled(False)
    def open_import():
        hand.clear_calibration_preview()
        preview_token[0] = None
        paths = hand.application_snapshot(include_curves=False)['calibration_storage']
        storage.set_text(f"Current device save path: {paths['current']}\nDefault data directory: {paths['data_dir']}\nEarlier default snapshot: {paths['default']}")
        import_info.set_text('Choose a calibration file to validate.'); load.set_enabled(False)
        importer.open()
    with ui.element('div').classes('console-shell'):
        with ui.row().classes('connection-bar items-center w-full'):
            ui.label('MM-Hand Console').classes('font-semibold text-base whitespace-nowrap')
            selectors, buttons, ages = {}, {}, {}
            for name in ('p4', 's3'):
                ui.label(name.upper()).classes('text-xs font-semibold')
                selectors[name] = ui.select({}, label='Device').props('dense outlined').classes('w-40')
                def connection(name=name):
                    state = hand.application_snapshot(include_curves=False)['connections'][name]
                    act(lambda: hand.request_disconnect(tactile=name == 's3') if state['connected'] else hand.request_connect(selectors[name].value or '', tactile=name == 's3'))
                buttons[name] = ui.button('Connect', on_click=connection).props('dense outline')
                ages[name] = ui.label('Rx age: —').classes('text-xs whitespace-nowrap')
                with ages[name]: ui.tooltip('Time since the latest data received on this connection, in milliseconds.')
            ui.space()
            save = ui.button('Save Data: ON', on_click=lambda: act(lambda: hand.request_recording(not hand.application_snapshot(include_curves=False)['recording']['enabled']))).props('dense outline')
            ui.button('■ STOP', on_click=lambda: act(hand.stop_motion), color='negative').props('dense').classes('font-bold px-5')
        with ui.row().classes('status-bar items-center w-full'):
            badges = {name: ui.label().classes('status-chip text-xs cursor-pointer') for name in ['System', 'Calibration', 'Overload', 'Collision', 'Multi-turn']}
        with ui.tabs(value=TABS[0]).classes('console-tabs w-full').props('dense no-caps active-color=primary outside-arrows mobile-arrows') as tabs:
            for name in TABS: ui.tab(name)
        for badge in badges.values(): badge.on('click', lambda: tabs.set_value('Settings'))
        refreshers = {}
        subtabs = {}
        with ui.splitter(horizontal=True, reverse=True, value=18, limits=(10, 70)).classes('console-workspace') as workspace:
            with workspace.before:
                with ui.tab_panels(tabs, value=TABS[0], animated=False).classes('page-panels').props('keep-alive') as panels:
                    for group in TABS:
                        with ui.tab_panel(group).classes('w-full'):
                            if group == 'Replay':
                                refreshers['Replay'] = replay.build(hand, act)
                                continue
                            with ui.tabs(value=SUBTABS[group][0]).classes('console-subtabs w-full').props('dense no-caps active-color=primary') as child_tabs:
                                for name in SUBTABS[group]: ui.tab(name)
                            subtabs[group] = child_tabs
                            with ui.tab_panels(child_tabs, value=SUBTABS[group][0], animated=False).classes('w-full').props('keep-alive'):
                                for name in SUBTABS[group]:
                                    with ui.tab_panel(name):
                                        if name == 'Joint Calibration': refreshers[name] = joint_calibration.build(hand, act, open_import, confirm)
                                        elif name == 'Motor Calibration': refreshers[name] = motor_calibration.build(hand, act, open_import)
                                        elif name == 'Feedback Control': refreshers[name] = feedback_control.build(hand, act)
                                        elif name == 'Direct Control': refreshers[name] = direct_control.build(hand, act, confirm)
                                        elif name == 'Curves & Recordings': refreshers[name] = plots.build(hand, act)
                                        elif name == 'Tactile': refreshers[name] = tactile.build(hand)
                                        elif name == 'Renumber Motor': refreshers[name] = renumber_motors.build(hand, act)
                                        elif name == 'Renumber Tactile': refreshers[name] = renumber_tactile.build(hand, act)
                                        elif name == 'Diagnostics': refreshers[name] = diagnostics.build(hand, act)
                                        elif name == 'Control Parameters': refreshers[name] = control_parameters.build(hand, act)
            with workspace.separator:
                with ui.element('div').classes('log-resize-grip'):
                    ui.tooltip('Drag up or down to resize System Log')
            with workspace.after:
                with ui.column().classes('system-log w-full'):
                    ui.label('System Log').classes('text-xs font-semibold')
                    output = ui.log(max_lines=2000)
        def current_leaf():
            group = tabs.value or TABS[0]
            return group if group == 'Replay' else (subtabs[group].value or SUBTABS[group][0])
    last_heavy = [0.0]
    def refresh():
        heavy = time.monotonic() - last_heavy[0] >= 0.2
        snapshot = hand.application_snapshot(include_curves=heavy)
        refresh_calibration_alerts(snapshot)
        choices = {d['port']: f"{d['port']} · {d['description']}" for d in snapshot['devices']}
        for name, state in snapshot['connections'].items():
            selector = selectors[name]
            options, value = device_selector_state(choices, selector.value, state)
            if selector.options != options: selector.set_options(options, value=value)
            elif selector.value != value: selector.set_value(value)
            selector.set_enabled(not state['connected'] and not state['pending'])
            buttons[name].set_text(state['label'])
            buttons[name].set_enabled(not state['pending'] and (state['connected'] or bool(selector.value)) and not snapshot['busy'])
            ages[name].set_text(state['data_status'] + ' · Rx age: ' + ('—' if state['age_ms'] is None else str(state['age_ms']) + ' ms'))
        for name, value in snapshot['badges'].items():
            display_name = 'Motor multi-turn position' if name == 'Multi-turn' else name
            display_value = 'UNKNOWN' if name == 'Multi-turn' and value == 'MISSING' else value
            badges[name].set_text(f'{display_name}: {display_value}')
            badges[name].style(f'background-color:{status_background(value)}')
        save.set_text('Save Data: ON' if snapshot['recording']['enabled'] else 'Save Data: OFF')
        preview = snapshot['import_preview']
        if preview and importer.value and preview['token'] != preview_token[0]:
            preview_token[0] = preview['token']
            import_info.set_text(f"File: {preview['name']}\nSaved at: {preview['saved_at']}   Profile: {preview['profile']}\nManual scope: 0x{preview['manual_joints']:06X}   Auto joint scope: 0x{preview['joints']:06X}\nMotor scope: 0x{preview['motors']:07X}\nValidation: Passed")
            load.set_enabled(True)
        events = hand.log_since(log_sequence[0])
        for line in events['lines']: output.push(line)
        log_sequence[0] = events['sequence']
        current = current_leaf()
        if heavy or current not in ('Curves & Recordings', 'Tactile'): refreshers[current](snapshot)
        if heavy: last_heavy[0] = time.monotonic()
    refresh()
    ui.timer(0.1, refresh)
