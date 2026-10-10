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
from .web import joint_calibration, motor_calibration, feedback_control, direct_control
from .web import replay, plots, tactile, diagnostics
from .web.common import show

TABS = ['Joint Calibration', 'Motor Calibration', 'Feedback Control', 'Direct Control',
        'Replay', 'Plots & Recordings', 'Tactile', 'Diagnostics']
CSS = '''
body {background:#F5F7FA;color:#1e293b}
.nicegui-content {padding:0;gap:0;height:100dvh;overflow:hidden}
.console-shell {display:flex;flex-direction:column;height:100dvh;width:100%;overflow:hidden}
.connection-bar {min-height:44px;padding:4px 12px;background:white;gap:8px;flex-shrink:0}
.connection-bar .q-field {font-size:12px}
.connection-bar .q-field__control,.connection-bar .q-field__marginal {height:32px;min-height:32px}
.status-bar {min-height:32px;padding:4px 12px;background:#eef2f7;gap:14px;flex-shrink:0}
.console-tabs {height:36px;min-height:36px;background:white;flex-shrink:0}
.console-tabs .q-tab {min-height:36px;padding:0 12px;font-size:12px}
.page-panels {flex:1;min-height:0;overflow:auto;width:100%;background:transparent}
.page-panels .q-tab-panel {padding:12px;display:flex;flex-direction:column;gap:12px}
.page-toolbar {width:100%;display:flex;align-items:center;justify-content:flex-start;flex-wrap:nowrap;gap:8px;overflow-x:auto;flex-shrink:0}
.page-toolbar > * {flex-shrink:0}
.page-toolbar .q-btn {min-height:32px;padding:4px 10px;white-space:nowrap}
.page-toolbar .q-field__control,.page-toolbar .q-field__marginal {height:32px;min-height:32px}
.finger-grid {display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px}
.q-card {border-radius:10px;box-shadow:0 1px 4px #182b4410;padding:12px;gap:8px}
.joint-row {border-top:1px solid #e8edf3;padding-top:8px}
.system-log {height:120px;background:white;border-top:1px solid #dce3eb;padding:6px 12px;flex-shrink:0;gap:4px}
.system-log .nicegui-log {flex:1;width:100%;font:11px ui-monospace,monospace;color:#475569}
.q-btn {text-transform:none;border-radius:6px;font-size:12px}
.tactile-grid {display:grid;grid-template-columns:repeat(5,minmax(0,1fr));gap:12px}
@media(max-width:1439px) {.finger-grid {grid-template-columns:1fr}.system-log {height:96px}}
@media(max-width:1000px) {.connection-bar {flex-wrap:wrap}.tactile-layout,.diagnostic-layout {flex-wrap:wrap!important}}
'''

def build_page(hand):
    ui.colors(primary='#2563eb', positive='#16a34a', warning='#ea880c', negative='#dc2626')
    ui.add_css(CSS)
    preview_token, log_sequence = [None], [0]
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
                ages[name] = ui.label('—ms').classes('text-xs whitespace-nowrap')
                with ages[name]: ui.tooltip('Feedback age since the latest received status, in milliseconds. This is not round-trip latency.')
            ui.space()
            save = ui.button('Save Data: ON', on_click=lambda: act(lambda: hand.request_recording(not hand.application_snapshot(include_curves=False)['recording']['enabled']))).props('dense outline')
            ui.button('■ STOP', on_click=lambda: act(hand.stop_motion), color='negative').props('dense').classes('font-bold px-5')
        with ui.row().classes('status-bar items-center w-full'):
            badges = {name: ui.label().classes('text-xs cursor-pointer') for name in ['System', 'Calibration', 'Overload', 'Collision', 'Multi-turn']}
        with ui.tabs(value=TABS[0]).classes('console-tabs w-full').props('dense no-caps active-color=primary outside-arrows mobile-arrows') as tabs:
            for name in TABS: ui.tab(name)
        for badge in badges.values(): badge.on('click', lambda: tabs.set_value('Diagnostics'))
        refreshers = {}
        with ui.tab_panels(tabs, value=TABS[0], animated=False).classes('page-panels').props('keep-alive'):
            with ui.tab_panel(TABS[0]): refreshers[TABS[0]] = joint_calibration.build(hand, act, open_import, confirm)
            with ui.tab_panel(TABS[1]): refreshers[TABS[1]] = motor_calibration.build(hand, act, open_import)
            with ui.tab_panel(TABS[2]): refreshers[TABS[2]] = feedback_control.build(hand, act)
            with ui.tab_panel(TABS[3]): refreshers[TABS[3]] = direct_control.build(hand, act, confirm)
            with ui.tab_panel(TABS[4]): refreshers[TABS[4]] = replay.build(hand, act)
            with ui.tab_panel(TABS[5]): refreshers[TABS[5]] = plots.build(hand, act)
            with ui.tab_panel(TABS[6]): refreshers[TABS[6]] = tactile.build(hand)
            with ui.tab_panel(TABS[7]): refreshers[TABS[7]] = diagnostics.build(hand, act)
        with ui.column().classes('system-log w-full'):
            ui.label('System Log').classes('text-xs font-semibold')
            output = ui.log(max_lines=2000)
    last_heavy = [0.0]
    def refresh():
        heavy = time.monotonic() - last_heavy[0] >= 0.2
        snapshot = hand.application_snapshot(include_curves=heavy)
        choices = {d['port']: f"{d['port']} · {d['description']}" for d in snapshot['devices']}
        for name, state in snapshot['connections'].items():
            selector, options = selectors[name], dict(choices)
            for port in (selector.value, state['port']):
                if port and port not in options: options[port] = port
            if selector.options != options: selector.set_options(options)
            if state['connected'] and state['port']: selector.set_value(state['port'])
            selector.set_enabled(not state['connected'] and not state['pending'])
            buttons[name].set_text(state['label'])
            buttons[name].set_enabled(not state['pending'] and (state['connected'] or bool(selector.value)) and not snapshot['busy'])
            ages[name].set_text(('—' if state['age_ms'] is None else str(state['age_ms'])) + 'ms')
        for name, value in snapshot['badges'].items():
            badges[name].set_text(f'{name}: {value}')
            color = '#16a34a' if value in ('Ready', 'KNOWN', 'CLEAR', 'SUCCESS') else '#dc2626' if value in ('ACTIVE', 'FAILED') else '#ea880c' if value in ('HOLD', 'MISSING', 'WARN') else '#64748b'
            badges[name].style(f'color:{color}')
        save.set_text('Save Data: ON' if snapshot['recording']['enabled'] else 'Save Data: OFF')
        preview = snapshot['import_preview']
        if preview and importer.value and preview['token'] != preview_token[0]:
            preview_token[0] = preview['token']
            import_info.set_text(f"File: {preview['name']}\nSaved at: {preview['saved_at']}   Profile: {preview['profile']}\nManual scope: 0x{preview['manual_joints']:06X}   Auto joint scope: 0x{preview['joints']:06X}\nMotor scope: 0x{preview['motors']:07X}\nValidation: Passed")
            load.set_enabled(True)
        events = hand.log_since(log_sequence[0])
        for line in events['lines']: output.push(line)
        log_sequence[0] = events['sequence']
        current = tabs.value or TABS[0]
        if heavy or current not in ('Plots & Recordings', 'Tactile'): refreshers[current](snapshot)
        if heavy: last_heavy[0] = time.monotonic()
    refresh()
    ui.timer(0.1, refresh)
