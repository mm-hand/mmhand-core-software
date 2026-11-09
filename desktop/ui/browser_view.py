"""Native console shell. UI owns drafts and display caches; HandBackend owns operations.

职责：展示快照并提交后端操作。
线程：NiceGUI 页面事件循环。
输入：后端快照及用户交互。
输出：原生控件和操作请求。
状态所有权：仅目标草稿和显示缓存；设备真值归后端。
禁止依赖：禁止直接操作协议、串口和控制器。
"""
import time
import json
from nicegui import core, ui
from desktop.core.user_issues import issue_text, operation_issue
from . import refresh_policy as policy
from .web import renumber_motors, joint_calibration, motor_calibration, feedback_control, direct_control
from .web import replay, plots, diagnostics, control_parameters, hand as hand_view
from .web import contact_calibration as contact_view
from .web.common import show

TABS = ['Calibration', 'Control', 'Plot', 'Settings']
SUBTABS = {
    'Calibration': ['Joint Calibration', 'Contact Calibration (optional)', 'Motor Calibration'],
    'Control': ['Feedback Control', 'Direct Control', 'Replay'],
    'Plot': ['Hand', 'Curves'],
    'Settings': ['Renumber Motor', 'Control Parameters', 'Diagnostics'],
}
def can_background(level):
    """CAN 色级：绿=通，红=断，灰=尚不可判。"""
    return {'ok': '#bbf7d0', 'can_error': '#fecaca'}.get(str(level), '#e2e8f0')


def status_background(value):
    """状态色卡：红=必须人工处理，黄=机器要自救/警告，蓝=正在动作，绿=可操作。"""
    status = str(value).upper()
    # 标签可能带 P4 编号后缀，如 'Joint Beyond Workspace (9)'，比对前去掉。
    base = status.split(' (')[0].strip()
    if base.startswith('NEED CALIBRATION') or base in ('PENDING', 'WARN', 'WARNING',
                                                       'SYNCING', 'JOINT BEYOND WORKSPACE',
                                                       'BEYOND'):
        return '#fde68a'
    if base == 'READY' or base.startswith('READY'):
        return '#bbf7d0'
    if base in ('FEEDBACK', 'DIRECT', 'CALIBRATING', 'RUNNING', 'CONTROLLING',
                'PULL JOINT INTO WORKSPACE'):
        return '#bfdbfe'
    if base in ('FAILED', 'FAULT', 'ERROR', 'ACTIVE', 'HOLD', 'MISSING',
                'STOPPED', 'LOCKED'):
        return '#fecaca'
    if base in ('KNOWN', 'CLEAR', 'SUCCESS', 'OFF', 'ARMED'):
        return '#bbf7d0'
    if base == 'IDLE':
        return '#d1fae5'
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
.ui-heartbeat {display:none}
.q-btn {text-transform:none;border-radius:6px;font-size:12px}
.tactile-grid {display:grid;grid-template-columns:repeat(5,minmax(0,1fr));gap:12px}
.renumber-motors-layout {display:grid;grid-template-columns:minmax(260px,1fr) minmax(0,2fr);gap:16px}
.renumber-motors-diagram {width:100%;max-width:720px;height:clamp(180px,calc(100dvh - 430px),380px);align-self:center}
@media(max-width:1000px) {.renumber-motors-layout {grid-template-columns:1fr}}
@media(max-width:1439px) {.finger-grid {grid-template-columns:1fr}}
@media(max-width:1000px) {.connection-bar {flex-wrap:wrap}.tactile-layout,.diagnostic-layout {flex-wrap:wrap!important}}
'''

# Injected once per page build through ui.add_body_html. The JavaScript only
# watches the hidden heartbeat label; it never runs on a hidden tab and gives up
# after MAX_AUTO_RELOADS consecutive reloads so a dead server is not hammered.
SELF_HEAL_SCRIPT = '''
(function () {
  'use strict';
  var HEARTBEAT_SELECTOR = '.ui-heartbeat';
  var STALE_MS = __STALE_MS__;
  var MAX_RELOADS = __MAX_RELOADS__;
  var RELOAD_DELAY_MS = __RELOAD_DELAY_MS__;
  var POLL_MS = __POLL_MS__;
  var STABLE_RESET_MS = 60000;
  var COUNT_KEY = 'mmhand-ui-auto-reloads';
  var TIME_KEY = 'mmhand-ui-auto-reload-at';
  var reloads = 0;
  var lastReloadAt = 0;
  var lastValue = null;
  var changedAt = Date.now();
  var reloading = false;
  var reloadTimer = null;
  var overlay = null;

  function readStorage() {
    try {
      reloads = parseInt(window.sessionStorage.getItem(COUNT_KEY) || '0', 10) || 0;
      lastReloadAt = parseInt(window.sessionStorage.getItem(TIME_KEY) || '0', 10) || 0;
    } catch (error) {
      reloads = 0;
      lastReloadAt = 0;
    }
    if (reloads > 0 && Date.now() - lastReloadAt > STABLE_RESET_MS) {
      reloads = 0;
      lastReloadAt = 0;
    }
  }

  function writeStorage() {
    try {
      window.sessionStorage.setItem(COUNT_KEY, String(reloads));
      window.sessionStorage.setItem(TIME_KEY, String(lastReloadAt));
    } catch (error) {
      return;
    }
  }

  function showOverlay(text) {
    if (!overlay) {
      overlay = document.createElement('div');
      overlay.setAttribute('role', 'alert');
      overlay.style.cssText = 'position:fixed;left:0;top:0;right:0;bottom:0;z-index:12000;'
        + 'display:flex;align-items:center;justify-content:center;padding:24px;'
        + 'background:rgba(15,23,42,0.78);color:#f8fafc;text-align:center;'
        + 'font:600 18px system-ui,sans-serif;';
      document.body.appendChild(overlay);
    }
    overlay.textContent = text;
    overlay.style.display = 'flex';
  }

  function hideOverlay() {
    if (overlay) { overlay.style.display = 'none'; }
  }

  function cancelPendingReload() {
    if (reloadTimer !== null) {
      window.clearTimeout(reloadTimer);
      reloadTimer = null;
    }
    reloading = false;
    hideOverlay();
  }

  function scheduleReload() {
    if (reloading) { return; }
    reloading = true;
    showOverlay('Connection lost — reloading…');
    reloadTimer = window.setTimeout(function () {
      reloads = reloads + 1;
      lastReloadAt = Date.now();
      writeStorage();
      window.location.reload();
    }, RELOAD_DELAY_MS);
  }

  function poll() {
    var node = document.querySelector(HEARTBEAT_SELECTOR);
    if (!node) { return; }
    var now = Date.now();
    var value = node.textContent || '';
    if (document.hidden) {
      lastValue = value;
      changedAt = now;
      return;
    }
    if (value !== lastValue) {
      lastValue = value;
      changedAt = now;
      if (reloading) { cancelPendingReload(); return; }
      hideOverlay();
      if (reloads > 0 && now - lastReloadAt > STABLE_RESET_MS) {
        reloads = 0;
        lastReloadAt = 0;
        writeStorage();
      }
      return;
    }
    if (now - changedAt <= STALE_MS) { return; }
    if (reloads < MAX_RELOADS) { scheduleReload(); return; }
    showOverlay('Connection lost — reload the page manually.');
  }

  readStorage();
  document.addEventListener('visibilitychange', function () { changedAt = Date.now(); });
  window.setInterval(poll, POLL_MS);
})();
'''


def self_heal_script():
    """Fill the reconnect thresholds from refresh_policy, the single source of truth."""
    return (SELF_HEAL_SCRIPT
            .replace('__STALE_MS__', str(int(policy.HEARTBEAT_STALE_S * 1000)))
            .replace('__MAX_RELOADS__', str(policy.MAX_AUTO_RELOADS))
            .replace('__RELOAD_DELAY_MS__', str(policy.AUTO_RELOAD_DELAY_MS))
            .replace('__POLL_MS__', str(policy.HEARTBEAT_POLL_MS)))


def build_page(hand):
    ui.colors(primary='#2563eb', positive='#16a34a', warning='#ea880c', negative='#dc2626')
    ui.add_css(CSS)
    # add_body_html only inserts raw HTML: script elements there never execute (and
    # without tags the source is rendered as page text). Run the script explicitly
    # once the client websocket is connected instead. Unit tests build pages without
    # the server event loop, so only schedule it when the loop exists.
    if core.loop is not None:
        ui.run_javascript(self_heal_script())
    # Fast ticks rewrite this label; the injected script watches it for a frozen page.
    heartbeat = ui.label('0').classes(policy.HEARTBEAT_LABEL_CLASS)
    preview_token, inspect_operation, log_sequence = [None], [None], [0]
    displayed_calibration_alerts = set()
    pending_calibration_alerts = []
    with ui.dialog().props('persistent') as calibration_dialog, ui.card().classes('w-[600px] max-w-full'):
        calibration_title = ui.label().classes('font-semibold text-lg')
        calibration_message = ui.label().classes('whitespace-pre-wrap text-sm')
        with ui.expansion('Details').classes('w-full'):
            calibration_evidence = ui.code('', language='json').classes('w-full')
        ui.button('Close', on_click=calibration_dialog.close).props('dense')
    displayed_firmware_alerts = set()
    firmware_alert_device = [None]
    with ui.dialog().props('persistent') as firmware_dialog, ui.card().classes('w-[600px] max-w-full'):
        ui.label('Motor firmware is too old').classes('text-red-600 font-bold')
        firmware_message = ui.label().classes('whitespace-pre-wrap text-sm')
        ui.button('Close', on_click=firmware_dialog.close).props('dense')

    def refresh_firmware_alerts(snapshot):
        device = snapshot.get('motor_firmware_device')
        if not snapshot.get('connections', {}).get('p4', {}).get('connected', False) or firmware_alert_device[0] != device:
            displayed_firmware_alerts.clear()
            firmware_alert_device[0] = device
        alerts = snapshot.get('motor_firmware_alerts', [])
        active = {(row['motor'], row['generation']) for row in alerts}
        displayed_firmware_alerts.intersection_update(active)
        new = [row for row in alerts if (row['motor'], row['generation']) not in displayed_firmware_alerts]
        if not new:
            return
        displayed_firmware_alerts.update((row['motor'], row['generation']) for row in new)
        firmware_message.set_text('Firmware 3.10 or newer is required. Affected motors cannot be controlled.\n' +
            '\n'.join(f"M{row['motor']:02d} / BUS{row['bus']} / ID{row['motor']}: {row['version']}"
                      for row in alerts))
        firmware_dialog.open()

    def refresh_calibration_alerts(snapshot):
        leaf = current_leaf()
        now = time.time()
        for operation in snapshot['operations']:
            if not policy.calibration_alert_visible(operation, leaf, now):
                continue
            operation_id = operation['operation_id']
            if operation_id in displayed_calibration_alerts:
                continue
            displayed_calibration_alerts.add(operation_id)
            pending_calibration_alerts.append((operation_id, operation['calibration_alert']))
        if pending_calibration_alerts and not calibration_dialog.value:
            operation_id, alert = pending_calibration_alerts.pop(0)
            calibration_title.set_text(alert['title'])
            calibration_title.style('color:#dc2626' if alert['severity'] == 'negative' else 'color:#ea880c')
            calibration_message.set_text(issue_text(alert.get('user_issue')) or alert['message'])
            calibration_evidence.set_content(json.dumps(alert, indent=2, default=str))
            hand.acknowledge_calibration_alert(operation_id)
            calibration_dialog.open()

    displayed_failed_operations = None
    def refresh_operation_failures(snapshot):
        """异步操作失败也立即提示；历史失败只在首次刷新时登记，不重复弹。"""
        nonlocal displayed_failed_operations
        failed = [op for op in snapshot.get('operations', ()) if op.get('status') == 'FAILED']
        if displayed_failed_operations is None:
            displayed_failed_operations = {op['operation_id'] for op in failed}
            return
        for operation in failed:
            operation_id = operation['operation_id']
            if operation_id in displayed_failed_operations:
                continue
            displayed_failed_operations.add(operation_id)
            if operation.get('calibration_alert') or operation.get('notification_shown'):
                continue
            ui.notify(issue_text(operation.get('user_issue') or operation_issue(
                operation.get('message') or 'Operation failed', scope=operation.get('scope', 'System')).to_dict()),
                      type='negative', timeout=10000, multi_line=True)
    def act(action):
        try:
            result = action()
            if not result.accepted:
                issue = hand.record_operation_issue(result.message)
                ui.notify(issue_text(issue), type='warning', multi_line=True, timeout=10000)
        except Exception as exc:
            issue = hand.record_operation_issue(str(exc), name='Action failed')
            ui.notify(issue_text(issue), type='negative', multi_line=True, timeout=10000)
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
            inspect_operation[0] = None
            import_info.set_text('Validating…'); load.set_enabled(False)
            content = await event.file.read()
            def request():
                result = hand.request_calibration_preview(event.file.name, content)
                if result.accepted:
                    inspect_operation[0] = result.operation_id
                else:
                    import_info.set_text('Validation failed: ' + result.message)
                return result
            act(request)
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
        inspect_operation[0] = None
        paths = hand.application_snapshot(include_curves=False)['calibration_storage']
        storage.set_text(f"Default data directory: {paths['data_dir']}")
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
            top_stop = ui.button('■ STOP', on_click=lambda: act(hand.stop_motion), color='negative').props('dense').classes('font-bold px-5')
        with ui.row().classes('status-bar items-center w-full flex-wrap gap-y-1'):
            badges = {name: ui.label().classes('status-chip text-xs cursor-pointer') for name in ['System', 'P4', 'CAN', 'Calibration', 'Mode', 'Overload', 'Collision', 'Joint workspace', 'Motor workspace', 'Multi-turn']}
        with ui.tabs(value=TABS[0]).classes('console-tabs w-full').props('dense no-caps active-color=primary outside-arrows mobile-arrows') as tabs:
            for name in TABS: ui.tab(name)
        def leave_direct_control():
            """没有电机坐标时取消 Direct 的归零提示：回到 Joint Calibration。"""
            tabs.set_value('Calibration')
            subtabs['Calibration'].set_value('Joint Calibration')
        for badge in badges.values(): badge.on('click', lambda: tabs.set_value('Settings'))
        refreshers = {}
        subtabs = {}
        with ui.splitter(horizontal=True, reverse=True, value=18, limits=(10, 70)).classes('console-workspace') as workspace:
            with workspace.before:
                with ui.tab_panels(tabs, value=TABS[0], animated=False).classes('page-panels').props('keep-alive') as panels:
                    for group in TABS:
                        with ui.tab_panel(group).classes('w-full'):
                            with ui.tabs(value=SUBTABS[group][0]).classes('console-subtabs w-full').props('dense no-caps active-color=primary') as child_tabs:
                                for name in SUBTABS[group]: ui.tab(name)
                            subtabs[group] = child_tabs
                            with ui.tab_panels(child_tabs, value=SUBTABS[group][0], animated=False).classes('w-full').props('keep-alive'):
                                for name in SUBTABS[group]:
                                    with ui.tab_panel(name):
                                        if name == 'Joint Calibration': refreshers[name] = joint_calibration.build(hand, act, open_import, confirm)
                                        elif name == 'Contact Calibration (optional)': refreshers[name] = contact_view.build(hand, act)
                                        elif name == 'Motor Calibration': refreshers[name] = motor_calibration.build(hand, act, open_import)
                                        elif name == 'Feedback Control': refreshers[name] = feedback_control.build(hand, act)
                                        elif name == 'Direct Control': refreshers[name] = direct_control.build(hand, act, confirm, leave_direct_control)
                                        elif name == 'Replay': refreshers[name] = replay.build(hand, act)
                                        elif name == 'Hand': refreshers[name] = hand_view.build(hand)
                                        elif name == 'Curves': refreshers[name] = plots.build(hand, act)
                                        elif name == 'Renumber Motor': refreshers[name] = renumber_motors.build(hand, act)
                                        elif name == 'Diagnostics': refreshers[name] = diagnostics.build(hand, act)
                                        elif name == 'Control Parameters': refreshers[name] = control_parameters.build(hand, act)
            with workspace.separator:
                with ui.element('div').classes('log-resize-grip'):
                    ui.tooltip('Drag up or down to resize System Log')
            with workspace.after:
                with ui.column().classes('system-log w-full'):
                    log_title = ui.label('System Log').classes('text-xs font-semibold')
                    output = ui.log(max_lines=2000)

    def current_leaf():
        group = tabs.value or TABS[0]
        child = subtabs[group].value if group in subtabs else None
        return policy.active_leaf(group, child, TABS, SUBTABS)

    def warn_control_page_switch(*_args):
        snapshot = hand.application_snapshot(include_curves=False)
        mode = snapshot.get('control_mode')
        leaf = current_leaf()
        if mode == 'FEEDBACK' and leaf != 'Feedback Control':
            ui.notify('Feedback control is active; changing pages does not stop motion. Press STOP to stop.', type='warning')
        elif mode == 'DIRECT' and leaf != 'Direct Control':
            ui.notify('Direct control is active; changing pages does not stop motion. Press STOP to stop.', type='warning')

    tabs.on_value_change(warn_control_page_switch)
    for child_tabs in subtabs.values():
        child_tabs.on_value_change(warn_control_page_switch)

    # Display caches: NiceGUI values are only written when they actually changed.
    text_cache, enabled_cache, style_cache = {}, {}, {}
    running = {'fast': False, 'heavy': False, 'plots': False}
    slow_ticks = {'fast': 0, 'heavy': 0, 'plots': 0}
    heartbeat_at = [0.0]
    plot_cache = {'leaf': None, 'snapshot': None}
    plot_last = {'leaf': None, 'ts': None}

    def set_text(key, widget, text):
        if policy.changed(text_cache, key, text):
            widget.set_text(text)

    def set_enabled(key, widget, enabled):
        if policy.changed(enabled_cache, key, enabled):
            widget.set_enabled(enabled)

    def set_style(key, widget, style):
        if policy.changed(style_cache, key, style):
            widget.style(style)

    def run_tick(kind, body):
        if not policy.backpressure_ok(running[kind]):
            return
        running[kind] = True
        started = time.perf_counter()
        try:
            body()
        finally:
            running[kind] = False
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            if policy.slow_tick(elapsed_ms):
                slow_ticks[kind] += 1
                total = sum(slow_ticks.values())
                output.push(f'UI slow tick: {kind} {elapsed_ms:.0f} ms (slow ticks: {total})')
                set_text('slow_ticks', log_title, f'System Log · slow ticks: {total}')

    def refresh_body():
        snapshot = hand.application_snapshot(include_curves=False)
        refresh_firmware_alerts(snapshot)
        refresh_calibration_alerts(snapshot)
        refresh_operation_failures(snapshot)
        choices = {d['port']: f"{d['port']} · {d['description']}" for d in snapshot['devices']}
        for name, state in snapshot['connections'].items():
            selector = selectors[name]
            options, value = device_selector_state(choices, selector.value, state)
            if selector.options != options: selector.set_options(options, value=value)
            elif selector.value != value: selector.set_value(value)
            set_enabled(('selector', name), selector, not state['connected'] and not state['pending'])
            set_text(('button', name), buttons[name], state['label'])
            set_enabled(('button', name), buttons[name],
                        not state['pending'] and (state['connected'] or bool(selector.value)) and not snapshot['busy'])
            age = '—' if state['age_ms'] is None else str(state['age_ms']) + ' ms'
            set_text(('age', name), ages[name], state['data_status'] + ' · Rx age: ' + age)
        for name, value in snapshot['badges'].items():
            display_name = 'Motor multi-turn position' if name == 'Multi-turn' else name
            display_value = 'UNKNOWN' if name == 'Multi-turn' and value == 'MISSING' else value
            set_text(('badge', name), badges[name], f'{display_name}: {display_value}')
            set_style(('badge-style', name), badges[name], f'background-color:{status_background(value)}')
        mode_value = snapshot.get('control_mode', 'UNKNOWN')
        set_text(('badge', 'Mode'), badges['Mode'], f'Mode: {mode_value}')
        set_style(('badge-style', 'Mode'), badges['Mode'], f'background-color:{status_background(mode_value)}')
        can = snapshot.get('can') or {}
        set_text(('badge', 'CAN'), badges['CAN'], can.get('label', 'CAN: UNKNOWN'))
        set_style(('badge-style', 'CAN'), badges['CAN'],
                  f"background-color:{can_background(can.get('level'))}")
        online = snapshot['communication'] == 'Online' and not snapshot['busy']
        set_text('top-stop', top_stop, '■ STOP')
        set_enabled('top-stop', top_stop, online)
        set_text('save', save, 'Save Data: ON' if snapshot['recording']['enabled'] else 'Save Data: OFF')
        preview = snapshot['import_preview']
        if preview and importer.value and preview['token'] != preview_token[0]:
            preview_token[0] = preview['token']
            import_info.set_text(f"File: {preview['name']}\nSaved at: {preview['saved_at']}   Profile: {preview['profile']}\nManual scope: 0x{preview['manual_joints']:06X}   Auto joint scope: 0x{preview['joints']:06X}\nMotor scope: 0x{preview['motors']:07X}\nValidation: Passed")
            load.set_enabled(True)
        elif inspect_operation[0] and preview_token[0] is None and importer.value:
            # A finished Inspect without a fresh preview must replace the
            # 'Validating…' placeholder, or the dialog looks stuck forever.
            operation = next((item for item in snapshot['operations']
                              if item['operation_id'] == inspect_operation[0]), None)
            if operation and operation['status'] == 'FAILED':
                inspect_operation[0] = None
                import_info.set_text('Validation failed: ' + operation['message'])
                load.set_enabled(False)
            elif operation and operation['status'] == 'SUCCESS':
                inspect_operation[0] = None
                import_info.set_text('Validation did not produce a preview. Close and reopen this dialog, then retry.')
                load.set_enabled(False)
        events = hand.log_since(log_sequence[0])
        for line in events['lines']: output.push(line)
        log_sequence[0] = events['sequence']
        now = time.monotonic()
        if policy.heartbeat_due(heartbeat_at[0], now):
            heartbeat_at[0] = now
            heartbeat.set_text(f'{time.time():.3f}')

    def refresh_heavy_body():
        leaf = current_leaf()
        if leaf in policy.PLOT_LEAVES:
            # Cache one curve-bearing snapshot for the slower chart timer.
            plot_cache['leaf'] = leaf
            plot_cache['snapshot'] = hand.application_snapshot(include_curves=policy.needs_curves(leaf))
            return
        refreshers[leaf](hand.application_snapshot(include_curves=False))

    def refresh_plots_body():
        leaf = current_leaf()
        now = time.monotonic()
        if not policy.should_refresh_plots(leaf, plot_last['leaf'], plot_last['ts'], now):
            return
        snapshot = plot_cache['snapshot']
        if snapshot is None or plot_cache['leaf'] != leaf:
            snapshot = hand.application_snapshot(include_curves=policy.needs_curves(leaf))
            plot_cache['leaf'] = leaf
            plot_cache['snapshot'] = snapshot
        refreshers[leaf](snapshot)
        plot_last['leaf'] = leaf
        plot_last['ts'] = now

    def refresh():
        run_tick('fast', refresh_body)

    def refresh_heavy():
        run_tick('heavy', refresh_heavy_body)

    def refresh_plots():
        run_tick('plots', refresh_plots_body)

    refresh()
    refresh_heavy()
    refresh_plots()
    fast_interval, heavy_interval, plots_interval = policy.select_intervals('Curves')
    ui.timer(fast_interval, refresh)
    ui.timer(heavy_interval, refresh_heavy)
    ui.timer(plots_interval, refresh_plots)
