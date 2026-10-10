"""Quality warnings and actual device faults remain separate; preserve raw evidence.

职责：展示快照并提交后端操作。
线程：NiceGUI 页面事件循环。
输入：后端快照及用户交互。
输出：原生控件和操作请求。
状态所有权：仅目标草稿和显示缓存；设备真值归后端。
禁止依赖：禁止直接操作协议、串口和控制器。
"""
import json
from nicegui import ui
from .common import label, table, set_rows, quality_name

def build(hand, act):
    with ui.row().classes('page-toolbar'):
        ui.button('Clear Faults', on_click=lambda: act(hand.request_clear_faults))
        headline = label()
    with ui.row().classes('w-full items-start flex-nowrap diagnostic-layout'):
        issues = table([('severity', 'Severity'), ('channel', 'Channel'), ('problem', 'Problem')], 'id').classes('flex-1')
        with ui.card().classes('w-96'):
            ui.label('Selected Issue').classes('font-semibold')
            detail = label()
    issues.on('rowClick', lambda e: detail.set_text(json.dumps(e.args[1].get('details', ''), indent=2, default=str)))
    ui.label('Operation History').classes('font-semibold')
    operations = table([(k, k.title()) for k in ('time', 'name', 'scope', 'phase', 'status', 'message')], 'operation_id')
    codes = {}
    for title in ['Task Runtime', 'Routes', 'Fingerprint', 'Raw Snapshot']:
        with ui.expansion(title).classes('w-full'):
            if title == 'Task Runtime':
                ui.switch('Subscribe to task diagnostics', on_change=lambda e: act(lambda: hand.request_task_diagnostics(e.value)))
            codes[title] = ui.code('', language='json').classes('w-full')
    def refresh(snapshot):
        diag = snapshot['diagnostics']
        headline.set_text(f"Motors: {sum(m['online'] is True for m in snapshot['motors'])}/27   Encoders: {sum(j['encoder'] == 'ONLINE' for j in snapshot['joints'])}/21   Owner: {diag['owner']}   Status age: {diag['status_age_ms']}ms")
        set_rows(issues, [dict(issue, problem=quality_name(issue['problem'])) for issue in snapshot['issues']])
        set_rows(operations, snapshot['operations'])
        for title, value in [('Task Runtime', diag['tasks']), ('Routes', diag['routes']), ('Fingerprint', diag['fingerprint']), ('Raw Snapshot', diag)]:
            text = json.dumps(value, indent=2, default=str)
            if codes[title].content != text:
                codes[title].set_content(text)
    return refresh
