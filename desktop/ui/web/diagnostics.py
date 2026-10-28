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
from .common import label, table, set_rows
from desktop.core.user_issues import issue_text

def build(hand, act):
    with ui.row().classes('page-toolbar'):
        headline = label()
    with ui.row().classes('w-full items-start flex-nowrap diagnostic-layout'):
        issues = table([('severity', 'Severity'), ('status', 'Status'), ('scope', 'Affected'), ('title', 'Problem')], 'id').classes('flex-1')
        with ui.card().classes('w-96'):
            ui.label('Selected Issue').classes('font-semibold')
            explanation = label().classes('whitespace-pre-wrap')
            with ui.expansion('Details').classes('w-full'):
                detail = ui.code('', language='json').classes('w-full')
    def select_issue(event):
        issue = event.args[1]
        explanation.set_text(issue_text(issue))
        detail.set_content(json.dumps(issue.get('details', {}), indent=2, default=str))
    issues.on('rowClick', select_issue)
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
        set_rows(issues, [dict(issue, id=f'{issue["category"]}:{issue["status"]}') for issue in snapshot.get('user_issues', [])])
        set_rows(operations, [dict(op, message=(op.get('user_issue') or {}).get('title', op.get('message', ''))) for op in snapshot['operations']])
        for title, value in [('Task Runtime', diag['tasks']), ('Routes', diag['routes']), ('Fingerprint', diag['fingerprint']), ('Raw Snapshot', diag)]:
            text = json.dumps(value, indent=2, default=str)
            if codes[title].content != text:
                codes[title].set_content(text)
    return refresh
