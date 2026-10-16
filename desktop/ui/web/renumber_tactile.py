"""Direct-S3 five-finger tactile inventory and batch ID editor.

职责：展示快照并提交用户操作。
线程：NiceGUI页面事件循环。
输入：后端只读快照及用户输入。
输出：原生控件和操作请求。
状态所有权：仅持有输入草稿和展示缓存。
禁止依赖：禁止串口、协议编码或直接硬件操作。
"""
from nicegui import ui
from .common import label

FINGERS = ((4, 'Thumb'), (3, 'Index'), (2, 'Middle'), (1, 'Ring'), (0, 'Little'))


def build(hand, act):
    inputs, generation = {}, [None]
    with ui.row().classes('page-toolbar'):
        scan = ui.button('Scan All Fingers', on_click=lambda: act(hand.request_tactile_scan))
        rename = ui.button('Renumber All', on_click=lambda: act(lambda: hand.request_tactile_renumber(
            {key: widget.value for key, widget in inputs.items()})))
    ui.label('S3 must be connected directly by USB. Fingertip: 0 · Distal segment: 1 · Proximal segment: 2. Blank New ID keeps the current ID.').classes('text-sm')
    ui.label('Multiple tactile sensors with the same ID on one bus cannot be identified or renumbered separately. Connect them separately before renumbering. IDs may repeat on different finger buses.').classes('text-sm text-amber-800')
    status, result = label(), label()
    rows = ui.column().classes('w-full')

    def refresh(snapshot):
        state = snapshot['id_reset']['tactile']
        available = state['available'] and not snapshot['busy']
        scan.set_enabled(available)
        rename.set_enabled(available and state['scanned'] and bool(state['rows']))
        status.set_text(state['message'] if state['available'] else 'Connect S3 directly by USB to scan and renumber.')
        result.set_text(state.get('last_result', ''))
        if generation[0] != state['generation']:
            generation[0] = state['generation']; rows.clear(); inputs.clear()
            with rows:
                for finger, name in FINGERS:
                    with ui.card().classes('w-full'):
                        ui.label(name).classes('font-semibold')
                        found = [row for row in state['rows'] if row['bus'] == finger]
                        if not found: ui.label('No sensor detected').classes('text-xs text-slate-500')
                        for row in found:
                            with ui.row().classes('items-center gap-4'):
                                ui.label(f"Current ID: {row['id']}").classes('w-28')
                                widget = ui.number('New ID', value=row.get('new_id'), min=0, max=2, step=1).props('dense outlined').classes('w-32')
                                inputs[(finger, row['id'])] = widget
                                ui.label('DP-S2015-Elite · 52 points' if row['model'] == 1 else
                                         'IP-S1610-Elite · 25 points' if row['model'] == 2 else 'Unknown model · read only').classes('text-xs')
        known = {(row['bus'], row['id']) for row in state['rows'] if (row['model'], row['points']) in ((1,52),(2,25))}
        for key, widget in inputs.items():
            widget.set_enabled(available and state['scanned'] and not state['recovering'] and key in known)
    return refresh
