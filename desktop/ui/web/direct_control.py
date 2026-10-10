"""Logical ABS drafts; existing backend owns direct control and tendon movement.

职责：展示快照并提交后端操作。
线程：NiceGUI 页面事件循环。
输入：后端快照及用户交互。
输出：原生控件和操作请求。
状态所有权：仅目标草稿和显示缓存；设备真值归后端。
禁止依赖：禁止直接操作协议、串口和控制器。
"""
from nicegui import ui
from .common import FINGERS, finger_cards, selection, target_editor, label, show

def build(hand, act, confirm):
    drafts, refreshers, initialized = {}, [], set()
    initial = hand.application_snapshot(include_curves=False)
    def send_selected():
        selected = hand.application_snapshot(include_curves=False)['selected_motors']
        act(lambda: hand.request_motors({m: drafts[m]['value'] for m in selected}))
    with ui.row().classes('page-toolbar'):
        ui.button('ENABLE Selected', on_click=lambda: act(hand.request_direct_enable))
        ui.button('Pause', on_click=lambda: act(hand.request_pause))
        ui.button('Clear Faults', on_click=lambda: act(hand.request_clear_faults))
        ui.button('Send Selected', on_click=send_selected)
        ui.button('Release Half Turn', on_click=lambda: confirm('Move selected motors to release 2048 counts?', lambda: hand.request_tendon_step(False)))
        ui.button('Tighten Half Turn', on_click=lambda: confirm('Move selected motors to tighten 2048 counts?', lambda: hand.request_tendon_step(True)))
        ui.label('Logical ABS (counts)').classes('text-sm text-slate-600')
    with finger_cards():
        for name, joints in FINGERS:
            indices = [m['motor'] for m in initial['motors'] if m['joint'] in joints]
            with ui.card().classes('w-full'):
                refreshers.append(selection(hand, 'selected_motors', indices, name))
                for m in indices:
                    with ui.column().classes('joint-row w-full gap-1'):
                        with ui.row().classes('items-center'):
                            refreshers.append(selection(hand, 'selected_motors', [m], f'M{m:02d}'))
                            feedback = label()
                        drafts[m], number, slider = target_editor(0, 0, 1, integer=True)
                        number.set_enabled(False); slider.set_enabled(False)
                        with ui.row().classes('items-center gap-2'):
                            send = ui.button('Send', on_click=lambda m=m: act(lambda: hand.request_motors({m: drafts[m]['value']}))).props('dense outline')
                            ui.button('−', on_click=lambda m=m: drafts[m].update(value=drafts[m]['value'] - 1)).props('dense outline')
                            ui.button('+', on_click=lambda m=m: drafts[m].update(value=drafts[m]['value'] + 1)).props('dense outline')
                            limits = label()
                    def refresh(snapshot, m=m, feedback=feedback, limits=limits, number=number, slider=slider, send=send):
                        row = snapshot['motors'][m]
                        feedback.set_text(f"BUS: {show(row['bus'])}   Route: {show(row['route'])}   Turn: {show(row['turn'])}\nRawNow: {show(row['raw'])}   SavedRaw: {show(row['saved_raw'])}   AbsNow: {show(row['abs'])}   Load: {show(row['load'])}   Joint Angle: {show(snapshot['joints'][row['joint']]['angle'], '°')}")
                        valid = row['limits'] is not None and row['abs'] is not None
                        number.set_enabled(valid); slider.set_enabled(valid); send.set_enabled(valid)
                        if valid:
                            lo, hi = row['limits']
                            if number._props.get('min') != lo or number._props.get('max') != hi:
                                for widget in (number, slider):
                                    widget.props(f'min={lo} max={hi}')
                            if m not in initialized:
                                drafts[m]['value'] = row['abs']
                                initialized.add(m)
                        limits.set_text(f"ABS Limits: {show(row['limits'])}")
                    refreshers.append(refresh)
    def refresh(snapshot):
        for update in refreshers:
            update(snapshot)
    return refresh
