"""Joint-scoped automatic calibration and all associated motor feedback.

职责：展示快照并提交后端操作。
线程：NiceGUI 页面事件循环。
输入：后端快照及用户交互。
输出：原生控件和操作请求。
状态所有权：仅目标草稿和显示缓存；设备真值归后端。
禁止依赖：禁止直接操作协议、串口和控制器。
"""
from nicegui import ui
from .common import FINGERS, finger_cards, selection, label, show, quality, table, set_rows

def build(hand, act, load_calibration):
    refreshers = []
    with ui.row().classes('page-toolbar'):
        ui.button('Load Joint & Motor Calibration', on_click=load_calibration)
        ui.button('Calibrate Selected', on_click=lambda: act(hand.request_calibration))
        selected = label()
    with finger_cards():
        for name, indices in FINGERS:
            with ui.card().classes('w-full'):
                refreshers.append(selection(hand, 'selected_joints', indices, name))
                for j in indices:
                    with ui.column().classes('joint-row w-full gap-1'):
                        with ui.row().classes('items-center'):
                            refreshers.append(selection(hand, 'selected_joints', [j], f'J{j:02d}'))
                            badge = quality()
                        motors = table([(k, v) for k, v in [('motor', 'Motor'), ('bus', 'BUS'), ('route', 'Route'), ('turn', 'Turn'), ('raw', 'RawNow'), ('saved_raw', 'SavedRaw'), ('abs', 'AbsNow'), ('load', 'Load')]], 'motor')
                        info = label()
                    def refresh(snapshot, j=j, badge=badge, motors=motors, info=info):
                        row, result = snapshot['joints'][j], snapshot['joints'][j]['result']
                        offset, shortfall = result.get('autoMinOffsetCounts'), result.get('autoMaxShortfallCounts')
                        causes = ('tendon friction, preload or mechanical resistance' if len(row['motors']) == 2
                                  else 'weak spring return, tendon friction, preload or mechanical resistance')
                        hint = 'Transaction failed. See System Log and Operation History for the actual reason.' if 'FAILED' in row['motor_quality'] else f"Return offset: {show(offset)} (warning ≥350); travel shortfall: {show(shortfall)} (warning ≥300) counts. Possible causes: {causes}; forces are not measured."
                        badge(row['motor_quality'], hint)
                        set_rows(motors, [dict(m, motor=f"M{m['motor']:02d}") for m in snapshot['motors'] if m['joint'] == j])
                        limits = ' / '.join(f"M{m['motor']:02d}: {show(m['limits'])}" for m in snapshot['motors'] if m['joint'] == j)
                        info.set_text(f"Joint Angle: {show(row['angle'], '°')}   ABS Limits: {limits}\nReturn offset: {show(offset)} counts   Travel shortfall: {show(shortfall)} counts")
                    refreshers.append(refresh)
    def refresh(snapshot):
        chosen = set(snapshot['selected_joints'])
        selected.set_text(f"Selected: {len(chosen)} joints / {sum(m['joint'] in chosen for m in snapshot['motors'])} motors")
        for update in refreshers:
            update(snapshot)
    return refresh
