"""Joint-scoped automatic calibration and all associated motor feedback.

职责：展示快照并提交后端操作。
线程：NiceGUI 页面事件循环。
输入：后端快照及用户交互。
输出：原生控件和操作请求。
状态所有权：仅目标草稿和显示缓存；设备真值归后端。
禁止依赖：禁止直接操作协议、串口和控制器。
"""
from nicegui import ui
from .common import (FINGERS, finger_cards, selection, label, show, quality, table, set_rows, joint_label, display_joints,
                     maintenance_allowed, maintenance_button, auto_span, request_calibration, select_all_and_request)


def detail_text(row, limits='—'):
    """Two compact lines: Auto endpoints/angle, then limits and warning counts."""
    result = row.get('result') or {}
    offset = result.get('autoMinOffsetCounts')
    shortfall = result.get('autoMaxShortfallCounts')
    return '\n'.join([
        f"Auto Min: {show(row.get('auto_min'))}   Auto Max: {show(row.get('auto_max'))}   "
        f"Auto Span: {show(auto_span(row))}   Joint Angle: {show(row.get('angle'), '°')}",
        f"ABS Limits: {limits}   Return offset: {show(offset)}   "
        f"Travel shortfall: {show(shortfall)} counts",
    ])


def build(hand, act, load_calibration):
    refreshers = []
    with ui.row().classes('page-toolbar'):
        ui.button('Load Joint & Motor Calibration', on_click=load_calibration)
        calibrate = maintenance_button(ui.button(
            'Calibrate Selected', on_click=lambda: act(lambda: request_calibration(hand, 'Motor Calibration'))))
        calibrate_all = maintenance_button(ui.button(
            'Calibrate All', on_click=lambda: act(lambda: select_all_and_request(
                hand, 'selected_joints', lambda: request_calibration(hand, 'Motor Calibration')))))
        selected = label()
    with finger_cards():
        for name, indices in FINGERS:
            with ui.card().classes('w-full'):
                refreshers.append(selection(hand, 'selected_joints', indices, name))
                for j in display_joints(indices):
                    with ui.column().classes('joint-row w-full gap-1'):
                        with ui.row().classes('items-center'):
                            refreshers.append(selection(hand, 'selected_joints', [j], joint_label(j)))
                            badge = quality()
                        motors = table([(k, v) for k, v in [('motor', 'Motor'), ('abs', 'ABS'), ('load', 'Load')]], 'motor')
                        info = label().classes('text-xs leading-tight')
                    def refresh(snapshot, j=j, badge=badge, motors=motors, info=info):
                        row, result = snapshot['joints'][j], snapshot['joints'][j]['result']
                        offset, shortfall = result.get('autoMinOffsetCounts'), result.get('autoMaxShortfallCounts')
                        causes = ('tendon friction, preload or mechanical resistance' if len(row['motors']) == 2
                                  else 'weak spring return, tendon friction, preload or mechanical resistance')
                        hint = 'Transaction failed. See System Log and Operation History for the actual reason.' if 'FAILED' in row['motor_quality'] else f"Return offset: {show(offset)} (warning ≥350); travel shortfall: {show(shortfall)} (warning ≥300) counts. Possible causes: {causes}; forces are not measured."
                        badge(row['motor_quality'], hint)
                        set_rows(motors, [dict(m, motor=f"M{m['motor']:02d}") for m in snapshot['motors'] if m['joint'] == j])
                        limits = ' / '.join(f"M{m['motor']:02d}: {show(m['limits'])}" for m in snapshot['motors'] if m['joint'] == j)
                        info.set_text(detail_text(row, limits))
                    refreshers.append(refresh)
    def refresh(snapshot):
        calibrate.set_enabled(maintenance_allowed(snapshot))
        calibrate_all.set_enabled(maintenance_allowed(snapshot))
        chosen = set(snapshot['selected_joints'])
        selected.set_text(f"Selected: {len(chosen)} joints / {sum(m['joint'] in chosen for m in snapshot['motors'])} motors")
        for update in refreshers:
            update(snapshot)
    return refresh
