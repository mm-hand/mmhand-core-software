"""Manual endpoint capture; confirmations and persistence belong to HandBackend.

职责：展示快照并提交后端操作。
线程：NiceGUI 页面事件循环。
输入：后端快照及用户交互。
输出：原生控件和操作请求。
状态所有权：仅目标草稿和显示缓存；设备真值归后端。
禁止依赖：禁止直接操作协议、串口和控制器。
"""
from nicegui import ui
from .common import FINGERS, finger_cards, selection, label, show, quality

def build(hand, act, load_calibration, confirm):
    refreshers = []
    with ui.row().classes('page-toolbar'):
        ui.button('Load Joint & Motor Calibration', on_click=load_calibration)
        ui.button('Clear Manual Selected', on_click=lambda: confirm('Clear selected Manual endpoints and their Auto/LUT results?', hand.request_clear_manual))
        ui.button('Release Selected Tendons', on_click=lambda: confirm('Release only the selected joints\' tendons. Their motor calibration and multi-turn anchors will be reset. Stop motion first.', hand.request_release_rope))
        counts = label()
    with finger_cards():
        for name, indices in FINGERS:
            with ui.card().classes('w-full'):
                refreshers.append(selection(hand, 'selected_joints', indices, name))
                for j in indices:
                    with ui.column().classes('joint-row w-full gap-1'):
                        with ui.row().classes('items-center gap-2'):
                            refreshers.append(selection(hand, 'selected_joints', [j], f'J{j:02d}'))
                            badge, raw = quality(), label()
                            with raw:
                                ui.tooltip('Raw Dir: ↑ increases raw counts; ↓ decreases raw counts as the joint angle increases.')
                        with ui.row().classes('items-center gap-2'):
                            low = label()
                            ui.button('Set Min', on_click=lambda j=j: act(lambda: hand.request_manual_capture(j, 'min'))).props('dense outline')
                            high = label()
                            ui.button('Set Max', on_click=lambda j=j: act(lambda: hand.request_manual_capture(j, 'max'))).props('dense outline')
                        info = label()
                    def refresh(snapshot, j=j, badge=badge, raw=raw, low=low, high=high, info=info):
                        row = snapshot['joints'][j]
                        hint = 'Transaction failed. See System Log and Operation History for the actual reason.' if 'FAILED' in row['joint_quality'] else f"Manual span: {show(row['span'])}; URDF: {row['urdf_span']}; difference: {show(row['difference'])} counts. Installation warning threshold >360 counts; possible magnetic encoder or joint installation error."
                        badge(row['joint_quality'], hint)
                        raw.set_text(f"Raw Dir: {'↑' if row['direction'] > 0 else '↓'}   Current Raw: {show(row['raw'])}")
                        low.set_text(f"Manual Min: {show(row['manual_min'])}"); high.set_text(f"Manual Max: {show(row['manual_max'])}")
                        info.set_text(f"Span: {show(row['span'])}   Auto Min: {show(row['auto_min'])}   Auto Max: {show(row['auto_max'])}   RAM/JSON: {row['persistence']}\nURDF Span: {row['urdf_span']}   URDF Min: {show(row['min'], '°')}   Current Min: {show(row['current_min'], '°')}   URDF Max: {show(row['max'], '°')}")
                    refreshers.append(refresh)
    def refresh(snapshot):
        counts.set_text(f"Manual: {sum(j['manual_valid'] for j in snapshot['joints'])}/21   Auto: {sum(j['auto_valid'] for j in snapshot['joints'])}/21")
        for update in refreshers:
            update(snapshot)
    return refresh
