"""Manual endpoint capture; confirmations and persistence belong to HandBackend.

职责：展示快照并提交后端操作。
线程：NiceGUI 页面事件循环。
输入：后端快照及用户交互。
输出：原生控件和操作请求。
状态所有权：仅目标草稿和显示缓存；设备真值归后端。
禁止依赖：禁止直接操作协议、串口和控制器。
"""
from nicegui import ui
from .common import (FINGERS, finger_cards, selection, label, show, quality,
                     maintenance_allowed, maintenance_button, json_status)

BADGE_TOOLTIP = ('Manual endpoint state. Release keeps the Manual endpoints; '
                 'automatic calibration state is on Motor Calibration.')


def detail_text(row):
    """One compact line below the Raw/Dir/Manual endpoint row."""
    return (f"Manual Span: {show(row['span'])}   "
            f"URDF Span: {show(row['urdf_span'])}   "
            f"URDF Min (deg): {show(row['min'])}   "
            f"URDF Max (deg): {show(row['max'])}   JSON: {json_status(row)}")


def build(hand, act, load_calibration, confirm):
    refreshers = []
    with ui.row().classes('page-toolbar'):
        load_button = maintenance_button(ui.button('Load Joint & Motor Calibration', on_click=load_calibration))
        clear_button = maintenance_button(ui.button('Clear Manual Selected', on_click=lambda: confirm(
            'Clear selected Manual endpoints and their Auto/LUT results?', hand.request_clear_manual)))
        release_button = maintenance_button(ui.button('Release Selected Tendons', on_click=lambda: confirm(
            "Release only the selected joints' tendons by another 1500 counts. Their motor calibration and multi-turn anchors will be reset. Stop motion first.",
            hand.request_release_rope)))
        counts = label()
    with finger_cards():
        for name, indices in FINGERS:
            with ui.card().classes('w-full'):
                refreshers.append(selection(hand, 'selected_joints', indices, name))
                for j in indices:
                    with ui.column().classes('joint-row w-full gap-1'):
                        with ui.row().classes('items-center gap-2'):
                            refreshers.append(selection(hand, 'selected_joints', [j], f'J{j:02d}'))
                            badge = quality()
                        with ui.row().classes('items-center gap-2 flex-nowrap'):
                            raw = label().classes('text-base font-bold text-slate-900 whitespace-nowrap')
                            with raw:
                                ui.tooltip('Current raw encoder count.')
                            min_value = ui.input('Manual Min', value='—').props('dense outlined readonly').classes('w-24')
                            min_button = maintenance_button(
                                ui.button('Set Min', on_click=lambda j=j: act(lambda: hand.request_manual_capture(j, 'min'))).props('dense outline'))
                            max_value = ui.input('Manual Max', value='—').props('dense outlined readonly').classes('w-24')
                            max_button = maintenance_button(
                                ui.button('Set Max', on_click=lambda j=j: act(lambda: hand.request_manual_capture(j, 'max'))).props('dense outline'))
                            capture_status = label()
                        info = label().classes('text-xs leading-tight')
                    def refresh(snapshot, j=j, badge=badge, info=info, raw=raw,
                                min_value=min_value, max_value=max_value,
                                min_button=min_button, max_button=max_button,
                                capture_status=capture_status):
                        row = snapshot['joints'][j]
                        allowed = maintenance_allowed(snapshot)
                        min_button.set_enabled(allowed); max_button.set_enabled(allowed)
                        capture_status.set_text(row.get('capture_status', ''))
                        raw.set_text(f"Raw: {show(row.get('raw'))}")
                        min_value.set_value(show(row['manual_min']))
                        max_value.set_value(show(row['manual_max']))
                        if 'FAILED' in row['joint_quality']:
                            hint = 'Transaction failed. See System Log and Operation History for the actual reason.'
                        else:
                            hint = (f"Manual span {show(row['span'])} vs URDF span {row['urdf_span']} counts; "
                                    f"difference {show(row['difference'])} counts. Installation warning threshold >360 counts. "
                                    + BADGE_TOOLTIP)
                        badge(row['joint_quality'], hint)
                        info.set_text(detail_text(row))
                    refreshers.append(refresh)
    def refresh(snapshot):
        allowed = maintenance_allowed(snapshot)
        for button in (load_button, clear_button, release_button):
            button.set_enabled(allowed)
        counts.set_text(f"Manual: {sum(j['manual_valid'] for j in snapshot['joints'])}/21")
        for update in refreshers:
            update(snapshot)
    return refresh
