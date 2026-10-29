"""Local target drafts; slider release and Send use backend authorization.

职责：展示快照并提交后端操作。
线程：NiceGUI 页面事件循环。
输入：后端快照及用户交互。
输出：原生控件和操作请求。
状态所有权：仅目标草稿和显示缓存；设备真值归后端。
禁止依赖：禁止直接操作协议、串口和控制器。
"""
from nicegui import ui
from .common import FINGERS, finger_cards, selection, target_editor, label, show, quality, control_note, select_all_and_request

def build(hand, act):
    drafts, refreshers, syncing = {}, [], [False]
    def send_selected():
        selected = hand.application_snapshot(include_curves=False)['selected_joints']
        act(lambda: hand.request_targets({j: drafts[j]['value'] for j in selected}))
    with ui.row().classes('page-toolbar'):
        ui.button('ENABLE Selected', on_click=lambda: act(hand.request_enable))
        ui.button('ENABLE All', on_click=lambda: act(lambda: select_all_and_request(hand, 'selected_joints', hand.request_enable)))
        ui.button('Send Selected', on_click=send_selected)
        ros = ui.switch('ROS2 INPUT', on_change=lambda e: None if syncing[0] else act(lambda: hand.request_ros_input(e.value)))
    control_note()
    initial = hand.application_snapshot(include_curves=False)
    operation_status = ui.label().classes('text-sm text-slate-700 whitespace-pre-wrap w-full')
    with finger_cards():
        for name, indices in FINGERS:
            with ui.card().classes('w-full'):
                refreshers.append(selection(hand, 'selected_joints', indices, name))
                for j in indices:
                    row = initial['joints'][j]
                    with ui.column().classes('joint-row w-full gap-1'):
                        with ui.row().classes('items-center'):
                            refreshers.append(selection(hand, 'selected_joints', [j], f'J{j:02d}'))
                            control = quality()
                            feedback = label()
                        drafts[j], number, slider = target_editor(
                            max(row['min'], min(row['max'], row['target'])), row['min'], row['max'],
                            on_commit=lambda value, j=j: act(lambda: hand.request_targets({j: value})))
                        with ui.row().classes('items-center'):
                            limits = label()
                            ui.button('Send', on_click=lambda j=j: act(lambda: hand.request_targets({j: drafts[j]['value']}))).props('dense outline')
                    def refresh(snapshot, j=j, feedback=feedback, limits=limits, control=control):
                        row = snapshot['joints'][j]
                        statuses = [row['feedback_control_status']]
                        if row.get('collision_detected'):
                            statuses.append('COLLISION DETECTED')
                        if row.get('angle_out_of_workspace'):
                            statuses.append('JOINT BEYOND WORKSPACE')
                        if any(snapshot['motors'][m].get('motor_beyond_workspace') for m in row['motors']):
                            statuses.append('MOTOR BEYOND WORKSPACE')
                        control(statuses, 'P4-confirmed feedback control, per-joint collision, joint-angle and motor workspace state.')
                        control_abs = ' / '.join(show(snapshot['motors'][m]['abs']) for m in row['motors'])
                        feedback.set_text(f"Enc: {row['encoder']}   Current: {show(row['angle'], '°')}   ROS2 Target: {show(row['ros_target'], '°')}\nRaw: {show(row['raw'])}   Control ABS: {control_abs}")
                        limits.set_text(f"Min: {show(row['min'], '°')}   Max: {show(row['max'], '°')}")
                    refreshers.append(refresh)
    def refresh(snapshot):
        operation_status.set_text(snapshot.get('message', ''))
        syncing[0] = True
        ros.set_value(snapshot['ros']['enabled'])
        ros.set_enabled(snapshot['ros']['available'] and (not snapshot['ros']['reason'] or snapshot['ros']['enabled']))
        syncing[0] = False
        for update in refreshers:
            update(snapshot)
    return refresh
