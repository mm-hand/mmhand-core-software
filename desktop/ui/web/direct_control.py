"""Logical ABS drafts; existing backend owns direct control and tendon movement.

职责：展示快照并提交后端操作。
线程：NiceGUI 页面事件循环。
输入：后端快照及用户交互。
输出：原生控件和操作请求。
状态所有权：仅目标草稿和显示缓存；设备真值归后端。
禁止依赖：禁止直接操作协议、串口和控制器。
"""
from nicegui import ui
from .common import (FINGERS, finger_cards, selection, target_editor, label, show, quality,
                     control_note, select_all_and_request)

ZERO_PROMPT = (
    'No motor calibration is loaded, so Direct Control has no motor coordinates. '
    'Load a calibration file that includes motor calibration, or continue to set every '
    "online motor's current position as ABS 0. Continuing clears motor calibration from "
    'the saved calibration file. Motor Calibration zeroes the motors again when it starts.')


def build(hand, act, confirm, leave=None):
    drafts, refreshers, initialized = {}, [], set()
    answered = [False]

    def cancel_zero():
        """未归零就不允许留在 Direct：关窗并回上一个页面。"""
        zero_dialog.close()
        answered[0] = False
        if leave is not None:
            leave()

    def confirm_zero():
        zero_dialog.close()

        def zero():
            result = hand.request_direct_zero_all()
            if not result.accepted:
                answered[0] = False  # 归零没开始：下次进来重新拦
            return result
        act(zero)

    # 弹窗只在建页时创建一次：渲染回调里只 open/close，沿用标定告警弹窗的写法。
    # persistent：ESC / 点遮罩都不能绕过，只能 Cancel（离开）或 Confirm（归零）。
    with ui.dialog().props('persistent') as zero_dialog, ui.card().classes('w-[600px] max-w-full'):
        ui.label(ZERO_PROMPT).classes('text-sm whitespace-pre-wrap')
        with ui.row():
            ui.button('Cancel', on_click=cancel_zero).props('flat dense')
            ui.button('Set All Motors to ABS 0', on_click=confirm_zero).props('dense color=primary')

    def prompt_zero(snapshot):
        """缺电机坐标时提示：载入标定文件，或把所有在线电机设为当前 ABS 0。"""
        if not snapshot.get('direct_zero_required'):
            answered[0] = False
            return
        if answered[0] or zero_dialog.value:
            return
        answered[0] = True
        zero_dialog.open()
    initial = hand.application_snapshot(include_curves=False)
    zero_generations = {row['motor']: row.get('abs_zero_generation', 0) for row in initial['motors']}
    def send_selected():
        selected = hand.application_snapshot(include_curves=False)['selected_motors']
        act(lambda: hand.request_motors({m: drafts[m]['value'] for m in selected}))
    with ui.row().classes('page-toolbar'):
        ui.button('ENABLE Selected', on_click=lambda: act(hand.request_direct_enable))
        ui.button('ENABLE All', on_click=lambda: act(lambda: select_all_and_request(hand, 'selected_motors', hand.request_direct_enable)))
        ui.button('Send Selected', on_click=send_selected)
        ui.label('Logical ABS (counts)').classes('text-sm text-slate-600')
    control_note()
    with finger_cards():
        for name, joints in FINGERS:
            indices = [m['motor'] for m in initial['motors'] if m['joint'] in joints]
            with ui.card().classes('w-full'):
                refreshers.append(selection(hand, 'selected_motors', indices, name))
                for m in indices:
                    with ui.column().classes('joint-row w-full gap-1'):
                        with ui.row().classes('items-center'):
                            refreshers.append(selection(hand, 'selected_motors', [m], f'M{m:02d}'))
                            control = quality()
                            feedback = label()
                        drafts[m], number, slider = target_editor(
                            0, 0, 1, integer=True,
                            on_commit=lambda value, m=m: act(lambda: hand.request_motors({m: value})))
                        number.set_enabled(False); slider.set_enabled(False)
                        with ui.row().classes('items-center gap-2'):
                            send = ui.button('Send', on_click=lambda m=m: act(lambda: hand.request_motors({m: drafts[m]['value']}))).props('dense outline')
                            ui.button('−', on_click=lambda m=m, number=number: number.set_value(drafts[m]['value'] - 1)).props('dense outline')
                            ui.button('+', on_click=lambda m=m, number=number: number.set_value(drafts[m]['value'] + 1)).props('dense outline')
                            limits = label()
                    def refresh(snapshot, m=m, feedback=feedback, limits=limits, number=number, slider=slider, send=send, control=control):
                        row = snapshot['motors'][m]
                        control([row['direct_control_status']], f"P4-confirmed direct control state for M{m:02d} / J{row['joint']:02d}.")
                        generation = row.get('abs_zero_generation', 0)
                        if generation != zero_generations[m] and row['abs'] is not None:
                            number.set_value(row['abs'])
                            zero_generations[m] = generation
                            initialized.add(m)
                        feedback.set_text(f"ABS: {show(row['abs'])}   Load: {show(row['load'])}   Joint Angle: {show(snapshot['joints'][row['joint']]['angle'], '°')}")
                        valid = row['limits'] is not None and row['abs'] is not None
                        number.set_enabled(valid); slider.set_enabled(valid); send.set_enabled(valid)
                        if valid:
                            lo, hi = row['limits']
                            if number._props.get('min') != lo or number._props.get('max') != hi:
                                for widget in (number, slider):
                                    widget._props.update(min=lo, max=hi)
                            if m not in initialized:
                                number.set_value(row['abs'])
                                initialized.add(m)
                        limits.set_text(f"ABS Limits: {show(row['limits'])}")
                    refreshers.append(refresh)
    def refresh(snapshot):
        for update in refreshers:
            update(snapshot)
        prompt_zero(snapshot)
    return refresh
