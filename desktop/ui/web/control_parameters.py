"""Control parameter editor for the two shared tendon groups and common safety settings.

职责：展示快照并提交用户操作。
线程：NiceGUI页面事件循环。
输入：后端只读快照及用户输入。
输出：原生控件和操作请求。
状态所有权：仅持有输入草稿和展示缓存。
禁止依赖：禁止串口、协议编码或直接硬件操作。
"""
from __future__ import annotations

import copy
from nicegui import ui
from desktop.core.control_parameters import BOUNDS

TITLES = {
    'joint_kp': 'Kp config',
    'joint_ki': 'Ki config',
    'joint_kd': 'Kd config',
    'dual_tendon': 'Dual-tendon pair recovery',
    'feedback': 'Encoder windows', 'reversal': 'Fast reversal',
    'collision': 'Collision thresholds', 'output': 'Output limits',
    'ros2_input': 'ROS2 One Euro and stationary anchor',
}


def build(hand, act):
    controls = {}
    with ui.row().classes('page-toolbar'):
        save = ui.button('Apply Parameters')
        with save:
            ui.tooltip('Apply the drafted parameter revision. The host writes it to the configuration and P4 applies the whole revision when idle.')
        defaults = ui.button('Restore Defaults', on_click=lambda: act(hand.request_restore_control_defaults))
        status = ui.label().classes('text-sm')
    ui.label('Kp/Ki/Kd, integral limit/noise/confirm, reversal confirm/takeup/exit are per joint type; pair recovery applies to AA/CMC. Collision, windows, output and ROS2 timing remain global in schema v5. P4 applies an entire revision only when idle.').classes('text-sm')
    with ui.column().classes('w-full gap-3'):
        for group, fields in BOUNDS.items():
            with ui.expansion(TITLES[group], value=group in ('joint_kp', 'joint_ki', 'joint_kd', 'dual_tendon')).classes('w-full bg-white rounded'):
                with ui.row().classes('w-full gap-3'):
                    for name, (lower, upper) in fields.items():
                        controls[(group, name)] = ui.number(name.upper() if group.startswith('joint_') else name.replace('_', ' '), min=lower, max=upper,
                                                           step=1 if isinstance(lower, int) and isinstance(upper, int) else 0.01).props('dense outlined').classes('w-44')
    draft_revision = [None]

    def submit():
        proposed = copy.deepcopy(hand.control_parameter_snapshot())
        proposed.pop('crc32', None)
        for (group, name), widget in controls.items():
            proposed[group][name] = widget.value
        act(lambda: hand.request_apply_control_parameters(proposed))
    save.on('click', submit)

    def refresh(snapshot):
        data = snapshot['control_parameters']
        if draft_revision[0] != data['revision']:
            for (group, name), widget in controls.items(): widget.set_value(data[group][name])
            draft_revision[0] = data['revision']
        status.set_text(f"Desired revision {data['revision']} · CRC32 {data['crc32']:08X} · P4: {snapshot['control_parameter_sync']}")
        save.set_enabled(not snapshot['busy'])
        defaults.set_enabled(not snapshot['busy'])
    return refresh
