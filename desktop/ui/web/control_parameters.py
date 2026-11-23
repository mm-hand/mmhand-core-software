"""Control parameter editor for P/I/D, fast reversal and adaptive control.

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
from desktop.core.control_parameters import BOUNDS, RAMP_BOUNDS
from . import common

TITLES = {
    'joint_kp': 'Kp config',
    'joint_ki': 'Ki config',
    'joint_kd': 'Kd config',
    'adaptive_kp': 'Adaptive Control',
    'dual_tendon': 'Dual-tendon pair recovery',
    'feedback': 'Encoder windows',
    'collision': 'Collision thresholds',
    'output': 'Output limits',
    'ros2_input': 'ROS2 One Euro and stationary anchor',
}

TYPES = (('AA', 'aa'), ('MCP', 'mcp'), ('PIP', 'pip'), ('DIP', 'dip'), ('CMC', 'cmc'))
ADAPTIVE_LOGIC = (
    ('Kp max scale', '2.0×'),
    ('Kp adjustment window', '10 s'),
    ('Kp up step', '8% with repeated lag and zero overshoot'),
    ('Kp lag evidence', '300 ms'),
    ('Kp down step', '10% with repeated overshoot'),
    ('Kp min scale', '0.3×'),
    ('Kp reduction evidence', 'Sustained lead in the confirmed direction after joint travel'),
    ('Learned scales', 'persist across STOP/Apply; reflash resets'),
)


def _joint_type(joint):
    return 'CMC' if joint == 20 else ('AA', 'MCP', 'PIP', 'DIP')[joint % 4]


def _adaptive_enabled(value):
    return bool(value and value > 0.5)


def _finger_name(joint):
    for name, joints in common.FINGERS:
        if joint in joints:
            return name
    return ''


def build(hand, act):
    controls = {}
    with ui.row().classes('page-toolbar'):
        save = ui.button('Apply Parameters')
        with save:
            ui.tooltip('Apply the drafted parameter revision. The host writes it to the configuration and P4 applies the whole revision when idle.')
        defaults = ui.button('Restore Defaults', on_click=lambda: act(hand.request_restore_control_defaults))
        status = ui.label().classes('text-sm')
    ui.label('P/I/D and fast reversal are per joint type; adaptive control adjusts Kp per joint. P4 applies an entire revision only when idle.').classes('text-sm')

    def number(group, name, label=None):
        limits = RAMP_BOUNDS if group == 'reversal_ramp' else BOUNDS[group]
        lower, upper = limits[name]
        controls[(group, name)] = ui.number(label or name.upper(), min=lower, max=upper,
                                            step=1 if isinstance(lower, int) and isinstance(upper, int) else 0.01) \
            .props('dense outlined').classes('w-full')

    adaptive_enable = [None]
    adaptive_status = [None]
    adaptive_table = [None]

    with ui.column().classes('w-full gap-3'):
        for group in ('joint_kp', 'joint_ki', 'joint_kd'):
            with ui.expansion(TITLES[group], value=True).classes('w-full bg-white rounded'):
                with ui.row().classes('w-full gap-3'):
                    for name in BOUNDS[group]:
                        number(group, name)

        with ui.expansion('Fast reversal', value=True).classes('w-full bg-white rounded'):
            with ui.row().classes('w-full gap-3 items-start'):
                for label, key in TYPES:
                    with ui.card().classes('flex-1 min-w-0'):
                        ui.label(label).classes('font-semibold text-sm')
                        with ui.column().classes('w-full gap-2'):
                            number('reversal', f'confirm_deg_{key}', 'Confirm deg')
                            number('reversal', f'takeup_max_{key}', 'Takeup max')
                            number('reversal', f'exit_travel_deg_{key}', 'Exit travel deg')
                            number('reversal_ramp', f'b0_{key}', 'B0')
                            number('reversal_ramp', f'vb_{key}', 'Base vB')
            with ui.row().classes('w-full gap-3'):
                number('reversal', 'confirm_ms', 'Confirm ms')
                number('reversal_ramp', 'takeup_timeout_ms', 'Takeup timeout ms')

        with ui.expansion('Adaptive Control', value=True).classes('w-full bg-white rounded'):
            with ui.row().classes('w-full items-center gap-3'):
                adaptive_enable[0] = ui.button('Enable Adaptive Control', on_click=lambda: toggle_adaptive())
                adaptive_enable[0].set_enabled(False)
                adaptive_status[0] = ui.label('Adaptive: waiting for P4').classes('text-sm')
            with ui.row().classes('w-full gap-3'):
                number('adaptive_kp', 'lag_limit_s', 'Lag limit s')
                number('adaptive_kp', 'overshoot_deg', 'Kp overshoot deg')
            adaptive_logic = common.table([('item', 'Adaptive logic'), ('value', 'Value')], key='item')
            common.set_rows(adaptive_logic, [{'item': name, 'value': value} for name, value in ADAPTIVE_LOGIC])
            adaptive_table[0] = common.table([
                ('joint', 'Joint'), ('finger', 'Finger'), ('type', 'Type'),
                ('base', 'Base Kp'), ('scale', 'Kp scale'), ('effective', 'Kp effective'),
            ])

        for group in ('dual_tendon', 'feedback', 'collision', 'output', 'ros2_input'):
            with ui.expansion(TITLES[group]).classes('w-full bg-white rounded'):
                with ui.row().classes('w-full gap-3'):
                    for name in BOUNDS[group]:
                        number(group, name, name.replace('_', ' '))

    def toggle_adaptive():
        proposed = copy.deepcopy(hand.control_parameter_snapshot())
        proposed.pop('crc32', None)
        proposed.pop('ramp_crc32', None)
        proposed['adaptive_kp']['enabled'] = 0 if _adaptive_enabled(proposed['adaptive_kp']['enabled']) else 1
        for (group, name), widget in controls.items():
            proposed[group][name] = widget.value
        act(lambda: hand.request_apply_control_parameters(proposed))

    def submit():
        proposed = copy.deepcopy(hand.control_parameter_snapshot())
        proposed.pop('crc32', None)
        proposed.pop('ramp_crc32', None)
        for (group, name), widget in controls.items():
            proposed[group][name] = widget.value
        act(lambda: hand.request_apply_control_parameters(proposed))
    save.on('click', submit)

    draft_revision = [None]

    def refresh(snapshot):
        data = snapshot['control_parameters']
        if draft_revision[0] != (data['revision'], data['ramp_revision']):
            for (group, name), widget in controls.items(): widget.set_value(data[group][name])
            draft_revision[0] = (data['revision'], data['ramp_revision'])
        status.set_text(f"Desired revision {data['revision']} / ramp {data['ramp_revision']} · CRC32 {data['crc32']:08X} ramp {data['ramp_crc32']:08X} · P4: {snapshot['control_parameter_sync']}")
        adaptive = snapshot.get('adaptive_kp') or {}
        scales = adaptive.get('scale') or []
        effective = adaptive.get('effective') or []
        rows = []
        for joint in range(21):
            joint_type = _joint_type(joint)
            base = data['joint_kp'][joint_type.lower()]
            rows.append({
                'joint': f'J{joint:02d}',
                'finger': _finger_name(joint),
                'type': joint_type,
                'base': f'{base:.3f}',
                'scale': f'×{scales[joint]:.3f}' if joint < len(scales) else '—',
                'effective': f'{effective[joint]:.3f}' if joint < len(effective) else '—',
            })
        common.set_rows(adaptive_table[0], rows)
        enabled = _adaptive_enabled(data['adaptive_kp']['enabled'])
        adaptive_enable[0].set_text('Disable Adaptive Control' if enabled else 'Enable Adaptive Control')
        # ponytail: only gated on busy; add the IDLE/stop-latched gate if the P4 ever applies outside idle.
        adaptive_enable[0].set_enabled(not snapshot['busy'])
        if adaptive.get('valid'):
            scale_values = [value for value in scales if value]
            span = f'{min(scale_values):.3f}–{max(scale_values):.3f}' if scale_values else '—'
            adaptive_status[0].set_text(
                f"Adaptive: {'ON' if adaptive.get('enabled') else 'OFF'} · "
                f"updated {adaptive.get('age_ms', 0)} ms ago · Kp scale {span}")
        else:
            adaptive_status[0].set_text('Adaptive: waiting for P4 1 Hz packet (showing configured base Kp)')
        save.set_enabled(not snapshot['busy'])
        defaults.set_enabled(not snapshot['busy'])
    return refresh
