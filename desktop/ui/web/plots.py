"""Single-joint angle plus per-motor control-component and load charts; Save Data is the sole switch.

职责：展示快照并处理本地显示交互。
线程：NiceGUI 页面事件循环。
输入：后端同步快照。
输出：原生控件。
状态所有权：仅显示缓存；设备真值归后端。
禁止依赖：禁止直接操作协议、串口和控制器。
"""
import time

from nicegui import ui

from .common import joint_label

WINDOW_S = 10.0
TERMS = ('P', 'I', 'D', 'Reversal', 'Pair recovery', 'Total')


def _motor_values(row, motor):
    """One P4 cycle's terms; None when this servo sample is invalid."""
    components, detail = row.get('components') or {}, row.get('detail') or {}
    if not detail.get('servo_valid_mask', 0) & (1 << motor):
        return None
    def get(source, key):
        values = source.get(key) or ()
        return values[motor] if motor < len(values) else None
    p, i, r = (get(components, key) for key in
               ('proportional_counts', 'integral_counts', 'reversal_counts'))
    pid, recovery = get(detail, 'pid_counts'), get(detail, 'recovery_counts')
    return {'P': p, 'I': i, 'D': None if None in (p, i, r, pid) else pid - p - i - r,
            'Reversal': r, 'Pair recovery': recovery,
            'Total': None if None in (pid, recovery) else pid + recovery}


def _motor_load(row, motor):
    """One curve sample's signed servo load; None when this sample carries no load."""
    loads = row.get('loads') or ()
    return loads[motor] if motor < len(loads) else None


def _options(title, x_name, y_name):
    return {'title': {'text': title, 'left': 'center'}, 'legend': {'top': 26},
            'tooltip': {'trigger': 'axis'}, 'xAxis': {'type': 'value', 'name': x_name},
            'yAxis': {'type': 'value', 'name': y_name},
            'grid': {'left': 65, 'right': 25, 'bottom': 50, 'top': 64}, 'series': []}


def build():
    cleared_at = [0.0]
    with ui.row().classes('page-toolbar'):
        ui.button('Clear Plot', on_click=lambda: cleared_at.__setitem__(0, time.time()))
        joint = ui.select({-1: 'None', **{j: joint_label(j) for j in range(21)}},
                          value=-1, label='Joint').props('dense outlined').classes('w-44')
    angle = ui.echart(_options('Joint angle', 'Time (s)', 'Angle (°)')).classes('w-full').style('height:26rem;flex-shrink:0')
    ui.label('Each motor: P, I, D, reversal, pair recovery and Total (= PID + pair recovery). '
             'D = PID − P − I − reversal.').classes('text-sm')
    with ui.row().classes('w-full gap-3'):
        motor_charts = [ui.echart(_options('', 'P4 time (s)', 'Counts'))
                        .classes('flex-1 min-w-0').style('height:22rem') for _ in range(2)]
    with ui.row().classes('w-full gap-3'):
        load_charts = [ui.echart(_options('', 'Time (s)', 'Load (counts)'))
                       .classes('flex-1 min-w-0').style('height:16rem') for _ in range(2)]
    for chart in motor_charts + load_charts:
        chart.set_visibility(False)

    def refresh(snapshot):
        joint_index = joint.value
        history = [row for row in snapshot['curves'] if row['host_time'] >= cleared_at[0]]
        start = history[0]['host_time'] if history else 0.0
        end = history[-1]['host_time'] - start if history else WINDOW_S
        begin = max(0.0, end - WINDOW_S)
        window = [row for row in history if row['host_time'] - start >= begin]
        keys = ((('Feedback', 'angles', 'solid'), ('Target', 'targets', 'dashed'))
                if joint_index >= 0 else ())
        angle.options['series'] = [dict(type='line', name=name, showSymbol=False, connectNulls=False,
            lineStyle={'type': style}, data=[[row['host_time'] - start, row[key][joint_index]] for row in window])
            for name, key, style in keys]
        angle.options['xAxis'].update(min=begin, max=end)
        angle.update()

        motors = snapshot['joints'][joint_index]['motors'] if joint_index >= 0 else ()
        rows = [row for row in snapshot.get('control_components', ()) if row['host_time'] >= cleared_at[0]]
        p4_end = rows[-1]['p4_time_ms'] / 1000 if rows else WINDOW_S
        p4_begin = max(0.0, p4_end - WINDOW_S)
        rows = [row for row in rows if row['p4_time_ms'] / 1000 >= p4_begin]
        times = [row['p4_time_ms'] / 1000 for row in rows]
        for chart, motor in zip(motor_charts, motors):
            values = [_motor_values(row, motor) for row in rows]
            chart.options['title']['text'] = f'J{joint_index:02d} · M{motor:02d}'
            chart.options['series'] = [dict(type='line', name=term, showSymbol=False, connectNulls=False,
                data=[[t, None if value is None else value[term]] for t, value in zip(times, values)])
                for term in TERMS]
            chart.options['xAxis'].update(min=p4_begin, max=p4_end)
            chart.update()
            chart.set_visibility(True)
        for chart in motor_charts[len(motors):]:
            chart.set_visibility(False)
        for chart, motor in zip(load_charts, motors):
            chart.options['title']['text'] = f'J{joint_index:02d} · M{motor:02d} load'
            chart.options['series'] = [dict(type='line', name='Load', showSymbol=False, connectNulls=False,
                data=[[row['host_time'] - start, _motor_load(row, motor)] for row in window])]
            chart.options['xAxis'].update(min=begin, max=end)
            chart.update()
            chart.set_visibility(True)
        for chart in load_charts[len(motors):]:
            chart.set_visibility(False)

    return refresh
