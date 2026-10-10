"""Five distinct joint plots and recording information; Save Data is the sole switch.

职责：展示快照并提交后端操作。
线程：NiceGUI 页面事件循环。
输入：后端快照及用户交互。
输出：原生控件和操作请求。
状态所有权：仅目标草稿和显示缓存；设备真值归后端。
禁止依赖：禁止直接操作协议、串口和控制器。
"""
from nicegui import ui
from .common import label, show

def build(hand, act):
    selectors, cleared_at = [], [0.0]
    with ui.row().classes('page-toolbar'):
        ui.button('Clear Plot', on_click=lambda: cleared_at.__setitem__(0, __import__('time').time()))
        for i in range(5):
            selectors.append(ui.select({-1: 'None', **{j: f'J{j:02d}' for j in range(21)}}, value=0 if i == 0 else -1, label='Joints' if i == 0 else None).props('dense outlined').classes('w-28'))
    chart = ui.echart({'legend': {}, 'tooltip': {'trigger': 'axis'}, 'xAxis': {'type': 'value', 'name': 'Time (s)'},
        'yAxis': {'type': 'value', 'name': 'Angle (°)'}, 'grid': {'left': 65, 'right': 25, 'bottom': 50}, 'series': []}).classes('w-full h-96')
    recording = label()
    with ui.expansion('Recording Options').classes('w-full'):
        syncing = [False]
        full = ui.switch('Full tactile data', on_change=lambda e: None if syncing[0] else act(lambda: hand.set_recording_options(e.value)))
        ui.label('Topics: packet frames, core state, joint feedback, motor feedback, control trace, tactile summary and operation events.').classes('text-sm')
    def refresh(snapshot):
        chosen = []
        for selector in selectors:
            if selector.value != -1 and selector.value not in chosen:
                chosen.append(selector.value)
            selector.set_options({-1: 'None', **{j: f'J{j:02d}' for j in range(21) if j == selector.value or j not in [s.value for s in selectors if s is not selector]}})
        history = [r for r in snapshot['curves'] if r['host_time'] >= cleared_at[0]]
        start = history[0]['host_time'] if history else 0
        series = []
        for j in chosen:
            for name, key, style in [('Feedback', 'angles', 'solid'), ('Target', 'targets', 'dashed')]:
                series.append(dict(type='line', name=f'J{j:02d} {name}', showSymbol=False, connectNulls=False,
                    lineStyle={'type': style}, data=[[r['host_time'] - start, r[key][j]] for r in history]))
        chart.options['series'] = series
        chart.update()
        record = snapshot['recording']
        state = 'Running' if record['running'] else 'Waiting' if record['enabled'] else 'Off'
        recording.set_text(f"Recording: {state}   Duration: {show(record['duration_s'], 's')}   Directory: {record['path'] or '—'}" + ('   Finalizing…' if record['finalizing'] else '') + (f"\nError: {record['error']}" if record['error'] else ''))
        syncing[0] = True
        full.set_value(record['full_tactile']); full.set_enabled(not record['running'])
        syncing[0] = False
    return refresh
