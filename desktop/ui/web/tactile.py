"""Full taxel heatmaps or explicitly labelled summary forces, without invented taxels.

职责：展示快照并提交后端操作。
线程：NiceGUI 页面事件循环。
输入：后端快照及用户交互。
输出：原生控件和操作请求。
状态所有权：仅目标草稿和显示缓存；设备真值归后端。
禁止依赖：禁止直接操作协议、串口和控制器。
"""
import math
from nicegui import ui
from .common import label, show

def build(hand):
    plots, selected = {}, [None]
    with ui.row().classes('page-toolbar'):
        source = label()
        axis = ui.toggle(['X', 'Y', 'Z', 'Magnitude'], value='Magnitude')
    with ui.row().classes('w-full items-start flex-nowrap tactile-layout'):
        with ui.element('div').classes('tactile-grid flex-1'):
            for f, name in enumerate(['Thumb', 'Index', 'Middle', 'Ring', 'Pinky']):
                with ui.column().classes('gap-2 w-full'):
                    ui.label(name).classes('font-semibold')
                    for m in range(3):
                        ui.button(f'Module {m}', on_click=lambda f=f, m=m: selected.__setitem__(0, (f, m))).props('flat dense')
                        plots[f, m] = ui.echart({'xAxis': {'type': 'category', 'data': [], 'show': False}, 'yAxis': {'type': 'category', 'data': [], 'show': False},
                            'grid': {'left': 2, 'right': 2, 'top': 2, 'bottom': 2},
                            'visualMap': {'show': False, 'min': 0, 'max': 100, 'inRange': {'color': ['#e2e8f0', '#93c5fd', '#2563eb', '#ef4444']}},
                            'series': [{'type': 'heatmap', 'data': []}]}).classes('w-full h-28 bg-slate-200 rounded')
        with ui.card().classes('w-60'):
            ui.label('Selected Module').classes('font-semibold')
            detail = label()
    ui.label('Color scale: 0–100 sensor units; signed axes −100–100. Missing or stale modules are grey. SUMMARY displays one module force, not taxels.').classes('text-xs text-slate-600')
    def component(vector):
        return math.sqrt(sum(v * v for v in vector)) if axis.value == 'Magnitude' else vector[['X', 'Y', 'Z'].index(axis.value)]
    def refresh(snapshot):
        tactile = snapshot['tactile']
        source.set_text(f"Source: {tactile['source']}   {tactile['level']}   Age: {show(tactile['age_ms'], 'ms')}")
        modules = {(m['finger'], m['module']): m for m in tactile['modules']}
        for key, plot in plots.items():
            module = modules.get(key, {})
            data = module.get('taxels') or ([[module['force']]] if module.get('force') is not None else [])
            height, width = len(data), len(data[0]) if data else 0
            plot.options['xAxis']['data'] = list(range(width)); plot.options['yAxis']['data'] = list(range(height))
            plot.options['visualMap']['min'] = 0 if axis.value == 'Magnitude' else -100
            plot.options['series'][0]['data'] = [[c, r, component(value)] for r, line in enumerate(data) for c, value in enumerate(line)]
            plot.update()
        module = modules.get(selected[0], {})
        force = module.get('force') or [None, None, None]
        detail.set_text(f"Finger: {show(module.get('finger'))}   Module: {show(module.get('module'))}\nFx: {show(force[0])}\nFy: {show(force[1])}\nFz: {show(force[2])}\nOnline: {'Yes' if module.get('valid') else 'No'}\nAge: {show(module.get('age_ms'), 'ms')}")
    return refresh
