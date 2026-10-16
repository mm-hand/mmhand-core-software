"""Flat hand view of manufacturer taxel coordinates and summary-only forces.

职责：展示快照并提交用户操作。
线程：NiceGUI页面事件循环。
输入：后端只读快照及用户输入。
输出：原生控件和操作请求。
状态所有权：仅持有输入草稿和展示缓存。
禁止依赖：禁止串口、协议编码或直接硬件操作。
"""
from nicegui import ui

from desktop.core.tactile_geometry import FINGERS, SEGMENTS, hand_svg
from .common import label, show


def build(hand):
    selected = [None]
    with ui.row().classes('page-toolbar'):
        source = label()
        axis = ui.toggle(['X', 'Y', 'Z', 'Magnitude'], value='Magnitude')
    with ui.row().classes('w-full items-start flex-wrap tactile-layout'):
        picture = ui.html(hand_svg([]), sanitize=False).classes('flex-1 min-w-[350px] max-w-[1100px]')
        with ui.card().classes('w-64'):
            ui.label('Module detail').classes('font-semibold')
            finger = ui.select({i: name for i, name in enumerate(FINGERS)}, label='Finger', value=1).props('dense outlined')
            segment = ui.select({i: name for i, name in enumerate(SEGMENTS)}, label='Sensor', value=0).props('dense outlined')
            detail = label()
            selected[0] = (finger, segment)
    ui.label('FULL: manufacturer XY taxel positions, one dot per measured point. SUMMARY: resultant force only (Σ). Grey: no fresh sensor data. Values use device raw units.').classes('text-xs text-slate-600')

    previous = [None]
    def refresh(snapshot):
        tactile = snapshot['tactile']
        source.set_text(f"Source: {tactile['source']}   {tactile['level']}   Age: {show(tactile['age_ms'], 'ms')}")
        rows = tactile['modules']
        key = (axis.value, repr(rows))
        if key != previous[0]:
            picture.set_content(hand_svg(rows, axis.value))
            previous[0] = key
        module = next((m for m in rows if m['finger'] == finger.value and m['module'] == segment.value), {})
        force = module.get('force') if module.get('valid') else None
        if force is None:
            detail.set_text('No fresh data for this sensor.')
        else:
            detail.set_text(f"{FINGERS[finger.value]} {SEGMENTS[segment.value]}\nFx: {show(force[0])}\nFy: {show(force[1])}\nFz: {show(force[2])}\nAge: {show(module.get('age_ms'), 'ms')}")
    return refresh
