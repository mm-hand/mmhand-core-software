"""Assembly ID view. Device transactions are owned by HandBackend.

职责：展示快照并提交用户操作。
线程：NiceGUI页面事件循环。
输入：后端只读快照及用户输入。
输出：原生控件和操作请求。
状态所有权：仅持有输入草稿和展示缓存。
禁止依赖：禁止串口、协议编码或直接硬件操作。
"""
from pathlib import Path
from nicegui import app, ui
from .common import label

IMAGE = Path(__file__).resolve().parents[2] / 'resources' / 'images' / 'joint_motor_ids.png'


def build(hand, act):
    if not any(getattr(route, 'path', '') == '/joint-motor-ids.png' for route in app.routes):
        app.add_static_file(url_path='/joint-motor-ids.png', local_file=IMAGE)
    inputs = {}
    generation = [None]
    with ui.row().classes('page-toolbar'):
        scan = ui.button('Scan All Buses', on_click=lambda: act(hand.request_id_scan))
        rename = ui.button('Renumber All', on_click=lambda: act(lambda: hand.request_motor_renumber({key: number.value for key, number in inputs.items()})))
    motor_status = label()
    ui.label('Multiple motors with the same ID on one bus cannot be identified or renumbered separately. Connect them separately before renumbering.').classes('text-sm text-amber-800')
    with ui.element('div').classes('renumber-motors-layout w-full'):
        with ui.column().classes('w-full min-w-0'):
            ui.label('Bus / Current ID / New ID').classes('text-sm font-semibold')
            rows = ui.column().classes('w-full')
            ui.label('Assembly target IDs are shown in the diagram. Product joint bindings remain fixed. IDs above 26 are outside the product control mapping.').classes('text-xs text-slate-500')
            ui.label('Renumber All also sets every scanned motor: maximum speed (84) = 254, maximum acceleration (85) = 254, acceleration multiplier (86) = 0. All values are read back before success.').classes('text-xs text-slate-700')
        with ui.column().classes('w-full min-w-0'):
            ui.image('/joint-motor-ids.png').props('fit=contain').classes('renumber-motors-diagram')

    def refresh(snapshot):
        motor = snapshot['id_reset']['motor']
        idle = not snapshot['busy']
        scan.set_enabled(motor['available'] and idle)
        rename.set_enabled(motor['available'] and idle and motor['scanned'] and bool(motor['rows']))
        motor_status.set_text(motor['message'] if motor['available'] else 'Connect P4 with fresh status to reset motor IDs.')
        if generation[0] != motor['generation']:
            generation[0] = motor['generation']; rows.clear(); inputs.clear()
            with rows:
                for row in motor['rows']:
                    key = (row['bus'], row['id'])
                    with ui.row().classes('items-center gap-4'):
                        ui.label(f"BUS{row['bus']}").classes('w-14')
                        ui.label(str(row['id'])).classes('w-14')
                        inputs[key] = ui.number('New ID', value=row.get('new_id',row['id']), min=0, max=252, step=1).props('dense outlined').classes('w-28')
        for number in inputs.values(): number.set_enabled(motor['available'] and idle and not motor['recovering'])
    return refresh
