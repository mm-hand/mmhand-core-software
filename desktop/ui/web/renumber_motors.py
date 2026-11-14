"""Assembly ID view. Device transactions are owned by HandBackend.

职责：展示快照并提交用户操作。
线程：NiceGUI页面事件循环。
输入：后端只读快照及用户输入。
输出：原生控件和操作请求。
状态所有权：仅持有输入草稿和展示缓存。
禁止依赖：禁止串口、协议编码或直接硬件操作。
"""
from pathlib import Path
from time import monotonic
from nicegui import app, ui
from .common import label

IMAGE = Path(__file__).resolve().parents[2] / 'resources' / 'images' / 'joint_motor_ids.png'

HOLD_BUZZ = """event => {
    if (!event.isPrimary || event.button !== 0) return;
    const button = event.currentTarget;
    button.setPointerCapture(event.pointerId);
    const timer = setInterval(() => emit('hold'), 150);
    const stop = () => {
        clearInterval(timer);
        button.removeEventListener('lostpointercapture', stop);
        window.removeEventListener('blur', stop);
        emit('stop');
    };
    button.addEventListener('lostpointercapture', stop);
    window.addEventListener('blur', stop);
    emit('start');
}"""


def build(hand, act):
    if not any(getattr(route, 'path', '') == '/joint-motor-ids.png' for route in app.routes):
        app.add_static_file(url_path='/joint-motor-ids.png', local_file=IMAGE)
    inputs = {}
    buzz = {}
    generation = [None]
    held = None

    def holding(key):
        current = held
        return current is not None and current[0] == key and monotonic() < current[1]

    def hold_buzz(key, event):
        nonlocal held
        if event.args == 'start':
            held = (key, monotonic() + 0.5)
            act(lambda: hand.request_motor_buzz(*key, held=lambda: holding(key)))
        elif holding(key):
            held = None if event.args == 'stop' else (key, monotonic() + 0.5)

    with ui.row().classes('page-toolbar'):
        scan = ui.button('Scan (0–30)', on_click=lambda: act(hand.request_id_scan))
        scan_full = ui.button('Scan (0–253)', on_click=lambda: act(lambda: hand.request_id_scan(253)))
        rename = ui.button('Renumber All', on_click=lambda: act(lambda: hand.request_motor_renumber({key: number.value for key, number in inputs.items()})))
    motor_status = label()
    ui.label('Multiple motors with the same ID on one bus cannot be identified or renumbered separately. Connect them separately before renumbering.').classes('text-sm text-amber-800')
    with ui.element('div').classes('renumber-motors-layout w-full'):
        with ui.column().classes('w-full min-w-0'):
            ui.label('Bus / Current ID / New ID').classes('text-sm font-semibold')
            rows = ui.column().classes('w-full')
            ui.label('Assembly target IDs are shown in the diagram. Product joint bindings remain fixed. IDs above 26 are outside the product control mapping.').classes('text-xs text-slate-500')
            ui.label('Hold Buzz to move the motor about 11.25° towards tendon release and back repeatedly. Motors sharing a BUS and ID move together.').classes('text-xs text-slate-500')
        with ui.column().classes('w-full min-w-0'):
            ui.image('/joint-motor-ids.png').props('fit=contain').classes('renumber-motors-diagram')

    def refresh(snapshot):
        motor = snapshot['id_reset']['motor']
        idle = not snapshot['busy']
        ready = motor['available'] and idle
        scan.set_enabled(ready)
        scan_full.set_enabled(ready)
        editable = ready and motor['scanned']
        rename.set_enabled(editable and bool(motor['rows']))
        motor_status.set_text(motor['message'] if motor['available'] else 'Connect P4 with fresh status to reset motor IDs.')
        if generation[0] != motor['generation']:
            generation[0] = motor['generation']; rows.clear(); inputs.clear(); buzz.clear()
            with rows:
                for row in motor['rows']:
                    key = (row['bus'], row['id'])
                    with ui.row().classes('items-center gap-4'):
                        ui.label(f"BUS{row['bus']}").classes('w-14')
                        ui.label(str(row['id'])).classes('w-14')
                        value = row['id'] if row['id'] <= 252 else None
                        inputs[key] = ui.number('New ID', value=value, min=0, max=252, step=1).props('dense outlined').classes('w-28')
                        buzz[key] = ui.button('Buzz').props('dense outline').style('touch-action: none')
                        buzz[key].on('pointerdown', lambda event, key=key: hold_buzz(key, event), js_handler=HOLD_BUZZ)
                        buzz[key].on('click', lambda _, key=key: act(lambda: hand.request_motor_buzz(*key)),
                                     js_handler='event => { if (event.detail === 0) emit(); }')
        for number in inputs.values():
            number.set_enabled(editable)
        for key, button in buzz.items():
            button.set_enabled(editable or holding(key))
    return refresh
