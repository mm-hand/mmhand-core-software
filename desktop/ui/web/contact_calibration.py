"""Contact calibration page: manual contact / close labels and a two-class fit.

职责：展示操作说明和计数，记录接触/未接触位姿，按勾选项后台求解，显示灰/蓝两只手。
线程：NiceGUI 页面事件循环；solve 在 cpu_bound worker 中运行。
输入：后端快照、用户点击。
输出：内存事件、求解结果、标定包的 contact_calibration 字段。
状态所有权：页面持有本次事件和结果。
禁止依赖：禁止直接操作串口、协议帧或控制器内部。
"""
from __future__ import annotations

from nicegui import run, ui

from desktop.core.contact_calibration import (
    clear_contact,
    counts_from_snapshot,
    infer_pair,
    initial_mappings,
    model_angles,
    save_contact,
    solve,
)

from .common import label
from .hand_scene import SCENE, HandScene, make_camera, register_meshes

INSTRUCTIONS = (
    'Select the fingertip, touch another finger → Record Contact, hover just above → '
    'Record Close, then Solve and Save.'
)
FINGER_LABELS = ((4, 'Thumb'), (3, 'Index'), (2, 'Middle'), (1, 'Ring'), (0, 'Pinky'))
FINGERTIP_OPTIONS = {index: name for index, name in FINGER_LABELS}


def _preconditions(snapshot):
    rows = snapshot.get('joints') or []
    manual = sum(1 for row in rows if row.get('manual_valid'))
    return [
        ("P4 online", snapshot.get('communication') == 'Online'),
        ("IDLE", snapshot.get('control_mode') == 'IDLE' and not snapshot.get('busy')),
        ("21/21 Manual", len(rows) == 21 and manual == 21),
    ]


def _mappings(snapshot):
    rows = snapshot.get('joints') or []
    return initial_mappings([row.get('manual_min') for row in rows],
                            [row.get('manual_max') for row in rows])


def _gap_text(result):
    return (f"Average gap between touching fingers: "
            f"{result['clearance_before_mm']:.1f} mm before, "
            f"{result['clearance_after_mm']:.1f} mm after.")


def build(hand, act):
    events, solution = [], [None]
    latest = [None]
    solving, stale = [False], [False]
    register_meshes()

    with ui.row().classes('page-toolbar'):
        record_contact = ui.button('Record Contact', color='primary')
        record_close = ui.button('Record Close (no contact)')
        delete = ui.button('Delete Last Sample')
        solve_button = ui.button('Solve')
        save = ui.button('Save Contact Calibration')
        clear = ui.button('Clear Contact Calibration')
    ui.label(INSTRUCTIONS).classes('text-sm whitespace-pre-wrap')
    with ui.row().classes('items-center gap-4'):
        fingertip = ui.select(FINGERTIP_OPTIONS, value=None, label='Fingertip') \
            .props('dense outlined').classes('w-32')
        encoder = ui.checkbox('Encoder correction (k/b)', value=True)
        geometry = ui.checkbox('Joint axis translation & rotation', value=False)
    ui.label('Gray: Joint Calibration pose · Blue: corrected pose').classes('text-xs text-slate-500')
    status = label()
    finger_counts = ui.label().classes('text-xs text-slate-600')
    scene = ui.scene(grid=False, camera=make_camera(), background_color='#ffffff',
                     **SCENE).classes('w-full')
    view = HandScene(scene)

    def create_models():
        view.add_model('gray', '#808080', 0.85)
        view.add_model('blue', '#2563eb', 0.85)
        view.show('blue', False)
    scene.on('init', create_models)

    def set_status(text):
        if status.text != text:
            status.set_text(text)

    def update_counts():
        per_finger = [[0, 0] for _ in range(5)]
        for event in events:
            pair = event.get('pair')
            fingers = (pair[0], pair[1]) if pair else (event.get('fingertip'),)
            column = 0 if event['y'] == 1 else 1
            for finger in {int(value) for value in fingers if value is not None}:
                per_finger[finger][column] += 1
        text = 'Finger contact/close: ' + ' · '.join(
            f'{name} {per_finger[index][0]}/{per_finger[index][1]}'
            for index, name in FINGER_LABELS)
        if finger_counts.text != text:
            finger_counts.set_text(text)

    def add_event(label_value):
        snapshot = hand.application_snapshot(include_curves=False)
        latest[0] = snapshot
        checks = _preconditions(snapshot)
        if not all(ok for _, ok in checks):
            set_status(' ; '.join(name for name, ok in checks if not ok) + ' required')
            return False
        values, reason = counts_from_snapshot(snapshot)
        if values is None:
            set_status(reason)
            return False
        selected = fingertip.value
        if selected is None:
            set_status('Select the fingertip first.')
            return False
        selected = int(selected)
        try:
            pair = infer_pair(values, _mappings(snapshot), selected)[:3]
        except (ValueError, OSError):
            pair = None
        events.append({'c': values, 'y': label_value,
                       'fingertip': selected, 'pair': pair})
        update_counts()
        return True

    def on_record(label_value):
        if solving[0]:
            return
        if not add_event(label_value):
            return
        if solution[0] is not None:
            stale[0] = True
            set_status('Samples changed. Press Solve to refit.')

    def delete_last():
        if solving[0]:
            return
        if not events:
            set_status('No samples to delete.')
            return
        events.pop()
        update_counts()
        if solution[0] is not None:
            stale[0] = True
            set_status('Last sample deleted. Press Solve to refit.')
        else:
            set_status('Last sample deleted.')

    async def run_solve():
        if solving[0]:
            return
        if not any(event['y'] == 1 for event in events):
            set_status('Record at least one contact sample first.')
            return
        if not encoder.value and not geometry.value:
            set_status('Select at least one correction.')
            return
        solving[0] = True
        try:
            mappings = _mappings(latest[0])
            values = list(events)
            set_status(f'Fitting {len(values)} samples…')
            try:
                result = await run.cpu_bound(
                    solve, values, mappings,
                    optimize_kb=bool(encoder.value),
                    optimize_geometry=bool(geometry.value))
            except Exception as exc:  # noqa: BLE001 - surface the solver reason
                set_status(f'Solve failed: {exc}')
                return
            solution[0] = result
            stale[0] = False
            set_status(_gap_text(result))
        finally:
            solving[0] = False

    def options_changed():
        if solving[0]:
            return
        solution[0] = None
        stale[0] = False
        if 'blue' in view.models:
            view.show('blue', False)
        set_status('Options changed. Press Solve.')

    def save_result():
        if solution[0] is None or stale[0] or solving[0]:
            set_status('Press Solve before saving.')
            return
        try:
            save_contact(hand.calibration_snapshot_path, solution[0])
        except Exception as exc:  # noqa: BLE001
            set_status(f'Save failed: {exc}')
            return
        set_status('Saved. Run Motor Calibration to apply the refined values.')

    def clear_result():
        try:
            clear_contact(hand.calibration_snapshot_path)
        except Exception as exc:  # noqa: BLE001
            set_status(f'Clear failed: {exc}')
            return
        events.clear()
        solution[0] = None
        stale[0] = False
        update_counts()
        if 'blue' in view.models:
            view.show('blue', False)
        set_status('Cleared. Motor Calibration will use the joint-calibration values.')

    record_contact.on_click(lambda: on_record(1))
    record_close.on_click(lambda: on_record(0))
    delete.on_click(delete_last)
    solve_button.on_click(run_solve)
    save.on_click(save_result)
    clear.on_click(clear_result)
    encoder.on_value_change(lambda _: options_changed())
    geometry.on_value_change(lambda _: options_changed())

    def refresh(snapshot):
        latest[0] = snapshot
        checks = _preconditions(snapshot)
        allowed = all(ok for _, ok in checks)
        busy = solving[0]
        ready = allowed and not busy and fingertip.value is not None
        record_contact.set_enabled(ready)
        record_close.set_enabled(ready)
        delete.set_enabled(bool(events) and not busy)
        solve_button.set_enabled(
            not busy and any(event['y'] == 1 for event in events))
        save.set_enabled(solution[0] is not None and not stale[0] and not busy)
        clear.set_enabled(not busy)
        fingertip.set_enabled(not busy)
        encoder.set_enabled(not busy)
        geometry.set_enabled(not busy)
        if not allowed:
            set_status(' · '.join(
                f"{'✓' if ok else '✗'} {name}" for name, ok in checks))
        elif fingertip.value is None:
            set_status('Select the fingertip first.')
        elif status.text == 'Select the fingertip first.':
            set_status('')
        values, _reason = counts_from_snapshot(snapshot)
        if values is not None and view.models:
            mappings = _mappings(snapshot)
            view.apply('gray', model_angles(
                values, [item['k'] for item in mappings],
                [item['b'] for item in mappings]))
            view.show('gray', True)
            if solution[0]:
                result = solution[0]
                view.set_origins('blue', result['origins'])
                view.apply('blue', model_angles(values, result['k'], result['b']))
                view.show('blue', True)
            else:
                view.show('blue', False)
        update_counts()

    update_counts()
    return refresh
