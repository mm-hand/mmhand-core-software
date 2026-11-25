"""Automatic tactile slide capture, fitting and preview.

职责：消费 P4 反馈历史，渲染接触对，拟合并保存标定结果。
禁止依赖：直接操作串口、协议帧或控制器内部。
"""
from __future__ import annotations

import time

from nicegui import background_tasks, run, ui

from desktop.core.contact_calibration import (
    CONTACT_GAP_MS,
    ContactCapture,
    build_trajectory,
    clear_contact,
    counts_from_snapshot,
    infer_pair,
    initial_mappings,
    model_angles,
    save_contact,
    solve_contact,
)
from desktop.core.tactile_geometry import SENSOR_LINKS, module_points

from .common import label
from .hand_scene import SCENE, HandScene, make_camera, register_meshes, rotation_from_force

INSTRUCTIONS = (
    'With the hand unloaded, zero the tactile sensors. Touch another finger with '
    'a fingertip sensor, slide while in contact, then lift to finish the trajectory.'
)
FINGER_NAMES = ('Pinky', 'Ring', 'Middle', 'Index', 'Thumb')
SEGMENT_NAMES = ('tip', 'distal', 'proximal')
GRAY = '#808080'
BLUE = '#2563eb'
TIP_MASK = sum(1 << (finger * 3) for finger in range(5))


def _preconditions(snapshot):
    rows = snapshot.get('joints') or []
    return [
        ('P4 online', snapshot.get('communication') == 'Online'),
        ('IDLE', snapshot.get('control_mode') == 'IDLE' and not snapshot.get('busy')),
        ('21/21 Manual', len(rows) == 21 and all(row.get('manual_valid') for row in rows)),
    ]


def _gap_text(result):
    return (f"Average gap between touching fingers: "
            f"{result['clearance_before_mm']:.1f} mm before, "
            f"{result['clearance_after_mm']:.1f} mm after.")


def _fit_text(result):
    return (f"{_gap_text(result)} Contact {result['contact_pass']}/"
            f"{result['contact_total']}, non-contact {result['noncontact_pass']}/"
            f"{result['noncontact_total']}; continuity "
            f"{result['continuity_before_mm']:.1f} → "
            f"{result['continuity_after_mm']:.1f} mm.")


def build(hand, _act):
    session = []
    pending = []
    solution = manual = stream = latest_force = zero_id = None
    stale = processing = solving = False
    capture = ContactCapture()
    markers, arrows = {}, {}
    highlighted = ()
    register_meshes()

    with ui.row().classes('page-toolbar'):
        zero = ui.button('Tactile sensor force zeroing', color='primary')
        zero.disable()
        delete = ui.button('Delete Last Contact Trajectory')
        solve_button = ui.button('Solve')
        save = ui.button('Save Contact Calibration')
        clear = ui.button('Clear Contact Calibration')
    ui.label(INSTRUCTIONS).classes('text-sm')
    with ui.row().classes('items-center gap-4'):
        encoder = ui.checkbox('Encoder correction (k/b)', value=True)
        geometry = ui.checkbox('Joint axis translation & rotation', value=False)
    ui.label('Gray: joint calibration | Light blue: fitted pose | Blue: contact pair').classes(
        'text-xs text-slate-500')
    live = label()
    status = label()
    summary = label().classes('text-xs text-slate-600')
    scene = ui.scene(grid=False, camera=make_camera(), background_color='#ffffff',
                     **SCENE).classes('w-full')
    view = HandScene(scene)
    sensor_by_link = {link: key for key, link in SENSOR_LINKS.items()}

    def add_sensor(link, parent):
        key = sensor_by_link.get(link)
        if key is None:
            return
        points = module_points(key[1])
        x, y, _ = points.mean(axis=0)
        # Start just above the housing so the link marker and arrow stay visible.
        with parent:
            with scene.group().move(x, y, points[:, 2].max() + 0.001):
                if key[1] == 0:
                    markers[link] = scene.sphere(0.0025).material(GRAY)
                with scene.group() as arrow:
                    scene.cylinder(0.001, 0.001, 0.7).move(0, 0.35, 0).material('#334155')
                    scene.cylinder(0, 0.0024, 0.3).move(0, 0.85, 0).material('#334155')
            arrows[key] = arrow
            arrow.visible(False)

    def create_models():
        view.add_model('gray', GRAY, 0.85, on_link=add_sensor)
        view.add_model('fitted', '#93c5fd', 0.55)
        view.show('fitted', False)

    scene.on('init', create_models)

    def highlight(links=()):
        nonlocal highlighted
        if highlighted == links or 'gray' not in view.models:
            return
        for old, color in ((highlighted, GRAY), (links, BLUE)):
            for link in old:
                if link in markers:
                    markers[link].material(color)
                else:
                    view.set_link_color('gray', (link,), color)
        highlighted = links

    def update_summary():
        parts = [f"#{i} {FINGER_NAMES[info['fingertip']]} "
                 f"{info['contacts']}/{info['noncontacts']}"
                 for i, (_, info) in enumerate(session, 1)]
        summary.set_text('Trajectories (contacts/non-contact): ' + ' | '.join(parts)
                         if parts else 'No trajectory recorded.')

    def reset_capture():
        nonlocal capture, latest_force, stream
        capture, latest_force, stream = ContactCapture(), None, None
        highlight()
        draw_forces(None)

    async def finish_slides():
        nonlocal processing, stale, manual
        try:
            while pending:
                fingertip, frames, endpoints = pending.pop(0)
                if manual is not None and endpoints != manual:
                    status.set_text('Manual Min/Max changed. Clear and capture again.')
                    continue
                status.set_text(f'Processing {len(frames)} frames...')
                try:
                    built = await run.cpu_bound(build_trajectory, frames, *endpoints,
                                                len(session), fingertip=fingertip)
                except Exception as exc:  # noqa: BLE001 - show the rejection reason
                    status.set_text(f'Trajectory rejected: {exc}')
                    continue
                if built:
                    manual = endpoints
                    session.append((built['events'], built['info']))
                    stale = True
                    update_summary()
                    status.set_text(f'Trajectory {len(session)} captured. Press Solve to fit.')
        finally:
            processing = False

    def busy():
        return capture.fingertip is not None or processing or solving or zero_id is not None

    def zero_now():
        nonlocal zero_id
        if busy() or not zero.enabled:
            return
        result = hand.request_tactile_force_zero()
        if result.accepted:
            zero_id = result.operation_id
            reset_capture()
            status.set_text('Zeroing tactile sensors...')
            zero.disable()
        else:
            status.set_text(result.message)

    def delete_last():
        nonlocal solution, stale, manual
        if busy() or not session:
            return
        session.pop()
        stale = bool(session)
        if not session:
            solution = manual = None
        update_summary()
        status.set_text(f'Deleted the last trajectory. {len(session)} remain.')

    async def solve_now():
        nonlocal solving, solution, stale
        if busy() or not session:
            return
        if not encoder.value and not geometry.value:
            status.set_text('Select at least one correction.')
            return
        events = [event for items, _ in session for event in items]
        solving = True
        status.set_text(f'Solving {len(events)} samples...')
        try:
            result = await run.cpu_bound(solve_contact, events, initial_mappings(*manual),
                                         optimize_kb=bool(encoder.value),
                                         optimize_geometry=bool(geometry.value))
            if result:
                solution, stale = result, False
                status.set_text(_fit_text(result))
        except Exception as exc:  # noqa: BLE001
            status.set_text(f'Solve failed: {exc}')
        finally:
            solving = False

    def save_result():
        if solution is None or stale or busy():
            return
        try:
            save_contact(hand.calibration_snapshot_path, solution)
        except Exception as exc:  # noqa: BLE001
            status.set_text(f'Save failed: {exc}')
            return
        status.set_text('Saved. Run Motor Calibration to apply the refined values.')

    def clear_result():
        nonlocal solution, stale, manual
        if busy():
            return
        try:
            clear_contact(hand.calibration_snapshot_path)
        except Exception as exc:  # noqa: BLE001
            status.set_text(f'Clear failed: {exc}')
            return
        session.clear()
        solution = manual = None
        stale = False
        reset_capture()
        update_summary()
        status.set_text('Cleared. Motor Calibration will use the joint-calibration values.')

    def options_changed(_):
        nonlocal solution, stale
        if not busy():
            solution, stale = None, False
            status.set_text('Options changed. Press Solve.')

    def draw_forces(frame):
        for (finger, segment), arrow in arrows.items():
            index = finger * 3 + segment
            if frame is not None and frame.valid_mask & (1 << index):
                rotation, magnitude = rotation_from_force(frame.values[index * 3 : index * 3 + 3])
                if rotation is not None:
                    arrow.rotate_R(rotation).scale(1, min(0.02, 0.005 + 0.00012 * magnitude), 1)
                    arrow.visible(True)
                    continue
            arrow.visible(False)

    def refresh(snapshot):
        nonlocal stream, capture, latest_force, processing, zero_id
        if snapshot is None:
            reset_capture()
            return
        if zero_id is not None:
            for operation in snapshot.get('operations', ()):
                if operation['operation_id'] == zero_id and operation['status'] in ('SUCCESS', 'FAILED'):
                    status.set_text(operation['message'])
                    zero_id = None
                    reset_capture()
                    break

        checks = _preconditions(snapshot)
        online, idle, _ = (ok for _, ok in checks)
        values, reason = counts_from_snapshot(snapshot)
        endpoints = tuple(tuple(row[key] for row in snapshot.get('joints', ()))
                          for key in ('manual_min', 'manual_max'))
        allowed = all(ok for _, ok in checks) and zero_id is None and not solving
        now = time.time()
        previous_stream = stream
        stream, frames = hand.read_contact_feedback(stream) if online else (None, ())
        if stream is None or (previous_stream is not None and stream[0] is not previous_stream[0]):
            capture, latest_force = ContactCapture(), None
        for frame in frames:
            if frame.kind == 'tactile_summary':
                latest_force = frame
        if latest_force is not None and now - latest_force.received_at > CONTACT_GAP_MS / 1000:
            latest_force = None

        if allowed:
            slides = capture.observe(frames, now)
            if slides:
                pending.extend((tip, rows, endpoints) for tip, rows in slides)
                if not processing:
                    processing = True
                    background_tasks.create(finish_slides())
        elif capture.latest is not None:
            capture = ContactCapture()

        locked = busy()
        zero.set_enabled(online and idle and not locked)
        delete.set_enabled(bool(session) and not locked)
        solve_button.set_enabled(bool(session) and not locked)
        save.set_enabled(solution is not None and not stale and not locked)
        clear.set_enabled(not locked)
        encoder.set_enabled(not locked)
        geometry.set_enabled(not locked)

        links = ()
        text = 'Slide a fingertip sensor along another finger.'
        if not all(ok for _, ok in checks) and zero_id is None:
            text = 'Required: ' + ', '.join(name for name, ok in checks if not ok)
        elif (latest_force is None or not latest_force.valid_mask & TIP_MASK) and zero_id is None:
            text = 'Waiting for fingertip force feedback.'
        elif values is None:
            text = reason or ''
        if values is not None and view.models:
            mappings = initial_mappings(*endpoints)
            view.apply('gray', model_angles(values, [m['k'] for m in mappings],
                                            [m['b'] for m in mappings]))
            if capture.fingertip is not None:
                f, other, segment, gap = infer_pair(values, mappings, capture.fingertip)
                target = SENSOR_LINKS[other, segment]
                if segment:
                    target = target.removesuffix('_skeleton_sensor')
                links = (SENSOR_LINKS[f, 0], target)
                text = (f'{FINGER_NAMES[f]} tip / {FINGER_NAMES[other]} '
                        f'{SEGMENT_NAMES[segment]} ({gap:.1f} mm)')
            if solution is not None:
                view.set_origins('fitted', solution['origins'])
                view.apply('fitted', model_angles(values, solution['k'], solution['b']))
        if 'fitted' in view.models:
            view.show('fitted', solution is not None and values is not None)
        highlight(links)
        draw_forces(latest_force)
        live.set_text(text if zero_id is None else '')

    zero.on_click(zero_now)
    delete.on_click(delete_last)
    solve_button.on_click(solve_now)
    save.on_click(save_result)
    clear.on_click(clear_result)
    encoder.on_value_change(options_changed)
    geometry.on_value_change(options_changed)
    update_summary()
    return refresh
