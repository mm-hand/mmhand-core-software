"""Contact calibration page: slide recording, non-contact samples and fit.

职责：录制指尖滑动轨迹，在线判定当前接触对并在 3D 中染红，按勾选项求解和保存标定包。
线程：NiceGUI 页面事件循环；轨迹处理和 solve 在 cpu_bound worker 中运行。
输入：后端快照、按钮/勾选框、P4 反馈历史。
输出：轨迹会话、在线高亮、求解结果、标定包 contact_calibration 字段。
状态所有权：页面持有本页会话与最新解；设备真值归后端。
禁止依赖：禁止直接操作串口、协议帧或控制器内部。
"""
from __future__ import annotations

import time

from nicegui import run, ui

from desktop.core.contact_calibration import (
    CONTACT_LINKS,
    STEP_M,
    build_trajectory,
    clear_contact,
    counts_from_snapshot,
    infer_pair,
    initial_mappings,
    model_angles,
    save_contact,
    solve_contact,
    tip_travel,
)

from .common import label
from .hand_scene import SCENE, HandScene, make_camera, register_meshes

INSTRUCTIONS = (
    'Touch another finger with one fingertip, press Start, slide while keeping '
    'contact, press Stop; repeat for other finger pairs, then Solve and Save.'
)
FINGER_LABELS = ((4, 'Thumb'), (3, 'Index'), (2, 'Middle'), (1, 'Ring'), (0, 'Pinky'))
FINGER_NAMES = dict(FINGER_LABELS)
SEGMENT_NAMES = ('tip', 'distal', 'proximal')
GRAY = '#808080'
RED = '#ef4444'


def _preconditions(snapshot, *, ignore_busy=False):
    rows = snapshot.get('joints') or []
    manual = sum(1 for row in rows if row.get('manual_valid'))
    busy = bool(snapshot.get('busy')) and not ignore_busy
    return [
        ("P4 online", snapshot.get('communication') == 'Online'),
        ("IDLE", snapshot.get('control_mode') == 'IDLE' and not busy),
        ("21/21 Manual", len(rows) == 21 and manual == 21),
    ]


def _mappings_from(manual_min, manual_max):
    return initial_mappings(manual_min, manual_max)


def _mappings(snapshot):
    rows = snapshot.get('joints') or []
    return _mappings_from([row.get('manual_min') for row in rows],
                          [row.get('manual_max') for row in rows])


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


def _finger_name(index):
    return FINGER_NAMES.get(int(index), '—')


def build(hand, act):
    session = []          # [{'events': [...], 'info': {...}}]
    latest = [None]
    solution = [None]
    stale = [False]
    recording = [False]
    processing = [False]
    solving = [False]
    manual = [None]       # (manual_min, manual_max) from the first trajectory
    live = []             # rolling counts for the online red highlight
    highlighted = []
    started_at = [0.0]
    status_text = ['']
    register_meshes()

    with ui.row().classes('page-toolbar'):
        start = ui.button('Start Recording Contact Trajectory', color='primary')
        stop = ui.button('Stop Current Recording')
        delete = ui.button('Delete Last Contact Trajectory')
        solve_button = ui.button('Solve')
        save = ui.button('Save Contact Calibration')
        clear = ui.button('Clear Contact Calibration')
    ui.label(INSTRUCTIONS).classes('text-sm whitespace-pre-wrap')
    with ui.row().classes('items-center gap-4'):
        encoder = ui.checkbox('Encoder correction (k/b)', value=True)
        geometry = ui.checkbox('Joint axis translation & rotation', value=False)
    ui.label('Gray: URDF + Joint Calibration · Blue: contact calibration · '
             'Red: links in contact while recording').classes('text-xs text-slate-500')
    status = label()
    summary = label().classes('text-xs text-slate-600')
    scene = ui.scene(grid=False, camera=make_camera(), background_color='#ffffff',
                     **SCENE).classes('w-full')
    view = HandScene(scene)

    def create_models():
        view.add_model('gray', GRAY, 0.85)
        view.add_model('blue', '#2563eb', 0.85)
        view.show('blue', False)
    scene.on('init', create_models)

    def set_status(text):
        if status_text[0] != text:
            status_text[0] = text
            status.set_text(text)

    def reset_highlight():
        if highlighted and 'gray' in view.models:
            view.set_link_color('gray', list(highlighted), GRAY)
        highlighted.clear()

    def update_summary():
        if not session:
            summary.set_text('No trajectory recorded.')
            return
        parts = []
        for index, item in enumerate(session, start=1):
            info = item['info']
            parts.append(f"#{index} {_finger_name(info.get('fingertip'))} "
                         f"{info.get('contacts', 0)}/{info.get('noncontacts', 0)}")
        summary.set_text('Trajectories (contacts/non-contact): ' + ' · '.join(parts))

    def snapshot_now():
        return hand.application_snapshot(include_curves=False)

    def begin():
        if recording[0] or processing[0] or solving[0]:
            return
        snapshot = snapshot_now()
        checks = _preconditions(snapshot)
        if not all(ok for _, ok in checks):
            set_status(' ; '.join(name for name, ok in checks if not ok) + ' required')
            return
        result = hand.begin_contact_capture()
        if not result.get('ok'):
            set_status(result.get('message') or 'Could not start recording')
            return
        if manual[0] is None:
            manual[0] = (list(result['manual_min']), list(result['manual_max']))
        elif (manual[0][0] != list(result['manual_min'])
              or manual[0][1] != list(result['manual_max'])):
            hand.end_contact_capture()
            set_status('Manual Min/Max changed; press Clear and record again.')
            return
        recording[0] = True
        live.clear()
        reset_highlight()
        started_at[0] = time.monotonic()
        set_status('Recording…')

    async def finish():
        if not recording[0]:
            return
        result = hand.end_contact_capture()
        recording[0] = False
        reset_highlight()
        if not result.get('ok'):
            set_status(result.get('message') or 'Recording failed')
            return
        processing[0] = True
        set_status(f"Processing {len(result['frames'])} frames…")
        try:
            built = await run.cpu_bound(
                build_trajectory, result['frames'],
                result['manual_min'], result['manual_max'], len(session))
        except Exception as exc:  # noqa: BLE001 - surface the trajectory reason
            set_status(f'Trajectory rejected: {exc}')
            return
        finally:
            processing[0] = False
        if not built:
            set_status('Trajectory processing was cancelled.')
            return
        session.append({'events': built['events'], 'info': built['info']})
        stale[0] = True
        update_summary()
        info = built['info']
        set_status(f"Trajectory {len(session)}: "
                   f"{_finger_name(info['fingertip'])} tip, "
                   f"{info['contacts']} contacts + {info['noncontacts']} "
                   f"non-contact samples. Press Solve to refit.")

    def delete_last():
        if recording[0] or processing[0] or solving[0] or not session:
            return
        session.pop()
        stale[0] = True
        update_summary()
        if not session:
            solution[0] = None
            stale[0] = False
            if 'blue' in view.models:
                view.show('blue', False)
        set_status(f'Deleted the last trajectory. {len(session)} remain. '
                   'Press Solve to refit.')

    async def solve_now():
        if recording[0] or processing[0] or solving[0] or not session:
            return
        if not encoder.value and not geometry.value:
            set_status('Select at least one correction.')
            return
        events = [event for item in session for event in item['events']]
        mappings = _mappings_from(*manual[0])
        solving[0] = True
        set_status(f'Solving {len(events)} samples…')
        try:
            result = await run.cpu_bound(
                solve_contact, events, mappings,
                optimize_kb=bool(encoder.value),
                optimize_geometry=bool(geometry.value))
        except Exception as exc:  # noqa: BLE001 - surface the solver reason
            set_status(f'Solve failed: {exc}')
            return
        finally:
            solving[0] = False
        if not result:
            set_status('Solve was cancelled.')
            return
        solution[0] = result
        stale[0] = False
        set_status(_fit_text(result))

    def save_result():
        if (solution[0] is None or stale[0] or recording[0]
                or processing[0] or solving[0]):
            set_status('Press Solve before saving.')
            return
        try:
            save_contact(hand.calibration_snapshot_path, solution[0])
        except Exception as exc:  # noqa: BLE001
            set_status(f'Save failed: {exc}')
            return
        set_status('Saved. Run Motor Calibration to apply the refined values.')

    def clear_result():
        if recording[0] or processing[0] or solving[0]:
            return
        try:
            clear_contact(hand.calibration_snapshot_path)
        except Exception as exc:  # noqa: BLE001
            set_status(f'Clear failed: {exc}')
            return
        session.clear()
        solution[0] = None
        stale[0] = False
        manual[0] = None
        live.clear()
        reset_highlight()
        update_summary()
        if 'blue' in view.models:
            view.show('blue', False)
        set_status('Cleared. Motor Calibration will use the joint-calibration values.')

    def options_changed():
        if processing[0] or solving[0]:
            return
        solution[0] = None
        stale[0] = False
        if 'blue' in view.models:
            view.show('blue', False)
        set_status('Options changed. Press Solve.')

    def refresh(snapshot):
        latest[0] = snapshot
        checks = _preconditions(snapshot, ignore_busy=recording[0])
        allowed = all(ok for _, ok in checks)
        busy = recording[0] or processing[0] or solving[0]
        start.set_enabled(allowed and not busy)
        stop.set_enabled(recording[0])
        delete.set_enabled(bool(session) and not busy)
        solve_button.set_enabled(bool(session) and not busy)
        save.set_enabled(solution[0] is not None and not stale[0] and not busy)
        clear.set_enabled(not busy)
        encoder.set_enabled(not busy)
        geometry.set_enabled(not busy)
        if recording[0]:
            if not allowed:
                hand.end_contact_capture()
                recording[0] = False
                reset_highlight()
                set_status('Recording discarded: ' + ' ; '.join(
                    name for name, ok in checks if not ok))
            else:
                values, reason = counts_from_snapshot(snapshot)
                if values is None:
                    set_status('Recording… ' + (reason or ''))
                else:
                    live.append(values)
                    del live[:-40]
                    if len(live) >= 2:
                        mappings = _mappings(snapshot)
                        travel = tip_travel(live, mappings)
                        fingertip = int(max(range(5), key=lambda f: travel[f]))
                        if travel[fingertip] >= STEP_M:
                            f, other, segment, gap = infer_pair(
                                values, mappings, fingertip)
                            links = [CONTACT_LINKS[f * 3],
                                     CONTACT_LINKS[other * 3 + segment]]
                            if links != highlighted and 'gray' in view.models:
                                reset_highlight()
                                view.set_link_color('gray', links, RED)
                                highlighted.extend(links)
                            set_status(
                                f"Recording… {time.monotonic() - started_at[0]:.1f} s · "
                                f"Live: {_finger_name(f)} tip ↔ "
                                f"{_finger_name(other)} {SEGMENT_NAMES[segment]}, "
                                f"gap {gap:.1f} mm")
                        else:
                            set_status(f"Recording… "
                                       f"{time.monotonic() - started_at[0]:.1f} s")
                    else:
                        set_status(f"Recording… "
                                   f"{time.monotonic() - started_at[0]:.1f} s")
        elif not processing[0] and not solving[0] and not allowed:
            set_status(' ; '.join(f"{'✓' if ok else '✗'} {name}"
                                  for name, ok in checks))
        values, _reason = counts_from_snapshot(snapshot)
        if values is not None and view.models:
            mappings = _mappings(snapshot)
            view.apply('gray', model_angles(
                values, [item['k'] for item in mappings],
                [item['b'] for item in mappings]))
            view.show('gray', True)
        if view.models and solution[0] is not None:
            values, _reason = counts_from_snapshot(snapshot)
            if values is not None:
                result = solution[0]
                view.set_origins('blue', result['origins'])
                view.apply('blue', model_angles(values, result['k'], result['b']))
                view.show('blue', True)
        elif 'blue' in view.models:
            view.show('blue', False)
        update_summary()

    start.on_click(begin)
    stop.on_click(finish)
    delete.on_click(delete_last)
    solve_button.on_click(solve_now)
    save.on_click(save_result)
    clear.on_click(clear_result)
    encoder.on_value_change(lambda _: options_changed())
    geometry.on_value_change(lambda _: options_changed())

    update_summary()
    set_status('Ready. Record a contact slide, then Solve and Save.')
    return refresh
