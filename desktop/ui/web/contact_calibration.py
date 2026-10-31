"""Contact calibration page: capture cross-finger contacts, solve k/b + geometry, apply.

职责：展示采集前置条件、接触计数和求解结果，并把修正后的 k/b 写入设备标定结果。
线程：NiceGUI 页面事件循环；solve 在 cpu_bound worker 中运行。
输入：后端快照、用户操作。
输出：接触事件、k/b/几何结果、设备标定导入请求和本地 JSON。
状态所有权：页面持有本次采集缓冲和待应用的 bundle 字节。
禁止依赖：禁止直接操作串口、协议帧或控制器内部。
"""
from __future__ import annotations

import json

from nicegui import run, ui

from desktop.core.contact_calibration import (
    CORRECTION_PATH,
    CaptureConfig,
    ContactRecorder,
    _initial_mappings,
    build_updated_bundle,
    save_solution,
    solve,
)

from .common import label


def _mappings(hand):
    state = hand.get_state_snapshot()
    mappings = []
    for joint, result in enumerate(state.joint_calib_results):
        item = _initial_mappings()[joint]
        if isinstance(result, dict) and result.get("success"):
            k = float(result.get("angleK", 0) or 0)
            b = float(result.get("angleB", 0) or 0)
            if k > 0:
                item = {"k": k, "b": b}
        mappings.append(item)
    return mappings


def _preconditions(snapshot):
    rows = snapshot.get("joints") or []
    tactile = snapshot.get("tactile") or {}
    modules = tactile.get("modules") or []
    checks = [
        ("P4 online", snapshot.get("communication") == "Online"),
        ("IDLE", snapshot.get("control_mode") == "IDLE"),
        ("S3 direct FULL", tactile.get("source") == "S3_SERIAL"
         and tactile.get("level") == "FULL"),
        ("15 tactile modules", len(modules) == 15
         and all(module.get("valid") for module in modules)),
        ("21/21 joints READY", len(rows) == 21
         and all("READY" in (row.get("joint_quality") or []) for row in rows)),
    ]
    return checks


def build(hand, act):
    recorder = [ContactRecorder()]
    capture = [False]
    solution = [None]
    apply_state = [""]
    with ui.row().classes('page-toolbar'):
        start = ui.button('Start Capture')
        stop = ui.button('Stop')
        solve_button = ui.button('Solve')
        save = ui.button('Save & Apply')
        clear = ui.button('Clear')
    status = ui.label('Check preconditions, then Start Capture.')
    progress = ui.label('')
    result_label = label()
    precondition = ui.label('').classes('text-xs whitespace-pre-wrap')

    def start_capture():
        recorder[0] = ContactRecorder(CaptureConfig())
        solution[0] = None
        apply_state[0] = ""
        capture[0] = True
        result_label.set_text('')
        status.set_text('Capturing: hold each contact pose still for about 0.5 s.')

    def stop_capture():
        capture[0] = False
        status.set_text('Stopped. Solve when all finger pairs have enough events.')

    async def do_solve():
        stop_capture()
        events = list(recorder[0].events)
        if len(events) < 20:
            status.set_text('Not enough contact events; capture at least 20.')
            return
        status.set_text(f'Solving {len(events)} events…')
        try:
            solution[0] = await run.cpu_bound(solve, events, _mappings(hand))
        except Exception as exc:  # noqa: BLE001 - surface the solver reason to the operator
            status.set_text(f'Solve failed: {exc}')
            return
        solved = solution[0]
        status.set_text(
            f"Solved {solved['events']} events: RMS {solved['rms_before_mm']:.2f} → "
            f"{solved['rms_after_mm']:.2f} mm; rejected {solved['rejected']}; "
            f"weak {solved['weak']}")
        result_label.set_text(json.dumps({
            "k": solved["k"], "b": solved["b"],
            "rms_before_mm": solved["rms_before_mm"],
            "rms_after_mm": solved["rms_after_mm"],
            "rejected": solved["rejected"], "weak": solved["weak"],
        }, indent=2))

    def save_and_apply():
        if solution[0] is None:
            status.set_text('Solve before saving.')
            return
        try:
            bundle = build_updated_bundle(solution[0], hand.calibration_snapshot_path)
            content = (json.dumps(bundle, ensure_ascii=False, indent=2) + "\n").encode()
            result = hand.request_calibration_preview(
                'contact_calibration.mmcal.json', content)
            if not getattr(result, 'accepted', False):
                status.set_text(f"Apply rejected: {getattr(result, 'message', '')}")
                return
            pending[0] = ('preview', None)
            apply_state[0] = 'preview'
            status.set_text('Bundle validated; loading calibration to P4…')
        except Exception as exc:  # noqa: BLE001
            status.set_text(f'Apply failed: {exc}')

    def clear_solution():
        try:
            CORRECTION_PATH.unlink(missing_ok=True)
        except OSError as exc:
            status.set_text(f'Clear failed: {exc}')
            return
        solution[0] = None
        result_label.set_text('')
        status.set_text('Local model correction deleted; reload the page to see the nominal model.')

    start.on_click(start_capture)
    stop.on_click(stop_capture)
    solve_button.on_click(do_solve)
    save.on_click(save_and_apply)
    clear.on_click(clear_solution)

    def refresh(snapshot_data):
        checks = _preconditions(snapshot_data)
        precondition.set_text(' · '.join(
            f"{'✓' if ok else '✗'} {name}" for name, ok in checks))
        start.set_enabled(not capture[0] and all(ok for _, ok in checks))
        stop.set_enabled(capture[0])
        solve_button.set_enabled(bool(recorder[0].events))
        save.set_enabled(solution[0] is not None and not apply_state[0])
        clear.set_enabled(solution[0] is not None or CORRECTION_PATH.exists())

        if capture[0]:
            recorder[0].observe(snapshot_data)
            pairs = {}
            for event in recorder[0].events:
                key = tuple(sorted((event['a']['finger'], event['b']['finger'])))
                pairs[key] = pairs.get(key, 0) + 1
            progress.set_text(
                f"Events {len(recorder[0].events)} · "
                + ' '.join(f"{a}-{b}:{count}" for (a, b), count in sorted(pairs.items()))
                + (' · skipped ' + json.dumps(recorder[0].skips, ensure_ascii=False)
                   if recorder[0].skips else ''))

        if apply_state[0] == 'preview':
            preview = snapshot_data.get('import_preview')
            if preview and preview.get('token'):
                result = hand.request_calibration_import(preview['token'])
                if not getattr(result, 'accepted', False):
                    status.set_text(f"Apply rejected: {getattr(result, 'message', '')}")
                    apply_state[0] = ''
                else:
                    apply_state[0] = 'import'
                    status.set_text('Loading calibration to P4…')
        elif apply_state[0] == 'import':
            operations = [item for item in snapshot_data.get('operations') or []
                          if item.get('name') == 'Load Calibration']
            latest = operations[-1] if operations else None
            if latest and latest.get('status') == 'SUCCESS':
                save_solution(
                    CORRECTION_PATH, solution[0],
                    calibration_path=hand.calibration_snapshot_path)
                apply_state[0] = ''
                status.set_text('Applied. Reload the page to see the calibrated model.')
            elif latest and latest.get('status') == 'FAILED':
                apply_state[0] = ''
                status.set_text(f"Apply failed: {latest.get('message', '')}")

    return refresh
