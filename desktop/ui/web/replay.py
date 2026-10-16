"""Existing CSV replay service, preview and playback progress.

职责：展示快照并提交后端操作。
线程：NiceGUI 页面事件循环。
输入：后端快照及用户交互。
输出：原生控件和操作请求。
状态所有权：仅目标草稿和显示缓存；设备真值归后端。
禁止依赖：禁止直接操作协议、串口和控制器。
"""
from nicegui import ui
from .common import label, show, table, set_rows, selection, FINGERS

def build(hand, act):
    async def upload(event):
        content = await event.file.read()
        act(lambda: hand.request_replay_upload(event.file.name, content))
    with ui.dialog() as importer, ui.card():
        ui.upload(label='Import CSV', on_upload=upload, auto_upload=True, max_file_size=50_000_000).props('accept=.csv').classes('w-80')
        ui.button('Close', on_click=importer.close).props('flat')
    with ui.row().classes('page-toolbar'):
        ui.button('Import CSV', on_click=importer.open)
        ui.button('Play', on_click=lambda: act(hand.request_replay_start))
        metadata = label()
    warnings = label()
    selectors = []
    with ui.row().classes('w-full gap-4'):
        for name, joints in FINGERS:
            with ui.column().classes('gap-1'):
                selectors.append(selection(hand, 'selected_joints', joints, name))
                for joint in joints:
                    selectors.append(selection(hand, 'selected_joints', [joint], f'J{joint:02d}'))
    preview = table([('joint', 'Joint'), ('angle', 'Current deg'), ('valid', 'Valid'), ('target', 'Replay Target'), ('error', 'Error')])
    with ui.row().classes('w-full items-center'):
        status = label()
        progress = ui.linear_progress(value=0).classes('flex-1')
    def refresh(snapshot):
        for update in selectors: update(snapshot)
        replay, file = snapshot['replay'], snapshot['replay'].get('file', {})
        metadata.set_text(f"File: {file.get('name', '—')}   Frames: {file.get('frames', '—')}   Duration: {show(file.get('duration_s'), 's')}   Rate: {show(file.get('rate_hz'), 'Hz')}")
        warnings.set_text('Warnings: ' + ('; '.join(file.get('warnings', []) + list(replay.get('warnings', []))) or '—'))
        targets, mask = replay.get('current_targets', []), replay.get('current_joint_mask', 0)
        rows = []
        for row in snapshot['joints']:
            j, angle = row['joint'], row['angle']
            target = targets[j] if j < len(targets) and mask & (1 << j) else None
            rows.append(dict(joint=f'J{j:02d}', angle=show(angle), valid='Yes' if angle is not None else 'No', target=show(target), error=show(target - angle if target is not None and angle is not None else None)))
        set_rows(preview, rows)
        total, frame = replay.get('total_frames', 0), replay.get('frame_index', 0)
        value = min(1, frame / total) if total else 0
        progress.set_value(value)
        status.set_text(f"Status: {replay.get('state', 'idle')}   Frame: {frame}/{total}   {value:.0%}   {replay.get('message', '')}")
    return refresh
