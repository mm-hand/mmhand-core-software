"""The calibration import dialog reports failed validation instead of freezing at 'Validating…'."""
import asyncio
import time
from types import SimpleNamespace

from nicegui import ui

from test_native_pages import page
from desktop.ui.browser_view import build_page


def console(page):
    hand, client = page
    build_page(hand)
    timer = next(element for element in client.elements.values() if isinstance(element, ui.timer)
                 and getattr(element.callback, '__qualname__', '') == 'build_page.<locals>.refresh')
    dialog = next(element for element in client.elements.values() if isinstance(element, ui.dialog)
                  and not element._props.get('persistent')
                  and any(isinstance(child, ui.label) and child.text == 'Load Joint & Motor Calibration'
                          for child in element.default_slot.children[0].default_slot.children))
    info = next(element for element in client.elements.values() if isinstance(element, ui.label)
                and element.text == 'Choose a calibration file to validate.')
    load = next(element for element in client.elements.values() if isinstance(element, ui.button)
                and element.text == 'Load to P4')
    upload = next(element for element in client.elements.values() if isinstance(element, ui.upload))
    return hand, timer.callback, dialog, info, load, upload


def upload_file(upload, name, content):
    async def read():
        return content
    asyncio.run(upload._upload_handlers[0](SimpleNamespace(file=SimpleNamespace(name=name, read=read))))


def wait_for_inspect(hand, timeout=5.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        operations = [item for item in hand._application_operations
                      if item['name'] == 'Inspect Calibration']
        if operations and operations[-1]['status'] in ('SUCCESS', 'FAILED'):
            return operations[-1]
        time.sleep(0.02)
    raise AssertionError('Inspect Calibration did not finish')


def test_failed_inspect_shows_error_and_keeps_load_disabled(page):
    hand, refresh, dialog, info, load, upload = console(page)
    dialog.open()
    upload_file(upload, 'broken.mmcal.json', b'not json')
    operation = wait_for_inspect(hand)
    assert operation['status'] == 'FAILED'
    refresh()
    assert info.text.startswith('Validation failed:')
    assert operation['message'] in info.text
    assert not load.enabled


def test_successful_preview_still_shows_file_and_enables_load(page):
    hand, refresh, dialog, info, load, upload = console(page)
    hand._import_previews['token-1'] = {'name': 'hand.mmcal.json', 'payload': {
        'saved_at_host_time': 1785258110.0, 'hand_profile': 'v2',
        'active_joint_mask': '0x000F000F', 'active_motor_mask': '0x01F0001F',
        'manual_reference_valid_bitmap': '0x001FFFFF'}}
    dialog.open()
    refresh()
    assert info.text.startswith('File: hand.mmcal.json')
    assert load.enabled
