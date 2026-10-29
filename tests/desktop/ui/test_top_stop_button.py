"""STOP 是唯一常驻的故障操作；RESET 由 Enable/维护操作自己完成。"""
import time

from nicegui import ui

from test_native_pages import page
from desktop.ui.browser_view import build_page


def console(page):
    hand, client = page
    build_page(hand)
    timer = next(element for element in client.elements.values()
                 if isinstance(element, ui.timer)
                 and getattr(element.callback, '__qualname__', '') == 'build_page.<locals>.refresh')
    return hand, timer.callback


def _buttons(page):
    _hand, client = page
    return [element for element in client.elements.values() if isinstance(element, ui.button)]


def _button(page, text):
    return next(element for element in _buttons(page) if element.text == text)


def test_stop_button_stays_stop_and_no_clear_faults_button(page):
    hand, refresh = console(page)
    refresh()
    _button(page, '■ STOP')
    assert not [button for button in _buttons(page) if button.text == 'Clear Faults']
    hand._estop_requested_at = time.time()
    refresh()
    # STOP 不改名，也不再有独立的 Clear Faults 按钮。
    _button(page, '■ STOP')
    assert not [button for button in _buttons(page) if button.text == 'Clear Faults']
