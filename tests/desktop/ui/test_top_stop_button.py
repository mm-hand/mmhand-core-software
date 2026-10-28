"""STOP 与 Clear Faults 是两个独立按钮；Clear Faults 只在真有事可清时可用。"""
import time
from unittest.mock import Mock

from nicegui import ui

from test_native_pages import page
from desktop.core.application_actions import OperationResult
from desktop.ui.browser_view import build_page


def console(page):
    hand, client = page
    build_page(hand)
    timer = next(element for element in client.elements.values()
                 if isinstance(element, ui.timer)
                 and getattr(element.callback, '__qualname__', '') == 'build_page.<locals>.refresh')
    return hand, timer.callback


def click(button):
    next(listener for listener in button._event_listeners.values()
         if listener.type == 'click').handler(None)


def _buttons(page):
    _hand, client = page
    return [element for element in client.elements.values() if isinstance(element, ui.button)]


def _button(page, text):
    return next(element for element in _buttons(page) if element.text == text)


def test_stop_button_stays_stop_after_latch_and_clear_faults_is_separate(page):
    hand, refresh = console(page)
    refresh()
    _button(page, '■ STOP')
    _button(page, 'Clear Faults')
    hand._estop_requested_at = time.time()
    refresh()
    # STOP 不再改名：锁存后两个按钮依旧同时存在。
    _button(page, '■ STOP')
    _button(page, 'Clear Faults')


def test_clear_faults_click_requests_reset(page):
    hand, refresh = console(page)
    refresh()
    hand._estop_requested_at = time.time()
    clear = Mock(return_value=OperationResult(True, None, 'cleared'))
    hand.request_clear_faults = clear
    refresh()
    click(_button(page, 'Clear Faults'))
    clear.assert_called_once_with()


def test_clear_faults_disabled_when_nothing_to_clear(page):
    hand, refresh = console(page)
    refresh()
    hand._estop_requested_at = 0.0
    refresh()
    # 没有锁存、没有故障、P4 也不在 0 号状态时，按钮灰着不可点。
    assert _button(page, 'Clear Faults').enabled is False
