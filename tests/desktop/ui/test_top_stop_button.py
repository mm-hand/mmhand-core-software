"""Top-right STOP button becomes Clear Faults while an E-STOP is latched."""
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


def test_stop_button_swaps_to_clear_faults_after_stop(page):
    hand, refresh = console(page)
    refresh()
    # Locate the global button by its live text.
    stop = next(element for element in _buttons(page) if element.text == '■ STOP')

    hand._estop_requested_at = time.time()
    clear = Mock(return_value=OperationResult(True, None, 'cleared'))
    hand.request_clear_faults = clear
    refresh()
    top = next(element for element in _buttons(page) if element.text == 'Clear Faults')
    click(top)
    clear.assert_called_once_with()
    assert not any(element.text == '■ STOP' for element in _buttons(page))


def _buttons(page):
    _hand, client = page
    return [element for element in client.elements.values() if isinstance(element, ui.button)]
