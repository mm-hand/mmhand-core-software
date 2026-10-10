"""Motor calibration alerts remain visible until closed on every console tab."""
from unittest.mock import Mock

import pytest
from nicegui import ui

from test_native_pages import page
from desktop.core.calibration_workflow import CalibrationWorkflowEvent
from desktop.ui.browser_view import TABS, build_page


def console(page):
    hand, client = page
    build_page(hand)
    # Diagnostics' ui.code components create their own copy-button timers.
    # Drive the console's actual snapshot refresh rather than the first timer.
    timers = [element for element in client.elements.values() if isinstance(element, ui.timer)
              and getattr(element.callback, '__qualname__', '') == 'build_page.<locals>.refresh']
    assert len(timers) == 1
    timer = timers[0]
    dialog = next(element for element in client.elements.values()
                  if isinstance(element, ui.dialog) and element._props.get('persistent'))
    tabs = next(element for element in client.elements.values() if isinstance(element, ui.tabs))
    card = next(element for element in dialog.default_slot.children if isinstance(element, ui.card))
    title, message = [element for element in card.default_slot.children if isinstance(element, ui.label)]
    close = next(element for element in card.default_slot.children
                 if isinstance(element, ui.button) and element.text == 'Close')
    return hand, timer.callback, dialog, tabs, title, message, close


def completed_calibration(hand, operation_id, *, warning=False, success=False, include_summary=True):
    hand._calibration_operation_id = hand._application_busy = operation_id
    hand._application_operations.append(dict(
        operation_id=operation_id, name='Selected-joint Calibration', time='12:00:00',
        scope='J03', status='RUNNING', message='Calibrating', phase='prepare'))
    status = 'SUCCESS' if success else 'WARN' if warning else 'HOST_FAILED'
    reason = 'all calibration joints completed' if warning or success else 'J03: actual servo feedback was lost'
    records = [dict(status='WARN', warning_summary=(
        dict(ready=True, message='J03: excessive tendon friction') if include_summary else None))] if warning else []
    hand._application_calibration_event(CalibrationWorkflowEvent(
        1, 'calibration_complete', warning or success, reason, joint_mask=8,
        details=dict(terminal_status=status, failed_joint_mask=0 if warning or success else 8,
                     records=records)))


def click(button):
    next(listener for listener in button._event_listeners.values() if listener.type == 'click').handler(None)


@pytest.mark.parametrize('tab', TABS)
@pytest.mark.parametrize('warning', [False, True], ids=['failed', 'warning'])
def test_calibration_completion_opens_persistent_popup_on_every_tab(page, tab, warning):
    hand, refresh, dialog, tabs, title, message, close = console(page)
    tabs.set_value(tab)
    completed_calibration(hand, 'calibration-1', warning=warning)
    refresh()

    assert tabs.value == tab
    assert dialog.value
    assert dialog._props['persistent']
    if warning:
        assert title.text == 'Motor Calibration Completed with Warnings'
        assert 'J03: excessive tendon friction' in message.text
    else:
        assert title.text == 'Motor Calibration Failed'
        assert 'J03: actual servo feedback was lost' in message.text
        assert 'Failed joints: J03' in message.text
    for _ in range(3):
        refresh()
        assert dialog.value
    click(close)
    assert not dialog.value
    for _ in range(3):
        refresh()
        assert not dialog.value


def test_pure_success_does_not_open_completion_popup(page, monkeypatch):
    hand, refresh, dialog, tabs, title, message, close = console(page)
    notification = Mock()
    monkeypatch.setattr(ui, 'notify', notification)
    completed_calibration(hand, 'calibration-ok', success=True)
    refresh()
    assert not dialog.value
    notification.assert_not_called()


def test_warning_without_summary_still_opens_popup(page):
    hand, refresh, dialog, tabs, title, message, close = console(page)
    completed_calibration(hand, 'calibration-warn', warning=True, include_summary=False)
    refresh()
    assert dialog.value
    assert 'Calibration ended with warnings' in message.text


def test_completed_alerts_are_queued_and_each_operation_opens_once(page):
    hand, refresh, dialog, tabs, title, message, close = console(page)
    completed_calibration(hand, 'calibration-fail')
    completed_calibration(hand, 'calibration-warn', warning=True)
    refresh()
    assert dialog.value and title.text == 'Motor Calibration Failed'
    refresh()
    assert title.text == 'Motor Calibration Failed'
    click(close)
    refresh()
    assert dialog.value and title.text == 'Motor Calibration Completed with Warnings'
    click(close)
    refresh()
    assert not dialog.value
    assert len(hand._application_operations) == 2


def test_completion_popup_does_not_use_timed_notification(page, monkeypatch):
    hand, refresh, dialog, tabs, title, message, close = console(page)
    notification = Mock()
    monkeypatch.setattr(ui, 'notify', notification)
    completed_calibration(hand, 'calibration-fail')
    refresh()
    assert dialog.value
    notification.assert_not_called()
