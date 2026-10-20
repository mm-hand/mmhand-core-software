"""Calibration alerts are scoped to their origin page and acknowledged once."""
import time
from unittest.mock import Mock

import pytest
from nicegui import ui

from test_native_pages import page
from desktop.core.calibration_workflow import CalibrationWorkflowEvent
from desktop.ui.browser_view import build_page


def console(page):
    hand, client = page
    build_page(hand)
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


def completed_calibration(hand, operation_id, *, warning=False, success=False,
                          include_summary=True, origin='Joint Calibration'):
    hand._calibration_operation_id = hand._application_busy = operation_id
    hand._application_operations.append(dict(
        operation_id=operation_id, name='Selected-joint Calibration', time='12:00:00',
        scope='J03', status='RUNNING', message='Calibrating', phase='prepare',
        origin=origin, finished_at=None, alert_acknowledged=False))
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


@pytest.mark.parametrize('warning', [False, True], ids=['failed', 'warning'])
def test_calibration_alert_opens_once_on_origin_page(page, warning):
    hand, refresh, dialog, tabs, title, message, close = console(page)
    completed_calibration(hand, 'calibration-1', warning=warning)
    refresh()

    assert dialog.value
    assert dialog._props['persistent']
    if warning:
        assert title.text == 'Selected-joint Calibration completed with warnings'
        assert 'J03: excessive tendon friction' in message.text
    else:
        assert title.text == 'Selected-joint Calibration failed'
        assert 'J03: actual servo feedback was lost' in message.text
        assert 'Failed joints: J03' in message.text
    click(close)
    assert not dialog.value
    for _ in range(3):
        refresh()
        assert not dialog.value
    assert hand._application_operations[-1]['alert_acknowledged'] is True


def test_calibration_alert_is_not_shown_on_another_page(page):
    hand, refresh, dialog, tabs, title, message, close = console(page)
    completed_calibration(hand, 'calibration-other', origin='Motor Calibration')
    refresh()
    assert not dialog.value
    assert hand._application_operations[-1]['alert_acknowledged'] is False


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
    assert dialog.value and title.text == 'Selected-joint Calibration failed'
    refresh()
    assert title.text == 'Selected-joint Calibration failed'
    click(close)
    refresh()
    assert dialog.value and title.text == 'Selected-joint Calibration completed with warnings'
    click(close)
    refresh()
    assert not dialog.value
    assert len(hand._application_operations) == 2
    assert all(item['alert_acknowledged'] for item in hand._application_operations)


def test_stale_alert_is_not_replayed(page):
    hand, refresh, dialog, tabs, title, message, close = console(page)
    completed_calibration(hand, 'calibration-stale')
    hand._application_operations[-1]['finished_at'] = time.time() - 901
    refresh()
    assert not dialog.value


def test_completion_popup_does_not_use_timed_notification(page, monkeypatch):
    hand, refresh, dialog, tabs, title, message, close = console(page)
    notification = Mock()
    monkeypatch.setattr(ui, 'notify', notification)
    completed_calibration(hand, 'calibration-fail')
    refresh()
    assert dialog.value
    notification.assert_not_called()
