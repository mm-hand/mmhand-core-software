"""Plot > Hand: equal split, light-green target and the palm stereo panel."""
import pytest
from nicegui import ui

from desktop.ui import browser_view
from desktop.ui.web import hand as hand_view
from test_native_pages import page


class FakePalm:
    """Stand-in for PalmCamera: only the read-only snapshot matters here."""

    def __init__(self, **state):
        self.state = dict(connected=False, device=None, sequence=0,
                          age_ms=None, error=None)
        self.state.update(state)

    def snapshot(self):
        return dict(self.state)

    def close(self):
        pass


def _init_scene(client):
    scene = next(element for element in client.elements.values()
                 if isinstance(element, ui.scene))
    handlers = [event.handler for event in scene._event_listeners.values()
                if event.type == 'init']
    handlers[-1]()
    return scene


def _timer(client, name):
    return next(element for element in client.elements.values()
                if isinstance(element, ui.timer)
                and element.callback.__qualname__.endswith(name))


def test_hand_page_pairs_the_scene_with_the_palm_panel(page):
    hand, client = page
    palm = FakePalm()
    hand_view.build(hand, palm, active=lambda: True)
    scene = _init_scene(client)
    models = [obj for obj in scene.objects.values() if obj.parent.id == 'scene']
    assert len(models) == 2
    measured, target = models
    current_meshes = [obj for obj in scene.objects.values()
                      if measured in obj.ancestors and type(obj).__name__ == 'Stl']
    target_meshes = [obj for obj in scene.objects.values()
                     if target in obj.ancestors and type(obj).__name__ == 'Stl']
    assert current_meshes and all(obj.color == '#808080'
                                  and obj.opacity == 0.85 for obj in current_meshes)
    assert target_meshes and all(obj.color == '#90EE90'
                                 and obj.opacity == 0.45 for obj in target_meshes)

    labels = [element.text for element in client.elements.values()
              if isinstance(element, ui.label)]
    assert 'Gray: latest corrected current pose · Light green: target' in labels
    assert 'MMHand palm stereo' in labels
    images = [element for element in client.elements.values()
              if isinstance(element, ui.interactive_image)]
    assert len(images) == 1
    status = next(element for element in client.elements.values()
                  if isinstance(element, ui.label) and 'palm-status' in element.classes)
    assert status.text == 'OFF'

    palm.state.update(connected=True, device='/dev/video0', sequence=7, age_ms=41.2)
    _timer(client, 'refresh_palm').callback()
    assert images[0].source.endswith('?7')
    assert status.text == '41 ms'

    palm.state.update(connected=False, device=None, sequence=8, age_ms=None)
    _timer(client, 'refresh_palm').callback()
    assert images[0].source.endswith('?8')
    assert status.text == 'OFF'


def test_palm_image_is_not_polled_while_the_hand_page_is_hidden(page):
    hand, client = page
    palm = FakePalm(connected=True, device='/dev/video0', sequence=4, age_ms=10.0)
    hand_view.build(hand, palm, active=lambda: False)
    image = next(element for element in client.elements.values()
                 if isinstance(element, ui.interactive_image))
    _timer(client, 'refresh_palm').callback()
    assert image.source == '/palm.jpg'


@pytest.mark.parametrize('device', [0, '/dev/video0'])
def test_connection_bar_offers_the_palm_camera_beside_p4_and_s3(page, monkeypatch, device):
    monkeypatch.setattr(browser_view, 'camera_devices', lambda: {device: 'Palm camera'})
    hand, client = page
    palm = FakePalm()
    browser_view.build_page(hand, palm)
    select = next(element for element in client.elements.values()
                  if isinstance(element, ui.select) and 'palm-select' in element.classes)
    button = next(element for element in client.elements.values()
                  if isinstance(element, ui.button) and 'palm-connect' in element.classes)
    refresh = next(element for element in client.elements.values()
                   if isinstance(element, ui.timer)
                   and element.callback.__qualname__ == 'build_page.<locals>.refresh')
    refresh.callback()
    assert button.text == 'Connect' and not button.enabled
    assert select.enabled  # the picker is usable until a camera is connected

    select.set_value(device)
    refresh.callback()
    assert button.enabled

    palm.state.update(connected=True, device=select.value, sequence=1, age_ms=5.0)
    refresh.callback()
    assert button.text == 'Disconnect'
    assert not select.enabled  # one node, one owner while connected

    palm.state.update(connected=False, device=None, sequence=2, age_ms=None)
    refresh.callback()
    assert button.text == 'Connect'
    assert select.enabled
    assert any(element.text == '■ STOP' for element in client.elements.values()
               if isinstance(element, ui.button))
