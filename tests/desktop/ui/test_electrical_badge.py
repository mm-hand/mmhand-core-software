"""State bar shows one CAN label; only faulty joints add a red encoder badge."""
from nicegui import Client, ui

from test_native_pages import page
from desktop.ui.browser_view import build_page
from desktop.ui.web.common import encoder_badge


def test_status_bar_shows_only_a_can_label(page):
    hand, client = page
    build_page(hand)
    labels = [element.text for element in client.elements.values() if isinstance(element, ui.label)]
    assert any((text or '').startswith('CAN:') for text in labels)
    assert not any((text or '').startswith(('electrics:', 'encoders:', 'electrical connect:')) for text in labels)


def test_encoder_badge_only_appears_for_faulty_joints():
    client = Client(ui.page('/test-encoder-badge'))
    with client:
        refresh = encoder_badge()
        refresh('can_error', 0)
        assert [element.text for element in client.elements.values() if isinstance(element, ui.badge)] == ['CAN ERROR']
        refresh('open', 5)
        assert [element.text for element in client.elements.values() if isinstance(element, ui.badge)] == ['ENCODER OPEN']
        refresh('ok', 5)
        assert not [element for element in client.elements.values() if isinstance(element, ui.badge)]
    client.delete()
