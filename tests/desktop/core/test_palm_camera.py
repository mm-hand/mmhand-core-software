"""Palm stereo camera: UVC enumeration, reader thread and JPEG state."""
import time
from pathlib import Path

import numpy as np
import pytest

from desktop.core import palm_camera


def test_camera_devices_lists_only_capture_nodes(tmp_path):
    names = tmp_path / 'names'
    names.mkdir()
    for node, label, index in (
        ('video0', 'Integrated Camera', '0'),
        ('video1', 'Integrated Camera', '1'),  # UVC metadata node
        ('video2', 'MMHand Palm Stereo', '0'),
    ):
        (names / node).mkdir()
        (names / node / 'name').write_text(label)
        (names / node / 'index').write_text(index)
        (tmp_path / node).write_text('')
    devices = palm_camera.camera_devices(pattern=str(tmp_path / 'video*'),
                                         names=str(names))
    labels = {Path(path).name: label.split(' · ')[-1]
              for path, label in devices.items()}
    assert labels == {'video0': 'Integrated Camera',
                      'video2': 'MMHand Palm Stereo'}


def test_camera_devices_falls_back_to_indices_without_video_nodes(tmp_path):
    devices = palm_camera.camera_devices(pattern=str(tmp_path / 'none*'),
                                         names=str(tmp_path / 'names'))
    assert list(devices) == [0, 1, 2, 3]


def test_connect_requires_a_device():
    camera = palm_camera.PalmCamera()
    with pytest.raises(ValueError, match='choose a palm camera'):
        camera.connect('')
    assert not camera.snapshot()['connected']


def test_failed_open_releases_the_capture_and_reports_it(monkeypatch):
    released = []

    class ClosedCapture:
        def __init__(self, device, backend):
            self.device = device

        def isOpened(self):
            return False

        def release(self):
            released.append(True)

    monkeypatch.setattr(palm_camera.cv2, 'VideoCapture', ClosedCapture)
    camera = palm_camera.PalmCamera()
    with pytest.raises(RuntimeError, match='cannot open palm camera'):
        camera.connect('/dev/video99')
    assert released == [True]
    assert not camera.snapshot()['connected']


def _streaming_capture(read):
    class Capture:
        def __init__(self, device, backend):
            self.device = device
            self.released = False

        def isOpened(self):
            return True

        def set(self, prop, value):
            return True

        def read(self):
            return read(self)

        def release(self):
            self.released = True

    return Capture


def _wait(predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        time.sleep(0.005)
    return predicate()


def test_connect_publishes_jpeg_and_disconnect_releases(monkeypatch):
    monkeypatch.setattr(palm_camera.cv2, 'VideoCapture', _streaming_capture(
        lambda capture: (True, np.full((48, 128, 3), 7, np.uint8))))
    camera = palm_camera.PalmCamera()
    camera.connect('/dev/video-test')

    assert _wait(lambda: camera.snapshot()['sequence'] >= 2)
    state = camera.snapshot()
    assert state['connected'] and state['device'] == '/dev/video-test'
    assert state['age_ms'] is not None and state['error'] is None
    assert camera.jpeg.startswith(b'\xff\xd8')

    camera.disconnect()
    state = camera.snapshot()
    assert not state['connected'] and state['device'] is None
    assert camera.jpeg == palm_camera.BLACK_JPEG
    assert state['sequence'] >= 3  # the black frame is a new publish


def test_reader_drops_the_camera_after_a_failed_frame(monkeypatch):
    def read(capture):
        capture.reads = getattr(capture, 'reads', 0) + 1
        if capture.reads > 2:
            return False, None
        return True, np.zeros((48, 128, 3), np.uint8)

    monkeypatch.setattr(palm_camera.cv2, 'VideoCapture',
                        _streaming_capture(read))
    camera = palm_camera.PalmCamera()
    camera.connect('/dev/video-test')
    assert _wait(lambda: camera.snapshot()['error'] is not None)
    state = camera.snapshot()
    assert not state['connected'] and state['device'] is None
    assert 'stopped delivering frames' in state['error']
    assert camera.jpeg == palm_camera.BLACK_JPEG
    camera.close()
