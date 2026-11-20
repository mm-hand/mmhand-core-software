"""Palm stereo camera: enumerate UVC capture nodes and own the latest JPEG.

职责：枚举可采集的 UVC 节点，持有掌心相机的打开/取流/释放和最新一帧 JPEG。
线程：connect/disconnect 在调用线程执行；取流在本模块自己的守护线程。
输入：V4L2 设备路径；UVC 原始双目并排帧。
输出：最新 JPEG 字节、序号、帧龄和错误。
状态所有权：本对象拥有 VideoCapture 和取流线程；UI 只读快照。
禁止依赖：禁止导入 NiceGUI、协议和控制器。

The enumeration and latest-frame reader mirror mmhand-teleop/input/camera.py,
so both applications pick the same capture nodes and drop V4L2 metadata nodes.
"""
from __future__ import annotations

import glob
import os
import threading
import time
from pathlib import Path

import cv2
import numpy as np

# The Waveshare AR0144 stereo module delivers the two eyes side by side in one
# UVC frame; UVC only exposes 1280x480 at 30 fps with MJPG.
SIZE = (1280, 480)
FPS = 30
FOURCC = 'MJPG'
JPEG_QUALITY = 82
_BACKEND = cv2.CAP_DSHOW if os.name == 'nt' else cv2.CAP_V4L2
# V4L2 exposes one extra "metadata" video node per camera. It shares the
# camera's name and reads back index 1, and cannot capture frames.
_NAME_DIR = '/sys/class/video4linux'


def camera_devices(pattern='/dev/video*', names=_NAME_DIR):
    """List the capture nodes a camera picker can offer, without opening any.

    Returns ``{device: label}``. Nodes V4L2 reports with a non-zero ``index``
    are metadata nodes and are skipped; a platform without ``/dev/video*``
    falls back to plain indices.
    """
    choices = {}
    for path in sorted(glob.glob(pattern)):
        node = Path(names, Path(path).name)
        try:
            if node.joinpath('index').read_text().strip() != '0':
                continue
        except OSError:
            pass  # no sysfs entry: trust the node
        try:
            label = node.joinpath('name').read_text().strip()
        except OSError:
            label = ''
        choices[path] = f'{path} · {label}' if label else path
    return choices or {index: f'camera {index}' for index in range(4)}


def _black_jpeg():
    """A valid blank frame, so the panel is black before the first stream frame."""
    ok, encoded = cv2.imencode('.jpg', np.zeros((SIZE[1], SIZE[0], 3), np.uint8))
    return encoded.tobytes() if ok else b''


BLACK_JPEG = _black_jpeg()


class PalmCamera:
    """One palm camera, one reader thread, latest JPEG readable without a lock."""

    def __init__(self):
        self._lock = threading.Lock()
        self._capture = None
        self._thread = None
        self.device = None
        self.error = None
        self.sequence = 0
        self.jpeg = BLACK_JPEG
        self.timestamp = None

    def connect(self, device):
        """Open one UVC node and start the reader; raises without a frame."""
        if not device:
            raise ValueError('choose a palm camera first')
        self.disconnect()
        capture = self._open(device)
        with self._lock:
            self._capture = capture
            self.device = device
            self.error = None
            self.timestamp = None
            self._thread = threading.Thread(
                target=self._read_loop, args=(capture,),
                name='palm-camera', daemon=True)
        self._thread.start()

    @staticmethod
    def _open(device):
        """Open and configure the node; a V4L2 node has exactly one owner."""
        capture = cv2.VideoCapture(device, _BACKEND)
        try:
            if not capture.isOpened():
                raise RuntimeError(
                    f"cannot open palm camera {device!r}: it may already be "
                    "held by another program, or the node may not offer a "
                    "colour format OpenCV can capture")
            if FOURCC is not None:
                capture.set(cv2.CAP_PROP_FOURCC,
                            cv2.VideoWriter_fourcc(*FOURCC))
            for prop, value in ((cv2.CAP_PROP_FRAME_WIDTH, SIZE[0]),
                                (cv2.CAP_PROP_FRAME_HEIGHT, SIZE[1]),
                                (cv2.CAP_PROP_FPS, FPS),
                                (cv2.CAP_PROP_BUFFERSIZE, 1)):
                capture.set(prop, value)
        except BaseException:
            capture.release()
            raise
        return capture

    def _read_loop(self, capture):
        while True:
            with self._lock:
                if self._capture is not capture:
                    return
            ok, frame = capture.read()
            if not ok:
                self._drop(capture, 'palm camera stopped delivering frames')
                return
            ok, encoded = cv2.imencode(
                '.jpg', frame, (cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY))
            if not ok:
                continue
            with self._lock:
                if self._capture is not capture:
                    return
                self.jpeg = encoded.tobytes()
                self.timestamp = time.monotonic()
                self.sequence += 1

    def _drop(self, capture, message):
        """Reader-side release after a failed read; never joins itself."""
        with self._lock:
            if self._capture is not capture:
                return
            self._capture = None
            self._thread = None
            self.device = None
            self.error = message
            self.jpeg = BLACK_JPEG
            self.sequence += 1
        capture.release()

    def disconnect(self):
        """Release the device and tell every open page to go back to OFF."""
        with self._lock:
            capture, thread = self._capture, self._thread
            self._capture = None
            self._thread = None
            self.device = None
            if capture is not None:
                self.jpeg = BLACK_JPEG
                self.sequence += 1
        if capture is not None:
            capture.release()
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)

    def snapshot(self):
        """Read-only state for the connection bar and the palm panel."""
        with self._lock:
            age = (None if self.timestamp is None
                   else max(0.0, time.monotonic() - self.timestamp))
            return {
                'connected': self._capture is not None,
                'device': self.device,
                'sequence': self.sequence,
                'age_ms': None if age is None else age * 1000.0,
                'error': self.error,
            }

    def close(self):
        self.disconnect()
