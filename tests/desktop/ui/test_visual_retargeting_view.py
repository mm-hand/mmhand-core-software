import math
import os
from dataclasses import dataclass
from types import SimpleNamespace

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from desktop.core.protocol import ENCODER_COUNT
from desktop.teleop.joint_target_adapter import URDF_DEG
import desktop.ui.ui_visual_retargeting_view as visual_module
from desktop.ui.ui_visual_retargeting_view import VisualRetargetingView
from desktop.ui.ui_qt_compat import ensure_app


_APP = ensure_app()


@dataclass(frozen=True)
class _Adaptation:
    source_space: str
    source_targets: tuple[float, ...]
    urdf_targets_deg: tuple[float, ...]


class _Context:
    def __init__(self, connected: bool = True) -> None:
        self.connected = bool(connected)
        self.calls = []
        self.actions = []
        self.statuses = []

    def is_comm_connected(self):
        return self.connected

    def send_joint_targets(self, targets, **kwargs):
        self.calls.append((list(targets), kwargs))
        return SimpleNamespace(sent=True, message="", targets=list(targets))

    def emit_action(self, text):
        self.actions.append(str(text))

    def set_status(self, text):
        self.statuses.append(str(text))


def _view(mask: int | None = None) -> tuple[VisualRetargetingView, _Context]:
    context = _Context()
    getter = None if mask is None else (lambda: int(mask))
    view = VisualRetargetingView(context, selected_joint_mask_getter=getter)
    return view, context


def test_visual_targets_are_adapted_before_send(monkeypatch):
    raw = [float(idx) for idx in range(ENCODER_COUNT)]
    adapted = [value + 100.0 for value in raw]
    calls = []

    def fake_adapt(values, *, source_space):
        source = tuple(float(value) for value in values)
        calls.append((source, source_space))
        return _Adaptation(
            source_space=source_space,
            source_targets=source,
            urdf_targets_deg=tuple(adapted),
        )

    monkeypatch.setattr(visual_module, "adapt_joint_targets", fake_adapt)
    view, context = _view()
    view._set_send_targets_enabled(True)

    view._on_joints_ready(raw, 1234)

    assert calls == [(tuple(raw), URDF_DEG)]
    targets, kwargs = context.calls[-1]
    assert targets == pytest.approx(adapted)
    assert kwargs["input_space"] == URDF_DEG
    assert list(kwargs["raw_targets"]) == raw
    assert view._last_targets == pytest.approx(adapted)
    assert view.angle_table.item(0, 1).text() == "100.00"


def test_selected_joints_use_adapted_targets_and_unselected_hold_feedback(monkeypatch):
    raw = [float(idx) for idx in range(ENCODER_COUNT)]
    adapted = [value + 10.0 for value in raw]

    def fake_adapt(values, *, source_space):
        source = tuple(float(value) for value in values)
        return _Adaptation(
            source_space=source_space,
            source_targets=source,
            urdf_targets_deg=tuple(adapted),
        )

    monkeypatch.setattr(visual_module, "adapt_joint_targets", fake_adapt)
    view, context = _view(mask=1 << 3)
    view._feedback_angles = [50.0 + idx for idx in range(ENCODER_COUNT)]
    view._set_send_targets_enabled(True)

    view._on_joints_ready(raw, 2345)

    targets, kwargs = context.calls[-1]
    assert targets[3] == pytest.approx(adapted[3])
    assert targets[2] == pytest.approx(52.0)
    assert targets[4] == pytest.approx(54.0)
    assert list(kwargs["raw_targets"]) == raw


@pytest.mark.parametrize(
    "bad_targets",
    (
        [0.0] * (ENCODER_COUNT - 1),
        [math.nan] * ENCODER_COUNT,
        [math.inf] * ENCODER_COUNT,
    ),
)
def test_invalid_visual_targets_are_dropped(bad_targets):
    view, context = _view()
    view._set_send_targets_enabled(True)

    view._on_joints_ready(bad_targets, 3456)

    assert context.calls == []
    assert context.statuses
    assert "Visual Retargeting target dropped" in context.statuses[-1]
    assert view.frame_label.text() == "Frame: -"
