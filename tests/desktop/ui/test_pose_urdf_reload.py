from types import SimpleNamespace

from desktop.core.angle_mapping import JOINT_SPECS
from desktop.core.protocol import ENCODER_COUNT
from desktop.ui import ui_pose_view
from desktop.ui.ui_pose_view import Pose3DView


def _reload_harness():
    calls = []
    harness = SimpleNamespace(
        _kin=None,
        _last_angles=[12.0] * ENCODER_COUNT,
        _closed=True,
        _paused=True,
        _renderer_initialized=False,
        _renderer_failed=False,
        _clear_scene=lambda: calls.append("clear"),
        _set_status=lambda text: calls.append(("status", text)),
        # 重载后画布已可用时，应立即按归零后的 21 路关节角重绘。
        is_renderer_ready=lambda: True,
        draw_angles=lambda angles: calls.append(("draw", list(angles))),
    )
    return harness, calls


def test_reload_urdf_clears_old_scene_and_resets_angles(monkeypatch):
    harness, calls = _reload_harness()
    loaded = SimpleNamespace(
        joint_by_name=lambda: {
            JOINT_SPECS[index].urdf_joint_name: object()
            for index in range(ENCODER_COUNT)
        }
    )
    monkeypatch.setattr(ui_pose_view, "UrdfKinematics", lambda _path: loaded)

    assert Pose3DView.reload_urdf(harness, reset_angles=True)
    assert harness._kin is loaded
    assert harness._closed is False
    assert harness._paused is False
    assert harness._last_angles == [0.0] * ENCODER_COUNT
    assert calls[0] == "clear"
    assert ("draw", [0.0] * ENCODER_COUNT) in calls


def test_reload_urdf_reports_load_failure(monkeypatch):
    harness, calls = _reload_harness()

    def fail_load(_path):
        raise ValueError("broken urdf")

    monkeypatch.setattr(ui_pose_view, "UrdfKinematics", fail_load)

    assert not Pose3DView.reload_urdf(harness, reset_angles=False)
    assert harness._kin is None
    assert any(
        isinstance(item, tuple)
        and item[0] == "status"
        and "broken urdf" in item[1]
        for item in calls
    )
