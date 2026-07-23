from desktop.core.data_recording.motion_sampler import (
    MotionCommandSampler,
    classify_motion_source,
)


def _row(source="ROS2:/raw_ik_target", *, live=True):
    return {
        "source": source,
        "live_enabled": 1 if live else 0,
        "sent_to_backend": 1,
        "blocked": 0,
        "command_joint_mask": "0x00000001",
    }


def test_continuous_stream_is_limited_and_keeps_tail():
    sampler = MotionCommandSampler(period_s=0.05)

    first = sampler.submit(_row(), now_monotonic=1.00)
    middle = sampler.submit(_row(), now_monotonic=1.02)
    tail = sampler.flush_due(now_monotonic=1.08)

    assert len(first) == 1
    assert middle == []
    assert len(tail) == 1
    assert first[0]["frame_kind"] == "continuous"
    assert tail[0]["stream_id"] == first[0]["stream_id"]


def test_discrete_ui_command_is_written_immediately():
    sampler = MotionCommandSampler(period_s=0.05)

    emitted = sampler.submit(_row("Send J03", live=False), now_monotonic=1.0)

    assert len(emitted) == 1
    assert emitted[0]["source_kind"] == "ui_feedback"
    assert emitted[0]["frame_kind"] == "discrete"
    assert emitted[0]["replayable"] == 1


def test_visual_retargeting_is_a_replayable_continuous_source():
    assert classify_motion_source("Visual Retargeting", True) == (
        "visual_retargeting",
        "continuous",
        True,
    )
