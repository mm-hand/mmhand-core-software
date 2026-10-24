"""Independent P4 time tables are interpolated before constructing LUT pairs."""

from concurrent.futures import ThreadPoolExecutor
from dataclasses import FrozenInstanceError

import pytest

from desktop.core.communication.feedback_history import (
    FeedbackFrame, FeedbackHistoryError, TimestampedFeedbackHistory,
)
from desktop.core.host_calibration.time_alignment import TimeAlignmentError, align_feedback


def frame(kind, time_ms, values, sequence, mask=None, arrival=0.0):
    return FeedbackFrame(kind, time_ms & 0xFFFFFFFF, tuple(values),
                         (1 << len(values)) - 1 if mask is None else mask, arrival, sequence)


def linear_frames():
    packets = [frame("encoder_raw", time_ms, (1000 + 2 * time_ms,), index)
               for index, time_ms in enumerate((0, 100, 200, 300), 1)]
    packets += [frame("servo_abs", time_ms, (500 + 6 * time_ms, -200 - 4 * time_ms), index)
                for index, time_ms in enumerate((50, 150, 250, 350), 5)]
    return packets


def test_async_linear_motion_restores_same_time_pair_instead_of_latest_read():
    result = align_feedback(linear_frames(), 0, (0, 1))
    assert [sample.timestamp_ms for sample in result.samples] == [50, 100, 150, 200, 250, 300]
    assert result.raw_point_count == result.abs_point_count == 4
    assert (result.overlap_start_ms, result.overlap_end_ms) == (50, 300)
    assert result.max_raw_gap_ms == result.max_abs_gap_ms == 100
    for sample in result.samples:
        assert sample.encoder == 1000 + 2 * sample.timestamp_ms
        assert sample.primary_abs == 500 + 6 * sample.timestamp_ms
        assert sample.secondary_abs == -200 - 4 * sample.timestamp_ms
        assert sample.primary_abs == 3 * sample.encoder - 2500


def test_arrival_delay_and_packet_order_do_not_change_p4_time_fit():
    packets = linear_frames()
    delayed = [frame(item.kind, item.p4_time_ms, item.values, item.sequence,
                     item.valid_mask, arrival=1000.0 - index)
               for index, item in enumerate(reversed(packets))]
    assert align_feedback(delayed, 0, (0, 1)) == align_feedback(packets, 0, (0, 1))


def test_piecewise_fit_preserves_a_measured_motion_reversal_without_global_smoothing():
    packets = [frame("encoder_raw", 0, (1000,), 1),
               frame("encoder_raw", 100, (1500,), 2),
               frame("encoder_raw", 200, (1000,), 3),
               frame("servo_abs", 25, (100,), 4),
               frame("servo_abs", 125, (300,), 5),
               frame("servo_abs", 225, (500,), 6)]
    result = align_feedback(packets, 0, (0,))
    assert [(sample.timestamp_ms, sample.encoder, sample.primary_abs)
            for sample in result.samples] == [
                (25, 1125, 100), (100, 1500, 250),
                (125, 1375, 300), (200, 1000, 450),
            ]


def test_physical_encoder_wrap_is_unwrapped_before_interpolation():
    packets = [frame("encoder_raw", 0, (16380,), 1),
               frame("encoder_raw", 100, (4,), 2),
               frame("servo_abs", 25, (20,), 3),
               frame("servo_abs", 75, (60,), 4)]
    result = align_feedback(packets, 0, (0,))
    assert [(sample.timestamp_ms, sample.encoder, sample.primary_abs)
            for sample in result.samples] == [
                (25, 16382, 20),
                (75, 2, 60),
            ]


def test_p4_uint32_rollover_keeps_chronological_common_timestamps():
    base = 0xFFFFFFC0
    packets = [frame("encoder_raw", base, (100,), 1),
               frame("encoder_raw", base + 100, (300,), 2),
               frame("servo_abs", base + 25, (1000,), 3),
               frame("servo_abs", base + 75, (1500,), 4)]
    result = align_feedback(packets, 0, (0,))
    assert [sample.timestamp_ms for sample in result.samples] == [0xFFFFFFD9, 11]
    assert [sample.encoder for sample in result.samples] == [150, 250]
    assert result.max_raw_gap_ms == 100
    assert result.overlap_end_ms == 11


def test_abs_interpolation_rounds_half_away_from_zero():
    packets = [frame("encoder_raw", 0, (100,), 1),
               frame("encoder_raw", 1, (101,), 2),
               frame("encoder_raw", 2, (102,), 3),
               frame("servo_abs", 0, (0, 0), 4),
               frame("servo_abs", 2, (1, -1), 5)]
    middle = align_feedback(packets, 0, (0, 1)).samples[1]
    assert (middle.primary_abs, middle.secondary_abs) == (1, -1)


def test_dual_abs_requires_both_motors_in_same_original_frame():
    packets = [frame("encoder_raw", 0, (100,), 1),
               frame("encoder_raw", 100, (200,), 2),
               frame("servo_abs", 0, (100, 200), 3, mask=1),
               frame("servo_abs", 50, (150, 250), 4, mask=2),
               frame("servo_abs", 100, (200, 300), 5, mask=1)]
    with pytest.raises(TimeAlignmentError, match="raw=2 ABS=0"):
        align_feedback(packets, 0, (0, 1))


def test_an_unselected_motor_losing_feedback_does_not_discard_the_selected_pair():
    packets = [frame("encoder_raw", 0, (100,), 1),
               frame("encoder_raw", 100, (200,), 2),
               frame("servo_abs", 0, (300, 400, 0), 3, mask=3),
               frame("servo_abs", 100, (400, 500, 0), 4, mask=3)]
    result = align_feedback(packets, 0, (0, 1))
    assert result.abs_point_count == 2
    assert len(result.samples) == 2


def test_nonoverlapping_measured_tables_are_not_extrapolated():
    packets = [frame("encoder_raw", 0, (100,), 1),
               frame("encoder_raw", 100, (200,), 2),
               frame("servo_abs", 200, (400,), 3),
               frame("servo_abs", 300, (500,), 4)]
    with pytest.raises(TimeAlignmentError, match="no common valid"):
        align_feedback(packets, 0, (0,))


@pytest.mark.parametrize("invalid_kind", ["encoder_raw", "servo_abs"])
def test_invalid_feedback_splits_the_function_and_is_never_bridged(invalid_kind):
    packets = []
    for kind in ("encoder_raw", "servo_abs"):
        for time_ms in (0, 25, 50, 75, 100):
            invalid = kind == invalid_kind and time_ms == 50
            packets.append(frame(kind, time_ms, (100 + time_ms,), len(packets) + 1,
                                 mask=0 if invalid else 1))
    result = align_feedback(packets, 0, (0,))
    assert [sample.timestamp_ms for sample in result.samples] == [0, 25, 75, 100]


@pytest.mark.parametrize("long_kind", ["encoder_raw", "servo_abs"])
def test_excessive_source_gap_does_not_interpolate_through_missing_telemetry(long_kind):
    packets = []
    for kind in ("encoder_raw", "servo_abs"):
        times = (0, 10, 1000, 1010) if kind == long_kind else (0, 5, 10, 500, 1000, 1005, 1010)
        packets += [frame(kind, time_ms, (100 + time_ms,), len(packets) + index + 1)
                    for index, time_ms in enumerate(times)]
    result = align_feedback(packets, 0, (0,))
    assert 500 not in [sample.timestamp_ms for sample in result.samples]
    assert result.max_raw_gap_ms <= 300
    assert result.max_abs_gap_ms <= 500


def test_a_source_gap_at_the_limit_is_accepted_but_above_it_is_split():
    packets = [frame("encoder_raw", 0, (100,), 1),
               frame("encoder_raw", 300, (400,), 2),
               frame("servo_abs", 0, (1000,), 3),
               frame("servo_abs", 150, (1300,), 4),
               frame("servo_abs", 300, (1600,), 5)]
    accepted = align_feedback(packets, 0, (0,))
    rejected_gap = align_feedback(packets, 0, (0,), raw_max_gap_ms=299)
    assert [sample.timestamp_ms for sample in accepted.samples] == [0, 150, 300]
    assert accepted.max_raw_gap_ms == 300
    assert [sample.timestamp_ms for sample in rejected_gap.samples] == [0, 300]


def test_duplicate_timestamps_use_latest_values_without_duplicate_samples():
    packets = linear_frames()
    packets.append(frame("servo_abs", 150, (1400, -800), 100))
    packets.reverse()
    result = align_feedback(packets, 0, (0, 1))
    assert len(result.samples) == 6
    assert [sample.primary_abs for sample in result.samples if sample.timestamp_ms == 150] == [1400]


def test_invalid_duplicate_timestamp_cannot_be_hidden_by_later_valid_packet():
    packets = linear_frames()
    packets += [frame("encoder_raw", 100, (1200,), 100, mask=0),
                frame("encoder_raw", 100, (1200,), 101)]
    result = align_feedback(packets, 0, (0, 1))
    assert [sample.timestamp_ms for sample in result.samples] == [200, 250, 300]


def test_history_cursor_carries_only_one_seed_per_source_and_post_mark_frames():
    history = TimestampedFeedbackHistory()
    history.record("encoder_raw", 0, [10], 1, 0.0)
    raw = history.record("encoder_raw", 10, [20], 1, 0.01)
    absolute = history.record("servo_abs", 5, [100], 1, 0.02)
    cursor = history.mark()
    later = history.record("servo_abs", 15, [110], 1, 0.03)
    assert history.read(cursor) == (raw, absolute, later)
    with pytest.raises(FrozenInstanceError):
        raw.p4_time_ms = 20
    with pytest.raises(FrozenInstanceError):
        cursor.sequence = 100


def test_history_same_time_invalid_event_is_preserved_and_exact_retransmit_suppressed():
    history = TimestampedFeedbackHistory()
    valid = history.record("encoder_raw", 10, (20,), 1, 0.1)
    cursor = history.mark()
    assert history.record("encoder_raw", 10, (20,), 1, 0.2) is valid
    invalid = history.record("encoder_raw", 10, (20,), 0, 0.3)
    assert history.read(cursor) == (valid, invalid)


def test_history_copies_mutable_packet_values():
    history = TimestampedFeedbackHistory()
    values = [1, 2]
    recorded = history.record("servo_abs", 0, values, 3, 0.0)
    values[0] = 100
    assert recorded.values == (1, 2)


def test_rebase_reset_invalidates_every_old_cursor():
    history = TimestampedFeedbackHistory()
    history.record("servo_abs", 10, (1000,), 1, 0.1)
    old = history.mark()
    history.reset()
    history.record("servo_abs", 20, (0,), 1, 0.2)
    with pytest.raises(FeedbackHistoryError, match="reset"):
        history.read(old)
    assert history.mark().seeds[0].values == (0,)


def test_ring_overflow_is_explicit_only_when_post_cursor_frames_were_lost():
    history = TimestampedFeedbackHistory(max_frames=2)
    history.record("servo_abs", 0, (100,), 1, 0.0)
    cursor = history.mark()
    history.record("servo_abs", 1, (101,), 1, 0.1)
    history.record("servo_abs", 2, (102,), 1, 0.2)
    assert len(history.read(cursor)) == 3  # The pre-mark seed remains available.
    history.record("servo_abs", 3, (103,), 1, 0.3)
    with pytest.raises(FeedbackHistoryError, match="overflow"):
        history.read(cursor)


def test_history_concurrent_sources_have_unique_ordered_sequences():
    history = TimestampedFeedbackHistory(max_frames=300)
    cursor = history.mark()

    def write(kind):
        for index in range(100):
            history.record(kind, index, (index,), 1, index * 0.01)

    with ThreadPoolExecutor(max_workers=2) as pool:
        list(pool.map(write, ("encoder_raw", "servo_abs")))
    packets = history.read(cursor)
    assert [packet.sequence for packet in packets] == list(range(1, 201))
    assert len([packet for packet in packets if packet.kind == "encoder_raw"]) == 100
    assert len([packet for packet in packets if packet.kind == "servo_abs"]) == 100


def test_insufficient_valid_history_has_an_explicit_alignment_error():
    with pytest.raises(TimeAlignmentError, match="no feedback history"):
        align_feedback([], 0, (0,))
    with pytest.raises(TimeAlignmentError, match="two valid timestamps"):
        align_feedback([frame("encoder_raw", 0, (1,), 1)], 0, (0,))
