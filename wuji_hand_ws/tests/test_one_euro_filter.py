import copy
import math
import unittest

import numpy as np

from wuji_mmhand_teleop.sources.one_euro_filter import OneEuroFilter3D
from wuji_mmhand_teleop.sources.wuji_sdk_source import WujiReceiver
from wuji_mmhand_teleop.types import WujiFrame


def make_frame(timestamp, recv_perf_time, offset=0.0):
    def node(x):
        return {"pos": [x + offset, 0.0, 0.0],
                "rot": [1.0, 0.0, 0.0, 0.0], "confidence": 0.9}
    skeleton = {
        "hand": node(0.00),
        "thumb": {"cmc": node(0.01), "mcp": node(0.02),
                  "ip": node(0.03), "tip": node(0.04)},
        "index": {"mcp": node(0.05), "pip": node(0.06),
                  "dip": node(0.07), "tip": node(0.08)},
    }
    return WujiFrame(
        seq=1, timestamp=timestamp, recv_time=100.0,
        recv_perf_time=recv_perf_time, dt=0.0, glove_id=0,
        side="left", skeleton=skeleton, frame_id="test",
    )


class OneEuroFilter3DTest(unittest.TestCase):
    def test_alpha_matches_rc_low_pass(self):
        expected = (1.0 / 60.0) / (1.0 / (2.0 * math.pi) + 1.0 / 60.0)
        self.assertAlmostEqual(
            OneEuroFilter3D.alpha(1.0, 1.0 / 60.0), expected)

    def test_static_noise_is_reduced(self):
        filter_3d = OneEuroFilter3D(min_cutoff=1.0, beta=20.0)
        raw = []
        filtered = []
        for index in range(600):
            value = 0.001 * (1.0 if index % 2 else -1.0)
            raw.append(value)
            filtered.append(
                filter_3d.update([value, 0.0, 0.0], index / 60.0)[0])
        raw_rms = float(np.sqrt(np.mean(np.square(raw[100:]))))
        filtered_rms = float(np.sqrt(np.mean(np.square(filtered[100:]))))
        self.assertLess(filtered_rms, raw_rms * 0.6)

    def test_beta_reduces_motion_lag(self):
        fixed = OneEuroFilter3D(min_cutoff=1.0, beta=0.0)
        adaptive = OneEuroFilter3D(min_cutoff=1.0, beta=20.0)
        for index in range(31):
            value = index * 0.001
            timestamp = index / 60.0
            fixed_value = fixed.update([value, 0.0, 0.0], timestamp)[0]
            adaptive_value = adaptive.update(
                [value, 0.0, 0.0], timestamp)[0]
        self.assertGreater(adaptive_value, fixed_value)
        self.assertLess(0.030 - adaptive_value, 0.030 - fixed_value)

    def test_bad_dt_or_long_gap_resets(self):
        filter_3d = OneEuroFilter3D()
        filter_3d.update([0.0, 0.0, 0.0], 1.0)
        self.assertEqual(
            filter_3d.update([1.0, 0.0, 0.0], 1.0).tolist(),
            [1.0, 0.0, 0.0])
        self.assertEqual(
            filter_3d.update([2.0, 0.0, 0.0], 2.0).tolist(),
            [2.0, 0.0, 0.0])


class WujiReceiverFilteringTest(unittest.TestCase):
    def test_only_distal_positions_are_filtered(self):
        receiver = WujiReceiver()
        receiver._filter_skeleton_positions(make_frame(1.0, 10.0))
        second = make_frame(
            1.0 + 1.0 / 60.0, 10.0 + 1.0 / 60.0, 0.01)
        original = copy.deepcopy(second.skeleton)
        receiver._filter_skeleton_positions(second)
        self.assertEqual(second.skeleton["hand"], original["hand"])
        self.assertEqual(
            second.skeleton["thumb"]["cmc"], original["thumb"]["cmc"])
        self.assertEqual(
            second.skeleton["index"]["mcp"], original["index"]["mcp"])
        self.assertNotEqual(
            second.skeleton["thumb"]["tip"]["pos"],
            original["thumb"]["tip"]["pos"])
        self.assertNotEqual(
            second.skeleton["index"]["pip"]["pos"],
            original["index"]["pip"]["pos"])
        self.assertEqual(
            second.skeleton["thumb"]["tip"]["rot"],
            original["thumb"]["tip"]["rot"])
        self.assertEqual(
            second.skeleton["thumb"]["tip"]["confidence"],
            original["thumb"]["tip"]["confidence"])

    def test_frozen_sdk_clock_switches_to_receive_clock(self):
        receiver = WujiReceiver()
        receiver._filter_skeleton_positions(make_frame(2.0, 20.0))
        receiver._filter_skeleton_positions(
            make_frame(2.0, 20.0 + 1.0 / 60.0, 0.01))
        self.assertEqual(receiver._filter_time_source, "receive")
        following = make_frame(2.0, 20.0 + 2.0 / 60.0, 0.02)
        raw_tip = following.skeleton["index"]["tip"]["pos"][0]
        receiver._filter_skeleton_positions(following)
        self.assertLess(
            following.skeleton["index"]["tip"]["pos"][0], raw_tip)

    def test_reset_restores_timestamp_detection(self):
        receiver = WujiReceiver()
        receiver._filter_skeleton_positions(make_frame(0.0, 30.0))
        self.assertEqual(receiver._filter_time_source, "receive")
        receiver._reset_skeleton_filters()
        receiver._filter_skeleton_positions(make_frame(4.0, 31.0))
        self.assertEqual(receiver._filter_time_source, "sdk")


if __name__ == "__main__":
    unittest.main()
