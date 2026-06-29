import json
import math
import unittest
from pathlib import Path

from mmhand_teleop.stream_writer import validate_angles, write_preview_stream


class StreamWriterTest(unittest.TestCase):
    def test_writes_viewer_compatible_json_atomically(self):
        directory = Path(__file__).parent
        output = directory / "_raw_ik_q_target_test.json"
        angles = [float(index) for index in range(21)]
        try:
            write_preview_stream(output, angles, timestamp=123.5)
            self.assertEqual(
                json.loads(output.read_text(encoding="utf-8")),
                {
                    "timestamp": 123.5,
                    "angles_deg": angles,
                    "source": "ros2_raw_ik_preview",
                },
            )
            self.assertEqual(list(directory.glob("*.tmp")), [])
        finally:
            output.unlink(missing_ok=True)
            Path(f"{output}.tmp").unlink(missing_ok=True)

    def test_rejects_invalid_angle_vectors(self):
        cases = (
            [0.0] * 20,
            [0.0] * 22,
            [0.0] * 20 + [math.nan],
            [0.0] * 20 + [math.inf],
        )
        for angles in cases:
            with self.subTest(angles=angles), self.assertRaises(ValueError):
                validate_angles(angles)


if __name__ == "__main__":
    unittest.main()
