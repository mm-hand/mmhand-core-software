import json
import math
import os
import time
from pathlib import Path
from typing import Iterable


JOINT_COUNT = 21


def validate_angles(values: Iterable[float]) -> list[float]:
    """Return one finite J00-J20 angle vector, or raise ValueError."""
    angles = [float(value) for value in values]
    if len(angles) != JOINT_COUNT:
        raise ValueError(f"expected {JOINT_COUNT} angles, got {len(angles)}")
    if not all(math.isfinite(value) for value in angles):
        raise ValueError("all angles must be finite")
    return angles


def write_preview_stream(
    output_path: str | os.PathLike[str],
    values: Iterable[float],
    *,
    timestamp: float | None = None,
) -> None:
    """Atomically write the JSON format consumed by raw_ik_urdf_viewer.py."""
    angles = validate_angles(values)
    path = Path(output_path).expanduser()
    if not str(path):
        raise ValueError("output_path must not be empty")
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "timestamp": float(time.time() if timestamp is None else timestamp),
        "angles_deg": angles,
        "source": "ros2_raw_ik_preview",
    }

    temp_path = Path(f"{path}.tmp")
    try:
        with temp_path.open("w", encoding="utf-8") as handle:
            json.dump(payload, handle, separators=(",", ":"), allow_nan=False)
            handle.flush()
        os.replace(temp_path, path)
    finally:
        if temp_path.exists():
            temp_path.unlink()
