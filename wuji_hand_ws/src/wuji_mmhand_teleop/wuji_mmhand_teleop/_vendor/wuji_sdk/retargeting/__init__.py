"""``wuji_sdk.retargeting`` — map human hand keypoints to Wuji Hand joint angles.

The heavy lifting lives in the compiled (closed-source) ``wuji_sdk._retargeting``
algorithm; this subpackage exposes the user-facing surface: the
:class:`RetargetSession` class plus helpers to locate the builtin tuning configs
/ URDF that ship inside the wheel.

Build a session with :meth:`RetargetSession.for_hand` — pass a
:class:`HandModel` and a ``wuji_sdk.Handedness``; the hand model picks the
builtin config, so you never deal with config paths. :func:`config_path` is for
*locating* a builtin config to inspect or tweak; ``RetargetSession.from_yaml``
takes a custom config path (advanced).

Teleoperation (driving a connected hand from a live keypoint source) is shown as
an example built on this surface — see
``public/examples/python/retargeting/1.teleop_real.py`` — not exposed as an API.
"""

from __future__ import annotations

import os

# RetargetSession + the HandModel enum are registered on the compiled
# extension's flat module; re-export them under this subpackage. The `side`
# argument reuses the shared `wuji_sdk.Handedness` enum.
from .._wuji_sdk import HandModel, RetargetSession

# Root of the bundled (compiled) retargeting data: configs/ + URDF.
_RETARGETING_DIR = os.path.join(os.path.dirname(__file__), "..", "_retargeting")
_CONFIG_DIR = os.path.normpath(os.path.join(_RETARGETING_DIR, "configs"))


def config_path(name: str) -> str:
    """Absolute path to a builtin tuning config bundled in the wheel.

    Args:
        name: config path relative to the builtin config root, e.g.
            ``"topo/topo_wuji_hand_2.yaml"``.

    Raises:
        FileNotFoundError: if no such builtin config ships in this wheel
            (e.g. the retargeting bundle was not built into the package).
    """
    path = os.path.normpath(os.path.join(_CONFIG_DIR, name))
    if not os.path.isfile(path):
        raise FileNotFoundError(f"builtin retargeting config not found: {name} ({path})")
    return path


__all__ = ["HandModel", "RetargetSession", "config_path"]
