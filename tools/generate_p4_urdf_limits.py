#!/usr/bin/env python3
"""兼容入口：P4 URDF 限位现由统一模型生成器负责。"""

from __future__ import annotations

import sys

from generate_robot_model import main


if __name__ == "__main__":
    print(
        "generate_p4_urdf_limits.py is deprecated; using generate_robot_model.py",
        file=sys.stderr,
    )
    raise SystemExit(main())
