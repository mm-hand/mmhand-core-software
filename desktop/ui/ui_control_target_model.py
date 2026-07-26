"""Reusable joint target workspace and limit calculations."""

import math
from typing import Optional

from desktop.core.angle_mapping import get_spec
from desktop.core.data_models import HandModel, JointWorkspaceRange, resolve_joint_workspace_range
from desktop.core.protocol import ENCODER_COUNT


class JointTargetModel:
    """Centralize joint target workspace and clamp behavior."""

    def __init__(self, mapping_version: int, urdf_crc: int) -> None:
        """Initialize this component."""
        self.mapping_version = int(mapping_version)
        self.urdf_crc = int(urdf_crc) & 0xFFFFFFFF
        self.default_min_deg = [get_spec(i).urdf_range_deg[0] for i in range(ENCODER_COUNT)]
        self.default_max_deg = [get_spec(i).urdf_range_deg[1] for i in range(ENCODER_COUNT)]

    def default_min(self, idx: int) -> float:
        """Return the default URDF minimum target for a joint."""
        if 0 <= idx < len(self.default_min_deg):
            return float(self.default_min_deg[idx])
        return float(self.default_min_deg[-1])

    def default_max(self, idx: int) -> float:
        """Return the default URDF maximum target for a joint."""
        if 0 <= idx < len(self.default_max_deg):
            return float(self.default_max_deg[idx])
        return float(self.default_max_deg[-1])

    def workspace(
        self,
        idx: int,
        state: Optional[HandModel],
    ) -> JointWorkspaceRange:
        """Resolve the calibrated target workspace for a joint."""
        return resolve_joint_workspace_range(
            state,
            idx,
            self.default_min(idx),
            self.default_max(idx),
            self.mapping_version,
            self.urdf_crc,
        )

    def calibrated_min(self, idx: int, state: Optional[HandModel]) -> Optional[float]:
        """Return the calibrated minimum target when the workspace is ready."""
        workspace = self.workspace(idx, state)
        return workspace.current_min_deg if workspace.ready else None

    def target_max(self, idx: int) -> float:
        """Return the effective maximum joint target."""
        return self.default_max(idx)

    def target_min(self, idx: int, state: Optional[HandModel]) -> float:
        """Return the effective minimum joint target."""
        calibrated = self.calibrated_min(idx, state)
        return calibrated if calibrated is not None else self.default_min(idx)

    def clamp(self, idx: int, value: float, state: Optional[HandModel]) -> float:
        """Clamp a joint target into the effective workspace."""
        try:
            value = float(value)
        except (TypeError, ValueError):
            value = self.target_min(idx, state)
        if not math.isfinite(value):
            value = self.target_min(idx, state)
        lo = self.target_min(idx, state)
        hi = self.target_max(idx)
        return max(lo, min(hi, value))
