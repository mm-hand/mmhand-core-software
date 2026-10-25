"""Seed a valid temporary calibration bundle for persistence tests."""
from desktop.core.device_state_store import BundleMultiTurnStore
from desktop.core.calibration_snapshot import _empty_v7_snapshot, atomic_write_snapshot
from desktop.core.protocol import VALID_MOTOR_MASK


class SeededBundleStore(BundleMultiTurnStore):
    def save(self, *, identity, **fields):
        if not self.path.exists():
            payload = _empty_v7_snapshot()
            payload.update(identity, active_motor_mask=hex(VALID_MOTOR_MASK))
            atomic_write_snapshot(self.path, payload)
        fields.pop("raw", None)
        fields.pop("p4_time_ms", None)
        return super().save(identity=identity, **fields)
