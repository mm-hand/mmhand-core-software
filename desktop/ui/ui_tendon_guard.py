"""Compatibility re-export for tendon guard helpers.

The tendon guard is application/backend logic now.  Keep this module so older
UI imports and visualization scripts continue to work.
"""

try:
    from tendon_guard import (
        JOINT_TO_PRIMARY_MOTOR_INDEX,
        MOTOR_TENDON_SHORTEN_ABS_SIGN,
        MOTOR_TO_JOINT_INDEX,
        TendonGuard,
        clamp_motor_abs,
        motor_abs_after_tendon_action,
        motor_target_from_servo_feedback,
        motor_tendon_shorten_abs_sign,
    )
except ImportError:  # pragma: no cover - package import fallback
    from ..tendon_guard import (
        JOINT_TO_PRIMARY_MOTOR_INDEX,
        MOTOR_TENDON_SHORTEN_ABS_SIGN,
        MOTOR_TO_JOINT_INDEX,
        TendonGuard,
        clamp_motor_abs,
        motor_abs_after_tendon_action,
        motor_target_from_servo_feedback,
        motor_tendon_shorten_abs_sign,
    )


__all__ = [
    "JOINT_TO_PRIMARY_MOTOR_INDEX",
    "MOTOR_TENDON_SHORTEN_ABS_SIGN",
    "MOTOR_TO_JOINT_INDEX",
    "TendonGuard",
    "clamp_motor_abs",
    "motor_abs_after_tendon_action",
    "motor_target_from_servo_feedback",
    "motor_tendon_shorten_abs_sign",
]
