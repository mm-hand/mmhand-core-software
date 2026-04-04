from typing import Callable, List, Optional

from hand_geometry import save_calib_zero_raw
from protocol import CALIB_LOAD_THRESHOLD, ENCODER_COUNT, ControlMode


def run_calibration(
    controller,
    progress_cb: Optional[Callable[[int, int, str], None]] = None,
    get_load: Optional[Callable[[int], float]] = None,
    control_mode: ControlMode = ControlMode.JOINT_ANGLE,
) -> Optional[List[int]]:
    import time

    if control_mode != ControlMode.JOINT_ANGLE:
        return None

    total = ENCODER_COUNT
    zero_raw: List[int] = []
    _get_load = get_load or (lambda _: 0.0)

    try:
        for joint_id in range(total):
            if progress_cb:
                progress_cb(joint_id, total, f"标定关节 {joint_id}")

            angles = [0.0] * ENCODER_COUNT
            step_deg = 2.0
            max_steps = 90
            found = False

            for step in range(max_steps):
                angles[joint_id] = step * step_deg
                controller.set_target_angles(angles)
                time.sleep(0.1)

                with controller.state_lock:
                    load = _get_load(joint_id)
                    enc = (
                        controller.current_state.encoders[joint_id].raw
                        if joint_id < len(controller.current_state.encoders)
                        else 0
                    )

                if load >= CALIB_LOAD_THRESHOLD:
                    zero_raw.append(enc)
                    found = True
                    break

            if not found:
                with controller.state_lock:
                    enc = (
                        controller.current_state.encoders[joint_id].raw
                        if joint_id < len(controller.current_state.encoders)
                        else 0
                    )
                zero_raw.append(enc)

        while len(zero_raw) < ENCODER_COUNT:
            zero_raw.append(0)

        zero_raw = zero_raw[:ENCODER_COUNT]
        if save_calib_zero_raw(zero_raw):
            if controller.comm:
                controller.comm.send_command(("calib_data", zero_raw))
            with controller.state_lock:
                controller.current_state.set_encoder_zeros(zero_raw)
                controller.current_state.calib_status = "SUCCESS"
            return zero_raw
    except Exception:
        return None
    finally:
        controller.stop()

    return None
