"""Application-level backend facade for Qt, ROS, teleop, and algorithms.

HandBackend owns the low-level HandController and exposes a stable, UI-free
application API.  Qt widgets should depend on this facade instead of reaching
into HandController or LowerComputerComm directly.
"""

from dataclasses import dataclass, field
import math
import threading
import time
from typing import Callable, Iterable, List, Optional, Tuple

from desktop.paths import TENDON_MANUAL_CALIB_PATH

from .angle_mapping import get_spec, urdf_manual_scaled_mapping_metadata
from .comm_layer import TactileSerialComm, list_ports as _list_ports
from .core_logic import HandController
from .data_models import HandModel, resolve_joint_workspace_range
from .protocol import ControlMode, ENCODER_COUNT
from .tendon_guard import TendonGuard


StateCallback = Callable[[HandModel], None]
JointTargetCallback = Callable[[dict], None]


def list_ports() -> List[tuple]:
    return _list_ports()


def clone_hand_model(model: HandModel) -> HandModel:
    snapshot = HandModel()
    snapshot.copy_state_from(model)
    snapshot.calib_status = str(getattr(model, "calib_status", "IDLE"))
    snapshot.timestamp = float(getattr(model, "timestamp", 0.0))
    snapshot.tactile_data = list(getattr(model, "tactile_data", []) or [])
    snapshot.tactile_online = bool(getattr(model, "tactile_online", False))
    snapshot.tactile_last_rx_ts = float(getattr(model, "tactile_last_rx_ts", 0.0) or 0.0)
    snapshot.tactile_module_online = list(getattr(model, "tactile_module_online", []) or [])
    snapshot.tactile_module_last_rx_ts = list(getattr(model, "tactile_module_last_rx_ts", []) or [])
    try:
        snapshot.set_target_angles(model.target_angles)
    except Exception:
        pass
    for i, enc in enumerate(snapshot.encoders):
        if i < len(getattr(model, "encoders", [])):
            enc.zero_raw = int(model.encoders[i].zero_raw)
    return snapshot


def merge_tactile_state(state: HandModel, tactile_state: HandModel) -> None:
    state.tactile_data = list(getattr(tactile_state, "tactile_data", []) or [])
    state.tactile_summary_seq = getattr(tactile_state, "tactile_summary_seq", None)
    state.tactile_full_seq = getattr(tactile_state, "tactile_full_seq", None)
    state.tactile_full_summary_seq = getattr(tactile_state, "tactile_full_summary_seq", None)
    state.tactile_full_crc = getattr(tactile_state, "tactile_full_crc", None)
    state.tactile_full_crc_errors = int(getattr(tactile_state, "tactile_full_crc_errors", 0))
    state.tactile_full_seq_mismatches = int(getattr(tactile_state, "tactile_full_seq_mismatches", 0))
    state.tactile_full_dropped = int(getattr(tactile_state, "tactile_full_dropped", 0))
    state.tactile_online = bool(getattr(tactile_state, "tactile_online", False))
    state.tactile_last_rx_ts = float(getattr(tactile_state, "tactile_last_rx_ts", 0.0) or 0.0)
    state.tactile_module_online = list(getattr(tactile_state, "tactile_module_online", []) or [])
    state.tactile_module_last_rx_ts = list(getattr(tactile_state, "tactile_module_last_rx_ts", []) or [])
    ts = float(getattr(tactile_state, "timestamp", 0.0) or 0.0)
    if ts > 0:
        state.timestamp = ts


@dataclass
class JointCommandResult:
    sent: bool
    blocked: List[int] = field(default_factory=list)
    clamped: List[int] = field(default_factory=list)
    message: str = ""
    targets: List[float] = field(default_factory=list)


class HandBackend:
    """UI-free orchestration layer above HandController.

    The low-level controller still owns protocol-level behavior.  This class
    owns application-level lifecycle, state fan-out, tactile serial management,
    tendon guard orchestration, and the command surface that UI/ROS clients use.
    """

    def __init__(
        self,
        controller: Optional[HandController] = None,
        *,
        tendon_guard: Optional[TendonGuard] = None,
        tendon_guard_config_path: Optional[str] = None,
    ) -> None:
        self._controller = controller or HandController()
        guard_path = tendon_guard_config_path or str(TENDON_MANUAL_CALIB_PATH)
        self.tendon_guard = tendon_guard or TendonGuard(guard_path)
        self.tactile_port = ""
        self.tactile_comm: Optional[TactileSerialComm] = None
        self._tactile_state: Optional[HandModel] = None
        self._next_tactile_reconnect_ts = 0.0
        self._state_callbacks: List[StateCallback] = []
        self._callback_lock = threading.Lock()
        self._joint_target_callbacks: List[JointTargetCallback] = []
        self._requested_active_joint_mask: Optional[int] = None
        try:
            self._workspace_mapping_version, self._workspace_urdf_crc = (
                urdf_manual_scaled_mapping_metadata()
            )
        except (OSError, TypeError, ValueError):
            self._workspace_mapping_version, self._workspace_urdf_crc = 0, 0
        self._joint_target_callback_lock = threading.Lock()
        self._ensure_controller_callback_registered()

    @property
    def raw_controller(self) -> HandController:
        return self._controller

    @property
    def comm(self):
        return getattr(self._controller, "comm", None)

    @property
    def current_state(self) -> HandModel:
        return self._controller.current_state

    @property
    def state_lock(self):
        return self._controller.state_lock

    @property
    def last_sync_status(self) -> str:
        return str(getattr(self._controller, "last_sync_status", ""))

    def _ensure_controller_callback_registered(self) -> None:
        callbacks = getattr(self._controller, "update_callbacks", None)
        if callbacks is None:
            return
        if self._on_controller_state_update not in callbacks:
            self._controller.register_update_callback(self._on_controller_state_update)

    def _on_controller_state_update(self, state: HandModel) -> None:
        snapshot = clone_hand_model(state)
        with self._callback_lock:
            callbacks = list(self._state_callbacks)
        for callback in callbacks:
            try:
                callback(snapshot)
            except Exception:
                pass

    def register_update_callback(self, callback: StateCallback) -> None:
        with self._callback_lock:
            if callback not in self._state_callbacks:
                self._state_callbacks.append(callback)
        self._ensure_controller_callback_registered()

    def unregister_update_callback(self, callback: StateCallback) -> None:
        with self._callback_lock:
            self._state_callbacks = [cb for cb in self._state_callbacks if cb != callback]

    def register_joint_target_callback(self, callback: JointTargetCallback) -> None:
        with self._joint_target_callback_lock:
            if callback not in self._joint_target_callbacks:
                self._joint_target_callbacks.append(callback)

    def unregister_joint_target_callback(self, callback: JointTargetCallback) -> None:
        with self._joint_target_callback_lock:
            self._joint_target_callbacks = [cb for cb in self._joint_target_callbacks if cb != callback]

    def _notify_joint_target_callbacks(self, payload: dict) -> None:
        with self._joint_target_callback_lock:
            callbacks = list(self._joint_target_callbacks)
        for callback in callbacks:
            try:
                callback(dict(payload))
            except Exception:
                pass

    def get_state_snapshot(self) -> HandModel:
        with self._controller.state_lock:
            return clone_hand_model(self._controller.current_state)

    def get_current_state(self) -> HandModel:
        return self._controller.current_state

    def connect(self, port: str) -> bool:
        self._ensure_controller_callback_registered()
        self._requested_active_joint_mask = None
        return bool(self._controller.initialize(str(port)))

    def initialize(self, port: str) -> bool:
        return self.connect(port)

    def shutdown(self, stop_lower: bool = True) -> None:
        self._requested_active_joint_mask = None
        self._controller.shutdown(stop_lower=stop_lower)

    def disconnect(self, stop_lower: bool = False) -> None:
        self.shutdown(stop_lower=stop_lower)

    def is_comm_connected(self) -> bool:
        comm = self.comm
        serial_obj = getattr(comm, "serial", None) if comm is not None else None
        return bool(comm and serial_obj and getattr(serial_obj, "is_open", False))

    def connect_tactile_serial(self, port: str) -> bool:
        self.disconnect_tactile_serial()
        if not port:
            return False
        comm = TactileSerialComm(str(port))
        if not comm.connect():
            return False
        self.tactile_comm = comm
        self.tactile_port = str(port)
        self._tactile_state = None
        self._next_tactile_reconnect_ts = 0.0
        return True

    def disconnect_tactile_serial(self) -> None:
        if self.tactile_comm is not None:
            try:
                self.tactile_comm.disconnect()
            except Exception:
                pass
        self.tactile_comm = None
        self._tactile_state = None
        self._next_tactile_reconnect_ts = 0.0

    def is_tactile_serial_connected(self) -> bool:
        comm = self.tactile_comm
        if comm is None:
            return False
        checker = getattr(comm, "is_connected", None)
        if callable(checker):
            return bool(checker())
        serial_obj = getattr(comm, "serial", None)
        return bool(serial_obj is not None and getattr(serial_obj, "is_open", False))

    def get_tactile_serial_port(self) -> str:
        return self.tactile_port or ""

    def consume_tactile_state_into(self, state: Optional[HandModel]) -> Tuple[bool, Optional[HandModel]]:
        comm = self.tactile_comm
        if comm is None:
            return False, state
        if not self.is_tactile_serial_connected():
            now = time.time()
            if self.tactile_port and now >= self._next_tactile_reconnect_ts:
                self._next_tactile_reconnect_ts = now + 1.0
                try:
                    comm.connect()
                except Exception:
                    pass
        latest = comm.get_latest_model()
        if latest is None:
            return False, state
        self._tactile_state = latest
        target = state or HandModel()
        merge_tactile_state(target, latest)
        return True, target

    def _joint_workspace_active_mask(self, state: Optional[HandModel]) -> int:
        valid_mask = (1 << ENCODER_COUNT) - 1
        if self._requested_active_joint_mask is not None:
            return int(self._requested_active_joint_mask) & valid_mask
        return int(getattr(state, "active_joint_mask", 0) or 0) & valid_mask

    def _apply_joint_workspace_guard(
        self,
        state: Optional[HandModel],
        targets: Iterable[float],
        *,
        source: str,
    ) -> Tuple[List[float], List[int], List[int], str]:
        values = list(targets)
        if len(values) != ENCODER_COUNT:
            blocked = list(range(ENCODER_COUNT))
            return [], blocked, [], f"Joint command blocked: expected {ENCODER_COUNT} targets"
        try:
            guarded = [float(value) for value in values]
        except (TypeError, ValueError):
            blocked = list(range(ENCODER_COUNT))
            return [], blocked, [], "Joint command blocked: targets must be numeric"
        non_finite = [idx for idx, value in enumerate(guarded) if not math.isfinite(value)]
        if non_finite:
            return [], non_finite, [], "Joint command blocked: targets must be finite"

        active_mask = self._joint_workspace_active_mask(state)
        if active_mask == 0:
            blocked = list(range(ENCODER_COUNT))
            return [], blocked, [], "Joint command blocked: active joint scope is empty"

        blocked: List[int] = []
        clamped: List[int] = []
        reasons: List[str] = []
        for joint_idx in range(ENCODER_COUNT):
            if (active_mask & (1 << joint_idx)) == 0:
                continue
            spec = get_spec(joint_idx)
            workspace = resolve_joint_workspace_range(
                state,
                joint_idx,
                spec.urdf_range_deg[0],
                spec.urdf_range_deg[1],
                self._workspace_mapping_version,
                self._workspace_urdf_crc,
            )
            if not workspace.ready:
                blocked.append(joint_idx)
                reasons.append(f"J{joint_idx:02d} {workspace.reason}")
                continue
            bounded = workspace.clamp(guarded[joint_idx])
            if abs(bounded - guarded[joint_idx]) > 1e-6:
                guarded[joint_idx] = bounded
                clamped.append(joint_idx)

        if blocked:
            detail = "; ".join(reasons[:4])
            if len(reasons) > 4:
                detail += f"; +{len(reasons) - 4} more"
            return [], blocked, clamped, f"Joint command blocked: {detail}"

        message = ""
        if clamped:
            joints = ", ".join(f"J{idx:02d}" for idx in clamped)
            suffix = f" ({source})" if source else ""
            message = f"Joint targets clamped to [Current Min, URDF Max]: {joints}{suffix}"
        return guarded, blocked, clamped, message

    def send_joint_targets(
        self,
        angles: Iterable[float],
        *,
        source: str = "",
        live: bool = False,
        input_space: str = "urdf_deg",
        raw_targets: Optional[Iterable[float]] = None,
    ) -> JointCommandResult:
        from desktop.teleop.joint_target_adapter import active_mapping_identity

        state = self.get_current_state()
        input_targets = list(angles)
        raw_input_targets = list(raw_targets) if raw_targets is not None else list(input_targets)
        mapping = active_mapping_identity()
        command_metadata = {
            "input_space": str(input_space or "urdf_deg"),
            "output_space": "urdf_deg",
            "raw_input_targets": raw_input_targets,
            "mapping_id": mapping.mapping_id,
            "mapping_version": mapping.mapping_version,
            "urdf_config_crc": mapping.urdf_config_crc,
        }
        guarded, workspace_blocked, clamped, workspace_message = self._apply_joint_workspace_guard(
            state,
            input_targets,
            source=source,
        )
        if workspace_blocked:
            result = JointCommandResult(
                sent=False,
                blocked=workspace_blocked,
                clamped=clamped,
                message=workspace_message,
                targets=list(input_targets),
            )
            self._notify_joint_target_callbacks(
                {
                    "source": source,
                    "live": bool(live),
                    "input_targets": input_targets,
                    "output_targets": list(input_targets),
                    "blocked": list(workspace_blocked),
                    "clamped": list(clamped),
                    "message": workspace_message,
                    "sent": False,
                    "device_timestamp": getattr(state, "timestamp", ""),
                    **command_metadata,
                }
            )
            return result

        guarded, tendon_blocked, tendon_message = self.tendon_guard.apply(state, guarded, source=source)
        if live:
            self._controller.set_target_angles_live(guarded)
        else:
            self._controller.set_target_angles(guarded)
        if source and not tendon_blocked and not str(source).startswith("Hold"):
            self._controller.request_run_feedback()
        message = tendon_message or workspace_message
        result = JointCommandResult(
            sent=True,
            blocked=list(tendon_blocked),
            clamped=list(clamped),
            message=message,
            targets=guarded,
        )
        self._notify_joint_target_callbacks(
            {
                "source": source,
                "live": bool(live),
                "input_targets": input_targets,
                "output_targets": guarded,
                "blocked": list(tendon_blocked),
                "clamped": list(clamped),
                "message": message,
                "sent": bool(result.sent),
                "device_timestamp": getattr(state, "timestamp", ""),
                **command_metadata,
            }
        )
        return result

    def start_joint_debug_packet_log(self, path: str, row_callback: Optional[Callable[[dict], None]] = None) -> bool:
        starter = getattr(self.comm, "start_joint_debug_packet_log", None)
        if starter is None:
            return False
        try:
            starter(path, row_callback=row_callback)
        except TypeError:
            starter(path)
        return True

    def stop_joint_debug_packet_log(self) -> None:
        stopper = getattr(self.comm, "stop_joint_debug_packet_log", None)
        if stopper is not None:
            stopper()

    def protocol_diag_items(self) -> List[Tuple[str, object]]:
        comm = self.comm
        if comm is None:
            return []
        fields = [
            "_rx_diag_total_packets",
            "_rx_diag_sensor_packets",
            "_rx_diag_joint_debug_packets",
            "_rx_diag_bytes",
            "_tx_diag_total_cmds",
            "_tx_diag_angle_live_cmds",
            "_tx_diag_bytes",
            "_stream_mode_requested",
            "_stream_mode_applied",
            "_stream_mode_ack_pending",
            "_stream_mode_ack_warned",
        ]
        return [(name, getattr(comm, name, "-")) for name in fields]

    # Controller command proxies.  These keep UI/ROS on the backend surface while
    # HandController remains the hardware-control primitive layer.
    def hold_current_motor_positions(self, delay_s: float = 0.15) -> bool:
        return self._controller.hold_current_motor_positions(delay_s=delay_s)

    def set_control_mode(self, mode: ControlMode) -> None:
        self._controller.set_control_mode(mode)

    def get_control_mode(self) -> ControlMode:
        return self._controller.get_control_mode()

    def set_pid_control(self, enabled: bool) -> None:
        self._controller.set_pid_control(enabled)

    def invalidate_joint_commands(self) -> int:
        return self._controller.invalidate_joint_commands()

    def pause(self) -> None:
        self._controller.pause()

    def resume(self) -> None:
        self._controller.resume()

    def set_target_angles(self, angles: List[float]) -> JointCommandResult:
        return self.send_joint_targets(angles, source="Backend set_target_angles", live=False)

    def set_target_angles_live(self, angles: List[float]) -> JointCommandResult:
        return self.send_joint_targets(angles, source="Backend set_target_angles_live", live=True)

    def set_motor_positions_raw(self, motor_pos_raw: List[int]) -> None:
        self._controller.set_motor_positions_raw(motor_pos_raw)

    def set_motor_positions_raw_sweep(self, motor_pos_raw: List[int]) -> None:
        self._controller.set_motor_positions_raw_sweep(motor_pos_raw)

    def set_motor_positions_absolute(self, motor_abs: List[int]) -> None:
        self._controller.set_motor_positions_absolute(motor_abs)

    def set_single_motor_position_absolute(self, motor_index: int, motor_abs: int) -> None:
        self._controller.set_single_motor_position_absolute(motor_index, motor_abs)

    def set_local_test_params(self, freq_hz: float, amplitude_deg: float) -> None:
        self._controller.set_local_test_params(freq_hz, amplitude_deg)

    def start_local_test(self) -> None:
        self._controller.start_local_test()

    def stop_local_test(self) -> None:
        self._controller.stop_local_test()

    def request_firmware_calibration(self) -> bool:
        return self._controller.request_firmware_calibration()

    def request_run_feedback(self) -> bool:
        return self._controller.request_run_feedback()

    def set_active_scope(self, active_motor_mask: int, active_joint_mask: int, require_can_feedback: bool) -> bool:
        ok = self._controller.set_active_scope(active_motor_mask, active_joint_mask, require_can_feedback)
        if ok:
            self._requested_active_joint_mask = int(active_joint_mask) & ((1 << ENCODER_COUNT) - 1)
        return ok

    def set_manual_encoder_references(self, manual_min_raw, manual_max_raw) -> bool:
        return self._controller.set_manual_encoder_references(
            list(manual_min_raw), list(manual_max_raw)
        )

    def set_manual_encoder_reference(
        self, joint: int, manual_min_raw: int, manual_max_raw: int
    ) -> bool:
        return self._controller.set_manual_encoder_reference(
            joint, manual_min_raw, manual_max_raw
        )

    def query_manual_encoder_references(self) -> bool:
        return self._controller.query_manual_encoder_references()

    def clear_manual_encoder_references(self) -> bool:
        return self._controller.clear_manual_encoder_references()

    def rope_init(self, active_joint_mask: int) -> bool:
        return self._controller.rope_init(active_joint_mask)

    def set_motor_abs_limit_lock(self, active_motor_mask: int, locked: bool) -> bool:
        return self._controller.set_motor_abs_limit_lock(active_motor_mask, locked)

    def set_multiturn_nvs_enabled(self, enabled: bool) -> bool:
        return self._controller.set_multiturn_nvs_enabled(enabled)

    def reset_multiturn_baseline_for_new_device(self) -> Tuple[bool, str]:
        return self._controller.reset_multiturn_baseline_for_new_device()

    def load_calibration_snapshot(self, snapshot: dict) -> Tuple[bool, str]:
        return self._controller.load_calibration_snapshot(snapshot)

    def start(self) -> None:
        self._controller.start()

    def stop(self) -> None:
        self._controller.stop()

    def reset(self) -> None:
        self._controller.reset()

    def is_started(self) -> bool:
        return self._controller.is_started()
