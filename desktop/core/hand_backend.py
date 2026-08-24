"""hand_backend.py - MMHand Desktop 模块契约。

职责：向 Qt、ROS2 和 Replay 提供统一的设备能力门面。
线程：除 comm_layer 的串口收发线程外不主动创建 GUI 线程；共享状态必须按既有锁读取。
输入：P4 字节流、上层类型化命令、配置和持久化数据。
输出：线程安全的 HandModel 快照、协议帧、ACK 和后端操作结果。
状态所有权：通信对象拥有串口与解析缓存；HandModel 的写入只允许在核心同步边界内发生。
禁止依赖：禁止依赖 Qt/UI、app、ROS2 或 Replay 页面，禁止把控件和展示规则带入核心层。
"""

from dataclasses import dataclass, field
import math
import threading
import time
from typing import Callable, Dict, Iterable, List, Optional, Tuple

from .angle_mapping import get_spec, urdf_manual_scaled_mapping_metadata
from .comm_layer import TactileSerialComm, list_ports as _list_ports
from .core_logic import HandController, ManualPrepareResult
from .data_models import HandModel, resolve_joint_workspace_range
from .protocol import ControlMode, ENCODER_COUNT
from .session_log import log_session


StateCallback = Callable[[HandModel], None]
JointTargetCallback = Callable[[dict], None]


def list_ports() -> List[tuple]:
    return _list_ports()


def clone_hand_model(model: HandModel) -> HandModel:
    snapshot = HandModel()
    snapshot.copy_state_from(model)
    snapshot.calib_status = str(getattr(model, "calib_status", "IDLE"))
    snapshot.tactile_data = list(getattr(model, "tactile_data", []) or [])
    snapshot.tactile_source = str(getattr(model, "tactile_source", "NONE"))
    snapshot.tactile_data_level = str(getattr(model, "tactile_data_level", "NONE"))
    snapshot.tactile_module_valid_mask = int(getattr(model, "tactile_module_valid_mask", 0) or 0)
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
    state.tactile_source = str(getattr(tactile_state, "tactile_source", "S3_SERIAL"))
    state.tactile_data_level = str(getattr(tactile_state, "tactile_data_level", "NONE"))
    state.tactile_module_valid_mask = int(
        getattr(tactile_state, "tactile_module_valid_mask", 0) or 0
    )
    state.tactile_online = bool(getattr(tactile_state, "tactile_online", False))
    state.tactile_last_rx_ts = float(getattr(tactile_state, "tactile_last_rx_ts", 0.0) or 0.0)
    state.tactile_module_online = list(getattr(tactile_state, "tactile_module_online", []) or [])
    state.tactile_module_last_rx_ts = list(getattr(tactile_state, "tactile_module_last_rx_ts", []) or [])
    ts = float(getattr(tactile_state, "host_rx_time", 0.0) or 0.0)
    if ts > 0:
        state.host_rx_time = ts


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
    and the command surface that UI/ROS clients use.
    """

    def __init__(
        self,
        controller: Optional[HandController] = None,
    ) -> None:
        self._controller = controller or HandController()
        self.tactile_port = ""
        self.tactile_comm: Optional[TactileSerialComm] = None
        self._tactile_state: Optional[HandModel] = None
        self._next_tactile_reconnect_ts = 0.0
        self._state_callbacks: List[StateCallback] = []
        self._callback_lock = threading.Lock()
        self._joint_target_callbacks: List[JointTargetCallback] = []
        # ROS2 回调运行在独立线程。授权检查、角度入队与 E-STOP 必须共用
        # 同一把锁，保证 STOP 入队后不会再插入新的 joint_target_run。
        self._ros2_output_lock = threading.RLock()
        self._ros2_output_armed = False
        self._ros2_armed_scope: Optional[Tuple[int, int, bool]] = None
        self._ros2_last_drop_reason = ""
        self._ros2_last_drop_log_ts = 0.0
        # generation令牌把Replay检查与每帧入队绑定，其他运动源可立即使旧令牌失效。
        self._replay_motion_lock = threading.RLock()
        self._replay_motion_generation = 0
        self._replay_motion_active = False
        self._requested_active_joint_mask: Optional[int] = None
        self._requested_active_scope: Optional[Tuple[int, int, bool]] = None
        try:
            self._workspace_mapping_version, self._workspace_urdf_crc = (
                urdf_manual_scaled_mapping_metadata()
            )
        except (OSError, TypeError, ValueError):
            self._workspace_mapping_version, self._workspace_urdf_crc = 0, 0
        self._joint_target_callback_lock = threading.Lock()
        self._ensure_controller_callback_registered()

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
        self.cancel_replay_motion("connect")
        self.set_ros2_output_armed(False, reason="connect")
        self._ensure_controller_callback_registered()
        self._requested_active_joint_mask = None
        self._requested_active_scope = None
        return bool(self._controller.initialize(str(port)))

    def initialize(self, port: str) -> bool:
        return self.connect(port)

    def shutdown(self, stop_lower: bool = False) -> None:
        """关闭上位机链路；默认交由 P4 租约 Hold，显式急停才下发 STOP。"""
        self.cancel_replay_motion("shutdown")
        with self._ros2_output_lock:
            self._set_ros2_output_armed_locked(False, reason="shutdown")
            self._requested_active_joint_mask = None
            self._requested_active_scope = None
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
        if latest is not None:
            self._tactile_state = latest
        tactile_state = self._tactile_state
        if tactile_state is None:
            return False, state
        if not bool(getattr(tactile_state, "tactile_online", False)):
            # S3直连只在数据新鲜时覆盖P4摘要；串口超时后立即回退到P4状态，
            # 避免离线的S3空快照持续遮住仍然在线的CAN中继。
            return latest is not None, self.get_state_snapshot()
        target = state or HandModel()
        # 即使本轮没有新S3包，也要把最近的新鲜S3全量快照覆盖到新到的P4状态上。
        merge_tactile_state(target, tactile_state)
        return latest is not None, target

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
        command_joint_mask: Optional[int] = None,
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
        if command_joint_mask is None:
            guard_mask = active_mask
        else:
            requested_mask = int(command_joint_mask) & ((1 << ENCODER_COUNT) - 1)
            outside_mask = requested_mask & ~active_mask
            if outside_mask:
                blocked = [
                    joint
                    for joint in range(ENCODER_COUNT)
                    if outside_mask & (1 << joint)
                ]
                return (
                    [],
                    blocked,
                    [],
                    "Joint command blocked: command mask exceeds active joint scope",
                )
            guard_mask = requested_mask & active_mask
            if guard_mask == 0:
                return (
                    [],
                    list(range(ENCODER_COUNT)),
                    [],
                    "Joint command blocked: command joint mask is empty",
                )

        blocked: List[int] = []
        clamped: List[int] = []
        reasons: List[str] = []
        for joint_idx in range(ENCODER_COUNT):
            # 单行UI、Replay和ROS2均只校验本帧真正控制的关节，未参与关节不应拖垮整包。
            if (guard_mask & (1 << joint_idx)) == 0:
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
            message = f"Joint targets clamped to [URDF Min, URDF Max]: {joints}{suffix}"
        return guarded, blocked, clamped, message

    def send_joint_targets(
        self,
        angles: Iterable[float],
        *,
        source: str = "",
        live: bool = False,
        input_space: str = "urdf_deg",
        raw_targets: Optional[Iterable[float]] = None,
        command_joint_mask: Optional[int] = None,
        motion_token: Optional[int] = None,
    ) -> JointCommandResult:
        """串行化Replay令牌检查与实际入队，杜绝STOP之后补入一帧旧目标。"""
        with self._replay_motion_lock:
            return self._send_joint_targets_locked(
                angles,
                source=source,
                live=live,
                input_space=input_space,
                raw_targets=raw_targets,
                command_joint_mask=command_joint_mask,
                motion_token=motion_token,
            )

    def _send_joint_targets_locked(
        self,
        angles: Iterable[float],
        *,
        source: str = "",
        live: bool = False,
        input_space: str = "urdf_deg",
        raw_targets: Optional[Iterable[float]] = None,
        command_joint_mask: Optional[int] = None,
        motion_token: Optional[int] = None,
    ) -> JointCommandResult:
        from desktop.core.joint_target_adapter import active_mapping_identity

        state = self.get_current_state()
        input_targets = list(angles)
        raw_input_targets = list(raw_targets) if raw_targets is not None else list(input_targets)
        is_replay = str(source or "").startswith("Replay:")
        if is_replay:
            if motion_token is None or not self.replay_motion_token_valid(motion_token):
                return JointCommandResult(
                    sent=False,
                    message="Replay command rejected: motion ownership expired",
                    targets=list(input_targets),
                )
        else:
            # 普通UI、ROS2或测试命令一旦到达，就撤销旧Replay令牌。
            # Replay线程会在下一控制周期退出，避免两个运动源同时写P4目标角。
            self.cancel_replay_motion(f"preempted by {source or 'joint command'}")
        mapping = active_mapping_identity()
        command_metadata = {
            "input_space": str(input_space or "urdf_deg"),
            "output_space": "urdf_deg",
            "raw_input_targets": raw_input_targets,
            "mapping_id": mapping.mapping_id,
            "mapping_version": mapping.mapping_version,
            "urdf_config_crc": mapping.urdf_config_crc,
            "joint_order_crc": mapping.joint_order_crc,
            "command_joint_mask": (
                None if command_joint_mask is None else int(command_joint_mask)
            ),
        }
        guarded, workspace_blocked, clamped, workspace_message = self._apply_joint_workspace_guard(
            state,
            input_targets,
            source=source,
            command_joint_mask=command_joint_mask,
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
                    "p4_time_ms": (
                        getattr(state, "p4_time_ms", "")
                        if getattr(state, "has_p4_time", False)
                        else ""
                    ),
                    **command_metadata,
                }
            )
            return result

        start_feedback = bool(source and not str(source).startswith("Hold"))
        if start_feedback:
            sent = self._controller.send_target_angles_and_run_feedback(
                guarded,
                live=live,
                joint_mask=command_joint_mask,
            )
        elif live:
            sent = self._controller.set_target_angles_live(
                guarded,
                joint_mask=command_joint_mask,
            )
        else:
            sent = self._controller.set_target_angles(guarded)
        message = workspace_message
        if not sent and not message:
            message = "Joint command was not queued; check connection, mode, and pause state"
        result = JointCommandResult(
            sent=bool(sent),
            blocked=[],
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
                "blocked": [],
                "clamped": list(clamped),
                "message": message,
                "sent": bool(result.sent),
                "p4_time_ms": (
                    getattr(state, "p4_time_ms", "")
                    if getattr(state, "has_p4_time", False)
                    else ""
                ),
                **command_metadata,
            }
        )
        return result

    @staticmethod
    def _state_active_scope(state: Optional[HandModel]) -> Tuple[int, int, bool]:
        """读取 P4 已确认的 active scope，供 ROS2 授权绑定与逐帧核验。"""
        return (
            int(getattr(state, "active_motor_mask", 0) or 0),
            int(getattr(state, "active_joint_mask", 0) or 0),
            bool(getattr(state, "require_can_feedback", False)),
        )

    def _set_ros2_output_armed_locked(self, armed: bool, *, reason: str) -> Tuple[bool, str]:
        """在已持有 ROS2 锁时更新授权；不得直接从锁外调用。"""
        if not armed:
            was_armed = self._ros2_output_armed
            self._ros2_output_armed = False
            self._ros2_armed_scope = None
            if was_armed:
                log_session("ROS2_GATE", f"DISARM reason={reason or 'requested'}")
            return True, "ROS2 joint output disabled"

        if not self.is_comm_connected():
            return False, "ROS2 enable blocked: P4 serial is not connected"

        state = self.get_current_state()
        scope = self._requested_active_scope or self._state_active_scope(state)
        if int(scope[1]) == 0:
            return False, "ROS2 enable blocked: active joint scope is empty"

        self._ros2_output_armed = True
        self._ros2_armed_scope = scope
        self._ros2_last_drop_reason = ""
        log_session(
            "ROS2_GATE",
            "ARM reason={} motor=0x{:08X} joint=0x{:08X} require_can={}".format(
                reason or "requested",
                int(scope[0]),
                int(scope[1]),
                int(bool(scope[2])),
            ),
        )
        return True, "ROS2 input enabled; waiting for a valid P4 state"

    def set_ros2_output_armed(self, armed: bool, *, reason: str = "") -> Tuple[bool, str]:
        """授权或撤销两条 ROS2 21 关节角输入链路。

        该授权只由独立 ROS2 INPUT 或 CLI 的独立按键建立，普通控制 ENABLE
        不得隐式放行外部目标。P4 在线、READY 和作用域一致性仍由每一帧发送前
        核验；E-STOP、Pause、断连和作用域切换会主动撤销授权。
        """
        with self._ros2_output_lock:
            return self._set_ros2_output_armed_locked(bool(armed), reason=reason)

    def ros2_output_armed(self) -> bool:
        with self._ros2_output_lock:
            return bool(self._ros2_output_armed)

    def set_ros2_output_enabled(self, enabled: bool) -> Tuple[bool, str]:
        """响应独立 ROS2 INPUT 开关，并复用线程安全的 ROS2 输出授权门。"""
        # UI 灰显不是安全门。CLI 或其他调用方也必须先启动普通控制器，
        # 防止提前预授权后某次普通 START 意外放行持续到来的遥操帧。
        if enabled and not self.is_started():
            return False, "ordinary controller is not enabled; press ENABLE first"
        if enabled:
            self.cancel_replay_motion("ROS2 input enabled")
        ok, message = self.set_ros2_output_armed(
            bool(enabled),
            reason="ROS2_OUTPUT_ENABLE" if enabled else "ROS2_OUTPUT_DISABLE",
        )
        if enabled and ok:
            return True, "ROS2 input enabled; waiting for the next ROS2 frame"
        if not enabled and ok:
            return True, "ROS2 input disabled"
        return ok, message

    def ros2_output_enabled(self) -> bool:
        """返回当前 ROS2 用户授权；运行门禁仍在每一帧发送前复核。"""
        return self.ros2_output_armed()

    def _log_ros2_drop_locked(self, reason: str) -> None:
        """同类 ROS2 丢帧日志每秒最多记录一次，避免 100 Hz 输入刷屏。"""
        now = time.monotonic()
        if reason == self._ros2_last_drop_reason and now - self._ros2_last_drop_log_ts < 1.0:
            return
        self._ros2_last_drop_reason = reason
        self._ros2_last_drop_log_ts = now
        log_session("ROS2_GATE", f"DROP reason={reason}")

    def send_ros2_joint_targets(
        self,
        angles: Iterable[float],
        *,
        source: str,
        input_space: str = "urdf_deg",
        raw_targets: Optional[Iterable[float]] = None,
    ) -> JointCommandResult:
        """在安全授权门后发送一帧 ROS2 关节角。

        该锁覆盖“检查授权 -> 发送角度”整个事务；E-STOP 使用同一把锁执行
        “撤权 -> STOP”，从线程时序上杜绝 STOP 后被下一帧重新启动。
        """
        values = list(angles)
        raw_values = list(raw_targets) if raw_targets is not None else list(values)
        with self._ros2_output_lock:
            if not self._ros2_output_armed:
                reason = "not_armed"
            elif not self.is_comm_connected():
                reason = "serial_offline"
            elif not self.is_started():
                reason = "controller_not_ready"
            else:
                state_scope = self._state_active_scope(self.get_current_state())
                if self._ros2_armed_scope is None or state_scope != self._ros2_armed_scope:
                    reason = "active_scope_mismatch"
                else:
                    # 把 ENABLE 时绑定的 scope 显式写入每一帧命令。即使串口队列
                    # 后续恰逢 UI 切换 scope，旧遥操帧也不能作用到新选中的关节。
                    ros2_joint_mask = int(self._ros2_armed_scope[1])
                    self._ros2_last_drop_reason = ""
                    self.set_control_mode(ControlMode.JOINT_ANGLE)
                    return self.send_joint_targets(
                        values,
                        source=source,
                        live=True,
                        input_space=input_space,
                        raw_targets=raw_values,
                        command_joint_mask=ros2_joint_mask,
                    )

            self._log_ros2_drop_locked(reason)
            return JointCommandResult(
                sent=False,
                message=f"ROS2 joint target dropped: {reason}",
                targets=values,
            )

    def begin_replay_motion(self) -> Tuple[bool, int, str]:
        """取得 Replay 独占令牌，并明确撤销 ROS2 输出授权。"""
        if not self.is_comm_connected():
            return False, 0, "Replay拒绝：P4串口未连接"
        if not self.is_started():
            return False, 0, "Replay拒绝：请先按ENABLE"
        with self._ros2_output_lock:
            self._set_ros2_output_armed_locked(False, reason="replay_start")
        with self._replay_motion_lock:
            self._replay_motion_generation += 1
            self._replay_motion_active = True
            token = self._replay_motion_generation
        log_session("REPLAY_OWNER", f"CLAIM token={token}")
        return True, token, "Replay motion ownership granted"

    def replay_motion_token_valid(self, token: int) -> bool:
        with self._replay_motion_lock:
            return bool(
                self._replay_motion_active
                and int(token) == int(self._replay_motion_generation)
            )

    def cancel_replay_motion(self, reason: str = "cancelled") -> bool:
        """撤销Replay令牌；工作线程无需跨线程强杀即可安全退出。"""
        with self._replay_motion_lock:
            if not self._replay_motion_active:
                return False
            self._replay_motion_active = False
            self._replay_motion_generation += 1
            generation = self._replay_motion_generation
        log_session(
            "REPLAY_OWNER",
            f"CANCEL generation={generation} reason={reason or 'cancelled'}",
        )
        return True

    def end_replay_motion(self, token: int) -> None:
        with self._replay_motion_lock:
            if (
                self._replay_motion_active
                and int(token) == int(self._replay_motion_generation)
            ):
                self._replay_motion_active = False
                log_session("REPLAY_OWNER", f"RELEASE token={token}")

    def protocol_diag_items(self) -> List[Tuple[str, object]]:
        comm = self.comm
        if comm is None:
            return []
        fields = [
            "_rx_diag_total_packets",
            "_rx_diag_sensor_packets",
            "_rx_diag_bytes",
            "_tx_diag_total_cmds",
            "_tx_diag_angle_live_cmds",
            "_tx_diag_bytes",
        ]
        return [(name, getattr(comm, name, "-")) for name in fields]

    def request_state_sync(self, sync_mask: int) -> bool:
        """透传静态状态显式同步请求，避免 UI 依赖固件周期广播。"""
        return self._controller.request_state_sync(sync_mask)

    def set_task_diag_subscription(self, enabled: bool) -> bool:
        """根据 Task Runtime 窗口生命周期控制诊断带宽。"""
        return self._controller.set_task_diag_subscription(enabled)

    def set_control_trace_subscription(self, enabled: bool) -> bool:
        """根据 Save Data 生命周期控制统一时间戳控制跟踪流。"""
        return self._controller.set_control_trace_subscription(enabled)

    # Controller command proxies.  These keep UI/ROS on the backend surface while
    # HandController remains the hardware-control primitive layer.
    def hold_current_motor_positions(self, delay_s: float = 0.15) -> bool:
        return self._controller.hold_current_motor_positions(delay_s=delay_s)

    def set_control_mode(self, mode: ControlMode) -> None:
        self._controller.set_control_mode(mode)

    def get_control_mode(self) -> ControlMode:
        return self._controller.get_control_mode()

    def invalidate_joint_commands(self) -> int:
        return self._controller.invalidate_joint_commands()

    def pause(self) -> None:
        self.cancel_replay_motion("pause")
        with self._ros2_output_lock:
            self._set_ros2_output_armed_locked(False, reason="pause")
            self._controller.pause()

    def resume(self) -> None:
        self._controller.resume()

    def set_target_angles(self, angles: List[float]) -> JointCommandResult:
        return self.send_joint_targets(angles, source="Backend set_target_angles", live=False)

    def set_target_angles_live(self, angles: List[float]) -> JointCommandResult:
        return self.send_joint_targets(angles, source="Backend set_target_angles_live", live=True)

    def set_motor_positions_absolute(self, motor_abs: List[int]) -> None:
        self._controller.set_motor_positions_absolute(motor_abs)

    def set_single_motor_position_absolute(self, motor_index: int, motor_abs: int) -> None:
        self._controller.set_single_motor_position_absolute(motor_index, motor_abs)

    def request_firmware_calibration(self) -> bool:
        return self._controller.request_firmware_calibration()

    def request_run_feedback(self) -> bool:
        return self._controller.request_run_feedback()

    def set_active_scope(self, active_motor_mask: int, active_joint_mask: int, require_can_feedback: bool) -> bool:
        self.cancel_replay_motion("active_scope_change")
        with self._ros2_output_lock:
            self._set_ros2_output_armed_locked(False, reason="active_scope_change")
            ok = self._controller.set_active_scope(active_motor_mask, active_joint_mask, require_can_feedback)
            if ok:
                valid_joint_mask = int(active_joint_mask) & ((1 << ENCODER_COUNT) - 1)
                self._requested_active_joint_mask = valid_joint_mask
                self._requested_active_scope = (
                    int(active_motor_mask),
                    valid_joint_mask,
                    bool(require_can_feedback),
                )
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

    def prepare_calibration_turn_zero(
        self, motor_mask: int
    ) -> Tuple[bool, str, int, int]:
        """标定前归零所选舵机，并返回成功与失败电机掩码。"""
        return self._controller.prepare_calibration_turn_zero(motor_mask)

    def manual_prepare_all_motors(self) -> ManualPrepareResult:
        """停止外部连续输入后执行全手 turn0 与 1500-count 预松事务。"""
        self.cancel_replay_motion("manual_prepare")
        with self._ros2_output_lock:
            self._set_ros2_output_armed_locked(False, reason="manual_prepare")
            return self._controller.manual_prepare_all_motors()

    def restore_manual_references_from_snapshot(
        self, snapshot: dict
    ) -> Tuple[bool, str]:
        """从便携标定 JSON 恢复 P4 Manual RAM，并等待逐项读回核验。"""
        return self._controller.restore_manual_references_from_snapshot(snapshot)

    def load_calibration_snapshot(self, snapshot: dict) -> Tuple[bool, str]:
        return self._controller.load_calibration_snapshot(snapshot)

    def start(self) -> None:
        self._controller.start()

    def stop(self) -> None:
        self.cancel_replay_motion("E_STOP")
        with self._ros2_output_lock:
            self._set_ros2_output_armed_locked(False, reason="E_STOP")
            self._controller.stop()

    def reset(self) -> None:
        self.cancel_replay_motion("reset")
        with self._ros2_output_lock:
            self._set_ros2_output_armed_locked(False, reason="reset")
            self._controller.reset()

    def is_started(self) -> bool:
        return self._controller.is_started()
