"""core_logic.py - MMHand Desktop 模块契约。

职责：封装面向业务的下位机命令序列和基础控制流程。
线程：串口、反馈发布与独立锚点落盘线程；坐标事务共用互斥锁，周期落盘不得阻塞反馈发布。
输入：P4 字节流、上层类型化命令、配置和持久化数据。
输出：线程安全的 HandModel 快照、协议帧、ACK 和后端操作结果。
状态所有权：通信对象拥有串口与解析缓存；HandModel 的写入只允许在核心同步边界内发生。
禁止依赖：禁止依赖 NiceGUI/UI、app、ROS2 或 Replay 页面，禁止把控件和展示规则带入核心层。
"""
# core_logic.py - 灵巧手控制器核心逻辑（关节角路数与协议 ENCODER_COUNT 一致，电机路数与 MOTOR_COUNT 一致）

import math
import json
import hashlib
import shutil
from functools import lru_cache
import queue
import threading
import time
from collections import namedtuple
from dataclasses import dataclass
from typing import List, NamedTuple, Optional, Tuple

from .angle_mapping import urdf_manual_scaled_mapping_metadata
from .calibration_snapshot import (
    CALIBRATION_SNAPSHOT_SCHEMA_VERSION,
    CalibrationSnapshotError,
    invalidate_calibration_snapshot_scope,
    joint_mask_for_motor_mask,
    motor_mask_for_joint_mask,
    atomic_write_snapshot,
    read_snapshot,
    update_manual_snapshot,
    validate_snapshot,
    validate_snapshot_for_load,
)
from .comm_layer import LowerComputerComm
from .multiturn_frame import shift_snapshot, turn_offsets
from .data_models import HandModel
from .device_state_store import (
    DeviceStateStoreError,
    BundleMultiTurnStore,
    canonical_product_fingerprint,
)
from .motor_mapping import MOTOR_TO_JOINT_INDEX, motor_abs_after_tendon_action
from desktop.paths import CALIB_ACTIVE_SNAPSHOT_PATH, CONFIG_DIR
from .protocol import (
    ServoOnlineCheck,
    DIRECT_SERVO_SPEED,
    DIRECT_SERVO_ACC,
    HOST_PROTOCOL_VERSION,
    CMD_ACTIVE_SCOPE,
    CMD_CLEAR_CALIBRATION_SCOPE,
    CMD_CALIB_LOAD_JOINT,
    CMD_CALIB_LOAD_LIMITS,
    CMD_MANUAL_ENCODER_REFERENCE,
    CMD_MANUAL_ENCODER_SINGLE,
    CMD_STATE_SYNC_REQUEST,
    CMD_ROPE_INIT,
    CMD_RESET,
    CMD_MOTOR_ABS_ZERO,
    ENCODER_COUNT,
    MOTOR_COUNT,
    PROTO_ACK_STATUS_OK,
    PROTO_ACK_STATUS_QUEUED,
    PROTO_ACK_STATUS_EMERGENCY_STOP_LATCHED,
    ControlMode,
    MOTOR_ABS_CMD_MAX,
    MOTOR_ABS_CMD_MIN,
    SYSTEM_STATE_IDLE,
    SYSTEM_STATE_CALIBRATION_RUNNING,
    SYSTEM_STATE_RECOVERY_REQUIRED,
    SYSTEM_STATE_RUN_FEEDBACK,
    SYSTEM_STATE_READY,
    SYSTEM_STATE_RUN_DIRECT,
    SERVO_TARGET_OWNER_CONTROL,

    SYSTEM_STATE_ROPE_INIT_RUNNING,
    SYSTEM_STATE_UNCONTROLLABLE,
    STATE_SYNC_ALL,
    STATE_SYNC_MANUAL_REFERENCE,
    STATE_SYNC_MULTITURN_STATE,
    manual_encoder_reference_crc,
    manual_encoder_reference_entry_crc,
)
from .session_log import log_session
from .user_issues import IssueMessage

LOWER_SYNC_TIMEOUT_S = 2.5
STATE_SYNC_RETRY_PERIOD_S = 1.0
LOWER_STATUS_STALE_AFTER_S = 1.0
ACTIVE_SCOPE_ACK_TIMEOUT_S = 1.0
CALIBRATE_ACK_TIMEOUT_S = 1.0
# P4 每个完整反馈轮次只归零并恢复一台舵机；27路最坏重试需要更长结果窗口。
CLEAR_CALIBRATION_ACK_TIMEOUT_S = 1.0
CALIB_LOAD_ACK_TIMEOUT_S = 1.0
MANUAL_REFERENCE_ACK_TIMEOUT_S = 2.0
SERVO_REBASE_ZERO_ACK_TIMEOUT_S = 1.5
SERVO_REBASE_ZERO_RESULT_TIMEOUT_S = 8.0
SERVO_COORDINATE_RESULT_SYNC_ATTEMPTS = 1
SERVO_COORDINATE_RESULT_SYNC_WAIT_S = 1.0
HOST_MULTITURN_MOVING_SAVE_PERIOD_S = 0.05
HOST_MULTITURN_IDLE_SAVE_PERIOD_S = 0.50
STARTUP_MULTITURN_RECOVERY_TIMEOUT_S = 20.0
MANUAL_PREPARE_RELEASE_COUNTS = 1500
MANUAL_PREPARE_FEEDBACK_TIMEOUT_S = 2.0
MANUAL_PREPARE_READY_TIMEOUT_S = 2.0
MANUAL_PREPARE_TRANSIENT_GRACE_S = 1.0
MANUAL_PREPARE_MOTION_TIMEOUT_S = 12.0
MANUAL_PREPARE_TARGET_TOLERANCE_COUNTS = 80


@lru_cache(maxsize=1)
def _active_urdf_repo_path() -> str:
    """读取生成来源中的仓库内路径；缺失时不得恢复运动锚点。"""
    try:
        source = json.loads((CONFIG_DIR / "robot_model.json").read_text(encoding="utf-8"))
        return str((source.get("source") or {}).get("source_urdf_path", ""))
    except (OSError, ValueError):
        return ""


_CoordinatePlan = namedtuple("_CoordinatePlan", [
    "label", "command_id", "has_result_field", "baseline_token_field", "success_field", "token_field",
    "matches", "record", "clear", "service_result", "invalidate_on_timeout",
    "success_message", "timeout_message", "rejected_message", "unsafe_state_message",
    "requires_sync", "precheck", "invalidation_reason",
])

_ABS_ZERO_PLAN = _CoordinatePlan(
    label="servo ABS rebase",
    command_id=CMD_MOTOR_ABS_ZERO,
    has_result_field="has_servo_rebase_zero_result",
    baseline_token_field="servo_rebase_zero_result_token",
    success_field="servo_rebase_zero_success_mask",
    token_field="servo_rebase_zero_result_token",
    matches=lambda s, token, sent_at, mask: HandController._servo_abs_zero_result_matches(s, token, sent_at, mask),
    record=lambda self, pending, token, sent_at, device_id, connection, comm: (
        setattr(self, "_servo_rebase_zero_unresolved_mask", pending),
        setattr(self, "_servo_rebase_zero_baseline_token", token),
        setattr(self, "_servo_rebase_zero_sent_at", sent_at),
        setattr(self, "_servo_rebase_zero_device_id", device_id),
        setattr(self, "_servo_rebase_zero_request_connection", connection),
    ),
    clear=lambda self, success: (
        setattr(self, "_servo_rebase_zero_unresolved_mask", 0),
        setattr(self, "_servo_rebase_zero_device_id", self._servo_rebase_zero_device_id if success else ""),
    ),
    service_result=lambda self, state: self._service_servo_rebase_zero_capture(state),
    invalidate_on_timeout=lambda self, pending, plan, state: self._invalidate_servo_rebase_zero_anchors(pending),
    success_message="servo ABS rebased; new feedback anchor capture pending",
    timeout_message="servo ABS rebase result timed out or cancelled; original transaction remains unresolved",
    rejected_message=lambda status: (
        IssueMessage("Cannot reset motor positions while Stop is active. Check the hand, click Clear Faults, then retry the original action.", "ACT-1005")
        if status == PROTO_ACK_STATUS_EMERGENCY_STOP_LATCHED else f"servo ABS rebase rejected (status={status}); hardware write not accepted"),
    unsafe_state_message="servo ABS zero requires fresh stopped status and online, unambiguous selected motors",
    requires_sync=True,
    precheck=None,
    invalidation_reason="servo_abs_rebase_zero",
)

class ManualPrepareResult(NamedTuple):
    """Device Check 全手预松事务结果；掩码用于精确失效本地缓存。"""

    ok: bool
    message: str
    reset_motor_mask: int
    failed_motor_mask: int
    invalidated_joint_mask: int
    reached_motor_mask: int


class ServoRebaseZeroResult(NamedTuple):
    """硬件 ABS 设零的最终结果，成功与失败由电机位图区分。"""

    ok: bool
    message: str
    result_token: int
    success_motor_mask: int
    failed_motor_mask: int


class HandController:
    """
    ===========================================================================
    HandController - 灵巧手控制器的核心业务逻辑层 (大脑)
    ===========================================================================
    定位：
    承上启下，向上对 UI 暴露控制接口（如设置目标角度、多圈直控绝对位置等），
    向下管理底层的物理通信 (`LowerComputerComm`)，并维护着最新的机器手系统状态 (`HandModel`)。

    主要职责：
    1. 生命周期管理：建立与断开串口通信，启动后台更新线程 `_update_loop`。
    2. 数据同步：通过回调函数 (`update_callbacks`) 将底层传来的状态实时推送给 UI 刷新。
    3. 命令打包过滤：负责对下发的关节目标 (joint angle) 或是电机目标 (motor absolute) 进行范围校验（clamp），
       并将其打包成安全的命令元组压入通信层的发送队列。
    ===========================================================================
    包含：ENCODER_COUNT 路关节目标角，MOTOR_COUNT 路直驱电机原始值。
    """

    def __init__(self):
        self._applied_output_margins = None
        self.comm: Optional[LowerComputerComm] = None
        self.current_state = HandModel()
        self.state_lock = threading.Lock()
        self.running = False
        self.update_callbacks: List[callable] = []
        self._last_update_callback_error = ""
        self._last_update_loop_error = ""
        self._last_coordinate_bookkeeping_error = ""

        self._control_mode = ControlMode.JOINT_ANGLE
        self._task_diag_subscribed = False
        self._control_trace_subscribed = False
        self._control_trace_divisor = 5
        self._started = False
        self._update_thread: Optional[threading.Thread] = None
        self._persistence_queue = queue.Queue(maxsize=1)
        self._persistence_stop = threading.Event()
        self._persistence_thread = None
        self._persistence_diagnostics = None
        self.last_sync_status = "Not connected"
        self._multiturn_store = BundleMultiTurnStore(CALIB_ACTIVE_SNAPSHOT_PATH)
        self._multiturn_store_payload: Optional[dict] = None
        self._multiturn_store_identity = None
        self._multiturn_store_checked_at = 0.0
        self._multiturn_store_last_error: Optional[str] = None
        self._multiturn_last_save_at = 0.0
        # Reset Zero 后 P4 会短暂把 logical ABS 标成可用，但在新的 Auto/LUT
        # 结果落盘前，该坐标不能成为下一次重启的恢复锚点。
        self._multiturn_anchor_capture_pending_mask = 0
        self._multiturn_anchor_capture_blocked_mask = 0
        self._multiturn_anchor_capture_last_error = ""
        self._multiturn_anchor_capture_lock = threading.Lock()
        # 写盘和坐标基准变更共用事务锁，避免后台把旧坐标写回已失效锚点。
        self._multiturn_io_lock = threading.RLock()
        self._servo_rebase_zero_call_lock = threading.Lock()
        self._coordinate_command_lock = threading.RLock()
        self._coordinate_cancel_generation = 0
        self._servo_rebase_zero_unresolved_mask = 0
        self._servo_rebase_zero_baseline_token = 0
        self._servo_rebase_zero_sent_at = 0.0
        self._servo_rebase_zero_capture_mask = 0
        self._servo_rebase_zero_capture_after_ms = 0
        self._servo_rebase_zero_rebased_mask = 0
        self._servo_rebase_zero_device_id = ""
        self._servo_rebase_zero_request_connection = 0
        self._servo_abs_zero_connection_generation = 0
        # 本地归零事务的新反馈已落盘后才递增；不使用设备历史结果作为代次。
        self._servo_abs_zero_generations = [0] * MOTOR_COUNT
        self._servo_abs_zero_generation_feedback_ms = [0] * MOTOR_COUNT
        self._servo_abs_zero_generation_device_ids = [""] * MOTOR_COUNT
        self._servo_abs_zero_generation_connections = [0] * MOTOR_COUNT
        self._state_sync_last_request_at = 0.0
        self._state_sync_waiting_logged = False
        self._host_calibration_io = None
        self._host_calibration_overlay = {}

    def create_host_calibration_io(self, cancel_check, selected_joint_mask,
                                   selected_motor_mask, *,
                                   snapshot_store=None):
        """创建 core 自动校准设备端口；UI 保持原工作流入口。"""
        from .host_calibration.runtime import HostCalibrationIO
        if self._host_calibration_io is not None:
            self._host_calibration_io.cancel()
        io = HostCalibrationIO(self, cancel_check, selected_joint_mask,
                               selected_motor_mask,
                               snapshot_store=snapshot_store)
        with self.state_lock:
            self._host_calibration_io = io
            self._host_calibration_overlay = {}
        return io

    def _cancel_host_calibration(self):
        if self._host_calibration_io is not None:
            self._host_calibration_io.cancel()

    def _apply_host_calibration_overlay(self):
        """仅覆盖宿主校准数据；P4 state/owner/fault 始终来自硬件。"""
        from copy import deepcopy
        for key, value in self._host_calibration_overlay.items():
            setattr(self.current_state, key, deepcopy(value))

    @staticmethod
    def _clamp_motor_abs(value: int) -> int:
        return max(MOTOR_ABS_CMD_MIN, min(MOTOR_ABS_CMD_MAX, int(value)))

    def _safe_motor_hold_targets_from_state(self) -> Optional[List[int]]:
        with self.state_lock:
            state = self.current_state
            if state.multi_turn_unknown_bitmap or not state.has_servo_angle_data:
                return None
            targets = [
                self._clamp_motor_abs(value) if online else 0
                for value, online in zip(state.servo_angles, state.servo_online)
            ]
            return targets if any(state.servo_online) else None

    def hold_current_motor_positions(self, delay_s: float = 0.15) -> bool:
        if not self.comm:
            return False
        targets = self._safe_motor_hold_targets_from_state()
        if not targets:
            return False
        try:
            self.comm.send_command(("motor_pos_abs", targets))
            if self._started:
                self.comm.send_command("start")
            if delay_s > 0:
                import time
                time.sleep(float(delay_s))
            return True
        except Exception as e:
            log_session("CTRL", f"hold current motor positions failed: {e}")
            print(f"Warning: Failed to hold current motor positions: {e}")
            return False

    def invalidate_direct_commands(self) -> None:
        if self.comm:
            self.comm.invalidate_pending_direct_commands()

    def enable_direct_hold(self, motor_mask: int) -> Tuple[bool, str, List[int]]:
        """Enable only from a fresh logical ABS snapshot; never rebuild from sliders."""
        mask = int(motor_mask) & ((1 << MOTOR_COUNT) - 1)
        with self.state_lock:
            state = HandModel()
            state.copy_state_from(self.current_state)
        targets = list(state.servo_angles)
        age = time.monotonic() - state.received_at('servo_abs_last_rx_ts')
        p4_age = ((state.p4_time_ms - state.servo_feedback_p4_time_ms + 0x80000000) & 0xFFFFFFFF) - 0x80000000
        valid = state.servo_online_bitmap & state.servo_route_controllable_bitmap
        reason = ""
        if not self.comm or not mask:
            reason = "not connected or empty motor scope"
        elif state.has_servo_online_status and any(mask & (1 << m) and status != ServoOnlineCheck.READY
                                                  for m, status in enumerate(state.servo_online_check)):
            reason = "selected motor firmware/configuration check has not passed"
        elif state.servo_route_scan_state == 1:
            reason = "full route scan is running"
        elif not state.has_servo_angle_data or not state.has_servo_route_status:
            reason = "waiting for logical ABS and routes"
        elif not state.has_p4_time or not 0 <= age <= 0.5 or abs(p4_age) > 500:
            reason = "logical ABS feedback is stale"
        elif mask & (~valid | state.multi_turn_unknown_bitmap):
            reason = "selected motor offline, unroutable or Turn Unknown"
        if not reason:
            if (state.system_state == SYSTEM_STATE_RUN_DIRECT and state.control_enabled
                    and state.servo_target_owner == 1 and mask & ~state.active_motor_mask == 0):
                return True, 'Direct control is already enabled', targets
            self.invalidate_direct_commands()
            self.set_control_mode(ControlMode.DIRECT_MOTOR)
            generation = self._coordinate_cancel_generation
            with self._coordinate_command_lock:
                if generation != self._coordinate_cancel_generation:
                    return False, 'Direct enable cancelled', targets
                state = self._coordinate_state()
                if (not 0 <= time.monotonic() - state.received_at('servo_abs_last_rx_ts') <= .5 or
                    mask & (~state.servo_online_bitmap | ~state.servo_route_controllable_bitmap | state.multi_turn_unknown_bitmap)):

                    return False, 'Current ABS became stale or unavailable', targets
                targets = list(state.servo_angles)
                token = (state.direct_target_token + 1) & 0xffffffff or 1
                submitted_at = time.monotonic()
                self.start()
                if not self.comm.send_command((
                        'motor_pos_abs', token, mask, targets, DIRECT_SERVO_SPEED, DIRECT_SERVO_ACC)):
                    return False, 'Current-position hold was not queued', targets
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline and generation == self._coordinate_cancel_generation:
                state = self._coordinate_state()
                if (state.direct_target_token == token and state.received_at('direct_target_result_rx_ts') >= submitted_at):
                    if state.direct_target_status != 2 or state.direct_target_applied_mask != mask or state.direct_target_rejected_mask:
                        return False, 'P4 rejected current-position hold', targets
                    if (state.control_enabled and state.servo_target_owner == 1 and
                            state.system_state == SYSTEM_STATE_RUN_DIRECT and state.active_motor_mask == mask
                            and 0 <= time.monotonic() - state.received_at('system_status_last_rx_ts') <= .6):
                        return True, 'Selected motors enabled; holding current ABS', targets
                time.sleep(.01)
            reason = 'Direct enable execution was not confirmed'

        selected = {i: targets[i] for i in range(min(len(targets), MOTOR_COUNT)) if mask & (1 << i)}
        log_session("DIRECT_ENABLE", f"mask=0x{mask:08X} feedback_ms={state.servo_feedback_p4_time_ms} "
                    f"age_s={age:.3f} ABS={selected} hold={selected if not reason else {}} reject={reason or 'none'}")
        return not reason, reason or "Direct motor hold requested", targets

    def set_control_mode(self, mode: ControlMode):
        """设置电机控制模式：JOINT_ANGLE 或 DIRECT_MOTOR"""
        self._control_mode = mode

    def get_control_mode(self) -> ControlMode:
        return self._control_mode

    def invalidate_joint_commands(self) -> int:
        """清除发送队列中待发的关节控制指令（角度包/angle_live）。"""
        if not self.comm:
            return 0
        try:
            return int(self.comm.invalidate_pending_joint_commands())
        except Exception:
            return 0



    def initialize(self, comm_port: str) -> bool:
        if not self._stop_persistence_worker():
            self.last_sync_status='Previous anchor write is still completing; connection not changed'
            return False
        self._reset_local_runtime_state("SYNCING")
        log_session("CTRL", f"initialize requested port={comm_port}")
        try:
            self.comm = LowerComputerComm(comm_port)
            self.comm.recording_packet_sink = getattr(self, "_recording_packet_sink", None)
        except (ValueError, RuntimeError) as exc:
            log_session("CTRL", f"initialize failed before connect: {exc}")
            print(f"Connection failed: {exc}")
            self.last_sync_status = str(exc)
            return False

        try:
            connected = self.comm.connect()
        except (ValueError, RuntimeError) as exc:
            log_session("CTRL", f"serial open failed port={comm_port}: {exc}")
            print(f"Connection failed: {exc}")
            self.last_sync_status = str(exc)
            return False

        if not connected:
            log_session("CTRL", f"serial open failed port={comm_port}")
            self.last_sync_status = "Serial open failed"
            return False

        self.running = True
        self._started = False
        self._start_persistence_worker()
        self._update_thread = threading.Thread(target=self._update_loop, daemon=True)
        self._update_thread.start()

        # 静态配置不再周期广播；每次建立串口链路后显式完成一次全量握手同步。
        self.request_state_sync(STATE_SYNC_ALL)
        sync_ok = self._wait_for_lower_sync(LOWER_SYNC_TIMEOUT_S)
        with self.state_lock:
            if self.current_state.calib_status == "SYNCING":
                self.current_state.calib_status = "IDLE"
        if sync_ok:
            self.last_sync_status = "Synced lower status/servo feedback"
        else:
            self.last_sync_status = "Connected; startup state sync pending, retrying"
        log_session("CTRL", self.last_sync_status)
        return True

    def request_state_sync(self, sync_mask: int = STATE_SYNC_ALL) -> bool:
        """请求 P4 回传静态状态、标定快照、路由和产品指纹。"""
        if not self.comm:
            return False
        sent = bool(self.comm.send_command(("state_sync", int(sync_mask))))
        if sent and int(sync_mask) == STATE_SYNC_ALL:
            self._state_sync_last_request_at = time.monotonic()
        return sent

    def _send_diagnostics_subscription(self) -> bool:
        flags = (1 if self._task_diag_subscribed else 0) | (
            2 if self._control_trace_subscribed else 0)
        return bool(self.comm.send_command(
            ("diagnostics_subscribe", flags, self._control_trace_divisor)))

    def set_task_diag_subscription(self, enabled: bool) -> bool:
        """仅在 Task Runtime 窗口打开期间订阅高频任务诊断。"""
        if not self.comm:
            return False
        self._task_diag_subscribed = bool(enabled)
        return self._send_diagnostics_subscription()

    def set_control_trace_subscription(self, enabled: bool, divisor: int = 5) -> bool:
        """数据记录期间订阅 P4 同周期目标/反馈快照。"""
        if not self.comm:
            return False
        self._control_trace_subscribed = bool(enabled)
        self._control_trace_divisor = int(divisor)
        return self._send_diagnostics_subscription()

    def set_recording_packet_sink(self, sink) -> None:
        self._recording_packet_sink = sink
        if self.comm:
            self.comm.recording_packet_sink = sink

    def _reset_local_runtime_state(self, calib_status: str = "IDLE") -> None:
        self._cancel_host_calibration()
        self._host_calibration_overlay = {}
        with self.state_lock:
            self.current_state = HandModel()
            self._applied_output_margins = None
            self.current_state.calib_status = str(calib_status)
        self._multiturn_store_payload = None
        self._multiturn_store_identity = None
        self._multiturn_store_last_error = None
        self._multiturn_last_save_at = 0.0
        with self._multiturn_io_lock:
            self._servo_abs_zero_connection_generation += 1
            # 重新连接不能把已发出但未确认/未落盘的归零当成未执行，
            # 否则旧 JSON 锚点可能被恢复到新的硬件圈数基准。
            pending_rebase = (
                self._servo_rebase_zero_unresolved_mask
                | self._servo_rebase_zero_capture_mask
            )
            with self._multiturn_anchor_capture_lock:
                self._multiturn_anchor_capture_pending_mask = 0
                self._multiturn_anchor_capture_blocked_mask = pending_rebase
                self._multiturn_anchor_capture_last_error = ""
            if not pending_rebase:
                self._servo_rebase_zero_device_id = ""
                self._servo_rebase_zero_baseline_token = 0
                self._servo_rebase_zero_sent_at = 0.0
                self._servo_rebase_zero_capture_after_ms = 0
                self._servo_rebase_zero_request_connection = self._servo_abs_zero_connection_generation
            self._servo_rebase_zero_rebased_mask &= pending_rebase
        self._state_sync_last_request_at = 0.0
        self._state_sync_waiting_logged = False
        self._started = False

        self._control_mode = ControlMode.JOINT_ANGLE

    def _wait_for_lower_sync(self, timeout_s: float) -> bool:
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        while time.monotonic() < deadline:
            with self.state_lock:
                state = self.current_state
                ready = self._startup_sync_ready(state)
            if ready:
                return True
            time.sleep(0.02)
        return False

    @staticmethod
    def _startup_sync_ready(state: HandModel) -> bool:
        """启动恢复所需的状态、反馈、路由和产品身份均已收到。"""
        has_servo = bool(state.has_servo_angle_data)
        return (
            bool(state.has_system_status_data)
            and has_servo
            and bool(state.has_servo_route_status)
            and bool(state.has_product_fingerprint)
        )

    def _service_startup_state_sync(self, state: HandModel) -> None:
        """静态握手包丢失或早于 P4 就绪时，限频补发全量同步请求。"""
        if self._startup_sync_ready(state):
            if self._state_sync_waiting_logged:
                log_session("SYNC", "required startup data received")
                self._state_sync_waiting_logged = False
            return
        if not self._state_sync_waiting_logged:
            log_session(
                "SYNC",
                "waiting for product fingerprint / servo route; retrying",
            )
            self._state_sync_waiting_logged = True
        now = time.monotonic()
        if now - self._state_sync_last_request_at >= STATE_SYNC_RETRY_PERIOD_S:
            self.request_state_sync(STATE_SYNC_ALL)

    def shutdown(self, stop_lower: bool = False):
        self._cancel_host_calibration()
        self._stop_persistence_worker()
        """
        关闭串口与后台线程。

        普通退出默认不发送锁存式 STOP：断开后由 P4 的上位机租约超时执行
        当前 ABS Hold。只有用户显式要求急停时才传入 stop_lower=True。
        """
        import time
        log_session("CTRL", f"shutdown requested stop_lower={int(bool(stop_lower))}")

        # 退出前补写最后一帧可信锚点，降低用户直接关闭上位机造成的状态窗口。
        with self.state_lock:
            final_state = HandModel()
            final_state.copy_state_from(self.current_state)
        self._persist_multiturn_state(final_state, force=True)

        # STOP 在 P4 中是必须 RESET 才能解除的 E-STOP，不能作为普通资源清理动作。
        if self.comm:
            try:
                if stop_lower:
                    self.comm.send_command("stop")
                    time.sleep(0.15)  # 等待硬件响应
            except Exception as e:
                log_session("CTRL", f"prepare lower for shutdown failed: {e}")
                print(f"Warning: Failed to prepare lower computer for shutdown: {e}")

        # Step 2: 标记停止运行
        self.running = False
        self._started = False

        # Step 3: 断开连接（停止 RX/TX 线程）
        if self.comm:
            try:
                self.comm.disconnect()
            except Exception as e:
                log_session("CTRL", f"disconnect failed: {e}")
                print(f"Warning: Failed to disconnect: {e}")
            time.sleep(0.1)  # 等待连接完全断开

        # Step 4: 等待 _update_loop 线程完全退出
        if self._update_thread and self._update_thread.is_alive():
            self._update_thread.join(timeout=2.0)
            if self._update_thread.is_alive():
                log_session("CTRL", "_update_thread did not exit within timeout")
                print("Warning: _update_thread did not exit within timeout")
        self._update_thread = None

        # Step 5: 清理资源
        with self.state_lock:
            self.update_callbacks.clear()  # 移除所有回调


        # Step 6: 重置内部状态为断开连接状态
        with self.state_lock:
            self.current_state.calib_status = "DISCONNECTED"

        log_session("CTRL", "shutdown complete")

    @property
    def calibration_data_identity(self) -> dict:
        with self.state_lock:
            reported = dict(getattr(self.current_state, "product_fingerprint", {}) or {})
            present = bool(getattr(self.current_state, "has_product_fingerprint", False))
        device_id = str(reported.get("device_id", ""))
        if (not present or int(reported.get("protocol_version", 0)) != HOST_PROTOCOL_VERSION
                or len(device_id) != 16 or device_id == "0000000000000000"
                or any(char not in "0123456789ABCDEF" for char in device_id)):
            raise CalibrationSnapshotError("valid P4 device identity unavailable")
        urdf_path = _active_urdf_repo_path()
        if not urdf_path:
            raise CalibrationSnapshotError("URDF source identity unavailable")
        return {"device_id": device_id, "urdf_repo_path": urdf_path,
                "product_fingerprint": canonical_product_fingerprint(reported)}

    @property
    def calibration_snapshot_path(self):
        return CALIB_ACTIVE_SNAPSHOT_PATH

    def _multiturn_identity(self, state: HandModel) -> Optional[dict]:
        device_id = self.servo_abs_zero_device_id(state)
        urdf_path = _active_urdf_repo_path()
        if not device_id or not urdf_path:
            return None
        if self._multiturn_store.path != self.calibration_snapshot_path:
            self._multiturn_store = BundleMultiTurnStore(self.calibration_snapshot_path)
            self._multiturn_store_identity = None
        return {"device_id": device_id, "urdf_repo_path": urdf_path,
                "product_fingerprint": canonical_product_fingerprint(state.product_fingerprint)}

    def _load_multiturn_store(self, identity: dict) -> Optional[dict]:
        # Explicit changes invalidate identity immediately; external replacements are checked at 1Hz.
        now = time.monotonic()
        if (self._multiturn_store_identity is not None
                and self._multiturn_store_identity[0] == identity
                and self._multiturn_store_payload is not None
                and now - self._multiturn_store_checked_at < 1.0):
            return self._multiturn_store_payload
        self._multiturn_store_checked_at = now
        try:
            version = (identity, self._multiturn_store.path.stat().st_mtime_ns)
        except OSError:
            version = (identity, None)
        # 读取失败（payload=None）绝不缓存成"这台机器没有锚点"，下次调用立刻重读。
        if (self._multiturn_store_identity != version
                or self._multiturn_store_payload is None):
            self._multiturn_store_identity = version
            try:
                self._multiturn_store_payload = self._multiturn_store.load(identity=identity)
                self._multiturn_store_last_error = None
            except (OSError, DeviceStateStoreError, CalibrationSnapshotError) as exc:
                self._multiturn_store_payload = None
                # 同一条错误只记一次，避免 20 Hz 重试刷屏。
                if str(exc) != self._multiturn_store_last_error:
                    self._multiturn_store_last_error = str(exc)
                    log_session("MULTITURN", f"host anchor unavailable: {exc}")
        return self._multiturn_store_payload

    @staticmethod
    def _format_motor_mask(motor_mask: int) -> str:
        return ",".join(
            f"M{motor_idx:02d}"
            for motor_idx in range(MOTOR_COUNT)
            if int(motor_mask) & (1 << motor_idx)
        ) or "none"

    def _multiturn_recovery_anchors(self, state: HandModel):
        identity = self._multiturn_identity(state)
        payload = self._load_multiturn_store(identity) if identity else None
        values = list(payload["logical_abs"]) if payload else [0] * MOTOR_COUNT
        mask = int(str(payload["valid_motor_mask"]), 0) if payload else 0
        # ponytail: 临时诊断，"恢复时锚点为空"定位完即删。
        if not payload or mask == 0:
            try:
                path = self.calibration_snapshot_path
                raw = read_snapshot(path)
                disk_mask = int(str(raw.get('multiturn_anchor_valid_bitmap', '0')), 0)
                mtime = path.stat().st_mtime
            except Exception as exc:
                disk_mask, mtime = f'read failed: {exc}', None
            log_session(
                "MULTITURN",
                f"anchor read identity={'yes' if identity else 'no'} "
                f"payload={'yes' if payload else 'no'} mask=0x{mask:08X} "
                f"disk_mask=0x{disk_mask:08X} mtime={mtime} "
                f"path={self.calibration_snapshot_path}",
            )
        return mask, values, [value % 4096 for value in values], identity is not None

    def _start_persistence_worker(self):
        if self._persistence_thread and self._persistence_thread.is_alive():
            raise RuntimeError('Previous anchor writer has not stopped')
        self._persistence_queue = queue.Queue(maxsize=1)
        jobs=self._persistence_queue
        self._persistence_stop.clear()
        def worker():
            while not self._persistence_stop.is_set():
                try: state, context = jobs.get(timeout=.1)
                except queue.Empty: continue
                try:
                    started=time.monotonic()
                    with self._multiturn_io_lock:
                        if context != self._persistence_context(): continue
                        if any(stamp and ((state.servo_feedback_p4_time_ms-stamp)&0xFFFFFFFF)>=0x80000000
                               for stamp in self._servo_abs_zero_generation_feedback_ms): continue
                        self._persist_multiturn_state_locked(state)
                        self._persistence_diagnostics=(context,state.host_multiturn_valid_mask,state.host_multiturn_status)
                    elapsed=time.monotonic()-started
                    if elapsed>.05:
                        log_session('MULTITURN',f'background anchor save slow duration_ms={elapsed*1000:.1f}')
                except Exception as exc:
                    log_session('MULTITURN',f'background anchor save failed: {exc}')
                finally: jobs.task_done()
                self._persistence_stop.wait(.005)  # let explicit coordinate work acquire the shared lock
        self._persistence_thread=threading.Thread(target=worker,name='mmhand-anchor-writer',daemon=True)
        self._persistence_thread.start()

    def _persistence_context(self):
        return (id(self.comm),self._servo_abs_zero_connection_generation,
                self._coordinate_cancel_generation,tuple(self._servo_abs_zero_generations))

    def _queue_multiturn_persistence(self,state):
        if self._persistence_stop.is_set(): return
        item=(state,self._persistence_context())
        try: self._persistence_queue.put_nowait(item)
        except queue.Full:
            try: self._persistence_queue.get_nowait(); self._persistence_queue.task_done()
            except queue.Empty: pass
            try: self._persistence_queue.put_nowait(item)
            except queue.Full: pass

    def _stop_persistence_worker(self):
        self._persistence_stop.set()
        thread=self._persistence_thread
        if thread and thread.is_alive(): thread.join(timeout=5)
        if thread and thread.is_alive():
            log_session('MULTITURN','anchor writer still completing IO; final save remains serialized')
            return False
        self._persistence_thread=None
        return True

    def _persist_multiturn_state(
        self, state: HandModel, *, force: bool = False
    ) -> bool:
        with self._multiturn_io_lock:
            return self._persist_multiturn_state_locked(state, force=force)

    def _persist_multiturn_state_locked(
        self, state: HandModel, *, force: bool = False
    ) -> bool:
        """按 20 Hz 运动/2 Hz 静止频率保存可信反馈，离线条目保留旧锚点。"""
        identity = self._multiturn_identity(state)
        if identity is None:
            state.host_multiturn_status = "waiting for product fingerprint"
            return False
        payload = self._load_multiturn_store(identity)
        now = time.monotonic()
        moving = int(getattr(state, "system_state", 0)) in (
            SYSTEM_STATE_RUN_DIRECT,
            SYSTEM_STATE_RUN_FEEDBACK,
            SYSTEM_STATE_CALIBRATION_RUNNING,
            SYSTEM_STATE_ROPE_INIT_RUNNING,
        )
        period = (
            HOST_MULTITURN_MOVING_SAVE_PERIOD_S
            if moving
            else HOST_MULTITURN_IDLE_SAVE_PERIOD_S
        )
        if not force and now - self._multiturn_last_save_at < period:
            if payload:
                state.host_multiturn_valid_mask = int(
                    str(payload.get("valid_motor_mask", "0")), 0
                ) & ((1 << MOTOR_COUNT) - 1)

                state.host_multiturn_status = "loaded"
            return False

        abs_values = list(payload.get("logical_abs", [])) if payload else []
        abs_values = (abs_values + [0] * MOTOR_COUNT)[:MOTOR_COUNT]
        valid_mask = (
            int(str(payload.get("valid_motor_mask", "0")), 0)
            if payload else 0
        ) & ((1 << MOTOR_COUNT) - 1)
        online_mask = int(state.servo_online_bitmap)
        if bool(state.has_servo_route_status):
            online_mask &= int(
                getattr(state, "servo_route_controllable_bitmap", 0) or 0
            )
        unknown_mask = int(state.multi_turn_unknown_bitmap)
        servo_abs = list(state.servo_angles or [])
        with self._multiturn_anchor_capture_lock:
            blocked_mask = self._multiturn_anchor_capture_blocked_mask
        rebase_delta_ms = (
            state.servo_feedback_p4_time_ms - self._servo_rebase_zero_capture_after_ms
        ) & 0xFFFFFFFF
        rebase_snapshot_ready = 0 < rebase_delta_ms < 0x80000000
        fresh_mask = 0
        if state.has_servo_angle_data:
            for motor_idx in range(MOTOR_COUNT):
                bit = 1 << motor_idx
                if (
                    (online_mask & bit)
                    and not (unknown_mask & bit)
                    and not (blocked_mask & bit)
                    and (
                        not (self._servo_rebase_zero_rebased_mask & bit)
                        or rebase_snapshot_ready
                    )
                    and motor_idx < len(servo_abs)
                ):
                    abs_values[motor_idx] = int(servo_abs[motor_idx])
                    fresh_mask |= bit
        if fresh_mask == 0 and not force:
            return False
        valid_mask |= fresh_mask
        try:
            payload = self._multiturn_store.save(
                identity=identity,
                valid_motor_mask=valid_mask,
                logical_abs=abs_values,
            )
        except (OSError, DeviceStateStoreError, CalibrationSnapshotError) as exc:
            state.host_multiturn_status = f"save failed: {exc}"
            log_session("MULTITURN", state.host_multiturn_status)
            return False
        self._multiturn_store_payload = payload
        self._multiturn_last_save_at = now
        state.host_multiturn_valid_mask = valid_mask

        state.host_multiturn_status = "saved"
        return True

    def _block_multiturn_anchor_capture(self, motor_mask: int) -> None:
        """防止 Reset Zero 后的临时坐标覆盖已失效的运行锚点。"""
        mask = int(motor_mask) & ((1 << MOTOR_COUNT) - 1)
        if mask == 0:
            return
        with self._multiturn_io_lock:
            with self._multiturn_anchor_capture_lock:
                self._multiturn_anchor_capture_blocked_mask |= mask
                self._multiturn_anchor_capture_pending_mask &= ~mask

    def _invalidate_multiturn_anchor_mask(self, motor_mask: int, reason: str) -> None:
        """硬件 Turn0 可能已改变时，宁可要求重标也不能保留旧恢复锚点。"""
        mask = int(motor_mask) & ((1 << MOTOR_COUNT) - 1)
        if mask == 0:
            return
        with self._multiturn_io_lock:
            payload = self._multiturn_store.invalidate_motor_mask(mask, reason=reason)
            self._multiturn_store_payload = payload

    def request_multiturn_anchor_capture(self, motor_mask: int) -> bool:
        """登记已完成标定的 Mxx，后台在拿到同帧可信反馈后再落盘。"""
        mask = int(motor_mask) & ((1 << MOTOR_COUNT) - 1)
        if mask == 0:
            return False
        with self._multiturn_io_lock:
            with self._multiturn_anchor_capture_lock:
                self._multiturn_anchor_capture_blocked_mask &= ~mask
                self._multiturn_anchor_capture_pending_mask |= mask
        log_session("MULTITURN", f"calibrated anchor capture queued mask=0x{mask:08X}")
        return True

    def multiturn_anchor_capture_status(self, motor_mask: int) -> Tuple[bool, str]:
        """返回本轮标定的原生ABS是否已保存。"""
        requested_mask = int(motor_mask) & ((1 << MOTOR_COUNT) - 1)
        if requested_mask == 0:
            return True, "no calibration motors require anchors"
        with self._multiturn_anchor_capture_lock:
            pending_mask = self._multiturn_anchor_capture_pending_mask
            last_error = self._multiturn_anchor_capture_last_error
        if pending_mask & requested_mask:
            return False, last_error or "waiting for trusted Mxx feedback"
        with self.state_lock:
            state = HandModel()
            state.copy_state_from(self.current_state)
        identity = self._multiturn_identity(state)
        runtime_mask = 0
        if identity is not None:
            payload = self._load_multiturn_store(identity) or {}
            runtime_mask = int(
                str(payload.get("valid_motor_mask", "0")), 0
            ) & ((1 << MOTOR_COUNT) - 1)
        if (
            identity is not None
            and (runtime_mask & requested_mask) == requested_mask
        ):
            return True, "native ABS recovery snapshot saved"
        return False, last_error or "native ABS recovery snapshot missing"

    def _service_multiturn_anchor_capture(self, state: HandModel) -> None:
        with self._multiturn_io_lock:
            self._service_multiturn_anchor_capture_locked(state)

    def _service_multiturn_anchor_capture_locked(self, state: HandModel) -> None:
        """确认已标定电机的原生ABS真正写入唯一恢复快照。"""
        with self._multiturn_anchor_capture_lock:
            pending_mask = self._multiturn_anchor_capture_pending_mask
        if pending_mask == 0:
            return

        online_mask = int(state.servo_online_bitmap)
        if bool(state.has_servo_route_status):
            online_mask &= int(
                getattr(state, "servo_route_controllable_bitmap", 0) or 0
            )
        unknown_mask = int(state.multi_turn_unknown_bitmap)
        servo_abs = list(state.servo_angles or [])
        feedback_ready = bool(state.has_servo_angle_data)
        eligible_mask = 0
        if feedback_ready:
            for motor_idx in range(MOTOR_COUNT):
                bit = 1 << motor_idx
                if (
                    (pending_mask & bit)
                    and (online_mask & bit)
                    and not (unknown_mask & bit)
                    and motor_idx < len(servo_abs)
                ):
                    eligible_mask |= bit
        waiting_mask = pending_mask & ~eligible_mask
        if waiting_mask:
            detail = (
                "calibrated anchor waiting "
                f"motors={self._format_motor_mask(waiting_mask)} "
                f"online=0x{online_mask:08X} unknown=0x{unknown_mask:08X} "
                f"feedback={int(feedback_ready)}"
            )
            with self._multiturn_anchor_capture_lock:
                if detail != self._multiturn_anchor_capture_last_error:
                    self._multiturn_anchor_capture_last_error = detail
                    log_session("MULTITURN", detail)
            state.host_multiturn_status = detail
        if eligible_mask == 0:
            return

        self._persist_multiturn_state(state, force=True)
        payload = self._multiturn_store_payload or {}
        saved_mask = int(str(payload.get("valid_motor_mask", "0")), 0)
        confirmed_mask = eligible_mask & saved_mask
        if confirmed_mask == 0:
            state.host_multiturn_status = "calibrated anchor runtime save pending"
            return
        with self._multiturn_anchor_capture_lock:
            self._multiturn_anchor_capture_pending_mask &= ~confirmed_mask
            remaining_mask = self._multiturn_anchor_capture_pending_mask
            if remaining_mask == 0:
                self._multiturn_anchor_capture_last_error = ""
        state.host_multiturn_status = (
            "calibrated anchor saved"
            if remaining_mask == 0
            else "calibrated anchor partially saved"
        )
        log_session(
            "MULTITURN",
            f"calibrated anchor saved runtime+snapshot mask=0x{confirmed_mask:08X}",
        )

    def reconcile_multiturn_frame(self, motor_mask: int) -> Tuple[bool, str]:
        """把标定包平移到电机当前坐标系，再让 P4 认账；不写电机。

        电机掉电会丢掉原生 ABS 的整圈部分，而 0x0B 的偏移窗口只有 ±4095，
        写不回去。这里改为平移标定包的绝对数字，再让 P4 把当前反馈认作坐标。
        """
        with self._multiturn_io_lock:
            if not self.comm:
                return False, 'serial not connected'
            with self.state_lock:
                state = HandModel()
                state.copy_state_from(self.current_state)
            anchors_mask, anchors, _raw, ready = self._multiturn_recovery_anchors(state)
            if not ready:
                return False, 'no multi-turn anchors to reconcile'
            with self._multiturn_anchor_capture_lock:
                blocked_mask = int(self._multiturn_anchor_capture_blocked_mask)
            usable = (int(motor_mask) & anchors_mask & ~blocked_mask
                      & int(state.servo_online_bitmap)
                      & int(state.servo_route_controllable_bitmap)
                      & ~int(state.servo_route_conflict_bitmap)
                      & ((1 << MOTOR_COUNT) - 1))
            if usable == 0:
                return True, 'no motor requires multi-turn reconciliation'
            currents = [int(value) for value in state.servo_angles]
            offsets = {}
            for motor in range(MOTOR_COUNT):
                if usable & (1 << motor):
                    turns = turn_offsets([anchors[motor]], [currents[motor]])[0]
                    if turns:
                        offsets[motor] = turns
            if offsets:
                try:
                    shifted = shift_snapshot(read_snapshot(self._multiturn_store.path), offsets)
                except (OSError, ValueError) as exc:
                    return False, f'multi-turn frame shift refused: {exc}'
                atomic_write_snapshot(self._multiturn_store.path, shifted)
                self._multiturn_store_payload = None
                self._multiturn_store_identity = None
                log_session('MULTITURN', 'reconcile frame ' + ', '.join(
                    f'M{motor:02d} k={turns:+d}' for motor, turns in sorted(offsets.items())))
                ok, message = self.load_calibration_snapshot(shifted)
                if not ok:
                    return False, f'multi-turn reconciliation reload failed: {message}'
            if not self.comm.send_command(('trust_multiturn', usable)):
                return False, 'P4 did not accept the multi-turn trust request'
            if offsets:
                return True, 'multi-turn frame reconciled for ' + self._format_motor_mask(usable)
            return True, 'multi-turn frames already match'

    def activate_calibration_snapshot(self, snapshot: dict, store) -> None:
        """安装快照并丢掉缓存；下一次对账重新读取新的绝对坐标。"""
        with self._multiturn_io_lock:
            store.activate(snapshot)
            # 导入可能与原文件摘要相同，也可能发生在 mtime 检查间隔内。
            self._multiturn_store_payload = None
            self._multiturn_store_identity = None


    def wait_for_multiturn_recovery(
        self,
        motor_mask: int,
        timeout_s: float = STARTUP_MULTITURN_RECOVERY_TIMEOUT_S,
        *,
        cancelled=None,
    ) -> Tuple[bool, str]:
        """等待路由就绪，把标定平移到电机当前坐标系，再等 P4 认账。"""
        requested_mask = int(motor_mask) & ((1 << MOTOR_COUNT) - 1)
        if requested_mask == 0:
            return True, "no calibrated motors require multi-turn recovery"
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        last_message = "waiting for P4 servo feedback"
        repair_requested_mask = 0
        reconciled = False
        while time.monotonic() < deadline:
            if cancelled is not None and cancelled():
                return False, "multi-turn recovery cancelled"
            with self.state_lock:
                state = HandModel()
                state.copy_state_from(self.current_state)
            feedback_ready = (
                bool(state.has_servo_angle_data)
                and bool(state.has_servo_route_status)
            )
            route_scan_complete = (
                int(state.servo_route_scan_state) == 2
                or (int(state.servo_route_scan_state) == 0
                    and int(state.servo_route_scan_progress) >= 100)
            )
            if not feedback_ready or not route_scan_complete:
                last_message = "waiting for complete P4 servo feedback and route scan"
                time.sleep(0.05)
                continue
            online_mask = int(state.servo_online_bitmap)
            target_mask = requested_mask & online_mask
            if target_mask == 0:
                return True, "calibration motors are currently offline; deferred recovery"
            route_mask = int(
                getattr(state, "servo_route_controllable_bitmap", 0) or 0
            )
            waiting_route = target_mask & ~route_mask
            if waiting_route:
                last_message = (
                    "waiting for P4 servo route: "
                    f"{self._format_motor_mask(waiting_route)}"
                )
                time.sleep(0.05)
                continue
            unknown_mask = int(
                getattr(state, "multi_turn_unknown_bitmap", 0) or 0
            ) & target_mask
            saved_mask, _saved_abs, _saved_raw, identity_ready = (
                self._multiturn_recovery_anchors(state)
            )
            if not identity_ready:
                last_message = "waiting for product identity and multi-turn anchors"
                time.sleep(0.05)
                continue

            # P4 当前仍可信时，把原生ABS保存到唯一标定包。
            known_mask = target_mask & ~unknown_mask
            if known_mask:
                anchors_ok, anchor_message = self.multiturn_anchor_capture_status(
                    known_mask
                )
                missing_repair = known_mask & ~repair_requested_mask
                if not anchors_ok and missing_repair:
                    self.request_multiturn_anchor_capture(missing_repair)
                    repair_requested_mask |= missing_repair
                if not anchors_ok:
                    last_message = "repairing trusted multi-turn anchors: " + anchor_message
                    time.sleep(0.05)
                    continue

            if unknown_mask == 0:
                offline_mask = requested_mask & ~target_mask
                if offline_mask:
                    return (
                        True,
                        "online multi-turn recovery confirmed; deferred offline: "
                        f"{self._format_motor_mask(offline_mask)}",
                    )
                return True, "multi-turn recovery confirmed"
            missing_anchor = unknown_mask & ~saved_mask
            if missing_anchor:
                return (
                    False,
                    "no persistent multi-turn anchor: "
                    f"{self._format_motor_mask(missing_anchor)}",
                )
            if not reconciled:
                ok, message = self.reconcile_multiturn_frame(target_mask)
                reconciled = True
                if not ok:
                    return False, message
                last_message = message
                continue
            last_message = (
                "waiting for P4 multi-turn trust: "
                f"{self._format_motor_mask(unknown_mask)}"
            )
            time.sleep(0.05)
        return False, last_message

    def _update_loop(self):
        while self.running:
            try:
                if not self.comm:
                    break
                new_model = self.comm.rx_queue.get(timeout=0.1)
                # 状态使用 latest-value 邮箱；ACK/安全规则走独立事件通道，逐条合并。
                ack_events = self.comm.drain_ack_events()
                safety_events = self.comm.drain_safety_events()
                self._service_startup_state_sync(new_model)
                # Optional coordinate bookkeeping must not wait for periodic
                # disk IO. Explicit coordinate transactions still use the same
                # blocking lock and cannot race with an anchor write.
                if self._multiturn_io_lock.acquire(blocking=False):
                    try:
                        self._service_servo_rebase_zero_capture(new_model)
                        self._service_multiturn_anchor_capture(new_model)
                    except Exception as exc:
                        # 坐标簿记失败不得跳过状态发布；异常留待下个模型重试，
                        # 否则持续异常会把 current_state 永久冻结成旧快照。
                        message = f"{type(exc).__name__}: {exc}"
                        if message != self._last_coordinate_bookkeeping_error:
                            self._last_coordinate_bookkeeping_error = message
                            log_session("CTRL", f"coordinate bookkeeping deferred: {message}")
                    finally: self._multiturn_io_lock.release()
                with self.state_lock:
                    self.current_state.copy_state_from(new_model)
                    diagnostics=self._persistence_diagnostics
                    if diagnostics and diagnostics[0]==self._persistence_context():
                        self.current_state.host_multiturn_valid_mask=diagnostics[1]
                        self.current_state.host_multiturn_status=diagnostics[2]
                    if ack_events:
                        ack_history = list(self.current_state.proto_ack_history)
                        ack_history.extend(dict(event) for event in ack_events)
                        self.current_state.proto_ack_history = ack_history[-64:]
                        latest_ack = ack_events[-1]
                        self.current_state.proto_ack_sequence = int(latest_ack["sequence"])
                        self.current_state.proto_ack_cmd = int(latest_ack["command"])
                        self.current_state.proto_ack_mode = int(latest_ack["mode"])
                        self.current_state.proto_ack_status = int(latest_ack["status"])
                        self.current_state.proto_ack_p4_time_ms = int(latest_ack["p4_time_ms"])
                    if safety_events:
                        history_by_sequence = {
                            int(item.get("sequence", 0)): dict(item)
                            for item in self.current_state.safety_event_history
                            if isinstance(item, dict)
                        }
                        for event in safety_events:
                            history_by_sequence[int(event.get("sequence", 0))] = dict(event)
                        self.current_state.safety_event_history = [
                            history_by_sequence[key]
                            for key in sorted(history_by_sequence)
                            if key > 0
                        ][-64:]
                        self.current_state.latest_safety_event = dict(safety_events[-1])
                        self.current_state.has_safety_event = True
                    self.current_state.calib_status = new_model.calib_status
                    self.current_state.tactile_data = new_model.tactile_data
                    self.current_state.host_rx_time = new_model.host_rx_time
                    self.current_state.p4_time_ms = new_model.p4_time_ms
                    self.current_state.has_p4_time = new_model.has_p4_time
                    self._apply_host_calibration_overlay()
                    if getattr(new_model, "has_system_status_data", False):
                        self._started = int(new_model.system_state) in (
                            SYSTEM_STATE_IDLE,
                            SYSTEM_STATE_RECOVERY_REQUIRED,
                            SYSTEM_STATE_RUN_FEEDBACK,
                            SYSTEM_STATE_READY,
                            SYSTEM_STATE_RUN_DIRECT,
                        )

                for cb in self.update_callbacks:
                    try:
                        cb(self.current_state)
                    except Exception as exc:
                        message = f"{type(exc).__name__}: {exc}"
                        if message != self._last_update_callback_error:
                            self._last_update_callback_error = message
                            log_session("CTRL", f"update callback failed: {message}")
                            print(f"Update callback failed: {message}")
                # Publish received feedback before periodic fsync/backup IO.
                # A slow disk must not delay this frame's state publication.
                self._queue_multiturn_persistence(new_model)
            except queue.Empty:
                continue
            except Exception as exc:
                if self.running:
                    message = f"{type(exc).__name__}: {exc}"
                    if message != self._last_update_loop_error:
                        self._last_update_loop_error = message
                        log_session("CTRL", f"update loop recovered: {message}")
                        print(f"Update loop recovered: {message}")

    def set_target_angles(self, angles: List[float]) -> bool:
        """设置 ENCODER_COUNT 路关节目标角（度），JOINT_ANGLE 模式"""
        if len(angles) != ENCODER_COUNT:
            raise ValueError(f"Expected {ENCODER_COUNT} joint angles")

        angles = self._validate_angles(angles)
        with self.state_lock:
            self.current_state.set_target_angles(angles)
        if self.comm and self._control_mode == ControlMode.JOINT_ANGLE:
            return bool(self.comm.send_command(self.current_state))
        return False

    def set_target_angles_live(
        self, angles: List[float], *, joint_mask: Optional[int] = None
    ) -> bool:
        """高频滑条跟随专用：仅保留最新目标，避免串口队列积压。"""
        if len(angles) != ENCODER_COUNT:
            raise ValueError(f"Expected {ENCODER_COUNT} joint angles")

        angles = self._validate_angles(angles)
        with self.state_lock:
            self.current_state.set_target_angles(angles)
        if self.comm and self._control_mode == ControlMode.JOINT_ANGLE:
            # tuple 命令走 comm_layer 高速通道（latest-only queue）。
            if joint_mask is not None:
                return bool(self.comm.send_command(
                    ("angle_live_masked", list(angles), int(joint_mask))
                ))
            return bool(self.comm.send_command(("angle_live", list(angles))))
        return False

    def send_target_angles_and_run_feedback(
        self,
        angles: List[float],
        *,
        live: bool = False,
        joint_mask: Optional[int] = None,
    ) -> bool:
        """把关节目标与 RUN_FEEDBACK 作为同一个队列事务发送。"""
        if not self.comm or self._control_mode != ControlMode.JOINT_ANGLE:
            return False
        if len(angles) != ENCODER_COUNT:
            raise ValueError(f"Expected {ENCODER_COUNT} joint angles")
        validated = self._validate_angles(angles)
        with self.state_lock:
            self.current_state.set_target_angles(validated)
        if joint_mask is not None:
            return bool(self.comm.send_command(
                ("joint_target_run_masked", list(validated), int(joint_mask))
            ))
        return bool(self.comm.send_command(("joint_target_run", list(validated))))

    def set_motor_positions_absolute(self, motor_abs: List[int]) -> None:
        """下发27路 logical ABS；每路仅做硬件 ABS 边界钳位。"""
        if len(motor_abs) != MOTOR_COUNT:
            raise ValueError(f"Expected {MOTOR_COUNT} absolute motor positions")
        raw = [self._clamp_motor_abs(value) for value in motor_abs]
        if self.comm and self._control_mode == ControlMode.DIRECT_MOTOR:
            self.comm.send_command(("motor_pos_abs", raw))

    def set_single_motor_position_absolute(self, motor_index: int, motor_abs: int) -> None:
        idx = int(motor_index)
        if idx < 0 or idx >= MOTOR_COUNT:
            raise ValueError(f"motor index out of range: {idx}")
        raw = self._clamp_motor_abs(motor_abs)
        if self.comm and self._control_mode == ControlMode.DIRECT_MOTOR:
            self.comm.send_command(("motor_pos_abs_one", idx, raw))

    def _validate_angles(self, angles: List[float]) -> List[float]:
        out: List[float] = []
        for a in angles:
            try:
                v = float(a)
            except (TypeError, ValueError):
                v = 0.0
            if not math.isfinite(v):
                v = 0.0
            out.append(v)
        return out


    def request_run_feedback(self) -> bool:
        if not self.comm:
            return False
        # A status packet can lag behind a freshly completed one-shot feedback
        # run. Always pair a new joint target with RUN_FEEDBACK so P4 can
        # reacquire ownership after it has already returned to READY.
        # P4 treats a duplicate request while RUN_FEEDBACK is active as
        # idempotent, without resetting the servo target queue.
        return bool(self.comm.send_command("run_feedback"))

    def set_manual_encoder_references(
        self,
        manual_min_raw: List[int],
        manual_max_raw: List[int],
        *, updated_joint_mask: int | None = None,
    ) -> bool:
        """Persist all 21 Manual endpoint pairs and verify the P4 readback."""
        if not self.comm:
            return False
        sent_at = time.monotonic()
        self.comm.send_command((
            "manual_encoder_reference",
            list(manual_min_raw),
            list(manual_max_raw),
        ))
        status = self._ack_status(CMD_MANUAL_ENCODER_REFERENCE, MANUAL_REFERENCE_ACK_TIMEOUT_S, sent_at)
        if status not in (PROTO_ACK_STATUS_OK, PROTO_ACK_STATUS_QUEUED):
            return False
        if status == PROTO_ACK_STATUS_QUEUED and not self._wait_for_manual_reference_commit_result(sent_at):
            return False
        expected_mask = (1 << ENCODER_COUNT) - 1
        expected_crc = manual_encoder_reference_crc(
            manual_min_raw, manual_max_raw
        )
        verified = self._verify_manual_encoder_readback(
            expected_mask,
            list(manual_min_raw),
            list(manual_max_raw),
            expected_crc,
        )
        if verified:
            try:
                update_manual_snapshot(
                    self.calibration_snapshot_path,
                    identity=self.calibration_data_identity,
                    valid_mask=expected_mask,
                    min_raw=list(manual_min_raw),
                    max_raw=list(manual_max_raw),
                    updated_joint_mask=updated_joint_mask,
                )
            except (OSError, CalibrationSnapshotError) as exc:
                log_session("MANUAL", f"desktop JSON save failed: {exc}")
                return False
            # Manual CRC changed: any Auto/LUT result computed against the old
            # Manual endpoints is no longer loadable for the updated joints.
            if updated_joint_mask:
                self.invalidate_runtime_calibration_scope(
                    motor_mask_for_joint_mask(updated_joint_mask),
                    updated_joint_mask,
                    reason="manual_reference",
                )
        return verified

    def _ack_status(self, command_id: int, timeout_s: float, sent_at: float):
        """等一条已发出命令的 ACK 并返回其状态码；超时返回 None。

        发命令仍是调用方的事（send_command）。``sent_at`` 取发命令前的
        monotonic，保证不会读到发送之前的旧 ACK。
        """
        ack = self.comm.wait_for_ack(command_id, timeout_s, received_after_monotonic=sent_at)
        return None if ack is None else int(ack[2])

    def query_manual_encoder_references(self, timeout_s: float = 2.0) -> bool:
        if not self.comm:
            return False
        sent_at = time.monotonic()
        self.comm.send_command(("state_sync", STATE_SYNC_MANUAL_REFERENCE))
        if self._ack_status(CMD_STATE_SYNC_REQUEST, timeout_s, sent_at) != PROTO_ACK_STATUS_OK:
            return False
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            with self.state_lock:
                received_at = float(
                    self.current_state.received_at('manual_reference_last_rx_time')
                    or 0.0
                )
            if received_at >= sent_at:
                return True
            time.sleep(0.01)
        return False

    def _wait_for_manual_reference_commit_result(
        self,
        received_after: float,
        timeout_s: float = MANUAL_REFERENCE_ACK_TIMEOUT_S,
    ) -> bool:
        """等待 Manual 写入 P4 RAM 的结果包，避免把 QUEUED 当成提交完成。"""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            with self.state_lock:
                state = self.current_state
                received_at = float(
                    state.received_at('manual_reference_last_rx_time') or 0.0
                )
                commit_status = int(
                    getattr(state, "manual_reference_commit_status", 0) or 0
                )
            if received_at >= received_after and commit_status != 0:
                return commit_status == ServoOnlineCheck.READY
            time.sleep(0.01)
        return False

    def _verify_manual_encoder_readback(
        self,
        expected_mask: int,
        expected_min: List[int],
        expected_max: List[int],
        expected_crc: int,
    ) -> bool:
        if not self.query_manual_encoder_references(
            MANUAL_REFERENCE_ACK_TIMEOUT_S
        ):
            return False
        with self.state_lock:
            state = self.current_state
            actual_mask = int(
                getattr(state, "manual_reference_valid_bitmap", 0) or 0
            )
            actual_crc = int(
                getattr(state, "manual_reference_crc", 0) or 0
            )
            actual_min = list(
                getattr(state, "manual_reference_min_raw", []) or []
            )
            actual_max = list(
                getattr(state, "manual_reference_max_raw", []) or []
            )
            committed = bool(
                getattr(state, "manual_reference_committed", False)
            )
            commit_status = int(
                getattr(state, "manual_reference_commit_status", 0) or 0
            )
        if actual_mask != (int(expected_mask) & ((1 << ENCODER_COUNT) - 1)):
            return False
        if actual_crc != (int(expected_crc) & 0xFFFFFFFF):
            return False
        if not committed or commit_status != ServoOnlineCheck.READY:
            return False
        if len(actual_min) != ENCODER_COUNT or len(actual_max) != ENCODER_COUNT:
            return False
        for joint in range(ENCODER_COUNT):
            if (actual_mask & (1 << joint)) == 0:
                continue
            if int(actual_min[joint]) != int(expected_min[joint]):
                return False
            if int(actual_max[joint]) != int(expected_max[joint]):
                return False
        return True

    def set_manual_encoder_reference(
        self, joint: int, manual_min_raw: int, manual_max_raw: int, *, cancelled=lambda: False
    ) -> bool:
        """Write one joint while preserving all existing Manual entries."""
        self.last_manual_write_phase = 'INSTALL_FAILED'
        def failed(phase):
            self.last_manual_write_phase = phase
            log_session('MANUAL', f'J{joint:02d} endpoint write failed at {phase}')
            return False
        joint = int(joint)
        if not 0 <= joint < ENCODER_COUNT or not self.comm or cancelled():
            return False
        if not self.query_manual_encoder_references(
            MANUAL_REFERENCE_ACK_TIMEOUT_S
        ):
            return failed('QUERY_FAILED')
        with self.state_lock:
            state = self.current_state
            expected_mask = int(
                getattr(state, "manual_reference_valid_bitmap", 0) or 0
            ) | (1 << joint)
            expected_min = list(
                getattr(state, "manual_reference_min_raw", [0] * ENCODER_COUNT)
            )
            expected_max = list(
                getattr(state, "manual_reference_max_raw", [0] * ENCODER_COUNT)
            )
        if len(expected_min) != ENCODER_COUNT or len(expected_max) != ENCODER_COUNT:
            return False
        expected_min[joint] = int(manual_min_raw)
        expected_max[joint] = int(manual_max_raw)
        if cancelled():
            return False
        sent_at = time.monotonic()
        self.comm.send_command((
            "manual_encoder_single",
            joint,
            int(manual_min_raw),
            int(manual_max_raw),
        ))
        ack = self.comm.wait_for_ack(
            CMD_MANUAL_ENCODER_SINGLE,
            MANUAL_REFERENCE_ACK_TIMEOUT_S,
            received_after_monotonic=sent_at,
        )
        if ack is None or int(ack[2]) not in (
            PROTO_ACK_STATUS_OK,
            PROTO_ACK_STATUS_QUEUED,
        ):
            return failed('ACK_TIMEOUT' if ack is None else f'ACK_REJECTED_{int(ack[2])}')
        if (
            int(ack[2]) == PROTO_ACK_STATUS_QUEUED
            and not self._wait_for_manual_reference_commit_result(sent_at)
        ):
            return failed('COMMIT_FAILED')
        expected_crc = manual_encoder_reference_crc(expected_min, expected_max)
        verified = self._verify_manual_encoder_readback(
            expected_mask,
            expected_min,
            expected_max,
            expected_crc,
        )
        if not verified:
            return failed('READBACK_FAILED')
        if verified and not cancelled():
            self.last_manual_write_phase = 'INSTALLED_NOT_SAVED'
            try:
                update_manual_snapshot(
                    self.calibration_snapshot_path,
                    identity=self.calibration_data_identity,
                    valid_mask=expected_mask,
                    min_raw=expected_min,
                    max_raw=expected_max,
                    updated_joint_mask=1 << joint,
                )
            except (OSError, CalibrationSnapshotError) as exc:
                log_session("MANUAL", f"desktop JSON save failed: {exc}")
                return False
            self.last_manual_write_phase = 'SAVED'
        return verified and not cancelled()

    def rope_init(self, active_joint_mask: int) -> bool:
        if not self.comm:
            return False
        sent_at = time.monotonic()
        self.comm.send_command(("rope_init", int(active_joint_mask)))
        ack = self.comm.wait_for_ack(
            CMD_ROPE_INIT,
            MANUAL_REFERENCE_ACK_TIMEOUT_S,
            received_after_monotonic=sent_at,
        )
        return bool(ack is not None and int(ack[2]) in (0, 2))

    def clear_manual_encoder_references(self) -> bool:
        """清除 P4 RAM 中的 Manual 表，并同步清除桌面标定 JSON 的对应内容。"""
        if not self.comm:
            return False
        sent_at = time.monotonic()
        self.comm.send_command("clear_manual_encoder_reference")
        ack = self.comm.wait_for_ack(
            CMD_MANUAL_ENCODER_REFERENCE,
            MANUAL_REFERENCE_ACK_TIMEOUT_S,
            received_after_monotonic=sent_at,
        )
        if ack is None or int(ack[2]) not in (
            PROTO_ACK_STATUS_OK,
            PROTO_ACK_STATUS_QUEUED,
        ):
            return False
        if (
            int(ack[2]) == PROTO_ACK_STATUS_QUEUED
            and not self._wait_for_manual_reference_commit_result(sent_at)
        ):
            return False
        if not self.query_manual_encoder_references(
            MANUAL_REFERENCE_ACK_TIMEOUT_S
        ):
            return False
        with self.state_lock:
            state = self.current_state
            cleared = (
                int(state.manual_reference_valid_bitmap)
                == 0
                and not bool(
                    getattr(state, "manual_reference_committed", False)
                )
            )
        if not cleared:
            return False
        try:
            update_manual_snapshot(
                self.calibration_snapshot_path,
                identity=self.calibration_data_identity,
                valid_mask=0,
                min_raw=[0] * ENCODER_COUNT,
                max_raw=[0] * ENCODER_COUNT,
            )
        except (OSError, CalibrationSnapshotError) as exc:
            log_session("MANUAL", f"desktop JSON clear failed: {exc}")
            return False
        return True

    def set_active_scope(self, active_motor_mask: int, active_joint_mask: int, require_can_feedback: bool) -> bool:
        if not self.comm:
            return False
        motor_mask = int(active_motor_mask) & 0xFFFFFFFF
        joint_mask = int(active_joint_mask) & 0xFFFFFFFF
        require_can = bool(require_can_feedback)
        sent_at = time.monotonic()
        with self.state_lock:
            state = self.current_state
            # RUN_DIRECT owns the current scope. Reusing its confirmed scope
            # avoids an illegal D7 rewrite when sending the next motor target.
            # Changes and stale/incomplete status still require the P4 ACK.
            if (
                not require_can
                and motor_mask != 0
                and motor_mask & ~((1 << MOTOR_COUNT) - 1) == 0
                and joint_mask & ~((1 << ENCODER_COUNT) - 1) == 0
                and state.has_system_status_data
                and state.has_active_scope_status
                and 0 <= sent_at - state.received_at('system_status_last_rx_ts') <= 0.6
                and state.system_state == SYSTEM_STATE_RUN_DIRECT
                and state.control_enabled
                and state.servo_target_owner == SERVO_TARGET_OWNER_CONTROL
                and state.control_mode == ControlMode.DIRECT_MOTOR.value
                # 已在更大范围内启用时，子集目标沿用现有 scope：P4 只在停机时
                # 接受 scope 重写，运行中重写会被拒（表现为 "Motor scope setup failed"）。
                and motor_mask & ~state.active_motor_mask == 0
                and joint_mask & ~state.active_joint_mask == 0
                and not state.require_can_feedback
            ):
                return True
        self.comm.send_command((
            "active_scope",
            motor_mask,
            joint_mask,
            require_can,
        ))
        return self._ack_status(CMD_ACTIVE_SCOPE, ACTIVE_SCOPE_ACK_TIMEOUT_S, sent_at) == PROTO_ACK_STATUS_OK

    def invalidate_runtime_calibration_scope(
        self, motor_mask: int, joint_mask: int, *, reason: str = "",
    ) -> None:
        """Clear the live calibration view for a scope without touching Manual.

        Release/Reset Zero/rebase invalidate the JSON snapshot; the same scope
        must disappear from the live HandModel too, otherwise Motor Calibration
        keeps showing stale Auto/LUT values until the next app restart.
        """
        motor_mask = int(motor_mask) & ((1 << MOTOR_COUNT) - 1)
        joint_mask = int(joint_mask) & ((1 << ENCODER_COUNT) - 1)
        if not motor_mask and not joint_mask:
            return
        with self.state_lock:
            state = self.current_state
            results = list(state.joint_calib_results or [])
            for joint in range(ENCODER_COUNT):
                if joint_mask & (1 << joint):
                    if joint < len(results):
                        results[joint] = None
            state.joint_calib_results = results
            state.auto_calibration_valid_bitmap = (
                int(state.auto_calibration_valid_bitmap) & ~joint_mask
            )
            state.calibration_warning_bitmap = (
                int(state.calibration_warning_bitmap) & ~joint_mask
            )
            state.motor_abs_limit_valid_bitmap = (
                int(state.motor_abs_limit_valid_bitmap) & ~motor_mask
            )
            mins = list(state.motor_abs_limit_min or [])
            maxs = list(state.motor_abs_limit_max or [])
            for motor in range(MOTOR_COUNT):
                if motor_mask & (1 << motor):
                    if motor < len(mins):
                        mins[motor] = 0
                    if motor < len(maxs):
                        maxs[motor] = 0
            state.motor_abs_limit_min = mins
            state.motor_abs_limit_max = maxs

        self._multiturn_store_identity = None
        log_session(
            "CALIB",
            f"runtime calibration invalidated motor=0x{motor_mask:08X} "
            f"joint=0x{joint_mask:06X} reason={reason or 'coordinate_change'}",
        )

    def _invalidate_servo_rebase_zero_anchors(self, motor_mask: int) -> None:
        """替换坐标前后的旧 runtime / Auto 快照都不能用于恢复。"""
        with self._multiturn_io_lock:
            state = self._coordinate_state()
            if (self._servo_rebase_zero_device_id
                    and (self.servo_abs_zero_device_id(state) != self._servo_rebase_zero_device_id
                         or self._servo_abs_zero_connection_generation != self._servo_rebase_zero_request_connection)):
                return
            self._invalidate_coordinate_anchors(motor_mask, "servo_abs_rebase_zero", state)

    def _invalidate_coordinate_anchors(self, motor_mask, reason, state):
        """从固定设备快照选择同一 namespace 的运行锚点及 Auto 后备锚点。"""
        with self._multiturn_io_lock:
            if self._multiturn_identity(state) is None:
                raise CalibrationSnapshotError("device identity unavailable during coordinate anchor invalidation")
            snapshot_path = self.calibration_snapshot_path
            self._invalidate_multiturn_anchor_mask(motor_mask, reason)
            invalidate_calibration_snapshot_scope(
                snapshot_path, motor_mask=motor_mask,
                joint_mask=joint_mask_for_motor_mask(motor_mask), reason=reason,
            )
            self.invalidate_runtime_calibration_scope(
                motor_mask, joint_mask_for_motor_mask(motor_mask), reason=reason,
            )


    @staticmethod
    def servo_abs_zero_device_id(state: HandModel) -> str:
        """沿用持久化身份的设备指纹门槛，仅校验，不切换文件或读取磁盘。"""
        reported = state.product_fingerprint if state.has_product_fingerprint else {}
        device_id = str(reported.get("device_id", ""))
        if (int(reported.get("protocol_version", 0)) != HOST_PROTOCOL_VERSION
                or len(device_id) != 16 or device_id == "0000000000000000"
                or any(char not in "0123456789ABCDEF" for char in device_id)):
            return ""
        return device_id

    @staticmethod
    def _servo_abs_zero_result_matches(state, baseline_token, sent_at, mask):
        expected = ((int(baseline_token) + 1) & 0xffffffff) or 1
        completed = state.servo_rebase_zero_success_mask | state.servo_rebase_zero_failed_mask
        return (state.has_servo_rebase_zero_result
                and state.servo_rebase_zero_result_token == expected
                and state.received_at('servo_rebase_zero_result_last_rx_ts') >= sent_at
                and completed == mask)

    def _coordinate_state(self):
        with self.state_lock:
            state = HandModel()
            state.copy_state_from(self.current_state)
        return state

    @staticmethod
    def _coordinate_safe_state(state, mask):
        return (state.has_system_status_data
                and 0 <= time.monotonic() - state.received_at('system_status_last_rx_ts') <= LOWER_STATUS_STALE_AFTER_S
                and not state.control_enabled and state.servo_target_owner == 0
                and state.system_state in (SYSTEM_STATE_UNCONTROLLABLE, SYSTEM_STATE_IDLE,
                                           SYSTEM_STATE_READY, SYSTEM_STATE_RECOVERY_REQUIRED)
                and state.has_servo_route_status and state.servo_route_scan_state != 1
                and not mask & ~state.servo_online_bitmap
                and not mask & ~state.servo_route_controllable_bitmap
                and not mask & state.servo_route_conflict_bitmap
                and (not state.has_servo_online_status or all(
                    not mask & (1 << m) or status == ServoOnlineCheck.READY
                    for m, status in enumerate(state.servo_online_check))))

    @staticmethod
    def _restore_feedback_ready(state):
        """当前 raw 只可与同快照、及时到达的 ABS 一起重建硬件坐标。"""
        delta = (int(state.system_status_p4_time_ms) - int(state.servo_feedback_p4_time_ms)) & 0xffffffff
        if delta >= 0x80000000:
            delta -= 0x100000000
        return (state.has_servo_angle_data
                and 0 <= time.monotonic() - state.received_at('servo_abs_last_rx_ts') <= LOWER_STATUS_STALE_AFTER_S
                and abs(delta) <= LOWER_STATUS_STALE_AFTER_S * 1000)

    @staticmethod
    def _coordinate_ack_status(ack, command_id, sent_at):
        """仅发送之后同命令的 ACK 可用于受理判断；逐电机成功仍依赖最终结果。"""
        if ack is None or len(ack) < 4 or int(ack[0]) != command_id:
            return None
        return int(ack[2])

    def _wait_coordinate_result(self, matches, cancelled, timeout_s, *, query_first=False, label="coordinate"):
        """丢失结果只查询现有事务；查询次数不计作新的硬件写请求。"""
        def query():
            with self._coordinate_command_lock:
                if cancelled():
                    return False
                self.request_state_sync(STATE_SYNC_MULTITURN_STATE)
                return True

        def probe_line(state, count):
            # 结果匹配诊断：轮询视角下的两类事务字段与状态包新鲜度。
            rx_ts = state.received_at('system_status_last_rx_ts')
            return (f"wait[{label}] probe#{count} "
                    f"rb(tok={int(state.servo_rebase_zero_result_token)} "
                    f"done=0x{(int(state.servo_rebase_zero_success_mask) | int(state.servo_rebase_zero_failed_mask)):08X} "
                    f"has={int(bool(state.has_servo_rebase_zero_result))}) "
                    f"rx_age={max(0.0, time.monotonic() - rx_ts):.2f}s "
                    f"dev={int(bool(self.servo_abs_zero_device_id(state)))}")

        queries = 0
        if query_first and query():
            queries = 1
            log_session("MULTITURN", f"wait[{label}] state-sync query before polling")
        started = time.monotonic()
        deadline = started + max(0.0, timeout_s)
        probes = 0
        next_probe_log = 0.0
        while True:
            state = self._coordinate_state()
            probes += 1
            if matches(state):
                log_session("MULTITURN",
                            f"wait[{label}] matched probes={probes} elapsed={time.monotonic() - started:.2f}s")
                return state
            if cancelled():
                log_session("MULTITURN", f"wait[{label}] cancelled probes={probes}")
                return None
            now = time.monotonic()
            if now - started >= next_probe_log:
                next_probe_log = (now - started) + 0.5
                log_session("MULTITURN", probe_line(state, probes))
            if now >= deadline:
                if queries >= SERVO_COORDINATE_RESULT_SYNC_ATTEMPTS:
                    log_session("MULTITURN", f"wait[{label}] timed out {probe_line(state, probes)}")
                    return None
                if not query():
                    log_session("MULTITURN", f"wait[{label}] query rejected {probe_line(state, probes)}")
                    return None
                queries += 1
                log_session("MULTITURN", f"wait[{label}] deadline hit; state-sync query #{queries}")
                deadline = time.monotonic() + min(
                    max(0.0, timeout_s), SERVO_COORDINATE_RESULT_SYNC_WAIT_S,
                )
                continue
            time.sleep(0.02)

    def _service_servo_rebase_zero_capture(self, state: HandModel) -> None:
        """最终结果先失效旧坐标，再保存结果之后的新原生ABS。"""
        with self._multiturn_io_lock:
            device_id = self.servo_abs_zero_device_id(state)
            if not device_id or device_id != self._servo_rebase_zero_device_id:
                return
            if self._servo_rebase_zero_request_connection != self._servo_abs_zero_connection_generation:
                return
            unresolved = self._servo_rebase_zero_unresolved_mask
            if (
                unresolved
                and self._servo_abs_zero_result_matches(
                    state, self._servo_rebase_zero_baseline_token,
                    self._servo_rebase_zero_sent_at, unresolved,
                )
            ):
                self._invalidate_servo_rebase_zero_anchors(unresolved)
                self._servo_rebase_zero_capture_mask = (
                    state.servo_rebase_zero_success_mask & unresolved
                )
                self._servo_rebase_zero_rebased_mask |= self._servo_rebase_zero_capture_mask
                self._servo_rebase_zero_capture_after_ms = (
                    state.servo_rebase_zero_result_p4_time_ms
                )
                self._servo_rebase_zero_unresolved_mask = 0
            capture_mask = self._servo_rebase_zero_capture_mask
            after_ms = self._servo_rebase_zero_capture_after_ms
            delta_ms = (state.servo_feedback_p4_time_ms - after_ms) & 0xFFFFFFFF
            if (
                not capture_mask
                or not state.has_servo_angle_data
                or not 0 < delta_ms < 0x80000000
            ):
                return
            eligible = capture_mask & state.servo_online_bitmap & ~state.multi_turn_unknown_bitmap
            if state.has_servo_route_status:
                eligible &= state.servo_route_controllable_bitmap
            if eligible:
                # 新坐标与旧Auto/LUT不能混用；唯一标定包保存新恢复记录。
                with self._multiturn_anchor_capture_lock:
                    self._multiturn_anchor_capture_blocked_mask &= ~eligible
                try:
                    saved = self._persist_multiturn_state(state, force=True)
                except Exception:
                    with self._multiturn_anchor_capture_lock:
                        self._multiturn_anchor_capture_blocked_mask |= eligible
                    raise
                if saved:
                    for motor in range(MOTOR_COUNT):
                        if eligible & (1 << motor):
                            self._servo_abs_zero_generations[motor] += 1
                            self._servo_abs_zero_generation_feedback_ms[motor] = state.servo_feedback_p4_time_ms
                            self._servo_abs_zero_generation_device_ids[motor] = device_id
                            self._servo_abs_zero_generation_connections[motor] = self._servo_abs_zero_connection_generation
                    self._servo_rebase_zero_capture_mask &= ~eligible
                else:
                    with self._multiturn_anchor_capture_lock:
                        self._multiturn_anchor_capture_blocked_mask |= eligible

    def rebase_servo_abs_zero(
        self, motor_mask: int, *, cancelled=lambda: False, send_gate=None,
    ) -> ServoRebaseZeroResult:
        """静止时将所选电机当前位置设为 ABS 0；调用者先释放运动 owner。

        P4用0x0B校正并以正常原生反馈确认；主机只提交一次，未知结果仅查询。
        """
        return self._set_servo_abs_zero(
            motor_mask, CMD_MOTOR_ABS_ZERO, "servo_rebase_zero",
            cancelled=cancelled, send_gate=send_gate,
        )

    def zero_direct_motor_abs(
        self, motor_mask: int, *, cancelled=lambda: False, send_gate=None
    ) -> ServoRebaseZeroResult:
        """Direct 专用置零；复用校准置零的结果、可信反馈及锚点事务。"""
        return self._set_servo_abs_zero(
            motor_mask, CMD_MOTOR_ABS_ZERO, "motor_abs_zero",
            cancelled=cancelled, send_gate=send_gate,
        )

    def servo_abs_zero_anchor_saved(self, motor_mask: int) -> bool:
        """所选成功电机的新实际坐标已由反馈写入运行 JSON。"""
        mask = int(motor_mask)
        with self._multiturn_io_lock, self._multiturn_anchor_capture_lock:
            payload = self._multiturn_store_payload or {}
            valid = int(str(payload.get("valid_motor_mask", "0")), 0)
            pending = (self._servo_rebase_zero_unresolved_mask
                       | self._servo_rebase_zero_capture_mask
                       | self._multiturn_anchor_capture_blocked_mask)
            return bool(mask and valid & mask == mask and not pending & mask)

    def get_servo_abs_zero_generation_state(self):
        """原子返回代次、反馈帧及设备/连接来源；调用者不持应用锁。"""
        with self._multiturn_io_lock:
            return (tuple(self._servo_abs_zero_generations),
                    tuple(self._servo_abs_zero_generation_feedback_ms),
                    tuple(self._servo_abs_zero_generation_device_ids),
                    tuple(self._servo_abs_zero_generation_connections),
                    self._servo_abs_zero_connection_generation)

    def get_servo_abs_zero_generations(self):
        """只读、单调的电机坐标变更代次，迟到或取消后落盘同样可见。"""
        return self.get_servo_abs_zero_generation_state()[0]

    def _set_servo_abs_zero(
        self, motor_mask: int, command_id: int, command_name: str,
        *, cancelled=lambda: False, send_gate=None,
    ) -> ServoRebaseZeroResult:
        """F0/F1共用一笔P4事务，不从主机重复改写电机。"""
        mask = int(motor_mask)
        if mask <= 0 or mask & ~((1 << MOTOR_COUNT) - 1):
            raise ValueError("servo ABS rebase requires a non-zero valid motor mask")
        ok, message, success, failed, token = self._coordinate_transaction(
            mask, _ABS_ZERO_PLAN, command_name, cancelled, send_gate, SERVO_REBASE_ZERO_ACK_TIMEOUT_S,
            SERVO_REBASE_ZERO_RESULT_TIMEOUT_S,
        )
        return ServoRebaseZeroResult(ok, message, token, success, failed)

    def _coordinate_transaction(self, mask, plan, command_name, cancelled, send_gate,
                                ack_timeout_s, result_timeout_s):
        """设零类事务：校验 → 发送一次 → 等待P4最终结果。

        ``plan`` 决定用哪套结果字段；返回 (成功, 消息, 成功位图, 失败位图, 结果token)。
        """
        success = 0
        result_token = 0

        def rejected(message):
            return False, message, success, mask & ~success, result_token

        if not self.comm or cancelled():
            return rejected("serial not connected or cancelled")
        if not self._servo_rebase_zero_call_lock.acquire(blocking=False):
            return rejected("another servo coordinate transaction is waiting")
        try:
            state = self._coordinate_state()
            device_id = self.servo_abs_zero_device_id(state)
            if not device_id:
                return rejected(f"valid P4 device identity unavailable for {plan.label}")
            if plan.requires_sync and not getattr(state, plan.has_result_field):
                self.request_state_sync(STATE_SYNC_MULTITURN_STATE)
                return rejected(f"waiting for P4 {plan.label} capability/result")
            comm = self.comm
            connection = self._servo_abs_zero_connection_generation
            cancel_generation = self._coordinate_cancel_generation

            def same_session():
                return (self.comm is comm
                        and self._servo_abs_zero_connection_generation == connection
                        and self.servo_abs_zero_device_id(self._coordinate_state()) == device_id)

            def changed():
                return (cancelled() or self._coordinate_cancel_generation != cancel_generation
                        or not same_session())

            if plan.precheck is None:
                pass
            else:
                early = plan.precheck(self, state, mask, plan, changed, rejected, send_gate or (lambda action: action()))
                if early is not None:
                    return early

            pending = mask
            state = self._coordinate_state()
            if changed():
                return rejected(f"{plan.label} cancelled or device connection changed")
            if not self._coordinate_safe_state(state, pending):
                return rejected(plan.unsafe_state_message)
            with self._multiturn_io_lock:
                blocked = (self._servo_rebase_zero_unresolved_mask or self._servo_rebase_zero_capture_mask
                           )
                if blocked:
                    self.request_state_sync(STATE_SYNC_MULTITURN_STATE)
                    return rejected(f"previous servo coordinate transaction is awaiting its result/new feedback (0x{blocked:08X})")
                with self._multiturn_anchor_capture_lock:
                    previous_blocked = self._multiturn_anchor_capture_blocked_mask
                    previous_capture = self._multiturn_anchor_capture_pending_mask
                self._block_multiturn_anchor_capture(pending)
                sent_at = time.monotonic()
                baseline_token = int(getattr(state, plan.baseline_token_field))
                plan.record(self, pending, baseline_token, sent_at, device_id, connection, comm)

            def send():
                # 外部应用/运动 gate 在此锁之前取得，STOP 无需等待 ACK/文件。
                with self._coordinate_command_lock:
                    if changed() or not self._coordinate_safe_state(self._coordinate_state(), pending):
                        return False
                    return comm.send_command((command_name, pending))

            sent = send_gate(send) if send_gate is not None else send()

            def restore_unaccepted():
                with self._multiturn_io_lock, self._multiturn_anchor_capture_lock:
                    plan.clear(self, success)
                    self._multiturn_anchor_capture_blocked_mask = (
                        self._multiturn_anchor_capture_blocked_mask & ~pending
                    ) | (previous_blocked & pending)
                    self._multiturn_anchor_capture_pending_mask |= previous_capture & pending

            if not sent:
                restore_unaccepted()
                return rejected(f"{plan.label} cancelled or command queue rejected")
            log_session("MULTITURN", f"{command_name} submitted mask=0x{pending:08X}")
            ack = comm.wait_for_ack(plan.command_id, ack_timeout_s, received_after_monotonic=sent_at)
            matches = lambda s: same_session() and plan.matches(s, baseline_token, sent_at, pending)
            status = self._coordinate_ack_status(ack, plan.command_id, sent_at)
            if (same_session() and not matches(self._coordinate_state())
                    and status is not None and status not in (PROTO_ACK_STATUS_OK, PROTO_ACK_STATUS_QUEUED)):
                restore_unaccepted()
                if changed():
                    return rejected(f"{plan.label} cancelled; hardware write was not accepted")
                return rejected(plan.rejected_message(status))
            result = self._wait_coordinate_result(
                matches, changed, result_timeout_s, query_first=status is None, label=command_name,
            )
            if result is None:
                if same_session():
                    plan.invalidate_on_timeout(self, pending, plan, state)
                return rejected(plan.timeout_message)
            success |= int(getattr(result, plan.success_field)) & pending
            result_token = int(getattr(result, plan.token_field))
            plan.service_result(self, result)
            if changed():
                return rejected(f"{plan.label} cancelled after submission; completed motor coordinates retained")
            remaining = mask & ~success
            if not remaining:
                return True, plan.success_message, success, 0, result_token
            if self._servo_rebase_zero_capture_mask:
                return rejected(
                    f"{plan.label} partly completed; failed: {self._format_motor_mask(remaining)}; "
                    "new trusted feedback/JSON save unconfirmed")
            return rejected(f"{plan.label} failed: {self._format_motor_mask(remaining)}")
        except (OSError, DeviceStateStoreError, CalibrationSnapshotError) as exc:
            return rejected(f"{plan.label} anchor invalidation failed: {exc}")
        finally:
            self._servo_rebase_zero_call_lock.release()

    def manual_prepare_all_motors(self, *, motor_mask=None, cancelled=lambda: False, motion_gate=lambda action: action()) -> ManualPrepareResult:
        """冻结指定的电机集合，归零并放绳 1500 counts。

        可重复调用：每次都在当前位置重新建立 turn0 基准，再放 1500 counts。
        """
        all_motors = (1 << MOTOR_COUNT) - 1
        if motor_mask is not None and (type(motor_mask) is not int or motor_mask <= 0 or motor_mask & ~all_motors):
            return ManualPrepareResult(False, 'Invalid selected tendon motor scope', 0, 0, 0, 0)
        requested_mask = all_motors if motor_mask is None else motor_mask
        empty = ManualPrepareResult(False, "", 0, requested_mask, 0, 0)
        if cancelled():
            return empty._replace(message='Release Rope cancelled')
        if not self.comm:
            return empty._replace(message="serial not connected")
        try:
            _ = self.calibration_data_identity
        except CalibrationSnapshotError as exc:
            return empty._replace(message=str(exc))

        pending_mask, refusal = self._manual_prepare_scope(motor_mask, requested_mask, empty)
        if refusal is not None:
            return refusal
        selected = [i for i in range(MOTOR_COUNT) if pending_mask & (1 << i)]
        log_session("RELEASE_ROPE", f"requested=0x{requested_mask:05X} pending=0x{pending_mask:05X}")

        reset, failure = self._manual_prepare_reset(pending_mask, cancelled, motion_gate)
        if failure is not None:
            return failure
        ok, reset_message, reset_mask, failed_mask, invalidated_joint_mask, snapshot_message = reset
        if not ok:
            return ManualPrepareResult(
                False, reset_message, reset_mask,
                failed_mask | (pending_mask & ~reset_mask), invalidated_joint_mask, 0,
            )
        if cancelled():
            return empty._replace(message='Release Rope cancelled after reset; Auto/LUT invalidated',
                                  reset_motor_mask=reset_mask, invalidated_joint_mask=invalidated_joint_mask)

        baseline = self._wait_release_baseline(pending_mask, selected, cancelled)
        if isinstance(baseline, str):
            return ManualPrepareResult(
                False, f"motor ABS zeroed, but {baseline}", reset_mask, 0, invalidated_joint_mask, 0,
            )

        outcome = self._run_release_motion(
            pending_mask, selected, baseline, reset_mask, invalidated_joint_mask, cancelled, motion_gate,
        )
        if isinstance(outcome, ManualPrepareResult):
            return outcome
        reached_mask = outcome
        return ManualPrepareResult(
            True,
            (
                f"{len(selected)} motor ABS zeroed; released {MANUAL_PREPARE_RELEASE_COUNTS} "
                f"counts; {snapshot_message}; automatic calibration is required"
            ),
            reset_mask,
            0,
            invalidated_joint_mask,
            reached_mask,
        )

    def _manual_prepare_scope(self, motor_mask, requested_mask, empty):
        """校验系统状态与在线/路由/ABS，返回 (待处理电机位图, 失败结果)。"""
        with self.state_lock:
            state = self.current_state
            system_state = int(state.system_state)
            online_mask = int(state.servo_online_bitmap)
            route_mask = int(state.servo_route_controllable_bitmap)
            has_routes = bool(state.has_servo_route_status)
            has_abs = (bool(state.has_servo_angle_data)
                       and 0 <= time.monotonic() - state.received_at('servo_abs_last_rx_ts') <= 0.5
                       and state.servo_route_scan_state != 1)

        allowed_states = (
            SYSTEM_STATE_UNCONTROLLABLE,
            SYSTEM_STATE_IDLE,
            SYSTEM_STATE_READY,
            SYSTEM_STATE_RECOVERY_REQUIRED,
        )
        if system_state not in allowed_states:
            return 0, empty._replace(
                message=f"Manual Prepare requires a stopped state; current={system_state}"
            )
        routable = online_mask & route_mask
        if motor_mask is not None and requested_mask & ~routable:
            missing = requested_mask & ~routable
            return 0, empty._replace(message=f'Selected tendon motors are offline or unroutable: 0x{missing:08X}')
        requested_mask &= routable
        if not requested_mask or not has_routes or not has_abs:
            return 0, empty._replace(message="Release Rope requires online motors with unique routes and ABS")

        return requested_mask, None

    def _manual_prepare_reset(self, pending_mask, cancelled, motion_gate):
        """归零硬件圈数并作废旧标定；返回 (结果元组, 失败结果)。"""
        result = self.rebase_servo_abs_zero(
            pending_mask, cancelled=cancelled, send_gate=motion_gate)
        ok, reset_message = result.ok, result.message
        reset_mask, failed_mask = result.success_motor_mask, result.failed_motor_mask
        invalidated_joint_mask = joint_mask_for_motor_mask(reset_mask)
        if reset_mask:
            # 硬件 turn0 已改变，UI 不能再显示旧 Auto/LUT；即使后续步骤失败也一样。
            self.invalidate_runtime_calibration_scope(
                reset_mask, invalidated_joint_mask, reason="manual_prepare",
            )
        try:
            snapshot_message = invalidate_calibration_snapshot_scope(
                self.calibration_snapshot_path,
                motor_mask=reset_mask,
                joint_mask=invalidated_joint_mask,
                reason="manual_prepare",
            ) if reset_mask else "no motors reset"
        except (OSError, CalibrationSnapshotError, ValueError) as exc:
            return None, ManualPrepareResult(
                False,
                f"motor ABS zeroed, but calibration JSON invalidation failed: {exc}",
                reset_mask,
                0,
                invalidated_joint_mask,
                0,
            )
        return (ok, reset_message, reset_mask, failed_mask, invalidated_joint_mask, snapshot_message), None

    def _wait_release_baseline(self, pending_mask, selected, cancelled):
        """等待Reset后的可信原生ABS；成功返回27路基准。"""
        deadline = time.monotonic() + MANUAL_PREPARE_FEEDBACK_TIMEOUT_S
        reason = "waiting for post-reset native ABS"
        while time.monotonic() < deadline:
            if cancelled():
                break
            with self.state_lock:
                state = self.current_state
                unknown = int(state.multi_turn_unknown_bitmap) & pending_mask
                online = int(state.servo_online_bitmap)
                fresh = 0 <= time.monotonic() - state.received_at('servo_abs_last_rx_ts') <= 0.5
                routable = (state.servo_route_controllable_bitmap & pending_mask) == pending_mask
                servo_abs = list(state.servo_angles or [])
            aligned = (
                fresh and routable and unknown == 0
                and (online & pending_mask) == pending_mask
                and len(servo_abs) >= MOTOR_COUNT
            )
            if aligned:
                return [int(servo_abs[i]) for i in range(MOTOR_COUNT)]
            reason = (
                "post-reset native ABS unavailable "
                f"(fresh={fresh} routable={routable} unknown=0x{unknown:05X} "
                f"online=0x{(online & pending_mask):05X})"
            )
            time.sleep(0.02)
        return reason

    def _run_release_motion(self, pending_mask, selected, baseline, reset_mask,
                            invalidated_joint_mask, cancelled, motion_gate):
        """申请直控、发送放绳目标并等待到位；返回 (到位位图, 快照消息) 或失败结果。"""
        def failed(message):
            return ManualPrepareResult(False, message, reset_mask, 0, invalidated_joint_mask, 0)

        targets = [
            self._clamp_motor_abs(
                motor_abs_after_tendon_action(baseline[motor_idx], motor_idx, -MANUAL_PREPARE_RELEASE_COUNTS)
            ) if motor_idx in selected else baseline[motor_idx]
            for motor_idx in range(MOTOR_COUNT)
        ]
        if not self.set_active_scope(pending_mask, 0, False):
            return failed("motor ABS zeroed, but selected-motor direct scope was rejected")
        self.set_control_mode(ControlMode.DIRECT_MOTOR)
        if cancelled():
            return failed('Release Rope cancelled before movement')

        def send_release():
            self.start()
            return self.comm.send_command(("motor_pos_abs", targets))
        if not motion_gate(send_release):
            self.reset()
            return failed("motor ABS zeroed, but the 1500-count release command was not queued")

        # P4 在 RESET/切直控后需要一拍才给 DIRECT_TARGET 结果；这里只等“就绪”。
        deadline = time.monotonic() + MANUAL_PREPARE_READY_TIMEOUT_S
        ready_reason = "waiting for P4 direct-target result"
        while time.monotonic() < deadline and not cancelled():
            with self.state_lock:
                state = self.current_state
                d_status = int(state.direct_target_status)
                d_applied = int(state.direct_target_applied_mask)
                d_rejected = int(state.direct_target_rejected_mask)
                has_direct_result = bool(state.has_direct_target_result)
                run_state = int(state.system_state)
                fresh = 0 <= time.monotonic() - state.received_at('servo_abs_last_rx_ts') <= 0.5
            if d_rejected & pending_mask:
                self.reset()
                return failed(f"motor ABS zeroed, but P4 rejected the direct release target on 0x{d_rejected & pending_mask:05X}")
            if has_direct_result:
                if d_status == 2 and (d_applied & pending_mask) == pending_mask:
                    ready_reason = "direct target applied"
                    break
                if d_status == 3:
                    self.reset()
                    return failed(f"motor ABS zeroed, but P4 rejected the direct release target on 0x{d_rejected or pending_mask:05X}")
            elif run_state == SYSTEM_STATE_RUN_DIRECT and fresh:
                ready_reason = "P4 entered RUN_DIRECT"
                break
            time.sleep(0.02)

        reached_mask, motion_reason, cancelled_during_motion = self._wait_release_targets(
            pending_mask, selected, targets, ready_reason, cancelled)
        # 维护动作结束必须释放 owner，不能把 RUN_DIRECT 留在设备里。
        self.reset()
        if cancelled_during_motion:
            return ManualPrepareResult(
                False,
                "Release Rope cancelled during release motion; turn counters reset and old Auto/LUT invalidated",
                reset_mask, 0, invalidated_joint_mask, reached_mask,
            )
        if reached_mask != pending_mask:
            return ManualPrepareResult(
                False,
                (
                    "turn counters reset and old Auto/LUT invalidated, but release "
                    f"did not settle; reached=0x{reached_mask:05X}/0x{pending_mask:05X}; "
                    f"reason={motion_reason}; {self._release_diagnostic(selected, targets)}"
                ),
                reset_mask, 0, invalidated_joint_mask, reached_mask,
            )
        return reached_mask

    def _wait_release_targets(self, pending_mask, selected, targets, ready_reason, cancelled):
        """等待放绳到位；短暂无效反馈给一段宽限，超时才判失败。"""
        reached_mask = 0
        deadline = time.monotonic() + MANUAL_PREPARE_MOTION_TIMEOUT_S
        invalid_since: Optional[float] = None
        motion_reason = ready_reason
        cancelled_during_motion = False
        while time.monotonic() < deadline:
            if cancelled():
                cancelled_during_motion = True
                break
            with self.state_lock:
                state = self.current_state
                online = int(state.servo_online_bitmap)
                unknown = int(state.multi_turn_unknown_bitmap) & pending_mask
                fresh = 0 <= time.monotonic() - state.received_at('servo_abs_last_rx_ts') <= 0.5
                routable = (state.servo_route_controllable_bitmap & pending_mask) == pending_mask
                servo_abs = list(state.servo_angles or [])
            transient = ""
            if not fresh:
                transient = "servo ABS feedback stale"
            elif not routable:
                transient = "servo route not controllable"
            elif unknown:
                transient = f"multi-turn unknown 0x{unknown:05X}"
            elif (online & pending_mask) != pending_mask:
                transient = f"servo offline 0x{online & ~pending_mask & ((1 << MOTOR_COUNT) - 1):05X}"
            if transient:
                now = time.monotonic()
                if invalid_since is None:
                    invalid_since = now
                    motion_reason = transient
                elif now - invalid_since > MANUAL_PREPARE_TRANSIENT_GRACE_S:
                    motion_reason = f"{transient} for >{MANUAL_PREPARE_TRANSIENT_GRACE_S:.1f}s"
                    break
            else:
                invalid_since = None
                if len(servo_abs) >= MOTOR_COUNT:
                    reached_mask = sum(
                        1 << motor_idx for motor_idx in selected
                        if abs(int(servo_abs[motor_idx]) - targets[motor_idx]) <= MANUAL_PREPARE_TARGET_TOLERANCE_COUNTS
                    )
                    if reached_mask == pending_mask:
                        motion_reason = "reached all selected motors"
                        break
            time.sleep(0.05)
        return reached_mask, motion_reason, cancelled_during_motion

    def _release_diagnostic(self, selected, targets):
        """失败时附上前 8 台电机的 当前->目标，方便现场排查。"""
        try:
            with self.state_lock:
                actual = list(self.current_state.servo_angles or [])
            return " ".join(
                f"M{motor_idx:02d}:{actual[motor_idx] if motor_idx < len(actual) else '?'}->{targets[motor_idx]}"
                for motor_idx in selected[:8]
            )
        except Exception:
            return "diagnostics unavailable"

    def _restore_snapshot_manual_references(
        self, snapshot: dict
    ) -> Tuple[bool, str]:
        """把便携 V6 JSON 中有效的 Manual 条目逐项提交到 P4 RAM 并读回核对。"""
        if not self.comm:
            return False, "serial not connected"
        try:
            snapshot_mask = int(
                str(snapshot.get("manual_reference_valid_bitmap", "0")), 0
            ) & ((1 << ENCODER_COUNT) - 1)
            snapshot_min = [
                int(value) for value in snapshot["manual_reference_min_raw"]
            ]
            snapshot_max = [
                int(value) for value in snapshot["manual_reference_max_raw"]
            ]
        except (KeyError, TypeError, ValueError):
            return False, "snapshot Manual endpoints are invalid"
        if (
            snapshot_mask == 0
            or len(snapshot_min) != ENCODER_COUNT
            or len(snapshot_max) != ENCODER_COUNT
        ):
            return False, "snapshot contains no valid Manual endpoints"

        if not self.query_manual_encoder_references(
            MANUAL_REFERENCE_ACK_TIMEOUT_S
        ):
            return False, "failed to read current Manual RAM before restore"
        with self.state_lock:
            current_mask = int(
                getattr(self.current_state, "manual_reference_valid_bitmap", 0)
                or 0
            )
            current_min = list(
                getattr(self.current_state, "manual_reference_min_raw", []) or []
            )
            current_max = list(
                getattr(self.current_state, "manual_reference_max_raw", []) or []
            )

        # 单路命令会在 P4 RAM 中保留其他条目，因此可恢复只有一只手指有效的
        # 部分快照，不会用默认 0 把未标定关节错误标记为有效。
        wrote_any = False
        for joint_idx in range(ENCODER_COUNT):
            if (snapshot_mask & (1 << joint_idx)) == 0:
                continue
            already_matches = (
                (current_mask & (1 << joint_idx)) != 0
                and joint_idx < len(current_min)
                and joint_idx < len(current_max)
                and int(current_min[joint_idx]) == snapshot_min[joint_idx]
                and int(current_max[joint_idx]) == snapshot_max[joint_idx]
            )
            if already_matches:
                continue
            sent_at = time.monotonic()
            self.comm.send_command((
                "manual_encoder_single",
                joint_idx,
                snapshot_min[joint_idx],
                snapshot_max[joint_idx],
            ))
            status = self._ack_status(CMD_MANUAL_ENCODER_SINGLE, MANUAL_REFERENCE_ACK_TIMEOUT_S, sent_at)
            if status not in (PROTO_ACK_STATUS_OK, PROTO_ACK_STATUS_QUEUED):
                return False, f"J{joint_idx:02d} Manual RAM restore rejected"
            if status == PROTO_ACK_STATUS_QUEUED and not self._wait_for_manual_reference_commit_result(sent_at):
                return False, f"J{joint_idx:02d} Manual commit timed out"
            wrote_any = True

        if wrote_any and not self.query_manual_encoder_references(
            MANUAL_REFERENCE_ACK_TIMEOUT_S
        ):
            return False, "failed to read Manual RAM after restore"
        with self.state_lock:
            state = self.current_state
            actual_mask = int(
                getattr(state, "manual_reference_valid_bitmap", 0) or 0
            )
            actual_min = list(
                getattr(state, "manual_reference_min_raw", []) or []
            )
            actual_max = list(
                getattr(state, "manual_reference_max_raw", []) or []
            )
        if (actual_mask & snapshot_mask) != snapshot_mask:
            return False, "P4 Manual RAM readback mask is incomplete"
        for joint_idx in range(ENCODER_COUNT):
            if (snapshot_mask & (1 << joint_idx)) == 0:
                continue
            if (
                joint_idx >= len(actual_min)
                or joint_idx >= len(actual_max)
                or int(actual_min[joint_idx]) != snapshot_min[joint_idx]
                or int(actual_max[joint_idx]) != snapshot_max[joint_idx]
            ):
                return False, f"J{joint_idx:02d} Manual RAM readback mismatch"
        return True, f"restored {snapshot_mask.bit_count()} Manual joint(s) to P4 RAM"

    def restore_manual_references_from_snapshot(
        self, snapshot: dict
    ) -> Tuple[bool, str]:
        """独立恢复便携 JSON 中的 Manual 端点，不让旧 Auto/LUT 阻断基础数据恢复。

        下位机 Flash 关闭后，P4 每次重启都会丢失 Manual RAM。启动恢复必须先
        完成本事务并读回核验，之后才能加载依赖 Manual CRC 的 Auto/LUT。
        """
        try:
            validate_snapshot(snapshot)
        except CalibrationSnapshotError as exc:
            return False, str(exc)
        return self._restore_snapshot_manual_references(snapshot)

    def load_calibration_snapshot(self, snapshot: dict) -> Tuple[bool, str]:
        if not self.comm:
            return False, "serial not connected"
        try:
            mapping_metadata = urdf_manual_scaled_mapping_metadata()
        except (OSError, TypeError, ValueError) as exc:
            return False, f"current URDF mapping is invalid: {exc}"
        validation_error = validate_snapshot_for_load(
            snapshot, mapping_metadata=mapping_metadata
        )
        if validation_error:
            return False, validation_error
        try:
            valid_bitmap = int(str(snapshot.get("motor_abs_limit_valid_bitmap", "0")), 0)
            active_joint_mask = int(str(snapshot.get("active_joint_mask", "0")), 0)
            mins = [int(v) for v in snapshot.get("motor_abs_limit_min", [])]
            maxs = [int(v) for v in snapshot.get("motor_abs_limit_max", [])]
            schema_version = int(snapshot.get("schema_version", 0) or 0)
        except (TypeError, ValueError) as exc:
            return False, f"invalid motor ABS limits: {exc}"
        if schema_version != CALIBRATION_SNAPSHOT_SCHEMA_VERSION:
            return False, f"unsupported snapshot schema v{schema_version}"
        if len(mins) != MOTOR_COUNT or len(maxs) != MOTOR_COUNT:
            return False, f"invalid motor ABS limit count: {len(mins)}/{len(maxs)}"

        joint_results = snapshot.get("joint_calib_results", [])
        if len(joint_results) != ENCODER_COUNT:
            return False, f"invalid joint calibration count: {len(joint_results)}"
        active_joint_mask &= (1 << ENCODER_COUNT) - 1
        if active_joint_mask == 0:
            for joint_idx, result in enumerate(joint_results):
                if isinstance(result, dict) and bool(result.get("success", False)):
                    active_joint_mask |= (1 << joint_idx)

        manual_ok, manual_message = self._restore_snapshot_manual_references(
            snapshot
        )
        if not manual_ok:
            return False, manual_message
        with self.state_lock:
            local_manual_mask = int(
                getattr(self.current_state, "manual_reference_valid_bitmap", 0)
                or 0
            )
            local_manual_min = list(
                getattr(self.current_state, "manual_reference_min_raw", []) or []
            )
            local_manual_max = list(
                getattr(self.current_state, "manual_reference_max_raw", []) or []
            )
            local_manual_committed = bool(
                getattr(self.current_state, "manual_reference_committed", False)
            )
        if not local_manual_committed:
            return False, "P4 Manual RAM image was not committed"
        if (local_manual_mask & active_joint_mask) != active_joint_mask:
            missing_mask = active_joint_mask & ~local_manual_mask
            missing_text = ",".join(
                f"J{idx:02d}"
                for idx in range(ENCODER_COUNT)
                if missing_mask & (1 << idx)
            )
            return False, f"P4 Manual references missing active joints: {missing_text}"
        if (
            len(local_manual_min) < ENCODER_COUNT
            or len(local_manual_max) < ENCODER_COUNT
        ):
            return False, "P4 Manual endpoint readback is incomplete"
        mismatched_manual_crc = [
            joint_idx for joint_idx in range(ENCODER_COUNT)
            if (active_joint_mask & (1 << joint_idx)) and
            int(joint_results[joint_idx].get("manualReferenceCrc", 0) or 0)
            != manual_encoder_reference_entry_crc(
                joint_idx,
                local_manual_min[joint_idx],
                local_manual_max[joint_idx],
            )
        ]
        if mismatched_manual_crc:
            mismatch_text = ",".join(
                f"J{idx:02d}" for idx in mismatched_manual_crc
            )
            return False, (
                f"snapshot Manual CRC mismatch: {mismatch_text}; "
                "re-run automatic calibration for these joints"
            )
        # 标定加载只覆盖快照实际包含的电机，P4 上其他已生效关节保持不变。
        raw_active_motor_mask = snapshot.get("active_motor_mask", "0")
        motor_update_mask = (
            int(raw_active_motor_mask, 0)
            if isinstance(raw_active_motor_mask, str)
            else int(raw_active_motor_mask or 0)
        )
        # 固件对 mask 为 0 的 CALIB_LOAD_LIMITS 回 INVALID_DATA；
        # Manual-only 快照没有电机范围可写，直接跳过。
        if motor_update_mask:
            limits_sent_at = time.monotonic()
            self.comm.send_command(
                ("calib_load_limits", motor_update_mask, valid_bitmap, mins, maxs)
            )
            if self._ack_status(CMD_CALIB_LOAD_LIMITS, CALIB_LOAD_ACK_TIMEOUT_S, limits_sent_at) != PROTO_ACK_STATUS_OK:
                return False, "motor ABS limit load ACK timeout/reject"

        loaded = 0
        for joint_idx, result in enumerate(joint_results):
            if (active_joint_mask & (1 << joint_idx)) == 0:
                continue
            if not isinstance(result, dict) or not bool(result.get("success", False)):
                continue
            payload = dict(result)
            payload["joint"] = joint_idx
            joint_sent_at = time.monotonic()
            self.comm.send_command(("calib_load_joint", payload))
            if self._ack_status(CMD_CALIB_LOAD_JOINT, CALIB_LOAD_ACK_TIMEOUT_S, joint_sent_at) != PROTO_ACK_STATUS_OK:
                return False, f"J{joint_idx:02d} load ACK timeout/reject"
            loaded += 1
        if loaded == 0 and active_joint_mask != 0:
            return False, "no valid joint calibration result in snapshot"
        log_session("CALIB_LOAD", manual_message)
        return True, f"loaded {loaded} joints" if loaded else manual_message

    def start(self):
        if self.comm:
            self.comm.send_command("start")
            # Firmware SYSTEM_STATUS is the source of truth. It will set
            # _started after the state machine reaches a controllable state.
            self._started = False


    def stop(self):
        """撤销待发目标和标定事务，向P4发送STOP。"""
        with self._coordinate_command_lock:
            self._coordinate_cancel_generation += 1
            self._cancel_host_calibration()
            self._host_calibration_overlay = {}
            if self.comm:
                self.comm.send_command("stop")
        with self.state_lock:
            if getattr(self.current_state, "calib_status", None) in ("PENDING", "RUNNING"):
                self.current_state.calib_status = "IDLE"
        self._started = False


    def reset(self):
        """发送显式故障复位指令，不恢复旧目标。"""
        self._cancel_host_calibration()
        self._host_calibration_overlay = {}
        with self._coordinate_command_lock:
            sent = bool(self.comm and self.comm.send_command("reset"))
            if sent:
                self._started = False
        return sent


    def is_started(self) -> bool:
        with self.state_lock:
            has_status = bool(getattr(self.current_state, "has_system_status_data", False))
            last_rx = float(self.current_state.received_at('system_status_last_rx_ts') or 0.0)
        status_fresh = has_status and last_rx > 0.0 and (time.monotonic() - last_rx) <= LOWER_STATUS_STALE_AFTER_S
        return self._started and status_fresh

    def register_update_callback(self, callback):
        self.update_callbacks.append(callback)
