"""core_logic.py - MMHand Desktop 模块契约。

职责：封装面向业务的下位机命令序列和基础控制流程。
线程：除 comm_layer 的串口收发线程外不主动创建 GUI 线程；共享状态必须按既有锁读取。
输入：P4 字节流、上层类型化命令、配置和持久化数据。
输出：线程安全的 HandModel 快照、协议帧、ACK 和后端操作结果。
状态所有权：通信对象拥有串口与解析缓存；HandModel 的写入只允许在核心同步边界内发生。
禁止依赖：禁止依赖 Qt/UI、app、ROS2 或 Replay 页面，禁止把控件和展示规则带入核心层。
"""
# core_logic.py - 灵巧手控制器核心逻辑（关节角路数与协议 ENCODER_COUNT 一致，电机路数与 MOTOR_COUNT 一致）

import json
import math
import queue
import threading
import time
from typing import List, Optional, Tuple

from desktop.paths import PACKAGE_DIR

from .angle_mapping import get_spec, urdf_manual_scaled_mapping_metadata
from .calibration_snapshot import (
    CALIBRATION_SNAPSHOT_SCHEMA_VERSION,
    CalibrationSnapshotError,
    validate_snapshot,
)
from .comm_layer import LowerComputerComm, list_ports
from .data_models import HandModel
from .motor_mapping import MOTOR_TO_JOINT_INDEX
from .protocol import (
    CMD_ACTIVE_SCOPE,
    CMD_CLEAR_CALIBRATION,
    CMD_CLEAR_CALIBRATION_SCOPE,
    CMD_CALIB_LOAD_JOINT,
    CMD_CALIB_LOAD_LIMITS,
    CMD_MOTOR_ABS_LIMIT_LOCK,
    CMD_MULTITURN_NVS_CONTROL,
    CMD_MANUAL_ENCODER_REFERENCE,
    CMD_MANUAL_ENCODER_QUERY,
    CMD_MANUAL_ENCODER_SINGLE,
    CMD_ROPE_INIT,
    CMD_PAUSE,
    CMD_RESET,
    CMD_RESTORE_MULTITURN,
    CMD_SERVO_RESET_ZERO,
    DUAL_TENDON_JOINT_INDICES,
    TENDON_LUT_MAX_SAMPLES,
    ENCODER_COUNT,
    MOTOR_COUNT,
    PROTO_ACK_STATUS_OK,
    PROTO_ACK_STATUS_QUEUED,
    ControlMode,
    MOTOR_ABS_CMD_MAX,
    MOTOR_ABS_CMD_MIN,
    SYSTEM_STATE_IDLE,
    SYSTEM_STATE_RECOVERY_REQUIRED,
    SYSTEM_STATE_RUN_FEEDBACK,
    SYSTEM_STATE_READY,
    SYSTEM_STATE_RUN_DIRECT,
    SYSTEM_STATE_PAUSED,
    STATE_SYNC_ALL,
    manual_encoder_reference_crc,
    manual_encoder_reference_entry_crc,
    tendon_lut_field,
)
from .session_log import log_session

LOWER_SYNC_TIMEOUT_S = 2.5
LOWER_STATUS_STALE_AFTER_S = 1.0
ACTIVE_SCOPE_ACK_TIMEOUT_S = 1.0
MOTOR_ABS_LIMIT_LOCK_ACK_TIMEOUT_S = 1.0
MULTITURN_NVS_CONTROL_ACK_TIMEOUT_S = 1.0
MULTITURN_NVS_CONTROL_RESULT_TIMEOUT_S = 2.0
MULTITURN_NVS_CONTROL_MAX_ATTEMPTS = 3
MULTITURN_NVS_CONTROL_RETRY_DELAY_S = 0.15
NEW_DEVICE_RESET_SETTLE_S = 0.30
NEW_DEVICE_RESET_RESET_ACK_TIMEOUT_S = 1.0
MULTITURN_RESTORE_ACK_TIMEOUT_S = 1.0
CLEAR_CALIBRATION_ACK_TIMEOUT_S = 1.0
CALIB_LOAD_ACK_TIMEOUT_S = 1.0
MANUAL_REFERENCE_ACK_TIMEOUT_S = 2.0
MULTITURN_RESTORE_RAW_TOL_COUNTS = 256
MULTITURN_RESTORE_RETRY_S = 0.75
MULTITURN_NVS_SAVE_CONFIRM_TIMEOUT_S = 2.0
SERVO_RESET_ZERO_ACK_TIMEOUT_S = 1.5
SERVO_RESET_ZERO_RESULT_TIMEOUT_S = 4.0
MULTITURN_CACHE_PATH = PACKAGE_DIR / "multiturn_cache.json"
MULTITURN_CACHE_SAVE_INTERVAL_S = 0.5
HOST_MULTITURN_CACHE_ENABLE = False


def _joint_calib_result_has_valid_tendon_lut(
    item: Optional[dict], joint_idx: int
) -> bool:
    if not isinstance(item, dict) or not bool(item.get("success", False)):
        return False
    if not bool(tendon_lut_field(item, "tendonLutValid", False)):
        return False
    try:
        count = int(tendon_lut_field(item, "tendonLutCount", 0))
    except (TypeError, ValueError):
        return False
    if count < 3 or count > TENDON_LUT_MAX_SAMPLES:
        return False
    expected_count = 2 if int(joint_idx) in DUAL_TENDON_JOINT_INDICES else 1
    motors = tendon_lut_field(item, "tendonLutMotor", [])
    if not isinstance(motors, list) or len(motors) < 2:
        return False
    try:
        expected_motors = [
            motor_idx for motor_idx, mapped_joint in enumerate(MOTOR_TO_JOINT_INDEX)
            if int(mapped_joint) == int(joint_idx)
        ]
        if len(expected_motors) != expected_count:
            return False
        if any(int(motors[i]) != expected_motors[i] for i in range(expected_count)):
            return False
        if expected_count == 1 and int(motors[1]) != 0xFF:
            return False
    except (TypeError, ValueError):
        return False
    motor_abs = tendon_lut_field(item, "tendonLutMotorAbs", [])
    if not isinstance(motor_abs, list) or len(motor_abs) < 2:
        return False
    for row in motor_abs[:expected_count]:
        if not isinstance(row, list) or len(row) < count:
            return False
    return True


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
        self.comm: Optional[LowerComputerComm] = None
        self.current_state = HandModel()
        self.state_lock = threading.Lock()
        self.running = False
        self.update_callbacks: List[callable] = []
        self._pid_enabled = False
        self._paused = False
        self._control_mode = ControlMode.JOINT_ANGLE
        self._started = False
        self._update_thread: Optional[threading.Thread] = None
        self.last_sync_status = "Not connected"
        self.last_multiturn_nvs_error = ""
        self._trusted_servo_abs: List[Optional[int]] = [None] * MOTOR_COUNT
        self._trusted_servo_raw: List[Optional[int]] = [None] * MOTOR_COUNT
        self._trusted_servo_online: List[bool] = [False] * MOTOR_COUNT
        self._last_multiturn_restore_attempt_s = 0.0
        self._last_multiturn_cache_save_s = 0.0
        # Host-side multi-turn persistence is intentionally disabled.
        # P4 NVS is the only trusted cross-power-cycle multi-turn source.
        self._clear_trusted_multiturn_cache()

    @staticmethod
    def _clamp_motor_abs(value: int) -> int:
        return max(MOTOR_ABS_CMD_MIN, min(MOTOR_ABS_CMD_MAX, int(value)))

    def _clamp_motor_abs_for_current_session(
        self, motor_index: int, value: int
    ) -> int:
        """按单台舵机当前上电会话的动态 logical 行程钳位直控目标。"""
        idx = int(motor_index)
        target = self._clamp_motor_abs(value)
        with self.state_lock:
            state = self.current_state
            valid_bitmap = int(
                getattr(state, "servo_logical_range_valid_bitmap", 0) or 0
            )
            mins = list(getattr(state, "servo_logical_range_min", []) or [])
            maxs = list(getattr(state, "servo_logical_range_max", []) or [])
        if (
            0 <= idx < MOTOR_COUNT
            and (valid_bitmap & (1 << idx)) != 0
            and idx < len(mins)
            and idx < len(maxs)
            and int(mins[idx]) <= int(maxs[idx])
        ):
            target = max(int(mins[idx]), min(int(maxs[idx]), target))
        return target

    @staticmethod
    def _raw_single_turn(value: int) -> int:
        return max(0, min(4095, int(value) % 4096))

    @staticmethod
    def _single_turn_diff_counts(a: int, b: int) -> int:
        diff = abs(int(a) % 4096 - int(b) % 4096) % 4096
        return min(diff, 4096 - diff)

    def _load_trusted_multiturn_cache(self) -> None:
        if not HOST_MULTITURN_CACHE_ENABLE:
            return
        try:
            with MULTITURN_CACHE_PATH.open("r", encoding="utf-8") as fh:
                data = json.load(fh)
            abs_values = data.get("abs", [])
            raw_values = data.get("raw", [])
            online_values = data.get("online", [])
            for i in range(MOTOR_COUNT):
                if i >= len(abs_values) or i >= len(raw_values) or i >= len(online_values):
                    continue
                if not bool(online_values[i]):
                    continue
                self._trusted_servo_abs[i] = self._clamp_motor_abs(int(abs_values[i]))
                self._trusted_servo_raw[i] = self._raw_single_turn(int(raw_values[i]))
                self._trusted_servo_online[i] = True
            log_session("MULTITURN", f"cache loaded {MULTITURN_CACHE_PATH}")
            print(f"[MULTITURN CACHE] loaded {MULTITURN_CACHE_PATH}")
        except FileNotFoundError:
            return
        except Exception as exc:
            log_session("MULTITURN", f"cache load failed: {exc}")
            print(f"[MULTITURN CACHE] load failed: {exc}")

    def _clear_trusted_multiturn_cache(self) -> None:
        self._trusted_servo_abs = [None] * MOTOR_COUNT
        self._trusted_servo_raw = [None] * MOTOR_COUNT
        self._trusted_servo_online = [False] * MOTOR_COUNT
        self._last_multiturn_restore_attempt_s = 0.0
        self._last_multiturn_cache_save_s = 0.0
        for path in (MULTITURN_CACHE_PATH, MULTITURN_CACHE_PATH.with_suffix(".tmp")):
            try:
                path.unlink()
            except FileNotFoundError:
                pass
            except Exception as exc:
                log_session("MULTITURN", f"cache clear failed for {path}: {exc}")
                print(f"[MULTITURN CACHE] clear failed for {path}: {exc}")
        log_session("MULTITURN", "cache cleared")
        print("[MULTITURN CACHE] cleared")

    def _save_trusted_multiturn_cache(self, force: bool = False) -> None:
        if not HOST_MULTITURN_CACHE_ENABLE:
            return
        now = time.time()
        if not force and now - self._last_multiturn_cache_save_s < MULTITURN_CACHE_SAVE_INTERVAL_S:
            return
        try:
            data = {
                "timestamp": now,
                "abs": [0 if v is None else int(v) for v in self._trusted_servo_abs],
                "raw": [0 if v is None else int(v) for v in self._trusted_servo_raw],
                "online": [bool(v) for v in self._trusted_servo_online],
            }
            tmp_path = MULTITURN_CACHE_PATH.with_suffix(".tmp")
            with tmp_path.open("w", encoding="utf-8") as fh:
                json.dump(data, fh, ensure_ascii=False, indent=2)
            tmp_path.replace(MULTITURN_CACHE_PATH)
            self._last_multiturn_cache_save_s = now
        except Exception as exc:
            log_session("MULTITURN", f"cache save failed: {exc}")
            print(f"[MULTITURN CACHE] save failed: {exc}")

    def _update_trusted_multiturn_cache(self, state: HandModel) -> None:
        if not HOST_MULTITURN_CACHE_ENABLE:
            return
        if not bool(getattr(state, "multiturn_nvs_enabled", True)):
            if any(self._trusted_servo_online) or MULTITURN_CACHE_PATH.exists() or MULTITURN_CACHE_PATH.with_suffix(".tmp").exists():
                self._clear_trusted_multiturn_cache()
            return
        unknown = int(getattr(state, "multi_turn_unknown_bitmap", 0))
        if not (getattr(state, "has_servo_angle_data", False) and getattr(state, "has_servo_raw_data", False)):
            return
        updated = False
        for i in range(MOTOR_COUNT):
            bit = 1 << i
            if (unknown & bit) != 0:
                continue
            if i >= len(state.servo_online) or i >= len(state.servo_raw_online):
                continue
            if not (state.servo_online[i] and state.servo_raw_online[i]):
                continue
            abs_value = self._clamp_motor_abs(state.servo_angles[i])
            raw_value = self._raw_single_turn(state.servo_raw_positions[i])
            if self._trusted_servo_abs[i] != abs_value or self._trusted_servo_raw[i] != raw_value:
                updated = True
            self._trusted_servo_abs[i] = abs_value
            self._trusted_servo_raw[i] = raw_value
            self._trusted_servo_online[i] = True
        if updated:
            self._save_trusted_multiturn_cache()

    def _maybe_restore_multiturn_from_cache(self, state: HandModel) -> None:
        if not HOST_MULTITURN_CACHE_ENABLE:
            return
        if not self.comm:
            return
        if not bool(getattr(state, "multiturn_nvs_enabled", True)):
            return
        unknown = int(getattr(state, "multi_turn_unknown_bitmap", 0))
        if unknown == 0 or not getattr(state, "has_servo_raw_data", False):
            return
        now = time.time()
        if now - self._last_multiturn_restore_attempt_s < MULTITURN_RESTORE_RETRY_S:
            return

        restore_abs = [0] * MOTOR_COUNT
        restore_mask = 0
        for i in range(MOTOR_COUNT):
            bit = 1 << i
            if (unknown & bit) == 0:
                continue
            if i >= len(state.servo_raw_online) or not state.servo_raw_online[i]:
                continue
            if not self._trusted_servo_online[i]:
                continue
            last_abs = self._trusted_servo_abs[i]
            last_raw = self._trusted_servo_raw[i]
            if last_abs is None or last_raw is None:
                continue
            current_raw = self._raw_single_turn(state.servo_raw_positions[i])
            if self._single_turn_diff_counts(current_raw, last_raw) > MULTITURN_RESTORE_RAW_TOL_COUNTS:
                continue
            turn = int(round((int(last_abs) - int(last_raw)) / 4096.0))
            restore_abs[i] = self._clamp_motor_abs(turn * 4096 + current_raw)
            restore_mask |= bit

        if restore_mask == 0:
            return
        self._last_multiturn_restore_attempt_s = now
        self.comm.send_command(("restore_multiturn", restore_mask, restore_abs))
        log_session("MULTITURN", f"restore request mask=0x{restore_mask:08X}")
        print(f"[MULTITURN RESTORE] request mask=0x{restore_mask:08X}")

    def _safe_motor_hold_targets_from_state(self) -> Optional[List[int]]:
        with self.state_lock:
            state = self.current_state
            has_abs = bool(getattr(state, "has_servo_angle_data", False))
            has_raw = bool(getattr(state, "has_servo_raw_data", False))
            unknown = int(getattr(state, "multi_turn_unknown_bitmap", 0))
            servo_abs = list(getattr(state, "servo_angles", []) or [])
            servo_online = list(getattr(state, "servo_online", []) or [])
            servo_raw = list(getattr(state, "servo_raw_positions", []) or [])
            servo_raw_online = list(getattr(state, "servo_raw_online", []) or [])

        if unknown != 0:
            log_session("MULTITURN", f"hold skipped unknown=0x{unknown:08X}")
            print(f"[MULTITURN HOLD] skipped: unknown=0x{unknown:08X}")
            return None

        if not has_abs and not has_raw:
            return None

        targets: List[int] = []
        any_feedback = False
        for i in range(MOTOR_COUNT):
            target: Optional[int] = None
            raw_single: Optional[int] = None
            if has_raw and i < len(servo_raw) and i < len(servo_raw_online) and servo_raw_online[i]:
                raw_single = self._raw_single_turn(servo_raw[i])
            if has_abs and i < len(servo_abs) and i < len(servo_online) and servo_online[i]:
                abs_value = self._clamp_motor_abs(servo_abs[i])
                # logical ABS 已使用 int32 且由 P4 上报动态行程，不再把合法的
                # 会话边界误认为旧 int16 饱和值并退回单圈 raw。
                target = abs_value
            elif raw_single is not None:
                target = raw_single

            if target is None:
                targets.append(0)
            else:
                targets.append(self._clamp_motor_abs(target))
                any_feedback = True
        return targets if any_feedback else None

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

    def set_control_mode(self, mode: ControlMode):
        """设置电机控制模式：JOINT_ANGLE 或 DIRECT_MOTOR"""
        self._control_mode = mode

    def get_control_mode(self) -> ControlMode:
        return self._control_mode

    def set_pid_control(self, enabled: bool):
        self._pid_enabled = enabled

    def invalidate_joint_commands(self) -> int:
        """清除发送队列中待发的关节控制指令（角度包/angle_live）。"""
        if not self.comm:
            return 0
        try:
            return int(self.comm.invalidate_pending_joint_commands())
        except Exception:
            return 0

    def pause(self):
        """Pause firmware output without clearing the cached target."""
        if self.comm:
            self.comm.send_command("pause")
        self._paused = True
        self._started = False

    def resume(self):
        """Resume a firmware PAUSED state with the regular START command."""
        if self.comm:
            self.comm.send_command("start")
        self._paused = False

    def is_paused(self) -> bool:
        return self._paused

    def toggle_pause(self) -> bool:
        """切换暂停状态，返回当前是否已暂停"""
        self._paused = not self._paused
        return self._paused

    def initialize(self, comm_port: str) -> bool:
        self._reset_local_runtime_state("SYNCING")
        log_session("CTRL", f"initialize requested port={comm_port}")
        try:
            self.comm = LowerComputerComm(comm_port)
        except (ValueError, RuntimeError) as exc:
            log_session("CTRL", f"initialize failed before connect: {exc}")
            print(f"连接失败: {exc}")
            self.last_sync_status = str(exc)
            return False

        try:
            connected = self.comm.connect()
        except (ValueError, RuntimeError) as exc:
            log_session("CTRL", f"serial open failed port={comm_port}: {exc}")
            print(f"连接失败: {exc}")
            self.last_sync_status = str(exc)
            return False

        if not connected:
            log_session("CTRL", f"serial open failed port={comm_port}")
            self.last_sync_status = "Serial open failed"
            return False

        self.running = True
        self._started = False
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
            self.last_sync_status = "Connected, but lower status/servo feedback sync timed out"
        log_session("CTRL", self.last_sync_status)
        return True

    def request_state_sync(self, sync_mask: int = STATE_SYNC_ALL) -> bool:
        """请求 P4 回传静态状态、标定快照、路由和产品指纹。"""
        if not self.comm:
            return False
        return bool(self.comm.send_command(("state_sync", int(sync_mask))))

    def set_task_diag_subscription(self, enabled: bool) -> bool:
        """仅在 Task Runtime 窗口打开期间订阅高频任务诊断。"""
        if not self.comm:
            return False
        return bool(self.comm.send_command(("task_diag_subscribe", bool(enabled))))

    def _reset_local_runtime_state(self, calib_status: str = "IDLE") -> None:
        with self.state_lock:
            self.current_state = HandModel()
            self.current_state.calib_status = str(calib_status)
        self._started = False
        self._pid_enabled = False
        self._paused = False
        self._control_mode = ControlMode.JOINT_ANGLE

    def _wait_for_lower_sync(self, timeout_s: float) -> bool:
        deadline = time.time() + max(0.0, float(timeout_s))
        while time.time() < deadline:
            with self.state_lock:
                state = self.current_state
                has_status = bool(getattr(state, "has_system_status_data", False))
                has_servo = bool(getattr(state, "has_servo_angle_data", False)) or bool(
                    getattr(state, "has_servo_raw_data", False)
                )
            if has_status and has_servo:
                return True
            time.sleep(0.02)
        return False

    def shutdown(self, stop_lower: bool = False):
        """
        关闭串口与后台线程。

        普通退出默认不发送锁存式 STOP：断开后由 P4 的上位机租约超时执行
        当前 ABS Hold。只有用户显式要求急停时才传入 stop_lower=True。
        """
        import time
        log_session("CTRL", f"shutdown requested stop_lower={int(bool(stop_lower))}")

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
        self._paused = False  # 恢复暂停状态
        
        # Step 6: 重置内部状态为断开连接状态
        with self.state_lock:
            self.current_state.calib_status = "DISCONNECTED"

        log_session("CTRL", "shutdown complete")

    def _update_loop(self):
        while self.running:
            try:
                if not self.comm:
                    break
                new_model = self.comm.rx_queue.get(timeout=0.1)
                with self.state_lock:
                    # Host-side multi-turn cache restore is disabled.
                    # P4 NVS is the only trusted persistence/restore source.
                    self.current_state.copy_state_from(new_model)
                    self.current_state.calib_status = new_model.calib_status
                    self.current_state.tactile_data = new_model.tactile_data
                    self.current_state.host_rx_time = new_model.host_rx_time
                    self.current_state.p4_time_ms = new_model.p4_time_ms
                    self.current_state.has_p4_time = new_model.has_p4_time
                    if getattr(new_model, "has_system_status_data", False):
                        self._started = int(new_model.system_state) in (
                            SYSTEM_STATE_IDLE,
                            SYSTEM_STATE_RECOVERY_REQUIRED,
                            SYSTEM_STATE_RUN_FEEDBACK,
                            SYSTEM_STATE_READY,
                            SYSTEM_STATE_RUN_DIRECT,
                        )
                        self._paused = int(new_model.system_state) == SYSTEM_STATE_PAUSED
                for cb in self.update_callbacks:
                    try:
                        cb(self.current_state)
                    except Exception:
                        pass
            except queue.Empty:
                continue
            except (AttributeError, Exception) as e:
                if self.running:
                    print(f"Update loop caught exception: {e}")
                break

    def set_target_angles(self, angles: List[float]) -> bool:
        """设置 ENCODER_COUNT 路关节目标角（度），JOINT_ANGLE 模式；暂停时不发送"""
        if self._paused:
            return False
        if len(angles) != ENCODER_COUNT:
            raise ValueError(f"需要 {ENCODER_COUNT} 个角度值")

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
        if self._paused:
            return False
        if len(angles) != ENCODER_COUNT:
            raise ValueError(f"需要 {ENCODER_COUNT} 个角度值")

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
        if self._paused or not self.comm or self._control_mode != ControlMode.JOINT_ANGLE:
            return False
        if len(angles) != ENCODER_COUNT:
            raise ValueError(f"需要 {ENCODER_COUNT} 个角度值")
        validated = self._validate_angles(angles)
        with self.state_lock:
            self.current_state.set_target_angles(validated)
        if joint_mask is not None:
            return bool(self.comm.send_command(
                ("joint_target_run_masked", list(validated), int(joint_mask))
            ))
        return bool(self.comm.send_command(("joint_target_run", list(validated))))

    def set_motor_positions_raw(self, motor_pos_raw: List[int]):
        """设置 MOTOR_COUNT 路电机目标编码器原始值，DIRECT_MOTOR 模式；暂停时不发送"""
        if self._paused:
            return
        if len(motor_pos_raw) != MOTOR_COUNT:
            raise ValueError(f"需要 {MOTOR_COUNT} 个编码器原始值")

        raw = [int(v) & 0xFFFF for v in motor_pos_raw]
        if self.comm and self._control_mode == ControlMode.DIRECT_MOTOR:
            self.comm.send_command(("motor_pos", raw))

    def set_motor_positions_raw_sweep(self, motor_pos_raw: List[int]):
        """设置 MOTOR_COUNT 路电机滑条扫动目标，DIRECT_MOTOR 模式；暂停时不发送"""
        if self._paused:
            return
        if len(motor_pos_raw) != MOTOR_COUNT:
            raise ValueError(f"需要 {MOTOR_COUNT} 个编码器原始值")

        raw = [int(v) & 0xFFFF for v in motor_pos_raw]
        if self.comm and self._control_mode == ControlMode.DIRECT_MOTOR:
            self.comm.send_command(("motor_pos_sweep", raw))

    def set_motor_positions_absolute(self, motor_abs: List[int]) -> None:
        """下发27路 logical ABS；每路先按 P4 当前会话动态行程独立钳位。"""
        if self._paused:
            return
        if len(motor_abs) != MOTOR_COUNT:
            raise ValueError(f"需要 {MOTOR_COUNT} 个绝对位置值")
        raw = [
            self._clamp_motor_abs_for_current_session(i, value)
            for i, value in enumerate(motor_abs)
        ]
        if self.comm and self._control_mode == ControlMode.DIRECT_MOTOR:
            self.comm.send_command(("motor_pos_abs", raw))

    def set_single_motor_position_absolute(self, motor_index: int, motor_abs: int) -> None:
        if self._paused:
            return
        idx = int(motor_index)
        if idx < 0 or idx >= MOTOR_COUNT:
            raise ValueError(f"motor index out of range: {idx}")
        raw = self._clamp_motor_abs_for_current_session(idx, motor_abs)
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

    def request_firmware_calibration(self) -> bool:
        if not self.comm:
            return False
        self.comm.send_command("calibrate")
        self._started = False
        return True

    def request_run_feedback(self) -> bool:
        if self._paused or not self.comm:
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
    ) -> bool:
        """Persist all 21 Manual endpoint pairs and verify the P4 readback."""
        if not self.comm:
            return False
        sent_at = time.time()
        self.comm.send_command((
            "manual_encoder_reference",
            list(manual_min_raw),
            list(manual_max_raw),
        ))
        ack = self.comm.wait_for_ack(
            CMD_MANUAL_ENCODER_REFERENCE,
            MANUAL_REFERENCE_ACK_TIMEOUT_S,
            received_after=sent_at,
        )
        if ack is None or int(ack[2]) not in (
            PROTO_ACK_STATUS_OK,
            PROTO_ACK_STATUS_QUEUED,
        ):
            return False
        if (
            int(ack[2]) == PROTO_ACK_STATUS_QUEUED
            and not self._wait_for_manual_reference_save_result(sent_at)
        ):
            return False
        expected_mask = (1 << ENCODER_COUNT) - 1
        expected_crc = manual_encoder_reference_crc(
            manual_min_raw, manual_max_raw
        )
        return self._verify_manual_encoder_readback(
            expected_mask,
            list(manual_min_raw),
            list(manual_max_raw),
            expected_crc,
        )

    def query_manual_encoder_references(self, timeout_s: float = 2.0) -> bool:
        if not self.comm:
            return False
        sent_at = time.time()
        self.comm.send_command("query_manual_encoder_reference")
        ack = self.comm.wait_for_ack(
            CMD_MANUAL_ENCODER_QUERY,
            timeout_s,
            received_after=sent_at,
        )
        if ack is None or int(ack[2]) != 0:
            return False
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            with self.state_lock:
                received_at = float(
                    getattr(
                        self.current_state,
                        "manual_reference_last_rx_time",
                        0.0,
                    )
                    or 0.0
                )
            if received_at >= sent_at:
                return True
            time.sleep(0.01)
        return False

    def _wait_for_manual_reference_save_result(
        self,
        received_after: float,
        timeout_s: float = MANUAL_REFERENCE_ACK_TIMEOUT_S,
    ) -> bool:
        """等待异步 Manual NVS 事务的结果包，避免把 QUEUED 当成写入完成。"""
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            with self.state_lock:
                state = self.current_state
                received_at = float(
                    getattr(state, "manual_reference_last_rx_time", 0.0) or 0.0
                )
                save_status = int(
                    getattr(state, "manual_reference_save_status", 0) or 0
                )
            if received_at >= received_after and save_status != 0:
                return save_status == 1
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
            nvs_valid = bool(
                getattr(state, "manual_reference_nvs_valid", False)
            )
            save_status = int(
                getattr(state, "manual_reference_save_status", 0) or 0
            )
        if actual_mask != (int(expected_mask) & ((1 << ENCODER_COUNT) - 1)):
            return False
        if actual_crc != (int(expected_crc) & 0xFFFFFFFF):
            return False
        if not nvs_valid or save_status != 1:
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
        self, joint: int, manual_min_raw: int, manual_max_raw: int
    ) -> bool:
        """Write one joint while preserving all existing Manual entries."""
        joint = int(joint)
        if not 0 <= joint < ENCODER_COUNT or not self.comm:
            return False
        if not self.query_manual_encoder_references(
            MANUAL_REFERENCE_ACK_TIMEOUT_S
        ):
            return False
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
        sent_at = time.time()
        self.comm.send_command((
            "manual_encoder_single",
            joint,
            int(manual_min_raw),
            int(manual_max_raw),
        ))
        ack = self.comm.wait_for_ack(
            CMD_MANUAL_ENCODER_SINGLE,
            MANUAL_REFERENCE_ACK_TIMEOUT_S,
            received_after=sent_at,
        )
        if ack is None or int(ack[2]) not in (
            PROTO_ACK_STATUS_OK,
            PROTO_ACK_STATUS_QUEUED,
        ):
            return False
        if (
            int(ack[2]) == PROTO_ACK_STATUS_QUEUED
            and not self._wait_for_manual_reference_save_result(sent_at)
        ):
            return False
        expected_crc = manual_encoder_reference_crc(expected_min, expected_max)
        return self._verify_manual_encoder_readback(
            expected_mask,
            expected_min,
            expected_max,
            expected_crc,
        )

    def rope_init(self, active_joint_mask: int) -> bool:
        if not self.comm:
            return False
        sent_at = time.time()
        self.comm.send_command(("rope_init", int(active_joint_mask)))
        ack = self.comm.wait_for_ack(
            CMD_ROPE_INIT,
            MANUAL_REFERENCE_ACK_TIMEOUT_S,
            received_after=sent_at,
        )
        return bool(ack is not None and int(ack[2]) in (0, 2))

    def clear_manual_encoder_references(self) -> bool:
        """Clear both Manual NVS slots and close the firmware motion gate."""
        if not self.comm:
            return False
        sent_at = time.time()
        self.comm.send_command("clear_manual_encoder_reference")
        ack = self.comm.wait_for_ack(
            CMD_MANUAL_ENCODER_REFERENCE,
            MANUAL_REFERENCE_ACK_TIMEOUT_S,
            received_after=sent_at,
        )
        if ack is None or int(ack[2]) not in (
            PROTO_ACK_STATUS_OK,
            PROTO_ACK_STATUS_QUEUED,
        ):
            return False
        if (
            int(ack[2]) == PROTO_ACK_STATUS_QUEUED
            and not self._wait_for_manual_reference_save_result(sent_at)
        ):
            return False
        if not self.query_manual_encoder_references(
            MANUAL_REFERENCE_ACK_TIMEOUT_S
        ):
            return False
        with self.state_lock:
            state = self.current_state
            return (
                int(getattr(state, "manual_reference_valid_bitmap", 0) or 0)
                == 0
                and not bool(
                    getattr(state, "manual_reference_nvs_valid", False)
                )
            )

    def set_active_scope(self, active_motor_mask: int, active_joint_mask: int, require_can_feedback: bool) -> bool:
        if not self.comm:
            return False
        self.comm.send_command((
            "active_scope",
            int(active_motor_mask) & 0xFFFFFFFF,
            int(active_joint_mask) & 0xFFFFFFFF,
            bool(require_can_feedback),
        ))
        ack = self.comm.wait_for_ack(CMD_ACTIVE_SCOPE, ACTIVE_SCOPE_ACK_TIMEOUT_S)
        return bool(ack is not None and int(ack[2]) == 0)

    def set_motor_abs_limit_lock(self, active_motor_mask: int, locked: bool) -> bool:
        if not self.comm:
            return False
        self.comm.send_command((
            "motor_abs_limit_lock",
            int(active_motor_mask) & 0xFFFFFFFF,
            bool(locked),
        ))
        ack = self.comm.wait_for_ack(CMD_MOTOR_ABS_LIMIT_LOCK, MOTOR_ABS_LIMIT_LOCK_ACK_TIMEOUT_S)
        return bool(ack is not None and int(ack[2]) == 0)

    def set_multiturn_nvs_enabled(self, enabled: bool) -> bool:
        if not self.comm:
            self.last_multiturn_nvs_error = "serial not connected"
            return False
        self.last_multiturn_nvs_error = ""
        target = bool(enabled)
        for attempt in range(1, MULTITURN_NVS_CONTROL_MAX_ATTEMPTS + 1):
            sent_at = time.time()
            self.comm.send_command(("multiturn_nvs_control", target))
            ack = self.comm.wait_for_ack(
                CMD_MULTITURN_NVS_CONTROL,
                MULTITURN_NVS_CONTROL_ACK_TIMEOUT_S,
                received_after=sent_at,
            )
            if ack is not None:
                status = int(ack[2])
                if status in (PROTO_ACK_STATUS_OK, PROTO_ACK_STATUS_QUEUED):
                    if (
                        status == PROTO_ACK_STATUS_QUEUED
                        and not self._wait_for_multiturn_nvs_control_result(
                            target,
                            sent_at,
                        )
                    ):
                        self.last_multiturn_nvs_error = (
                            "firmware queued command but final Flash state was not confirmed"
                        )
                        log_session(
                            "MULTITURN",
                            f"NVS control enabled={int(target)} result timeout attempt={attempt}",
                        )
                        return False
                    if not target:
                        self._clear_trusted_multiturn_cache()
                    log_session(
                        "MULTITURN",
                        f"NVS control enabled={int(target)} confirmed attempt={attempt}",
                    )
                    return True
                self.last_multiturn_nvs_error = f"firmware rejected command (status={status})"
                log_session(
                    "MULTITURN",
                    f"NVS control enabled={int(target)} rejected status={status} attempt={attempt}",
                )
                return False
            log_session(
                "MULTITURN",
                f"NVS control enabled={int(target)} ACK timeout attempt={attempt}",
            )
            if attempt < MULTITURN_NVS_CONTROL_MAX_ATTEMPTS:
                time.sleep(MULTITURN_NVS_CONTROL_RETRY_DELAY_S)

        self.last_multiturn_nvs_error = (
            f"no ACK after {MULTITURN_NVS_CONTROL_MAX_ATTEMPTS} attempts; "
            "wait for servo hotplug recovery or update P4 firmware"
        )
        return False

    def _wait_for_multiturn_nvs_control_result(
        self,
        target: bool,
        received_after: float,
        timeout_s: float = MULTITURN_NVS_CONTROL_RESULT_TIMEOUT_S,
    ) -> bool:
        """等待 P4 在 Flash 事务完成后回传与请求一致的系统状态。"""
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            with self.state_lock:
                state = self.current_state
                has_status = bool(
                    getattr(state, "has_system_status_data", False)
                )
                received_at = float(
                    getattr(state, "system_status_last_rx_ts", 0.0) or 0.0
                )
                actual = bool(
                    getattr(state, "multiturn_nvs_enabled", True)
                )
            if has_status and received_at >= received_after and actual == bool(target):
                return True
            time.sleep(0.01)
        return False

    def _wait_for_multiturn_nvs_save(
        self,
        baseline_request_token: int,
        baseline_fail_count: int,
        required_mask: int,
        timeout_s: float = MULTITURN_NVS_SAVE_CONFIRM_TIMEOUT_S,
    ) -> Tuple[bool, str]:
        deadline = time.time() + timeout_s
        required_mask = int(required_mask)
        while time.time() < deadline:
            with self.state_lock:
                state = self.current_state
                has_status = bool(getattr(state, "has_multiturn_nvs_status", False))
                request_token = int(getattr(state, "multiturn_nvs_save_request_token", 0) or 0)
                result_token = int(getattr(state, "multiturn_nvs_save_result_token", 0) or 0)
                valid_mask = int(getattr(state, "multiturn_nvs_save_valid_mask", 0) or 0)
                fail_count = int(getattr(state, "multiturn_nvs_save_fail_count", 0) or 0)
                snapshot_valid = bool(getattr(state, "multiturn_nvs_snapshot_valid", False))

            if not has_status:
                return False, "firmware does not report NVS save status; update P4 firmware"
            if fail_count > baseline_fail_count:
                return False, f"NVS save failed; fail_count {baseline_fail_count}->{fail_count}"
            if request_token > baseline_request_token and result_token >= request_token:
                if snapshot_valid and (valid_mask & required_mask) == required_mask:
                    return True, f"NVS snapshot saved token={result_token} mask=0x{valid_mask:08X}"
                return False, (
                    f"NVS save completed but mask invalid: "
                    f"valid=0x{valid_mask:08X} required=0x{required_mask:08X}"
                )
            time.sleep(0.02)

        return False, "NVS save confirmation timeout"

    @staticmethod
    def _joint_mask_for_motor_mask(motor_mask: int) -> int:
        """由桌面映射预览受影响关节；P4 仍会用 HardwareMap 再次校验。"""
        joint_mask = 0
        for motor_idx, joint_idx in enumerate(MOTOR_TO_JOINT_INDEX[:MOTOR_COUNT]):
            if (int(motor_mask) & (1 << motor_idx)) == 0:
                continue
            if 0 <= int(joint_idx) < ENCODER_COUNT:
                joint_mask |= 1 << int(joint_idx)
        return joint_mask & ((1 << ENCODER_COUNT) - 1)

    def reset_multiturn_baseline_for_new_device(
        self,
    ) -> Tuple[bool, str, int, int]:
        """仅修复在线且 Turn Unknown 的舵机，并失效其所属关节标定。"""
        if not self.comm:
            return False, "serial not connected", 0, 0
        with self.state_lock:
            state = self.current_state
            has_raw = bool(getattr(state, "has_servo_raw_data", False))
            raw_positions = list(getattr(state, "servo_raw_positions", []) or [])
            raw_online = list(getattr(state, "servo_raw_online", []) or [])
            flash_was_enabled = bool(getattr(state, "multiturn_nvs_enabled", True))
            unknown_mask = (
                int(getattr(state, "multi_turn_unknown_bitmap", 0) or 0)
                & ((1 << MOTOR_COUNT) - 1)
            )

        if unknown_mask == 0:
            return (
                False,
                "all motor turns are already trusted; New Device Reset is not allowed",
                0,
                0,
            )

        if not has_raw:
            return False, "waiting for servo raw feedback", 0, 0

        restore_abs = [0] * MOTOR_COUNT
        restore_mask = 0
        for i in range(MOTOR_COUNT):
            bit = 1 << i
            if (
                (unknown_mask & bit) == 0
                or i >= len(raw_positions)
                or i >= len(raw_online)
                or not bool(raw_online[i])
            ):
                continue
            restore_abs[i] = self._raw_single_turn(int(raw_positions[i]))
            restore_mask |= bit

        if restore_mask == 0:
            return False, "Turn Unknown motors are offline; reconnect servo power first", 0, 0

        affected_joint_mask = self._joint_mask_for_motor_mask(restore_mask)
        if affected_joint_mask == 0:
            return False, "cannot map Turn Unknown motors to joints", 0, 0

        # New Device Reset 是维护恢复，不是用户急停。这里发送 RESET 会撤销
        # 旧 owner/目标、关闭扭矩并清除可恢复锁存；若发送 STOP，则
        # emergency_stop_latched 会一直保留，导致紧接着执行的全手标定被拒绝。
        reset_sent_at = time.time()
        self.reset()
        reset_ack = self.comm.wait_for_ack(
            CMD_RESET,
            NEW_DEVICE_RESET_RESET_ACK_TIMEOUT_S,
            received_after=reset_sent_at,
        )
        if (
            reset_ack is None
            or int(reset_ack[2]) not in (
                PROTO_ACK_STATUS_OK,
                PROTO_ACK_STATUS_QUEUED,
            )
        ):
            return (
                False,
                "maintenance reset ACK timeout/reject before turn0 restore",
                restore_mask,
                affected_joint_mask,
            )

        # 等待 RESET 状态切换和热插拔配置事务稳定，再发送恢复命令；
        # 否则二进制 ACK 可能与舵机重连诊断日志同时到达，造成误判超时。
        time.sleep(NEW_DEVICE_RESET_SETTLE_S)

        with self.state_lock:
            baseline_save_request_token = int(getattr(self.current_state, "multiturn_nvs_save_request_token", 0) or 0)
            baseline_save_fail_count = int(getattr(self.current_state, "multiturn_nvs_save_fail_count", 0) or 0)

        # 热插拔恢复需要先关闭扭矩并重新配置舵机多圈模式。最多重试数次，
        # 但每次仍由 P4 硬门禁确认“在线 + configured + Turn Unknown”。
        ack = None
        for _attempt in range(5):
            restore_sent_at = time.time()
            self.comm.send_command(("restore_multiturn", restore_mask, restore_abs))
            ack = self.comm.wait_for_ack(
                CMD_RESTORE_MULTITURN,
                MULTITURN_RESTORE_ACK_TIMEOUT_S,
                received_after=restore_sent_at,
            )
            if ack is not None and int(ack[2]) == 0:
                break
            time.sleep(0.20)
        if ack is None or int(ack[2]) != 0:
            return (
                False,
                "scoped turn0 restore rejected; wait for servo hotplug recovery/configuration",
                restore_mask,
                affected_joint_mask,
            )

        verified = False
        deadline = time.time() + 1.5
        while time.time() < deadline:
            with self.state_lock:
                state = self.current_state
                unknown = int(getattr(state, "multi_turn_unknown_bitmap", 0) or 0)
                has_abs = bool(getattr(state, "has_servo_angle_data", False))
                servo_abs = list(getattr(state, "servo_angles", []) or [])
                servo_online = list(getattr(state, "servo_online", []) or [])
            if (unknown & restore_mask) == 0 and has_abs:
                all_match = True
                for i in range(MOTOR_COUNT):
                    if ((restore_mask >> i) & 0x01) == 0:
                        continue
                    if i >= len(servo_abs) or i >= len(servo_online) or not bool(servo_online[i]):
                        all_match = False
                        break
                    if int(servo_abs[i]) != int(restore_abs[i]):
                        all_match = False
                        break
                if all_match:
                    verified = True
                    break
            time.sleep(0.02)
        if not verified:
            self._clear_trusted_multiturn_cache()
            return (
                False,
                "restore turn0 request accepted, but feedback did not confirm the new baseline",
                restore_mask,
                affected_joint_mask,
            )

        nvs_save_message = ""
        if flash_was_enabled:
            save_ok, save_message = self._wait_for_multiturn_nvs_save(
                baseline_save_request_token,
                baseline_save_fail_count,
                restore_mask,
            )
            if not save_ok:
                self._clear_trusted_multiturn_cache()
                return (
                    False,
                    f"turn0 reset confirmed, but Flash snapshot was not saved: {save_message}",
                    restore_mask,
                    affected_joint_mask,
                )
            nvs_save_message = f"; {save_message}"
        else:
            nvs_save_message = "; Flash OFF, new turn0 exists only in P4 RAM"

        clear_sent_at = time.time()
        self.comm.send_command(("clear_calibration_scope", restore_mask))
        ack = self.comm.wait_for_ack(
            CMD_CLEAR_CALIBRATION_SCOPE,
            CLEAR_CALIBRATION_ACK_TIMEOUT_S,
            received_after=clear_sent_at,
        )
        if ack is None or int(ack[2]) != 0:
            self._clear_trusted_multiturn_cache()
            return (
                False,
                "turn0 reset confirmed, but scoped calibration clear ACK timeout/reject",
                restore_mask,
                affected_joint_mask,
            )

        self._clear_trusted_multiturn_cache()

        count = bin(restore_mask).count("1")
        flash_text = "Flash ON" if flash_was_enabled else "Flash OFF"
        return True, (
            f"reset {count} Turn Unknown motor(s) to current raw as turn0; "
            f"invalidated only affected joint calibration/ABS limits; "
            f"{flash_text}{nvs_save_message}"
        ), restore_mask, affected_joint_mask

    def reset_overflow_servo_turns(
        self,
    ) -> Tuple[bool, str, int, int, int]:
        """重置硬件 ABS 严格越过 ±30715 的舵机，并等待 P4 完成 Flash 闭环。"""
        if not self.comm:
            return False, "serial not connected", 0, 0, 0

        with self.state_lock:
            state = self.current_state
            if not bool(getattr(state, "has_servo_reset_zero_status", False)):
                return (
                    False,
                    "P4 firmware does not report Reset Zero status; update firmware first",
                    0,
                    0,
                    0,
                )
            overflow_mask = (
                int(getattr(state, "servo_turn_overflow_bitmap", 0) or 0)
                & ((1 << MOTOR_COUNT) - 1)
            )
            flash_enabled = bool(getattr(state, "multiturn_nvs_enabled", True))
            baseline_result_token = int(
                getattr(state, "servo_reset_zero_result_token", 0) or 0
            )
            reset_status = int(
                getattr(state, "servo_reset_zero_status", 0) or 0
            )

        if overflow_mask == 0:
            return False, "no servo hardware ABS exceeds ±30715", 0, 0, 0
        if not flash_enabled:
            return (
                False,
                "Flash is OFF; enable Flash before Reset Zero so the new turn0 can be persisted",
                0,
                0,
                0,
            )
        if reset_status == 1:
            return False, "another Reset Zero transaction is running", 0, 0, 0

        self.stop()
        time.sleep(NEW_DEVICE_RESET_SETTLE_S)
        sent_at = time.time()
        self.comm.send_command(("servo_reset_zero", overflow_mask))
        ack = self.comm.wait_for_ack(
            CMD_SERVO_RESET_ZERO,
            SERVO_RESET_ZERO_ACK_TIMEOUT_S,
            received_after=sent_at,
        )
        if ack is None or int(ack[2]) not in (0, 2):
            status = "timeout" if ack is None else str(int(ack[2]))
            return (
                False,
                f"Reset Zero command rejected (status={status})",
                0,
                overflow_mask,
                0,
            )

        # ACK 仅表示事务已受理。必须等 result token 前进后再读取成功掩码，
        # 避免 UI 把“硬件已清零、Flash 尚未替换”误认为完整成功。
        deadline = time.time() + SERVO_RESET_ZERO_RESULT_TIMEOUT_S
        while time.time() < deadline:
            with self.state_lock:
                state = self.current_state
                result_token = int(
                    getattr(state, "servo_reset_zero_result_token", 0) or 0
                )
                success_mask = int(
                    getattr(state, "servo_reset_zero_success_mask", 0) or 0
                ) & ((1 << MOTOR_COUNT) - 1)
                failed_mask = int(
                    getattr(state, "servo_reset_zero_failed_mask", 0) or 0
                ) & ((1 << MOTOR_COUNT) - 1)
                invalid_joint_mask = int(
                    getattr(
                        state,
                        "servo_reset_zero_invalidated_joint_mask",
                        0,
                    )
                    or 0
                ) & ((1 << ENCODER_COUNT) - 1)
                status = int(
                    getattr(state, "servo_reset_zero_status", 0) or 0
                )
            if result_token != baseline_result_token and status in (2, 3):
                self._clear_trusted_multiturn_cache()
                completed = success_mask & overflow_mask
                missing = overflow_mask & ~completed
                if status == 2 and failed_mask == 0 and missing == 0:
                    return (
                        True,
                        f"Reset Zero completed for {completed.bit_count()} motor(s)",
                        completed,
                        0,
                        invalid_joint_mask,
                    )
                failed = failed_mask | missing
                return (
                    False,
                    "some servos were reset but the transaction was not fully persisted",
                    completed,
                    failed,
                    invalid_joint_mask,
                )
            time.sleep(0.02)

        return (
            False,
            "Reset Zero ACK received, but result status timed out",
            0,
            overflow_mask,
            0,
        )

    def load_calibration_snapshot(self, snapshot: dict) -> Tuple[bool, str]:
        if not self.comm:
            return False, "serial not connected"
        try:
            validate_snapshot(snapshot)
        except CalibrationSnapshotError as exc:
            return False, str(exc)
        try:
            valid_bitmap = int(str(snapshot.get("motor_abs_limit_valid_bitmap", "0")), 0)
            active_joint_mask = int(str(snapshot.get("active_joint_mask", "0")), 0)
            mins = [int(v) for v in snapshot.get("motor_abs_limit_min", [])]
            maxs = [int(v) for v in snapshot.get("motor_abs_limit_max", [])]
            schema_version = int(snapshot.get("schema_version", 0) or 0)
            locked_bitmap = int(
                str(snapshot.get("motor_abs_limit_locked_bitmap", "0")), 0
            )
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
        missing_joints = [
            joint_idx for joint_idx in range(ENCODER_COUNT)
            if (active_joint_mask & (1 << joint_idx)) and (
                not isinstance(joint_results[joint_idx], dict) or
                not bool(joint_results[joint_idx].get("success", False))
            )
        ]
        if missing_joints:
            missing_text = ",".join(f"J{idx:02d}" for idx in missing_joints)
            return False, f"snapshot missing active joint calibration: {missing_text}"
        try:
            expected_mapping_version, expected_urdf_crc = (
                urdf_manual_scaled_mapping_metadata()
            )
        except (OSError, TypeError, ValueError) as exc:
            return False, f"current URDF mapping is invalid: {exc}"
        incompatible_mapping = [
            joint_idx for joint_idx in range(ENCODER_COUNT)
            if (active_joint_mask & (1 << joint_idx)) and (
                int(joint_results[joint_idx].get("mappingVersion", 0) or 0)
                != expected_mapping_version
                or int(joint_results[joint_idx].get("urdfConfigCrc", 0) or 0)
                != expected_urdf_crc
            )
        ]
        if incompatible_mapping:
            mapping_text = ",".join(f"J{idx:02d}" for idx in incompatible_mapping)
            return False, (
                f"snapshot URDF mapping mismatch: {mapping_text}; "
                f"expected v{expected_mapping_version}/0x{expected_urdf_crc:08X}"
            )

        # V4 的四个范围字段必须全部等于当前 URDF Min/Max。这里在发送前
        # 拒绝旧动态 Min 快照，P4 仍会使用本机 Manual 再计算一次形成双重门禁。
        incompatible_ranges = []
        for joint_idx in range(ENCODER_COUNT):
            if (active_joint_mask & (1 << joint_idx)) == 0:
                continue
            result = joint_results[joint_idx]
            urdf_min, urdf_max = get_spec(joint_idx).urdf_range_deg
            try:
                values = (
                    float(result.get("angleMin")),
                    float(result.get("currentMinDeg")),
                    float(result.get("angleMax")),
                    float(result.get("currentMaxDeg")),
                )
            except (TypeError, ValueError):
                incompatible_ranges.append(joint_idx)
                continue
            if (
                not all(math.isfinite(value) for value in values)
                or abs(values[0] - urdf_min) > 0.01
                or abs(values[1] - urdf_min) > 0.01
                or abs(values[2] - urdf_max) > 0.01
                or abs(values[3] - urdf_max) > 0.01
            ):
                incompatible_ranges.append(joint_idx)
        if incompatible_ranges:
            range_text = ",".join(f"J{idx:02d}" for idx in incompatible_ranges)
            return False, (
                f"snapshot V4 range mismatch: {range_text}; "
                "re-run automatic calibration with the current firmware"
            )

        if not self.query_manual_encoder_references(
            MANUAL_REFERENCE_ACK_TIMEOUT_S
        ):
            return False, "failed to read current Manual references from P4"
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
            local_manual_nvs_valid = bool(
                getattr(self.current_state, "manual_reference_nvs_valid", False)
            )
        if not local_manual_nvs_valid:
            return False, "P4 Manual references are not persisted"
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
        missing_tendon_lut = [
            joint_idx for joint_idx in range(ENCODER_COUNT)
            if (active_joint_mask & (1 << joint_idx)) and
            not _joint_calib_result_has_valid_tendon_lut(
                joint_results[joint_idx], joint_idx
            )
        ]
        if missing_tendon_lut:
            missing_text = ",".join(f"J{idx:02d}" for idx in missing_tendon_lut)
            return False, f"snapshot missing tendon LUT: {missing_text}"

        locked_bitmap &= valid_bitmap
        limits_sent_at = time.time()
        # 标定加载只覆盖快照实际包含的电机，P4 上其他已生效关节保持不变。
        self.comm.send_command(
            (
                "calib_load_limits",
                int(snapshot.get("active_motor_mask", "0"), 0)
                if isinstance(snapshot.get("active_motor_mask"), str)
                else int(snapshot.get("active_motor_mask", 0) or 0),
                valid_bitmap,
                locked_bitmap,
                mins,
                maxs,
            )
        )
        ack = self.comm.wait_for_ack(
            CMD_CALIB_LOAD_LIMITS,
            CALIB_LOAD_ACK_TIMEOUT_S,
            received_after=limits_sent_at,
        )
        if not ack or int(ack[2]) != 0:
            return False, "motor ABS limit load ACK timeout/reject"

        loaded = 0
        for joint_idx, result in enumerate(joint_results):
            if (active_joint_mask & (1 << joint_idx)) == 0:
                continue
            if not isinstance(result, dict) or not bool(result.get("success", False)):
                continue
            payload = dict(result)
            payload["joint"] = joint_idx
            joint_sent_at = time.time()
            self.comm.send_command(("calib_load_joint", payload))
            ack = self.comm.wait_for_ack(
                CMD_CALIB_LOAD_JOINT,
                CALIB_LOAD_ACK_TIMEOUT_S,
                received_after=joint_sent_at,
            )
            if not ack or int(ack[2]) != 0:
                return False, f"J{joint_idx:02d} load ACK timeout/reject"
            loaded += 1
        if loaded == 0:
            return False, "no valid joint calibration result in snapshot"
        return True, f"loaded {loaded} joints"

    def start(self):
        if self.comm:
            self.comm.send_command("start")
            # Firmware SYSTEM_STATUS is the source of truth. It will set
            # _started after the state machine reaches a controllable state.
            self._started = False
            self._paused = False

    def stop(self):
        """停止：暂停时也可发送，便于紧急停"""
        if self.comm:
            self.comm.send_command("stop")
        with self.state_lock:
            if getattr(self.current_state, "calib_status", None) in ("PENDING", "RUNNING"):
                self.current_state.calib_status = "IDLE"
        self._started = False
        self._paused = False

    def reset(self):
        """发送重置指令；暂停时不发送"""
        if self.comm:
            self.comm.send_command("reset")
        self._started = False
        self._paused = False

    def is_started(self) -> bool:
        with self.state_lock:
            has_status = bool(getattr(self.current_state, "has_system_status_data", False))
            last_rx = float(getattr(self.current_state, "system_status_last_rx_ts", 0.0) or 0.0)
        status_fresh = has_status and last_rx > 0.0 and (time.time() - last_rx) <= LOWER_STATUS_STALE_AFTER_S
        return self._started and status_fresh

    def register_update_callback(self, callback):
        self.update_callbacks.append(callback)
