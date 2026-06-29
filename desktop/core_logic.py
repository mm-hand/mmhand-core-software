# core_logic.py - 灵巧手控制器核心逻辑（关节角路数与协议 ENCODER_COUNT 一致，电机路数与 MOTOR_COUNT 一致）

import json
import math
import queue
import threading
import time
from pathlib import Path
from typing import List, Optional, Tuple

from comm_layer import LowerComputerComm, list_ports
from data_models import HandModel
from hand_geometry import load_calib_zero_raw
from protocol import (
    CMD_ACTIVE_SCOPE,
    CMD_CALIB_DATA,
    CMD_CLEAR_CALIBRATION,
    CMD_CALIB_LOAD_JOINT,
    CMD_CALIB_LOAD_LIMITS,
    CMD_MOTOR_ABS_LIMIT_LOCK,
    CMD_MULTITURN_NVS_CONTROL,
    CMD_PAUSE,
    CMD_RESTORE_MULTITURN,
    CMD_SIDE_SWAY_ADJUST,
    ENCODER_COUNT,
    MOTOR_COUNT,
    ControlMode,
    MOTOR_ABS_CMD_MAX,
    MOTOR_ABS_CMD_MIN,
    SYSTEM_STATE_IDLE,
    SYSTEM_STATE_RUN_FEEDBACK,
    SYSTEM_STATE_READY,
    SYSTEM_STATE_RUN_DIRECT,
    SYSTEM_STATE_PAUSED,
)
from session_log import log_session

LEGACY_CALIB_DATA_SYNC_ENABLE = False
LOWER_SYNC_TIMEOUT_S = 2.5
LOWER_STATUS_STALE_AFTER_S = 1.0
LEGACY_CALIB_DATA_ACK_TIMEOUT_S = 1.0
ACTIVE_SCOPE_ACK_TIMEOUT_S = 1.0
MOTOR_ABS_LIMIT_LOCK_ACK_TIMEOUT_S = 1.0
MULTITURN_NVS_CONTROL_ACK_TIMEOUT_S = 1.0
MULTITURN_NVS_CONTROL_MAX_ATTEMPTS = 3
MULTITURN_NVS_CONTROL_RETRY_DELAY_S = 0.15
NEW_DEVICE_RESET_SETTLE_S = 0.30
MULTITURN_RESTORE_ACK_TIMEOUT_S = 1.0
CLEAR_CALIBRATION_ACK_TIMEOUT_S = 1.0
CALIB_LOAD_ACK_TIMEOUT_S = 1.0
MULTITURN_RESTORE_RAW_TOL_COUNTS = 256
MULTITURN_RESTORE_RETRY_S = 0.75
MULTITURN_CACHE_PATH = Path(__file__).resolve().parent / "multiturn_cache.json"
MULTITURN_CACHE_SAVE_INTERVAL_S = 0.5
HOST_MULTITURN_CACHE_ENABLE = False


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

    @staticmethod
    def _is_motor_abs_at_limit(value: int) -> bool:
        margin = 16
        value = int(value)
        return value >= (MOTOR_ABS_CMD_MAX - margin) or value <= (MOTOR_ABS_CMD_MIN + margin)

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
                target = raw_single if raw_single is not None and self._is_motor_abs_at_limit(abs_value) else abs_value
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

        if not self.comm.connect():
            log_session("CTRL", f"serial open failed port={comm_port}")
            self.last_sync_status = "Serial open failed"
            return False

        self.running = True
        self._started = False
        self._update_thread = threading.Thread(target=self._update_loop, daemon=True)
        self._update_thread.start()

        sync_ok = self._wait_for_lower_sync(LOWER_SYNC_TIMEOUT_S)
        legacy_ok = self._sync_legacy_calib_data_if_enabled() if sync_ok else None
        with self.state_lock:
            if self.current_state.calib_status == "SYNCING":
                self.current_state.calib_status = "IDLE"
        if sync_ok:
            if legacy_ok is True:
                self.last_sync_status = "Synced lower status/servo feedback; legacy calib_data ACK OK"
            elif legacy_ok is False:
                self.last_sync_status = "Synced lower status/servo feedback; legacy calib_data ACK timeout"
            else:
                self.last_sync_status = "Synced lower status/servo feedback"
        else:
            self.last_sync_status = "Connected, but lower status/servo feedback sync timed out"
        log_session("CTRL", self.last_sync_status)
        return True

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

    def _sync_legacy_calib_data_if_enabled(self) -> Optional[bool]:
        if not LEGACY_CALIB_DATA_SYNC_ENABLE or not self.comm:
            return None
        zero_raw = load_calib_zero_raw()
        if not zero_raw:
            return None
        try:
            self.comm.send_command(("calib_data", zero_raw))
            ack = self.comm.wait_for_ack(CMD_CALIB_DATA, LEGACY_CALIB_DATA_ACK_TIMEOUT_S)
            if ack is None:
                return False
            with self.state_lock:
                self.current_state.set_encoder_zeros(zero_raw)
            return True
        except Exception as exc:
            print(f"Warning: legacy calib_data sync failed: {exc}")
            return False

    def shutdown(self, stop_lower: bool = True):
        """
        安全关闭控制器：停止下位机、等待线程、清理资源。
        """
        import time
        log_session("CTRL", f"shutdown requested stop_lower={int(bool(stop_lower))}")

        # Step 1: 停止下位机上的运动（即使暂停也要发送 STOP）
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
                    self.current_state.timestamp = new_model.timestamp
                    if getattr(new_model, "has_system_status_data", False):
                        self._started = int(new_model.system_state) in (
                            SYSTEM_STATE_IDLE,
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

    def set_target_angles(self, angles: List[float]):
        """设置 ENCODER_COUNT 路关节目标角（度），JOINT_ANGLE 模式；暂停时不发送"""
        if self._paused:
            return
        if len(angles) != ENCODER_COUNT:
            raise ValueError(f"需要 {ENCODER_COUNT} 个角度值")

        angles = self._validate_angles(angles)
        with self.state_lock:
            self.current_state.set_target_angles(angles)
        if self.comm and self._control_mode == ControlMode.JOINT_ANGLE:
            self.comm.send_command(self.current_state)

    def set_target_angles_live(self, angles: List[float]):
        """高频滑条跟随专用：仅保留最新目标，避免串口队列积压。"""
        if self._paused:
            return
        if len(angles) != ENCODER_COUNT:
            raise ValueError(f"需要 {ENCODER_COUNT} 个角度值")

        angles = self._validate_angles(angles)
        with self.state_lock:
            self.current_state.set_target_angles(angles)
        if self.comm and self._control_mode == ControlMode.JOINT_ANGLE:
            # tuple 命令走 comm_layer 高速通道（latest-only queue）。
            self.comm.send_command(("angle_live", list(angles)))

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
        """多圈绝对位置（CMD_MOTOR_POS_ABS），与下位机 clampServoPos 范围一致。"""
        if self._paused:
            return
        if len(motor_abs) != MOTOR_COUNT:
            raise ValueError(f"需要 {MOTOR_COUNT} 个绝对位置值")
        raw = [max(MOTOR_ABS_CMD_MIN, min(MOTOR_ABS_CMD_MAX, int(v))) for v in motor_abs]
        if self.comm and self._control_mode == ControlMode.DIRECT_MOTOR:
            self.comm.send_command(("motor_pos_abs", raw))

    def set_single_motor_position_absolute(self, motor_index: int, motor_abs: int) -> None:
        if self._paused:
            return
        idx = int(motor_index)
        if idx < 0 or idx >= MOTOR_COUNT:
            raise ValueError(f"motor index out of range: {idx}")
        raw = max(MOTOR_ABS_CMD_MIN, min(MOTOR_ABS_CMD_MAX, int(motor_abs)))
        if self.comm and self._control_mode == ControlMode.DIRECT_MOTOR:
            self.comm.send_command(("motor_pos_abs_one", idx, raw))

    def set_local_test_params(self, freq_hz: float, amplitude_deg: float):
        """设置本地测试参数：频率（Hz）和幅值（度）"""
        if self.comm:
            self.comm.send_command(("local_test_params", freq_hz, amplitude_deg))

    def set_tendon_guard_config(self, config: List[Tuple[bool, int, int]]) -> None:
        if self.comm:
            self.comm.send_command(("tendon_guard", config))

    def start_local_test(self):
        """开始本地测试模式"""
        if self.comm:
            self.comm.send_command("local_test_start")

    def stop_local_test(self):
        """停止本地测试模式"""
        if self.comm:
            self.comm.send_command("local_test_stop")

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

    def request_side_sway_adjust(self) -> bool:
        if not self.comm:
            return False
        self.comm.send_command("side_sway_adjust")
        self._started = False
        return True

    def request_run_feedback(self) -> bool:
        if self._paused or not self.comm:
            return False
        with self.state_lock:
            state = int(getattr(self.current_state, "system_state", -1))
        if state == SYSTEM_STATE_RUN_FEEDBACK:
            return True
        if state != SYSTEM_STATE_READY:
            return False
        self.comm.send_command("run_feedback")
        return True

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
                if status == 0:
                    if not target:
                        self._clear_trusted_multiturn_cache()
                    log_session(
                        "MULTITURN",
                        f"NVS control enabled={int(target)} ACK OK attempt={attempt}",
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

    def reset_multiturn_baseline_for_new_device(self) -> Tuple[bool, str]:
        if not self.comm:
            return False, "serial not connected"
        with self.state_lock:
            state = self.current_state
            has_raw = bool(getattr(state, "has_servo_raw_data", False))
            raw_positions = list(getattr(state, "servo_raw_positions", []) or [])
            raw_online = list(getattr(state, "servo_raw_online", []) or [])
            flash_was_enabled = bool(getattr(state, "multiturn_nvs_enabled", True))

        if not has_raw:
            return False, "waiting for servo raw feedback"

        restore_abs = [0] * MOTOR_COUNT
        restore_mask = 0
        for i in range(MOTOR_COUNT):
            if i >= len(raw_positions) or i >= len(raw_online) or not bool(raw_online[i]):
                continue
            restore_abs[i] = self._raw_single_turn(int(raw_positions[i]))
            restore_mask |= (1 << i)

        if restore_mask == 0:
            return False, "no online motor raw feedback"

        self.stop()
        # Let STOP and a simultaneous servo hotplug burst settle before the
        # Flash command; otherwise the binary ACK can be lost among diagnostics.
        time.sleep(NEW_DEVICE_RESET_SETTLE_S)

        if not self.set_multiturn_nvs_enabled(False):
            detail = self.last_multiturn_nvs_error or "unknown NVS control error"
            rollback = ""
            if flash_was_enabled:
                if self.set_multiturn_nvs_enabled(True):
                    rollback = "; Flash restored to ON after uncertain clear result"
                else:
                    rollback_detail = self.last_multiturn_nvs_error or "unknown NVS control error"
                    rollback = f"; Flash restore also failed: {rollback_detail}"
            return False, f"failed to clear lower Flash snapshots: {detail}{rollback}"

        flash_restore_warning = ""
        if flash_was_enabled and not self.set_multiturn_nvs_enabled(True):
            detail = self.last_multiturn_nvs_error or "unknown NVS control error"
            flash_restore_warning = f"; Flash re-enable failed: {detail}"

        restore_sent_at = time.time()
        self.comm.send_command(("restore_multiturn", restore_mask, restore_abs))
        ack = self.comm.wait_for_ack(
            CMD_RESTORE_MULTITURN,
            MULTITURN_RESTORE_ACK_TIMEOUT_S,
            received_after=restore_sent_at,
        )
        if ack is None or int(ack[2]) != 0:
            return False, "restore turn0 request ACK timeout/reject" + flash_restore_warning

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
            return False, "restore turn0 request accepted, but feedback did not confirm the new baseline"

        clear_sent_at = time.time()
        self.comm.send_command("clear_calibration")
        ack = self.comm.wait_for_ack(
            CMD_CLEAR_CALIBRATION,
            CLEAR_CALIBRATION_ACK_TIMEOUT_S,
            received_after=clear_sent_at,
        )
        if ack is None or int(ack[2]) != 0:
            self._clear_trusted_multiturn_cache()
            return False, "turn0 reset confirmed, but calibration/ABS limit clear ACK timeout/reject"

        self._clear_trusted_multiturn_cache()

        count = bin(restore_mask).count("1")
        flash_text = "Flash ON" if flash_was_enabled and not flash_restore_warning else "Flash OFF"
        return True, f"reset {count} motor(s) to current raw as turn0; cleared calibration/ABS limits; {flash_text}{flash_restore_warning}"

    def load_calibration_snapshot(self, snapshot: dict) -> Tuple[bool, str]:
        if not self.comm:
            return False, "serial not connected"
        try:
            valid_bitmap = int(str(snapshot.get("motor_abs_limit_valid_bitmap", "0")), 0)
            mins = [int(v) for v in snapshot.get("motor_abs_limit_min", [])]
            maxs = [int(v) for v in snapshot.get("motor_abs_limit_max", [])]
        except (TypeError, ValueError) as exc:
            return False, f"invalid motor ABS limits: {exc}"
        if len(mins) != MOTOR_COUNT or len(maxs) != MOTOR_COUNT:
            return False, f"invalid motor ABS limit count: {len(mins)}/{len(maxs)}"

        locked_bitmap = valid_bitmap
        self.comm.send_command(("calib_load_limits", valid_bitmap, locked_bitmap, mins, maxs))
        ack = self.comm.wait_for_ack(CMD_CALIB_LOAD_LIMITS, CALIB_LOAD_ACK_TIMEOUT_S)
        if not ack or int(ack[2]) != 0:
            return False, "motor ABS limit load ACK timeout/reject"

        joint_results = snapshot.get("joint_calib_results", [])
        if len(joint_results) != ENCODER_COUNT:
            return False, f"invalid joint calibration count: {len(joint_results)}"
        loaded = 0
        for joint_idx, result in enumerate(joint_results):
            if not isinstance(result, dict) or not bool(result.get("success", False)):
                continue
            payload = dict(result)
            payload["joint"] = joint_idx
            self.comm.send_command(("calib_load_joint", payload))
            ack = self.comm.wait_for_ack(CMD_CALIB_LOAD_JOINT, CALIB_LOAD_ACK_TIMEOUT_S)
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
