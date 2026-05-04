# core_logic.py - 灵巧手控制器核心逻辑（关节角路数与协议 ENCODER_COUNT 一致，电机路数与 MOTOR_COUNT 一致）

import math
import queue
import threading
from typing import List, Optional

from calibration import run_calibration
from comm_layer import LowerComputerComm, list_ports
from data_models import HandModel
from hand_geometry import has_calib_file, load_calib_zero_raw
from protocol import (
    ENCODER_COUNT,
    MOTOR_COUNT,
    ControlMode,
    MOTOR_ABS_CMD_MAX,
    MOTOR_ABS_CMD_MIN,
)


class HandController:
    """灵巧手控制器：ENCODER_COUNT 路关节目标角，MOTOR_COUNT 路直驱电机原始值。"""

    def __init__(self):
        self.comm: Optional[LowerComputerComm] = None
        self.current_state = HandModel()
        if not has_calib_file():
            self.current_state.calib_status = "NO_CALIB"

        self.state_lock = threading.Lock()
        self.running = False
        self.update_callbacks: List[callable] = []
        self._pid_enabled = False
        self._paused = False
        self._control_mode = ControlMode.JOINT_ANGLE
        self._started = False
        self._update_thread: Optional[threading.Thread] = None

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
        """暂停：停止向下位机发送控制指令（数据接收照常）"""
        self._paused = True

    def resume(self):
        """恢复：重新允许发送控制指令"""
        self._paused = False

    def is_paused(self) -> bool:
        return self._paused

    def toggle_pause(self) -> bool:
        """切换暂停状态，返回当前是否已暂停"""
        self._paused = not self._paused
        return self._paused

    def initialize(self, comm_port: str) -> bool:
        try:
            self.comm = LowerComputerComm(comm_port)
        except (ValueError, RuntimeError) as exc:
            print(f"连接失败: {exc}")
            return False

        if not self.comm.connect():
            return False

        self.running = True
        self._started = False
        self._update_thread = threading.Thread(target=self._update_loop, daemon=True)
        self._update_thread.start()

        # 连接成功后下发 calib_deg.json 中的编码器零点
        zero_raw = load_calib_zero_raw()
        if zero_raw:
            self.comm.send_command(("calib_data", zero_raw))
            with self.state_lock:
                self.current_state.set_encoder_zeros(zero_raw)
        else:
            with self.state_lock:
                self.current_state.calib_status = "NO_CALIB"
        return True

    def shutdown(self):
        """
        安全关闭控制器：停止下位机、等待线程、清理资源。
        """
        import time
        
        # Step 1: 停止下位机上的运动（即使暂停也要发送 STOP）
        if self.comm:
            try:
                self.comm.send_command("stop")
                time.sleep(0.15)  # 等待硬件响应
            except Exception as e:
                print(f"Warning: Failed to send stop command to lower computer: {e}")
        
        # Step 2: 标记停止运行
        self.running = False
        self._started = False
        
        # Step 3: 断开连接（停止 RX/TX 线程）
        if self.comm:
            try:
                self.comm.disconnect()
            except Exception as e:
                print(f"Warning: Failed to disconnect: {e}")
            time.sleep(0.1)  # 等待连接完全断开
        
        # Step 4: 等待 _update_loop 线程完全退出
        if self._update_thread and self._update_thread.is_alive():
            self._update_thread.join(timeout=2.0)
            if self._update_thread.is_alive():
                print("Warning: _update_thread did not exit within timeout")
        self._update_thread = None
        
        # Step 5: 清理资源
        with self.state_lock:
            self.update_callbacks.clear()  # 移除所有回调
        self._paused = False  # 恢复暂停状态
        
        # Step 6: 重置内部状态为断开连接状态
        with self.state_lock:
            self.current_state.calib_status = "DISCONNECTED"

    def _update_loop(self):
        while self.running:
            try:
                if not self.comm:
                    break
                # Keep only the latest RX frame to avoid UI latency caused by
                # backlog when producer rate is higher than consumer rate.
                new_model = self.comm.rx_queue.get(timeout=0.1)
                while True:
                    try:
                        newer = self.comm.rx_queue.get_nowait()
                        new_model = newer
                    except queue.Empty:
                        break
                with self.state_lock:
                    self.current_state.copy_state_from(new_model)
                    self.current_state.calib_status = new_model.calib_status
                    self.current_state.tactile_data = new_model.tactile_data
                    self.current_state.timestamp = new_model.timestamp
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

    def set_tendon_guard_config(self, configs: List[tuple]) -> None:
        """
        下发腱绳 guard 配置到下位机（每关节: enabled, sign, x1_abs）。
        该配置属于控制参数，不依赖当前控制模式。
        """
        if not self.comm:
            return
        if len(configs) != ENCODER_COUNT:
            raise ValueError(f"需要 {ENCODER_COUNT} 组 tendon guard 配置")
        self.comm.send_command(("tendon_guard", list(configs)))

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

    def calibrate(self, progress_cb=None, done_cb=None):
        """
        上位机主导的校准：逐关节寻零，保存至 calib_deg.json 并下发下位机。
        progress_cb(current, total, msg)、done_cb(success, zero_raw_or_none)。
        """
        if self._paused:
            return
        if not self.comm:
            if done_cb:
                done_cb(False, None)
            return

        def _run():
            result = run_calibration(self, progress_cb=progress_cb)
            if done_cb:
                done_cb(result is not None, result)

        threading.Thread(target=_run, daemon=True).start()

    def start(self):
        if self._paused:
            return
        if self.comm:
            self.comm.send_command("start")
            self._started = True

    def stop(self):
        """停止：暂停时也可发送，便于紧急停"""
        if self.comm:
            self.comm.send_command("stop")
        self._started = False

    def reset(self):
        """发送重置指令；暂停时不发送"""
        if self._paused:
            return
        if self.comm:
            self.comm.send_command("reset")
        self._started = False

    def is_started(self) -> bool:
        return self._started

    def register_update_callback(self, callback):
        self.update_callbacks.append(callback)
