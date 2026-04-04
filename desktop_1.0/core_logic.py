import queue
import threading
import math
from typing import List, Optional

from calibration import run_calibration
from comm_layer import LowerComputerComm, list_ports
from data_models import HandModel
from hand_geometry import has_calib_file, load_calib_zero_raw
from protocol import ENCODER_COUNT, MOTOR_COUNT, ControlMode

class HandController:
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

    def set_control_mode(self, mode: ControlMode):
        self._control_mode = mode

    def get_control_mode(self) -> ControlMode:
        return self._control_mode

    def set_pid_control(self, enabled: bool):
        self._pid_enabled = enabled

    def pause(self):
        self._paused = True

    def resume(self):
        self._paused = False

    def is_paused(self) -> bool:
        return self._paused

    def toggle_pause(self) -> bool:
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
        threading.Thread(target=self._update_loop, daemon=True).start()

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
        self.running = False
        self._started = False
        if self.comm:
            self.comm.disconnect()

    def _update_loop(self):
        while self.running:
            try:
                new_model = self.comm.rx_queue.get(timeout=0.1)
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

    def set_target_angles(self, angles: List[float]):
        if self._paused:
            return
        if len(angles) != ENCODER_COUNT:
            raise ValueError(f"需要 {ENCODER_COUNT} 个角度值")

        angles = self._validate_angles(angles)
        with self.state_lock:
            self.current_state.set_target_angles(angles)
        if self.comm and self._control_mode == ControlMode.JOINT_ANGLE:
            self.comm.send_command(self.current_state)

    def set_motor_positions_raw(self, motor_pos_raw: List[int]):
        if self._paused:
            return
        if len(motor_pos_raw) != MOTOR_COUNT:
            raise ValueError(f"需要 {MOTOR_COUNT} 个编码器原始值")

        raw = [int(v) & 0xFFFF for v in motor_pos_raw]
        if self.comm and self._control_mode == ControlMode.DIRECT_MOTOR:
            self.comm.send_command(("motor_pos", raw))

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
        if self.comm:
            self.comm.send_command("stop")
        self._started = False

    def reset(self):
        if self._paused:
            return
        if self.comm:
            self.comm.send_command("reset")
        self._started = False

    def is_started(self) -> bool:
        return self._started

    def register_update_callback(self, callback):
        self.update_callbacks.append(callback)
