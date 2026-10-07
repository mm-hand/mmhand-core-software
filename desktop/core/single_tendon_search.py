"""Single tendon mechanical search migrated from SingleMotorCalibrationMachine.

职责：依据已授予的标定许可和新鲜反馈推进单腱机械端点搜索。
线程：只由标定专用工作线程串行调用，不创建线程或阻塞等待。
输入：会话许可、采样时间、方向坐标、舵机 ABS 和负载。
输出：标定目标以及待拟合端点与样本，不直接安装或保存结果。
状态所有权：本对象唯一持有当前关节搜索阶段与停稳窗口。
禁止依赖：禁止依赖 Qt、串口和整机状态机，不能自行授权运动。
"""

from dataclasses import dataclass
from typing import Callable

from statemachine import State, StateMachine


@dataclass(frozen=True)
class SingleSearchParameters:
    startup_wait_ms: int = 1200
    servo_max_age_ms: int = 500
    encoder_max_age_ms: int = 300
    target_ack_ms: int = 100
    speed_counts_per_second: int = 400
    loop_ms: int = 20
    window_ms: int = 200
    window_servo_counts: int = 50
    stall_degrees: float = 0.75
    tighten_load: int = 250
    release_max_counts: int = 8192
    release_follow_degrees: float = 12.0
    sync_target_tolerance_counts: int = 40
    sync_settle_counts: int = 20
    stable_samples: int = 2
    fit_step_degrees: float = 5.0


@dataclass(frozen=True)
class SearchFeedback:
    received_ms: int
    encoder_sample_ms: int
    servo_sample_ms: int
    encoder_oriented: int
    servo_abs: int
    load_magnitude: int | None
    permission_valid: bool = False


@dataclass(frozen=True)
class MechanicalEndpoints:
    tight_encoder: int
    tight_abs: int
    released_encoder: int
    released_abs: int
    samples: tuple[tuple[float, int, int], ...]


class SingleTendonSearch(StateMachine):
    """Event-driven mechanical search; firmware remains the permission authority."""

    waiting = State(initial=True)
    tightening = State()
    settling = State()
    releasing = State()
    completed = State(final=True)
    failed = State(final=True)
    cancelled = State(final=True)

    feedback_ready = waiting.to(tightening)
    tight_found = tightening.to(settling)
    settled = settling.to(releasing)
    release_found = releasing.to(completed)
    fail = (waiting.to(failed) | tightening.to(failed) |
            settling.to(failed) | releasing.to(failed))
    cancel = (waiting.to(cancelled) | tightening.to(cancelled) |
              settling.to(cancelled) | releasing.to(cancelled))

    def __init__(self, *, started_ms: int, direction: int,
                 abs_range: tuple[int, int], publish: Callable[[int], bool],
                 revoke: Callable[[], None], parameters=SingleSearchParameters()):
        if direction not in (-1, 1) or abs_range[0] >= abs_range[1]:
            raise ValueError("invalid calibration direction or ABS range")
        self.parameters = parameters
        self.started_ms = started_ms
        self.direction = direction
        self.abs_range = abs_range
        self.publish = publish
        self.revoke = revoke
        self.failure_reason = ""
        self.result = None
        self._last_feedback_ms = -1
        self._stable = 0
        self._samples = []
        self._next_fit_degrees = 0.0
        self._remainder = 0
        self._revoked = False
        super().__init__()

    def _finish(self):
        if not self._revoked:
            self._revoked = True
            self.revoke()

    def on_enter_failed(self):
        self._finish()

    def on_enter_cancelled(self):
        self._finish()

    def on_enter_completed(self):
        self._finish()

    def _reject(self, reason):
        self.failure_reason = reason
        self.fail()

    @staticmethod
    def _travel(first, second):
        return abs((second - first + 8192) % 16384 - 8192) * 360.0 / 16384

    def _window(self, feedback):
        self._window_ms = feedback.received_ms
        self._window_abs = feedback.servo_abs
        self._window_encoder = feedback.encoder_oriented

    def _send(self, target):
        if not self.abs_range[0] <= target <= self.abs_range[1]:
            self._reject("target outside calibration session range")
            return False
        try:
            accepted = self.publish(target)
        except Exception as exc:
            self._reject(f"calibration target publish failed: {exc}")
            return False
        if not accepted:
            self._reject("calibration target rejected")
            return False
        return True

    def tick(self, now_ms: int):
        """Called by the worker clock even when feedback stops arriving."""
        if self.current_state.final:
            return
        if self.current_state == self.waiting:
            if now_ms - self.started_ms >= self.parameters.startup_wait_ms:
                self._reject("initial feedback timeout")
        elif now_ms - self._last_feedback_ms > min(
                self.parameters.servo_max_age_ms, self.parameters.encoder_max_age_ms):
            self._reject("feedback lost")

    def observe(self, feedback: SearchFeedback):
        if self.current_state.final:
            return
        if not feedback.permission_valid:
            self._reject("calibration permission lost")
            return
        params = self.parameters
        now = feedback.received_ms
        fresh = (0 <= now - feedback.servo_sample_ms <= params.servo_max_age_ms
                 and 0 <= now - feedback.encoder_sample_ms <= params.encoder_max_age_ms)
        if not fresh:
            if self.current_state == self.waiting:
                self.tick(now)
            else:
                self._reject("stale encoder or servo feedback")
            return
        if now <= self._last_feedback_ms:
            return
        self._last_feedback_ms = now
        if self.current_state == self.waiting:
            self._command_abs = feedback.servo_abs
            self._command_ms = now
            self._window(feedback)
            self.feedback_ready()
            return
        if self.current_state == self.settling:
            if abs(feedback.servo_abs - self._hold_abs) > params.sync_target_tolerance_counts:
                self._hold_abs = feedback.servo_abs
                self._stable = 0
                self._window(feedback)
            if not self._send(self._hold_abs):
                return
            if now - self._window_ms < params.window_ms:
                return
            stable = abs(feedback.servo_abs - self._window_abs) <= params.sync_settle_counts
            self._stable = self._stable + 1 if stable else 0
            self._window(feedback)
            if self._stable >= params.stable_samples:
                self._command_abs = feedback.servo_abs
                self._command_ms = now
                self._stable = 0
                self._remainder = 0
                self.settled()
            return
        releasing = self.current_state == self.releasing
        if releasing:
            actual_release = self.direction * (self._tight_abs - feedback.servo_abs)
            if actual_release > params.release_max_counts:
                self._reject("single tendon release exceeded two turns")
                return
            travel = self._travel(self._tight_encoder, feedback.encoder_oriented)
            if travel + 0.1 >= self._next_fit_degrees:
                while travel + 0.1 >= self._next_fit_degrees:
                    self._next_fit_degrees += params.fit_step_degrees
                fit_sample = (travel, feedback.encoder_oriented, feedback.servo_abs)
                if not self._samples or fit_sample[1:] != self._samples[-1][1:]:
                    self._samples.append(fit_sample)
        if now - self._window_ms >= params.window_ms:
            moving = abs(feedback.servo_abs - self._window_abs) >= params.window_servo_counts
            stalled = self._travel(self._window_encoder, feedback.encoder_oriented) <= params.stall_degrees
            self._window(feedback)
            if not releasing:
                if (moving and stalled and feedback.load_magnitude is not None
                        and feedback.load_magnitude >= params.tighten_load):
                    self._tight_encoder = feedback.encoder_oriented
                    self._tight_abs = feedback.servo_abs
                    self._hold_abs = feedback.servo_abs
                    self._samples = [(0.0, self._tight_encoder, self._tight_abs)]
                    self.tight_found()
                    self._send(self._hold_abs)
                    return
            else:
                self._stable = self._stable + 1 if stalled else 0
                if moving and self._stable >= params.stable_samples and travel >= params.release_follow_degrees:
                    self._samples.append((travel, feedback.encoder_oriented, feedback.servo_abs))
                    self.result = MechanicalEndpoints(self._tight_encoder, self._tight_abs,
                        feedback.encoder_oriented, feedback.servo_abs, tuple(self._samples))
                    self.release_found()
                    return
        elapsed = now - self._command_ms
        if elapsed >= params.loop_ms:
            counts, self._remainder = divmod(elapsed * params.speed_counts_per_second + self._remainder, 1000)
            target = self._command_abs + self.direction * counts * (-1 if releasing else 1)
            if releasing and self.direction * (self._tight_abs - target) > params.release_max_counts:
                self._reject("release target would exceed two turns")
                return
            if counts and self._send(target):
                self._command_abs = target
            self._command_ms = now
