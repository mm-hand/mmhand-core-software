"""按CSV原始时间轴逐帧播放完整遥操作动作。"""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Callable, List, Optional, Tuple

from desktop.core.angle_mapping import get_spec, urdf_manual_scaled_mapping_metadata
from desktop.core.data_models import resolve_joint_workspace_range
from desktop.core.hand_backend import HandBackend
from desktop.core.protocol import (
    ENCODER_COUNT,
    SYSTEM_STATE_READY,
    SYSTEM_STATE_RUN_FEEDBACK,
)
from desktop.teleop.joint_target_adapter import active_mapping_identity

from .replay_models import ReplayTrace


# 等待下一原始帧期间仍按50 ms检查一次Stop和P4运行状态。
REPLAY_WAIT_POLL_SECONDS = 0.05
# 不能为了追赶时间轴而静默跳过姿态；调度严重落后时直接中止并提示用户重试。
REPLAY_MAX_SCHEDULING_LAG_SECONDS = 0.25
# 20 Hz记录关键帧之间按10 ms插值，下位机闭环控制仍保持100 Hz。
REPLAY_OUTPUT_PERIOD_SECONDS = 0.01
# 大于该间隔视为原动作暂停或采集间断，不跨越空白区缓慢插值。
REPLAY_MAX_INTERPOLATION_GAP_SECONDS = 0.2
REPLAY_START_SYSTEM_STATES = (SYSTEM_STATE_READY, SYSTEM_STATE_RUN_FEEDBACK)


def replay_start_state_allowed(system_state: object) -> bool:
    """Return whether P4 can accept a Replay start command."""
    try:
        value = int(system_state)
    except (TypeError, ValueError):
        return False
    return value in REPLAY_START_SYSTEM_STATES


@dataclass(frozen=True)
class ReplayStatus:
    state: str = "idle"
    message: str = "Replay空闲"
    frame_index: int = 0
    total_frames: int = 0
    elapsed_s: float = 0.0
    duration_s: float = 0.0
    current_targets: Tuple[float, ...] = ()
    current_joint_mask: int = 0


@dataclass(frozen=True)
class ReplayOutputSample:
    """Replay线程实际发送的一帧；连续关键帧之间可包含插值样本。"""

    time_s: float
    targets: Tuple[float, ...]
    command_joint_mask: int


class ReplayController:
    """管理Replay控制权、时间调度和运行期安全门禁。"""

    def __init__(
        self,
        backend: HandBackend,
        *,
        event_sink: Optional[Callable[[str, str], None]] = None,
    ) -> None:
        self.backend = backend
        self._event_sink = event_sink
        self._lock = threading.RLock()
        self._status = ReplayStatus()
        self._thread: Optional[threading.Thread] = None
        self._stop_event: Optional[threading.Event] = None

    def set_event_sink(self, sink: Optional[Callable[[str, str], None]]) -> None:
        self._event_sink = sink

    def status(self) -> ReplayStatus:
        with self._lock:
            return self._status

    def is_running(self) -> bool:
        with self._lock:
            return bool(self._thread and self._thread.is_alive())

    def start(self, trace: ReplayTrace) -> tuple[bool, str]:
        trace.validate()
        with self._lock:
            if self._thread and self._thread.is_alive():
                return False, "Replay已经运行"
            state = self.backend.get_state_snapshot()
            active_mask = int(trace.active_joint_mask)
            if active_mask == 0:
                active_mask = int(getattr(state, "active_joint_mask", 0) or 0)
            reason = self._entry_reject_reason(trace, state, active_mask)
            if reason:
                self._set_status(
                    "rejected",
                    reason,
                    total_frames=trace.frame_count,
                    duration_s=trace.duration_s,
                )
                self._emit("replay_rejected", reason)
                return False, reason

            ok, token, message = self.backend.begin_replay_motion()
            if not ok:
                self._set_status(
                    "rejected",
                    message,
                    total_frames=trace.frame_count,
                    duration_s=trace.duration_s,
                )
                return False, message

            stop_event = threading.Event()
            output_samples = self._build_output_samples(trace, active_mask)
            self._stop_event = stop_event
            self._thread = threading.Thread(
                target=self._run,
                args=(trace, output_samples, active_mask, token, stop_event),
                name="mmhand-replay-csv",
                daemon=False,
            )
            self._set_status(
                "running",
                f"正在播放 {trace.name}",
                total_frames=len(output_samples),
                duration_s=trace.duration_s,
            )
            self._thread.start()
        self._emit(
            "replay_start",
            f"file={trace.path}; frames={trace.frame_count}; duration_s={trace.duration_s:.3f}",
        )
        return True, "Replay已启动"

    def stop(self, reason: str = "用户停止") -> bool:
        with self._lock:
            event = self._stop_event
            running = bool(self._thread and self._thread.is_alive())
        if not running or event is None:
            return False
        event.set()
        self.backend.cancel_replay_motion(reason)
        if reason in ("用户停止", "Replay UI Stop"):
            self._hold_current_joint_feedback()
        self._emit("replay_stop", reason)
        return True

    def shutdown(self, timeout_s: float = 3.0) -> None:
        self.stop("窗口关闭")
        with self._lock:
            thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=max(0.0, float(timeout_s)))

    def _entry_reject_reason(self, trace: ReplayTrace, state, active_mask: int) -> str:
        """播放前一次性检查模型、READY、active scope和整条轨迹工作空间。"""
        system_state = int(getattr(state, "system_state", -1))
        if not replay_start_state_allowed(system_state):
            return "P4必须处于READY/RUN_FEEDBACK才能开始Replay"
        current_active = int(getattr(state, "active_joint_mask", 0) or 0)
        if active_mask == 0:
            return "当前active joint scope为空"
        if active_mask & ~current_active:
            return (
                "CSV active scope超出P4当前范围："
                f"csv=0x{active_mask:08X}, current=0x{current_active:08X}"
            )

        mapping = active_mapping_identity()
        if trace.urdf_crc and int(trace.urdf_crc) != int(mapping.urdf_config_crc):
            return "CSV的URDF CRC与当前模型不一致"
        if trace.mapping_id and trace.mapping_id != mapping.mapping_id:
            return "CSV的mapping_id与当前模型不一致"
        if trace.mapping_version and int(trace.mapping_version) != int(mapping.mapping_version):
            return "CSV的mapping_version与当前模型不一致"

        mapping_version, urdf_crc = urdf_manual_scaled_mapping_metadata()
        invalid: list[str] = []
        for joint in range(ENCODER_COUNT):
            if not (active_mask & (1 << joint)):
                continue
            spec = get_spec(joint)
            workspace = resolve_joint_workspace_range(
                state,
                joint,
                spec.urdf_range_deg[0],
                spec.urdf_range_deg[1],
                mapping_version,
                urdf_crc,
            )
            if not workspace.ready:
                invalid.append(f"J{joint:02d}未标定")
                continue
            values = [
                frame.target_urdf_deg[joint]
                for frame in trace.frames
                if (int(frame.command_joint_mask) or active_mask) & (1 << joint)
            ]
            if not values:
                continue
            low = min(values)
            high = max(values)
            if low < workspace.current_min_deg - 1e-6:
                invalid.append(f"J{joint:02d}最低{low:.2f}°")
            elif high > workspace.current_max_deg + 1e-6:
                invalid.append(f"J{joint:02d}最高{high:.2f}°")
        if invalid:
            return "CSV轨迹超出当前工作空间：" + "；".join(invalid[:6])
        return ""

    @staticmethod
    def _build_output_samples(
        trace: ReplayTrace,
        fallback_active_mask: int,
    ) -> List[ReplayOutputSample]:
        """把20 Hz连续关键帧插值成100 Hz，离散命令仍只生成一个样本。"""
        samples: List[ReplayOutputSample] = []
        for index, frame in enumerate(trace.frames):
            frame_mask = int(frame.command_joint_mask) or int(fallback_active_mask)
            samples.append(
                ReplayOutputSample(
                    float(frame.time_s),
                    tuple(float(value) for value in frame.target_urdf_deg),
                    frame_mask,
                )
            )
            if frame.frame_kind != "continuous" or index + 1 >= len(trace.frames):
                continue
            next_frame = trace.frames[index + 1]
            next_mask = int(next_frame.command_joint_mask) or int(fallback_active_mask)
            gap_s = float(next_frame.time_s) - float(frame.time_s)
            if (
                next_frame.frame_kind != "continuous"
                or next_frame.stream_id != frame.stream_id
                or next_mask != frame_mask
                or gap_s <= REPLAY_OUTPUT_PERIOD_SECONDS
                or gap_s > REPLAY_MAX_INTERPOLATION_GAP_SECONDS
            ):
                continue
            offset_s = REPLAY_OUTPUT_PERIOD_SECONDS
            while offset_s < gap_s - 1e-9:
                alpha = offset_s / gap_s
                targets = tuple(
                    float(start) + (float(end) - float(start)) * alpha
                    for start, end in zip(
                        frame.target_urdf_deg,
                        next_frame.target_urdf_deg,
                    )
                )
                samples.append(
                    ReplayOutputSample(
                        float(frame.time_s) + offset_s,
                        targets,
                        frame_mask,
                    )
                )
                offset_s += REPLAY_OUTPUT_PERIOD_SECONDS
        samples.sort(key=lambda sample: sample.time_s)
        return samples

    def _runtime_reject_reason(self, token: int, active_mask: int) -> str:
        if not self.backend.replay_motion_token_valid(token):
            return "Replay控制权已被其他命令撤销"
        state = self.backend.get_state_snapshot()
        system_state = int(getattr(state, "system_state", -1))
        if system_state not in (SYSTEM_STATE_READY, SYSTEM_STATE_RUN_FEEDBACK):
            return f"P4离开READY/RUN_FEEDBACK，state={system_state}"
        # 碰撞 HOLD 只冻结对应关节的舵机 ABS，不是遥操/Replay暂停条件。
        # 必须继续发送后续目标，P4才能在目标反向回撤至少1°时解除 HOLD。
        current_active = int(getattr(state, "active_joint_mask", 0) or 0)
        if active_mask & ~current_active:
            return "active scope在Replay期间发生变化"
        return ""

    def _run(
        self,
        trace: ReplayTrace,
        output_samples: List[ReplayOutputSample],
        active_mask: int,
        token: int,
        stop_event: threading.Event,
    ) -> None:
        start_time = time.monotonic()
        abort_reason = ""
        frame_index = 0
        try:
            while frame_index < len(output_samples):
                if stop_event.is_set():
                    abort_reason = "Replay已停止"
                    break
                abort_reason = self._runtime_reject_reason(token, active_mask)
                if abort_reason:
                    break

                sample = output_samples[frame_index]
                deadline = start_time + float(sample.time_s)
                while True:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0.0:
                        break
                    if stop_event.wait(min(REPLAY_WAIT_POLL_SECONDS, remaining)):
                        abort_reason = "Replay已停止"
                        break
                    abort_reason = self._runtime_reject_reason(token, active_mask)
                    if abort_reason:
                        break
                if abort_reason:
                    break

                elapsed = time.monotonic() - start_time
                scheduling_lag = elapsed - float(sample.time_s)
                if scheduling_lag > REPLAY_MAX_SCHEDULING_LAG_SECONDS:
                    abort_reason = (
                        f"Replay调度落后{scheduling_lag:.3f}s，"
                        "为避免漏帧或突发补发已中止"
                    )
                    break
                result = self.backend.send_joint_targets(
                    sample.targets,
                    source=f"Replay:{trace.name}",
                    live=True,
                    command_joint_mask=sample.command_joint_mask,
                    motion_token=token,
                )
                if not result.sent:
                    abort_reason = result.message or "Replay目标发送失败"
                    break

                self._set_status(
                    "running",
                    f"正在播放 {trace.name}",
                    frame_index=frame_index + 1,
                    total_frames=len(output_samples),
                    elapsed_s=min(trace.duration_s, elapsed),
                    duration_s=trace.duration_s,
                    current_targets=sample.targets,
                    current_joint_mask=sample.command_joint_mask,
                )
                frame_index += 1
        except Exception as exc:
            abort_reason = f"Replay异常：{exc}"
        finally:
            self.backend.end_replay_motion(token)
            with self._lock:
                if self._stop_event is stop_event:
                    self._stop_event = None
            if abort_reason:
                last_sample = (
                    output_samples[frame_index - 1]
                    if frame_index > 0
                    else None
                )
                self._set_status(
                    "aborted",
                    abort_reason,
                    total_frames=len(output_samples),
                    duration_s=trace.duration_s,
                    current_targets=(
                        last_sample.targets if last_sample is not None else ()
                    ),
                    current_joint_mask=(
                        last_sample.command_joint_mask
                        if last_sample is not None
                        else 0
                    ),
                )
                self._emit("replay_abort", abort_reason)
            else:
                final_sample = output_samples[-1]
                self._set_status(
                    "completed",
                    f"Replay完成：{trace.name}",
                    frame_index=len(output_samples),
                    total_frames=len(output_samples),
                    elapsed_s=trace.duration_s,
                    duration_s=trace.duration_s,
                    current_targets=final_sample.targets,
                    current_joint_mask=final_sample.command_joint_mask,
                )
                self._emit("replay_complete", f"file={trace.path}")

    def _set_status(
        self,
        state: str,
        message: str,
        *,
        frame_index: int = 0,
        total_frames: int = 0,
        elapsed_s: float = 0.0,
        duration_s: float = 0.0,
        current_targets: Tuple[float, ...] = (),
        current_joint_mask: int = 0,
    ) -> None:
        with self._lock:
            self._status = ReplayStatus(
                state=state,
                message=message,
                frame_index=int(frame_index),
                total_frames=int(total_frames),
                elapsed_s=float(elapsed_s),
                duration_s=float(duration_s),
                current_targets=tuple(float(value) for value in current_targets),
                current_joint_mask=int(current_joint_mask),
            )

    def _emit(self, event_type: str, detail: str) -> None:
        sink = self._event_sink
        if sink is not None:
            try:
                sink(event_type, detail)
            except Exception:
                pass

    def _hold_current_joint_feedback(self) -> None:
        """普通Stop冻结当前关节角；E-STOP由更高层直接切断控制，不在此补发。"""
        try:
            state = self.backend.get_state_snapshot()
            angles = list(getattr(state, "angles", []) or [])
            joint_mask = int(getattr(state, "active_joint_mask", 0) or 0)
            if len(angles) != ENCODER_COUNT or joint_mask == 0:
                return
            self.backend.send_joint_targets(
                angles,
                source="Hold Replay Stop",
                live=True,
                command_joint_mask=joint_mask,
            )
        except Exception:
            pass
