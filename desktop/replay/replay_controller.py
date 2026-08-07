"""replay_controller.py - MMHand Desktop 模块契约。

职责：按 CSV 原始时间轴调度完整动作轨迹并处理停止和重定位。
线程：解析在调用线程执行，播放调度在 Replay 工作线程执行。
输入：运行 CSV、模型契约、活动关节范围和播放控制事件。
输出：带时间轴的稀疏关节帧、进度、诊断和停止结果。
状态所有权：轨迹对象由 ReplayService 拥有；设备运行状态只读且归 HandBackend。
禁止依赖：禁止依赖 Qt、串口和 ROS2，禁止直接修改 HandModel 或下位机状态。
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import Callable, Dict, List, Optional, Tuple

from desktop.core.angle_mapping import get_spec, urdf_manual_scaled_mapping_metadata
from desktop.core.data_models import resolve_joint_workspace_range
from desktop.core.hand_backend import HandBackend
from desktop.core.protocol import (
    ENCODER_COUNT,
    SYSTEM_STATE_READY,
    SYSTEM_STATE_RUN_FEEDBACK,
)
from desktop.teleop.joint_target_adapter import active_mapping_identity

from .replay_models import ALL_JOINTS_MASK, ReplayFrame, ReplayPreparation, ReplayTrace


# 等待下一原始帧期间仍按50 ms检查一次Stop和P4运行状态。
REPLAY_WAIT_POLL_SECONDS = 0.05
# Windows调度落后超过该值时整体后移时间轴，禁止突发补发积压姿态。
REPLAY_TIMELINE_REBASE_THRESHOLD_SECONDS = 0.25
# 20 Hz记录关键帧之间按10 ms插值，下位机闭环控制仍保持100 Hz。
REPLAY_OUTPUT_PERIOD_SECONDS = 0.01
# 大于该间隔视为原动作暂停或采集间断，不跨越空白区缓慢插值。
REPLAY_MAX_INTERPOLATION_GAP_SECONDS = 0.2
REPLAY_START_SYSTEM_STATES = (SYSTEM_STATE_READY, SYSTEM_STATE_RUN_FEEDBACK)


def replay_start_state_allowed(system_state: object) -> bool:
    """Replay首次启动或重复启动时，P4必须处于可接管的负反馈状态。"""
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
    source_joint_mask: int = 0
    effective_joint_mask: int = 0
    skipped_joint_mask: int = 0
    clamped_joint_mask: int = 0
    clamped_sample_count: int = 0
    timeline_rebase_count: int = 0
    timeline_shift_s: float = 0.0
    warnings: Tuple[str, ...] = ()


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
            reason, preparation, output_samples = self._prepare_replay(trace, state)
            if reason:
                self._set_status(
                    "rejected",
                    reason,
                    total_frames=trace.frame_count,
                    duration_s=trace.duration_s,
                )
                self._emit("replay_rejected", reason)
                return False, reason
            assert preparation is not None
            if not output_samples:
                reason = "Replay没有可发送的有效关节帧"
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
            self._stop_event = stop_event
            self._thread = threading.Thread(
                target=self._run,
                args=(
                    trace,
                    output_samples,
                    preparation,
                    token,
                    stop_event,
                ),
                name="mmhand-replay-csv",
                daemon=False,
            )
            self._set_status(
                "running",
                f"正在播放 {trace.name}",
                total_frames=len(output_samples),
                duration_s=trace.duration_s,
                source_joint_mask=preparation.source_joint_mask,
                effective_joint_mask=preparation.effective_joint_mask,
                skipped_joint_mask=preparation.skipped_joint_mask,
                clamped_joint_mask=preparation.clamped_joint_mask,
                clamped_sample_count=preparation.clamped_sample_count,
                warnings=preparation.warnings,
            )
            self._thread.start()
        self._emit(
            "replay_start",
            (
                f"file={trace.path}; frames={trace.frame_count}; "
                f"duration_s={trace.duration_s:.3f}; "
                f"source_mask=0x{preparation.source_joint_mask:08X}; "
                f"effective_mask=0x{preparation.effective_joint_mask:08X}; "
                f"skipped_mask=0x{preparation.skipped_joint_mask:08X}; "
                f"clamped_samples={preparation.clamped_sample_count}"
            ),
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

    def _prepare_replay(
        self,
        trace: ReplayTrace,
        state,
    ) -> tuple[str, Optional[ReplayPreparation], List[ReplayOutputSample]]:
        """将历史轨迹降级到当前可用关节，并在插值前钳入当前标定工作空间。"""
        system_state = int(getattr(state, "system_state", -1))
        if not replay_start_state_allowed(system_state):
            return "P4必须处于READY/RUN_FEEDBACK才能开始Replay", None, []
        current_active = int(getattr(state, "active_joint_mask", 0) or 0) & ALL_JOINTS_MASK
        if current_active == 0:
            return "当前active joint scope为空", None, []
        source_mask = int(trace.active_joint_mask) & ALL_JOINTS_MASK
        if source_mask == 0:
            source_mask = current_active

        mapping = active_mapping_identity()
        warnings = list(trace.warnings)
        if trace.joint_order_crc:
            if int(trace.joint_order_crc) != int(mapping.joint_order_crc):
                return "CSV的J00-J20关节顺序与当前模型不一致", None, []
            if trace.mapping_id and trace.mapping_id != mapping.mapping_id:
                warnings.append(
                    "mapping_id不同，但J00-J20关节顺序一致；按当前工作空间降级播放"
                )
            if trace.mapping_version and int(trace.mapping_version) != int(
                mapping.mapping_version
            ):
                warnings.append(
                    "mapping_version不同，但关节顺序一致；按当前工作空间降级播放"
                )
        elif trace.mapping_id and trace.mapping_id != mapping.mapping_id:
            return (
                "旧CSV缺少关节顺序签名且mapping_id不同，无法安全确认J00-J20含义",
                None,
                [],
            )
        elif trace.mapping_version and int(trace.mapping_version) != int(mapping.mapping_version):
            return (
                "旧CSV缺少关节顺序签名且mapping_version不同，无法安全确认J00-J20含义",
                None,
                [],
            )
        elif trace.mapping_id or trace.mapping_version or trace.urdf_crc:
            warnings.append("CSV缺少joint_order_crc，只能依赖mapping ID/version确认关节顺序")
        if trace.urdf_crc and int(trace.urdf_crc) != int(mapping.urdf_config_crc):
            warnings.append("URDF范围CRC不同；历史角度将钳位到当前标定工作空间")

        mapping_version, urdf_crc = urdf_manual_scaled_mapping_metadata()
        workspaces: Dict[int, object] = {}
        workspace_ready_mask = 0
        for joint in range(ENCODER_COUNT):
            bit = 1 << joint
            if not (source_mask & current_active & bit):
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
            if workspace.ready:
                workspaces[joint] = workspace
                workspace_ready_mask |= bit

        value_union_mask = 0
        for frame in trace.frames:
            frame_source_mask = int(frame.command_joint_mask) or source_mask
            value_union_mask |= frame_source_mask & int(frame.joint_value_mask)
        effective_mask = (
            source_mask
            & current_active
            & workspace_ready_mask
            & value_union_mask
            & ALL_JOINTS_MASK
        )
        unavailable_mask = source_mask & ~current_active
        uncalibrated_mask = source_mask & current_active & ~workspace_ready_mask
        missing_value_mask = source_mask & ~value_union_mask
        skipped_mask = source_mask & ~effective_mask
        if unavailable_mask:
            warnings.append(
                "跳过当前P4 active scope外关节："
                + self._format_joint_mask(unavailable_mask)
            )
        if uncalibrated_mask:
            warnings.append(
                "跳过当前Manual/Auto未就绪关节："
                + self._format_joint_mask(uncalibrated_mask)
            )
        if missing_value_mask:
            warnings.append(
                "跳过CSV没有有效数值的关节："
                + self._format_joint_mask(missing_value_mask)
            )
        if effective_mask == 0:
            return "CSV与当前P4状态没有可共同控制的已标定关节", None, []

        baseline = list(getattr(state, "angles", []) or [])
        if len(baseline) != ENCODER_COUNT:
            baseline = [0.0] * ENCODER_COUNT
        baseline = [
            float(value)
            if isinstance(value, (int, float)) and math.isfinite(float(value))
            else 0.0
            for value in baseline
        ]
        prepared_frames, clamp_counts = self._prepare_clamped_frames(
            trace,
            effective_mask,
            workspaces,
            baseline,
        )
        if not prepared_frames:
            return "CSV有效关节在所有帧中都没有可发送数值", None, []

        clamped_joint_mask = 0
        for joint in clamp_counts:
            clamped_joint_mask |= 1 << joint
        clamped_sample_count = sum(clamp_counts.values())
        if clamped_sample_count:
            warnings.append(
                f"{self._format_joint_mask(clamped_joint_mask)}共有"
                f"{clamped_sample_count}个关键帧目标被钳位"
            )
        preparation = ReplayPreparation(
            source_joint_mask=source_mask,
            current_active_mask=current_active,
            workspace_ready_mask=workspace_ready_mask,
            effective_joint_mask=effective_mask,
            skipped_joint_mask=skipped_mask,
            uncalibrated_joint_mask=uncalibrated_mask,
            clamped_joint_mask=clamped_joint_mask,
            clamped_sample_count=clamped_sample_count,
            clamped_counts_by_joint=tuple(sorted(clamp_counts.items())),
            warnings=tuple(dict.fromkeys(warnings)),
        )
        return (
            "",
            preparation,
            self._build_output_samples(prepared_frames, effective_mask),
        )

    @staticmethod
    def _prepare_clamped_frames(
        trace: ReplayTrace,
        effective_mask: int,
        workspaces: Dict[int, object],
        baseline: List[float],
    ) -> tuple[List[ReplayFrame], Dict[int, int]]:
        """未参与本帧的槽位保持最近目标；有效关节在插值前先钳到当前范围。"""
        prepared: List[ReplayFrame] = []
        current_targets = list(baseline)
        clamp_counts: Dict[int, int] = {}
        source_mask = int(trace.active_joint_mask) or int(effective_mask)
        for frame in trace.frames:
            requested_mask = int(frame.command_joint_mask) or source_mask
            frame_mask = (
                requested_mask
                & int(frame.joint_value_mask)
                & int(effective_mask)
                & ALL_JOINTS_MASK
            )
            if frame_mask == 0:
                continue
            for joint in range(ENCODER_COUNT):
                if not (frame_mask & (1 << joint)):
                    continue
                original = float(frame.target_urdf_deg[joint])
                bounded = float(workspaces[joint].clamp(original))
                current_targets[joint] = bounded
                if abs(bounded - original) > 1e-6:
                    clamp_counts[joint] = clamp_counts.get(joint, 0) + 1
            prepared.append(
                ReplayFrame(
                    time_s=float(frame.time_s),
                    target_urdf_deg=list(current_targets),
                    frame_kind=frame.frame_kind,
                    source_kind=frame.source_kind,
                    stream_id=frame.stream_id,
                    command_joint_mask=frame_mask,
                    joint_value_mask=frame_mask,
                )
            )
        return prepared, clamp_counts

    @staticmethod
    def _build_output_samples(
        frames: List[ReplayFrame],
        fallback_active_mask: int,
    ) -> List[ReplayOutputSample]:
        """把20 Hz连续关键帧插值成100 Hz，离散命令仍只生成一个样本。"""
        samples: List[ReplayOutputSample] = []
        for index, frame in enumerate(frames):
            frame_mask = int(frame.command_joint_mask) or int(fallback_active_mask)
            samples.append(
                ReplayOutputSample(
                    float(frame.time_s),
                    tuple(float(value) for value in frame.target_urdf_deg),
                    frame_mask,
                )
            )
            if frame.frame_kind != "continuous" or index + 1 >= len(frames):
                continue
            next_frame = frames[index + 1]
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

    @staticmethod
    def _format_joint_mask(mask: int) -> str:
        labels = [
            f"J{joint:02d}"
            for joint in range(ENCODER_COUNT)
            if int(mask) & (1 << joint)
        ]
        return ",".join(labels) if labels else "-"

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
        preparation: ReplayPreparation,
        token: int,
        stop_event: threading.Event,
    ) -> None:
        start_time = time.monotonic()
        timeline_shift_s = 0.0
        timeline_rebase_count = 0
        abort_reason = ""
        frame_index = 0
        try:
            while frame_index < len(output_samples):
                if stop_event.is_set():
                    abort_reason = "Replay已停止"
                    break
                abort_reason = self._runtime_reject_reason(
                    token, preparation.effective_joint_mask
                )
                if abort_reason:
                    break

                sample = output_samples[frame_index]
                deadline = start_time + timeline_shift_s + float(sample.time_s)
                while True:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0.0:
                        break
                    if stop_event.wait(min(REPLAY_WAIT_POLL_SECONDS, remaining)):
                        abort_reason = "Replay已停止"
                        break
                    abort_reason = self._runtime_reject_reason(
                        token, preparation.effective_joint_mask
                    )
                    if abort_reason:
                        break
                if abort_reason:
                    break

                elapsed = time.monotonic() - start_time
                scheduling_lag = elapsed - timeline_shift_s - float(sample.time_s)
                if scheduling_lag > REPLAY_TIMELINE_REBASE_THRESHOLD_SECONDS:
                    # 调度延迟时整体后移剩余时间轴：当前帧正常发送，后续帧不会突发补发。
                    timeline_shift_s += scheduling_lag
                    timeline_rebase_count += 1
                    self._emit(
                        "replay_timeline_rebase",
                        (
                            f"file={trace.path}; lag_ms={scheduling_lag * 1000.0:.1f}; "
                            f"count={timeline_rebase_count}; "
                            f"total_shift_ms={timeline_shift_s * 1000.0:.1f}"
                        ),
                    )
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
                    # 进度按动作时间轴而非墙钟时间计算；重定位后的等待不应让进度虚增。
                    elapsed_s=min(trace.duration_s, float(sample.time_s)),
                    duration_s=trace.duration_s,
                    current_targets=sample.targets,
                    current_joint_mask=sample.command_joint_mask,
                    source_joint_mask=preparation.source_joint_mask,
                    effective_joint_mask=preparation.effective_joint_mask,
                    skipped_joint_mask=preparation.skipped_joint_mask,
                    clamped_joint_mask=preparation.clamped_joint_mask,
                    clamped_sample_count=preparation.clamped_sample_count,
                    timeline_rebase_count=timeline_rebase_count,
                    timeline_shift_s=timeline_shift_s,
                    warnings=preparation.warnings,
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
                    source_joint_mask=preparation.source_joint_mask,
                    effective_joint_mask=preparation.effective_joint_mask,
                    skipped_joint_mask=preparation.skipped_joint_mask,
                    clamped_joint_mask=preparation.clamped_joint_mask,
                    clamped_sample_count=preparation.clamped_sample_count,
                    timeline_rebase_count=timeline_rebase_count,
                    timeline_shift_s=timeline_shift_s,
                    warnings=preparation.warnings,
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
                    source_joint_mask=preparation.source_joint_mask,
                    effective_joint_mask=preparation.effective_joint_mask,
                    skipped_joint_mask=preparation.skipped_joint_mask,
                    clamped_joint_mask=preparation.clamped_joint_mask,
                    clamped_sample_count=preparation.clamped_sample_count,
                    timeline_rebase_count=timeline_rebase_count,
                    timeline_shift_s=timeline_shift_s,
                    warnings=preparation.warnings,
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
        source_joint_mask: int = 0,
        effective_joint_mask: int = 0,
        skipped_joint_mask: int = 0,
        clamped_joint_mask: int = 0,
        clamped_sample_count: int = 0,
        timeline_rebase_count: int = 0,
        timeline_shift_s: float = 0.0,
        warnings: Tuple[str, ...] = (),
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
                source_joint_mask=int(source_joint_mask),
                effective_joint_mask=int(effective_joint_mask),
                skipped_joint_mask=int(skipped_joint_mask),
                clamped_joint_mask=int(clamped_joint_mask),
                clamped_sample_count=int(clamped_sample_count),
                timeline_rebase_count=int(timeline_rebase_count),
                timeline_shift_s=float(timeline_shift_s),
                warnings=tuple(str(value) for value in warnings),
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
