"""Algorithm mode: repeated Open Pose / Fist Pose endurance exercise.

The Algorithm launcher calls ``run(backend, stop_event=...)`` in a background
thread. This module waits until the selected active joint scope is READY, then
performs a bounded Open -> Fist test. All commands still use HandBackend, so
host-side dynamic workspace clamping and P4 safety checks remain active.
"""

from __future__ import annotations

import threading
import time
from typing import Optional, Tuple

from desktop.core.angle_mapping import get_spec, urdf_manual_scaled_mapping_metadata
from desktop.core.data_models import HandModel, resolve_joint_workspace_range
from desktop.core.hand_backend import HandBackend
from desktop.core.protocol import (
    ENCODER_COUNT,
    SYSTEM_STATE_READY,
    SYSTEM_STATE_RECOVERY_REQUIRED,
    SYSTEM_STATE_RUN_FEEDBACK,
)


# One cycle is: Open Pose -> hold -> Fist Pose -> hold.
CYCLE_COUNT = 100
POSE_HOLD_SECONDS = 3.0
READY_POLL_SECONDS = 0.2
COMMAND_SOURCE = "Algorithm Pose Cycle"
OPEN_POSE_CURRENT_MIN_OFFSET_DEG = 5.0
FIST_POSE_URDF_MAX_OFFSET_DEG = 7.0


class PoseCycleController:
    """Owns one optional Open/Fist worker for the Algorithm UI."""

    def __init__(self, backend: HandBackend) -> None:
        self._backend = backend
        self._lock = threading.Lock()
        self._stop_event: Optional[threading.Event] = None
        self._thread: Optional[threading.Thread] = None

    def is_running(self) -> bool:
        with self._lock:
            return bool(self._thread and self._thread.is_alive())

    def start(self) -> bool:
        """Start a new cycle run; return False when a run is already active."""
        with self._lock:
            if self._thread and self._thread.is_alive():
                return False
            self._stop_event = threading.Event()
            self._thread = threading.Thread(
                target=self._run,
                args=(self._stop_event,),
                name="mmhand-algorithm-pose-cycle",
                daemon=False,
            )
            self._thread.start()
        print("[Algorithm] enabled: waiting for READY before starting pose cycles")
        return True

    def stop(self, reason: str = "stop requested") -> bool:
        """Cancel pose-cycle commands without changing the current hand target."""
        with self._lock:
            stop_event = self._stop_event
            thread = self._thread
        if stop_event is None or thread is None or not thread.is_alive():
            return False
        stop_event.set()
        print(f"[Algorithm] cancelled: {reason}")
        return True

    def shutdown(self, timeout_s: float = 3.0) -> None:
        """Cancel and join the worker before the application exits."""
        self.stop("application closing")
        with self._lock:
            thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=max(0.0, float(timeout_s)))

    def _run(self, stop_event: threading.Event) -> None:
        try:
            run_pose_cycles(self._backend, stop_event=stop_event)
        finally:
            with self._lock:
                if self._stop_event is stop_event:
                    self._stop_event = None


def _stop_requested(stop_event: Optional[threading.Event]) -> bool:
    return bool(stop_event and stop_event.is_set())


def _active_joint_mask(state: HandModel) -> int:
    return int(getattr(state, "active_joint_mask", 0) or 0) & ((1 << ENCODER_COUNT) - 1)


def _state_ready_reason(state: Optional[HandModel]) -> str:
    if state is None:
        return "state unavailable"

    system_state = int(getattr(state, "system_state", -1))
    if system_state not in (
        SYSTEM_STATE_READY,
        SYSTEM_STATE_RECOVERY_REQUIRED,
        SYSTEM_STATE_RUN_FEEDBACK,
    ):
        return f"system_state={system_state}, waiting for READY"

    active_mask = _active_joint_mask(state)
    if active_mask == 0:
        return "active joint scope is empty"

    mapping_version, urdf_crc = urdf_manual_scaled_mapping_metadata()
    missing = []
    for joint in range(ENCODER_COUNT):
        if (active_mask & (1 << joint)) == 0:
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
            missing.append(f"J{joint:02d} {workspace.reason}")

    if missing:
        suffix = "" if len(missing) <= 4 else f"; +{len(missing) - 4} more"
        return "workspace not READY: " + "; ".join(missing[:4]) + suffix
    return ""


def _wait_until_ready(
    backend: HandBackend,
    stop_event: Optional[threading.Event],
    *,
    poll_seconds: float,
) -> Optional[HandModel]:
    last_reason = ""
    while not _stop_requested(stop_event):
        state = backend.get_state_snapshot()
        reason = _state_ready_reason(state)
        if not reason:
            print("[Algorithm] active scope READY; pose cycle starts")
            return state
        if reason != last_reason:
            print(f"[Algorithm] waiting: {reason}")
            last_reason = reason
        if stop_event:
            stop_event.wait(max(0.01, poll_seconds))
        else:
            time.sleep(max(0.01, poll_seconds))
    return None


def _base_targets(state: HandModel) -> list[float]:
    values = list(getattr(state, "angles", []) or [])
    values = [float(value) for value in values[:ENCODER_COUNT]]
    values.extend([0.0] * (ENCODER_COUNT - len(values)))
    return values


def build_open_pose_targets(state: HandModel) -> list[float]:
    """Build Open Pose from Current Min + 5 deg for active joints only."""
    targets = _base_targets(state)
    active_mask = _active_joint_mask(state)
    mapping_version, urdf_crc = urdf_manual_scaled_mapping_metadata()
    for joint in range(ENCODER_COUNT):
        if active_mask & (1 << joint):
            spec = get_spec(joint)
            workspace = resolve_joint_workspace_range(
                state,
                joint,
                spec.urdf_range_deg[0],
                spec.urdf_range_deg[1],
                mapping_version,
                urdf_crc,
            )
            targets[joint] = workspace.open_pose_target(
                OPEN_POSE_CURRENT_MIN_OFFSET_DEG
            )
    return targets


def build_fist_pose_targets(state: HandModel) -> list[float]:
    """Build Fist Pose at URDF Max - 7 deg for active joints only."""
    targets = _base_targets(state)
    active_mask = _active_joint_mask(state)
    for joint in range(ENCODER_COUNT):
        if active_mask & (1 << joint):
            urdf_max = float(get_spec(joint).urdf_range_deg[1])
            targets[joint] = urdf_max - FIST_POSE_URDF_MAX_OFFSET_DEG
    return targets


def _send_pose(backend: HandBackend, state: HandModel, name: str) -> Tuple[bool, str]:
    targets = (
        build_open_pose_targets(state)
        if name == "Open"
        else build_fist_pose_targets(state)
    )
    result = backend.send_joint_targets(
        targets,
        source=f"{COMMAND_SOURCE} {name}",
        live=False,
    )
    if not bool(getattr(result, "sent", False)):
        return False, str(getattr(result, "message", "joint command rejected"))
    blocked = list(getattr(result, "blocked", []) or [])
    if blocked:
        labels = ", ".join(f"J{int(joint):02d}" for joint in blocked)
        return False, f"tendon guard blocked {labels}"
    clamped = list(getattr(result, "clamped", []) or [])
    clamp_text = ""
    if clamped:
        clamp_text = " clamped=" + ",".join(f"J{int(joint):02d}" for joint in clamped)
    return True, f"{name} sent{clamp_text}"


def _hold_and_monitor(
    backend: HandBackend,
    stop_event: Optional[threading.Event],
    hold_seconds: float,
) -> Tuple[bool, str]:
    deadline = time.monotonic() + max(0.0, hold_seconds)
    while time.monotonic() < deadline:
        if _stop_requested(stop_event):
            return False, "stop requested"
        reason = _state_ready_reason(backend.get_state_snapshot())
        if reason:
            return False, reason
        remaining = deadline - time.monotonic()
        wait_seconds = min(0.1, max(0.0, remaining))
        if stop_event:
            stop_event.wait(wait_seconds)
        elif wait_seconds > 0.0:
            time.sleep(wait_seconds)
    return True, ""


def run_pose_cycles(
    backend: HandBackend,
    stop_event: Optional[threading.Event] = None,
    *,
    cycle_count: int = CYCLE_COUNT,
    hold_seconds: float = POSE_HOLD_SECONDS,
    ready_poll_seconds: float = READY_POLL_SECONDS,
) -> int:
    """Run completed Open->Fist cycles and return the number fully finished."""
    state = _wait_until_ready(
        backend,
        stop_event,
        poll_seconds=ready_poll_seconds,
    )
    if state is None:
        print("[Algorithm] pose cycle cancelled before READY")
        return 0

    completed = 0
    for cycle in range(1, max(0, int(cycle_count)) + 1):
        for pose_name in ("Open", "Fist"):
            if _stop_requested(stop_event):
                print(f"[Algorithm] stopped after {completed} completed cycle(s)")
                return completed

            state = backend.get_state_snapshot()
            reason = _state_ready_reason(state)
            if reason:
                print(f"[Algorithm] cycle {cycle:03d} {pose_name} aborted: {reason}")
                return completed

            sent, detail = _send_pose(backend, state, pose_name)
            if not sent:
                print(f"[Algorithm] cycle {cycle:03d} {pose_name} rejected: {detail}")
                return completed
            print(f"[Algorithm] cycle {cycle:03d}/{cycle_count:03d}: {detail}; hold {hold_seconds:.1f}s")

            held, reason = _hold_and_monitor(backend, stop_event, hold_seconds)
            if not held:
                print(f"[Algorithm] cycle {cycle:03d} {pose_name} interrupted: {reason}")
                return completed
        completed = cycle

    print(f"[Algorithm] completed {completed}/{cycle_count} Open/Fist cycle(s); final pose is Fist")
    return completed


def run(
    backend: HandBackend,
    stop_event: Optional[threading.Event] = None,
) -> None:
    """Algorithm launcher entry point used by ``desktop/app/ui_runner.py``."""
    if not isinstance(backend, HandBackend):
        backend = HandBackend(backend)
    run_pose_cycles(backend, stop_event)
