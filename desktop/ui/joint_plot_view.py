import csv
import math
import os
import time
from collections import deque
from datetime import datetime
from typing import Dict, List, Optional, Tuple
import tkinter as tk
from tkinter import ttk

import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np

from data_models import HandModel
from protocol import ENCODER_COUNT

JOINT_RT_MAX_PLOT_JOINTS = 5

class JointRealtimePlotMixin:
    def _build_joint_debug_packet_csv_path(csv_path: str) -> str:
        base, _ = os.path.splitext(csv_path)
        return f"{base}_joint_debug_packets.csv"

    def _build_joint_debug_packet_image_path(csv_path: str) -> str:
        base, _ = os.path.splitext(csv_path)
        return f"{base}.png"

    def _build_joint_rt_image_path(self, csv_path: Optional[str] = None) -> str:
        if csv_path:
            base, _ = os.path.splitext(csv_path)
            return f"{base}_joint_rt.png"
        current_csv = self._data_logger.path if hasattr(self, "_data_logger") else ""
        if current_csv:
            base, _ = os.path.splitext(current_csv)
            return f"{base}_joint_rt.png"
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        return os.path.join(self._run_data_root_dir(), f"joint_rt_{ts}.png")

    @staticmethod
    def _parse_joint_rt_csv_float(value: object) -> float:
        try:
            out = float(value)
        except Exception:
            return float("nan")
        return out if np.isfinite(out) else float("nan")

    def _resolve_joint_rt_export_indices(self) -> List[int]:
        selected = self._selected_joint_rt_indices()
        if selected:
            return selected
        idx = int(getattr(self, "_joint_rt_joint_index", 0))
        if 0 <= idx < ENCODER_COUNT:
            return [idx]
        return [0]

    def _resolve_joint_rt_csv_path(self, csv_path: Optional[str] = None) -> str:
        candidate = os.path.abspath(csv_path) if csv_path else ""
        if candidate and os.path.isfile(candidate):
            return candidate
        current_csv = self._data_logger.path if hasattr(self, "_data_logger") else ""
        current_csv = os.path.abspath(current_csv) if current_csv else ""
        if current_csv and os.path.isfile(current_csv):
            return current_csv
        return ""

    def _load_joint_rt_export_series(
        self,
        csv_path: str,
        joint_indices: List[int],
    ) -> Tuple[Dict[int, List[float]], Dict[int, List[float]], Dict[int, List[float]], str]:
        host_ts_abs: List[float] = []
        host_actual_map: Dict[int, List[float]] = {idx: [] for idx in joint_indices}
        host_target_map: Dict[int, List[float]] = {idx: [] for idx in joint_indices}
        debug_time_map: Dict[int, List[float]] = {idx: [] for idx in joint_indices}
        debug_actual_map: Dict[int, List[float]] = {idx: [] for idx in joint_indices}
        debug_target_map: Dict[int, List[float]] = {idx: [] for idx in joint_indices}
        debug_t0_map: Dict[int, float] = {}
        debug_last_ts_map: Dict[int, float] = {}

        with open(csv_path, "r", encoding="utf-8", newline="") as f:
            reader = csv.DictReader(f)
            if not reader.fieldnames:
                raise ValueError(f"CSV header is empty: {csv_path}")

            required = {"ts_wall"}
            for idx in joint_indices:
                required.add(f"joint_curr_deg_j{idx:02d}")
                required.add(f"joint_target_deg_j{idx:02d}")
            missing = [key for key in required if key not in reader.fieldnames]
            if missing:
                raise ValueError(f"CSV missing required columns: {missing}")

            for row in reader:
                host_t_abs = self._parse_joint_rt_csv_float(row.get("ts_wall", ""))
                if np.isfinite(host_t_abs):
                    host_ts_abs.append(float(host_t_abs))
                for idx in joint_indices:
                    actual_key = f"joint_curr_deg_j{idx:02d}"
                    target_key = f"joint_target_deg_j{idx:02d}"
                    if np.isfinite(host_t_abs):
                        host_actual_map[idx].append(self._parse_joint_rt_csv_float(row.get(actual_key, "")))
                    raw_target = str(row.get(target_key, "")).strip()
                    if raw_target == "" or raw_target == NA_MODE_MARKER:
                        host_target = float("nan")
                    else:
                        host_target = self._parse_joint_rt_csv_float(raw_target)
                    if np.isfinite(host_t_abs):
                        host_target_map[idx].append(host_target)

                    debug_ts_key = f"joint_debug_ts_ms_j{idx:02d}"
                    debug_target_key = f"joint_debug_target_deg_j{idx:02d}"
                    debug_actual_key = f"joint_debug_actual_deg_j{idx:02d}"
                    if debug_ts_key not in reader.fieldnames:
                        continue
                    debug_ts = self._parse_joint_rt_csv_float(row.get(debug_ts_key, ""))
                    debug_target = self._parse_joint_rt_csv_float(row.get(debug_target_key, ""))
                    debug_actual = self._parse_joint_rt_csv_float(row.get(debug_actual_key, ""))
                    if (not np.isfinite(debug_ts)) or (not np.isfinite(debug_target)) or (not np.isfinite(debug_actual)):
                        continue
                    if debug_ts <= 0.0:
                        continue
                    prev_debug_ts = debug_last_ts_map.get(idx)
                    if prev_debug_ts is not None and debug_ts <= prev_debug_ts:
                        continue
                    if idx not in debug_t0_map:
                        debug_t0_map[idx] = float(debug_ts)
                    debug_last_ts_map[idx] = float(debug_ts)
                    debug_time_map[idx].append((float(debug_ts) - debug_t0_map[idx]) / 1000.0)
                    debug_target_map[idx].append(float(debug_target))
                    debug_actual_map[idx].append(float(debug_actual))

        if not host_ts_abs and not any(debug_time_map[idx] for idx in joint_indices):
            raise ValueError(f"No usable CSV rows: {csv_path}")

        host_time_map: Dict[int, List[float]] = {}
        if host_ts_abs:
            t0 = host_ts_abs[0]
            host_t_rel = [float(t - t0) for t in host_ts_abs]
            for idx in joint_indices:
                host_time_map[idx] = list(host_t_rel)

        time_map: Dict[int, List[float]] = {}
        actual_map: Dict[int, List[float]] = {}
        target_map: Dict[int, List[float]] = {}
        used_debug = 0
        used_host = 0
        for idx in joint_indices:
            if debug_time_map[idx]:
                time_map[idx] = debug_time_map[idx]
                actual_map[idx] = debug_actual_map[idx]
                target_map[idx] = debug_target_map[idx]
                used_debug += 1
            else:
                time_map[idx] = host_time_map.get(idx, [])
                actual_map[idx] = host_actual_map[idx]
                target_map[idx] = host_target_map[idx]
                used_host += 1

        if used_debug and not used_host:
            time_label = "Device Time (s)"
        elif used_debug and used_host:
            time_label = "Time (s, device preferred)"
        else:
            time_label = "Host Time (s)"
        return time_map, actual_map, target_map, time_label

    def _load_joint_debug_packet_export_series(
        self,
        csv_path: str,
    joint_indices: List[int],
    ) -> Tuple[Dict[int, List[float]], Dict[int, List[float]], Dict[int, List[float]]]:
        time_map: Dict[int, List[float]] = {idx: [] for idx in joint_indices}
        actual_map: Dict[int, List[float]] = {idx: [] for idx in joint_indices}
        target_map: Dict[int, List[float]] = {idx: [] for idx in joint_indices}
        last_ts_map: Dict[int, float] = {}

        with open(csv_path, "r", encoding="utf-8", newline="") as f:
            reader = csv.DictReader(f)
            if not reader.fieldnames:
                raise ValueError(f"CSV header is empty: {csv_path}")
            required = {
                "device_ts_ms",
                "joint_index",
                "target_deg",
                "actual_deg",
            }
            missing = [key for key in required if key not in reader.fieldnames]
            if missing:
                raise ValueError(f"CSV missing required columns: {missing}")

            joint_index_set = set(int(idx) for idx in joint_indices)
            for row in reader:
                try:
                    joint_index = int(str(row.get("joint_index", "")).strip())
                except Exception:
                    continue
                if joint_index not in joint_index_set:
                    continue

                device_ts = self._parse_joint_rt_csv_float(row.get("device_ts_ms", ""))
                target_deg = self._parse_joint_rt_csv_float(row.get("target_deg", ""))
                actual_deg = self._parse_joint_rt_csv_float(row.get("actual_deg", ""))
                if (not np.isfinite(device_ts)) or (not np.isfinite(target_deg)) or (not np.isfinite(actual_deg)):
                    continue
                if device_ts <= 0.0:
                    continue
                prev_ts = last_ts_map.get(joint_index)
                if prev_ts is not None and device_ts <= prev_ts:
                    continue
                last_ts_map[joint_index] = float(device_ts)
                time_map[joint_index].append(float(device_ts) / 1000.0)
                target_map[joint_index].append(float(target_deg))
                actual_map[joint_index].append(float(actual_deg))

        if not any(time_map[idx] for idx in joint_indices):
            raise ValueError(f"No usable packet CSV rows: {csv_path}")
        return time_map, actual_map, target_map

    @staticmethod
    def _estimate_series_lag(
        self,
        t_rel: List[float],
        y_target: List[float],
        y_actual: List[float],
    ) -> Optional[Dict[str, float]]:
        if len(t_rel) < 20 or len(y_target) != len(t_rel) or len(y_actual) != len(t_rel):
            return None

        t = np.asarray(t_rel, dtype=float)
        target = np.asarray(y_target, dtype=float)
        actual = np.asarray(y_actual, dtype=float)
        finite_mask = np.isfinite(t) & np.isfinite(target) & np.isfinite(actual)
        if int(np.sum(finite_mask)) < 20:
            return None
        t = t[finite_mask]
        target = target[finite_mask]
        actual = actual[finite_mask]

        keep = [0]
        for i in range(1, len(t)):
            if t[i] > t[keep[-1]]:
                keep.append(i)
        if len(keep) < 20:
            return None
        t = t[keep]
        target = target[keep]
        actual = actual[keep]

        base_target = float(target[0])
        start_idx = 0
        for i, value in enumerate(target):
            if abs(float(value) - base_target) > 0.3:
                start_idx = max(0, i - 3)
                break
        t = t[start_idx:]
        target = target[start_idx:]
        actual = actual[start_idx:]
        if len(t) < 20:
            return None

        dt = float(np.median(np.diff(t)))
        if (not np.isfinite(dt)) or dt <= 0.0:
            return None

        grid = np.arange(float(t[0]), float(t[-1]), dt)
        if len(grid) < 32:
            return None

        target_i = np.interp(grid, t, target)
        actual_i = np.interp(grid, t, actual)
        x = target_i - float(np.mean(target_i))
        y = actual_i - float(np.mean(actual_i))
        if len(x) < 32 or np.allclose(x, 0.0) or np.allclose(y, 0.0):
            return None

        freqs = np.fft.rfftfreq(len(grid), d=dt)
        target_fft = np.fft.rfft(x)
        if len(freqs) < 2:
            return None
        dominant_idx = int(np.argmax(np.abs(target_fft[1:])) + 1)
        dominant_freq = float(freqs[dominant_idx])
        if (not np.isfinite(dominant_freq)) or dominant_freq <= 0.0:
            return None

        actual_fft = np.fft.rfft(y)
        phase_target = float(np.angle(target_fft[dominant_idx]))
        phase_actual = float(np.angle(actual_fft[dominant_idx]))
        phase_diff = phase_target - phase_actual
        phase_diff = (phase_diff + math.pi) % (2.0 * math.pi) - math.pi
        lag_s = float(phase_diff / (2.0 * math.pi * dominant_freq))

        best_corr = None
        best_lag = 0.0
        max_lag_steps = int(round(1.0 / dt))
        for lag_steps in range(-max_lag_steps, max_lag_steps + 1):
            if lag_steps < 0:
                xs = x[-lag_steps:]
                ys = y[: len(y) + lag_steps]
            elif lag_steps > 0:
                xs = x[:-lag_steps]
                ys = y[lag_steps:]
            else:
                xs = x
                ys = y
            if len(xs) < 20:
                continue
            corr = float(np.corrcoef(xs, ys)[0, 1])
            if not np.isfinite(corr):
                continue
            if best_corr is None or corr > best_corr:
                best_corr = corr
                best_lag = float(lag_steps * dt)

        return {
            "lag_s": lag_s,
            "phase_deg": float(phase_diff * 180.0 / math.pi),
            "freq_hz": dominant_freq,
            "corr": float(best_corr) if best_corr is not None else float("nan"),
            "xcorr_lag_s": best_lag,
        }

    def _save_joint_rt_plot_snapshot(self, csv_path: Optional[str] = None, *, reason: str = "") -> Optional[str]:
        try:
            source_csv = self._resolve_joint_rt_csv_path(csv_path)
            if not source_csv:
                return None
            joint_indices = self._resolve_joint_rt_export_indices()
            time_map, actual_map, target_map, time_label = self._load_joint_rt_export_series(source_csv, joint_indices)
            out_path = self._build_joint_rt_image_path(csv_path)
            folder = os.path.dirname(out_path)
            if folder:
                os.makedirs(folder, exist_ok=True)

            fig = plt.Figure(figsize=(10.5, 5.6), dpi=100)
            ax = fig.add_subplot(111)
            ax.set_xlabel(time_label, fontsize=10)
            ax.set_ylabel("Angle (deg)", fontsize=10)
            ax.grid(True, alpha=0.28)

            cmap = plt.get_cmap("tab10")
            plotted = False
            vals: List[float] = []
            x_max = 0.0
            for idx in joint_indices:
                t_rel = time_map.get(idx, [])
                y_actual = actual_map.get(idx, [])
                y_target = target_map.get(idx, [])
                if len(y_actual) != len(t_rel) or len(y_target) != len(t_rel):
                    continue
                if not t_rel:
                    continue
                plotted = True
                x_max = max(x_max, float(t_rel[-1]))
                base_color = cmap(idx % 10)
                ax.plot(
                    t_rel,
                    y_actual,
                    linestyle="-",
                    linewidth=1.8,
                    color=base_color,
                    label=f"J{idx:02d} Actual",
                )
                ax.plot(
                    t_rel,
                    y_target,
                    linestyle="--",
                    linewidth=1.6,
                    color=base_color,
                    alpha=0.58,
                    label=f"J{idx:02d} Target",
                )
                vals.extend([float(v) for v in y_actual if np.isfinite(v)])
                vals.extend([float(v) for v in y_target if np.isfinite(v)])

            if x_max > 0.0:
                ax.set_xlim(0.0, max(0.8, x_max + 0.05))
            else:
                ax.set_xlim(0.0, 1.0)

            if vals:
                y_min = min(vals)
                y_max = max(vals)
                if y_min == y_max:
                    y_max = y_min + 1.0
                margin = (y_max - y_min) * 0.12
                ax.set_ylim(y_min - margin, y_max + margin)
            else:
                ax.set_ylim(-1.0, 1.0)

            joint_label = ", ".join([f"J{i:02d}" for i in joint_indices]) if joint_indices else "J00"
            ax.set_title(f"Target vs Actual (Full Run): {joint_label}", fontsize=11)
            if plotted:
                ax.legend(loc="upper right", fontsize=8, ncol=2)
            fig.tight_layout()
            fig.savefig(out_path, dpi=150, bbox_inches="tight")
            fig.clear()

            suffix = f" ({reason})" if reason else ""
            self._emit_action(f"Joint RT plot saved from CSV{suffix}: {out_path}")
            return out_path
        except Exception as exc:
            self._emit_action(f"Joint RT plot save failed: {exc}")
            return None

    def _save_joint_debug_packet_plot_snapshot(
        self,
        csv_path: str,
        *,
        reason: str = "",
    ) -> Optional[str]:
        try:
            source_csv = os.path.abspath(csv_path) if csv_path else ""
            if not source_csv or not os.path.isfile(source_csv):
                return None
            joint_indices = self._resolve_joint_rt_export_indices()
            time_map, actual_map, target_map = self._load_joint_debug_packet_export_series(source_csv, joint_indices)
            out_path = self._build_joint_debug_packet_image_path(source_csv)
            folder = os.path.dirname(out_path)
            if folder:
                os.makedirs(folder, exist_ok=True)

            fig = plt.Figure(figsize=(10.5, 5.6), dpi=100)
            ax = fig.add_subplot(111)
            ax.set_xlabel("device_ts_ms / 1000 (s)", fontsize=10)
            ax.set_ylabel("Angle (deg)", fontsize=10)
            ax.grid(True, alpha=0.28)

            cmap = plt.get_cmap("tab10")
            plotted = False
            vals: List[float] = []
            x_min = float("inf")
            x_max = float("-inf")
            summary_lines: List[str] = []
            for idx in joint_indices:
                t_rel = time_map.get(idx, [])
                y_actual = actual_map.get(idx, [])
                y_target = target_map.get(idx, [])
                if len(y_actual) != len(t_rel) or len(y_target) != len(t_rel):
                    continue
                if not t_rel:
                    continue
                plotted = True
                x_min = min(x_min, float(t_rel[0]))
                x_max = max(x_max, float(t_rel[-1]))
                base_color = cmap(idx % 10)
                ax.plot(
                    t_rel,
                    y_actual,
                    linestyle="-",
                    linewidth=1.8,
                    color=base_color,
                    label=f"J{idx:02d} Actual",
                )
                ax.plot(
                    t_rel,
                    y_target,
                    linestyle="--",
                    linewidth=1.6,
                    color=base_color,
                    alpha=0.75,
                    label=f"J{idx:02d} Target",
                )
                vals.extend([float(v) for v in y_actual if np.isfinite(v)])
                vals.extend([float(v) for v in y_target if np.isfinite(v)])
                lag_info = self._estimate_series_lag(t_rel, y_target, y_actual)
                if lag_info:
                    summary_lines.append(
                        f"J{idx:02d}: lag~{lag_info['lag_s'] * 1000.0:.0f} ms, "
                        f"phase~{lag_info['phase_deg']:.1f} deg @ {lag_info['freq_hz']:.2f} Hz"
                    )

            if not plotted:
                raise ValueError(f"No plottable packet series for joints: {joint_indices}")

            if vals:
                vmin = min(vals)
                vmax = max(vals)
                if np.isfinite(vmin) and np.isfinite(vmax):
                    if math.isclose(vmin, vmax):
                        pad = 1.0 if abs(vmin) < 1.0 else abs(vmin) * 0.1
                        ax.set_ylim(vmin - pad, vmax + pad)
                    else:
                        pad = (vmax - vmin) * 0.08
                        ax.set_ylim(vmin - pad, vmax + pad)

            if np.isfinite(x_min) and np.isfinite(x_max) and x_max > x_min:
                ax.set_xlim(x_min, x_max)

            title = f"Joint Debug Packets: {', '.join([f'J{idx:02d}' for idx in joint_indices])}"
            if summary_lines:
                title += f"\n{summary_lines[0]}"
            ax.set_title(title, fontsize=12)
            if summary_lines:
                ax.text(
                    0.01,
                    0.99,
                    "\n".join(summary_lines),
                    transform=ax.transAxes,
                    va="top",
                    ha="left",
                    fontsize=9,
                    bbox={"facecolor": "white", "alpha": 0.78, "edgecolor": "#cccccc"},
                )
            ax.legend(loc="upper right", fontsize=9, ncol=1)
            fig.tight_layout()
            fig.savefig(out_path)
            plt.close(fig)

            suffix = f" ({reason})" if reason else ""
            if summary_lines:
                self._emit_action(
                    f"Joint debug packet plot saved{suffix}: {out_path} | {summary_lines[0]}"
                )
            else:
                self._emit_action(f"Joint debug packet plot saved{suffix}: {out_path}")
            return out_path
        except Exception as exc:
            self._emit_action(f"Joint debug packet plot save failed: {exc}")
            return None

    def _open_joint_rt_window(self) -> None:
        win = getattr(self, "_joint_rt_window", None)
        if win is not None:
            try:
                if win.winfo_exists():
                    if getattr(self, "_joint_rt_canvas", None) is None:
                        self._setup_joint_realtime_plot(win)
                    win.deiconify()
                    win.lift()
                    win.focus_set()
                    return
            except Exception:
                pass

        win = tk.Toplevel(self.root)
        win.title("Joint Real-Time Curve")
        win.geometry("760x360")
        win.protocol("WM_DELETE_WINDOW", self._on_joint_rt_window_close)
        self._joint_rt_window = win
        self._setup_joint_realtime_plot(win)

    def _on_joint_rt_window_close(self) -> None:
        self._save_joint_rt_plot_snapshot(reason="window close")
        self._destroy_joint_rt_plot()

    def _setup_joint_realtime_plot(self, parent) -> None:
        self._destroy_joint_rt_plot(close_window=False)
        box = ttk.LabelFrame(parent, text="Joint Real-Time Curve")
        self._joint_rt_container = box
        box.pack(fill=tk.BOTH, expand=True, padx=6, pady=(2, 4))

        top = ttk.Frame(box)
        top.pack(fill=tk.X, padx=6, pady=(6, 2))
        ttk.Label(top, text="Joints:").pack(side=tk.LEFT)
        values = ["None"] + [f"J{i:02d}" for i in range(ENCODER_COUNT)]
        if len(getattr(self, "_joint_rt_joint_vars", [])) != JOINT_RT_MAX_PLOT_JOINTS:
            self._joint_rt_joint_vars = [tk.StringVar(value="None") for _ in range(JOINT_RT_MAX_PLOT_JOINTS)]
        for i, var in enumerate(self._joint_rt_joint_vars):
            var.set(self._normalize_joint_rt_value(var.get()))
            combo = ttk.Combobox(
                top,
                textvariable=var,
                values=values,
                state="readonly",
                width=6,
            )
            combo.pack(side=tk.LEFT, padx=(4 if i == 0 else 2, 2))
            combo.bind("<<ComboboxSelected>>", lambda _e, slot=i: self._on_joint_rt_selection_changed(slot))
        ttk.Button(top, text="Clear", width=8, command=self._on_joint_rt_clear).pack(side=tk.LEFT)

        self._joint_rt_fig = plt.Figure(figsize=(7.2, 3.0), dpi=100)
        self._joint_rt_ax = self._joint_rt_fig.add_subplot(111)
        self._joint_rt_lines = {}
        self._joint_rt_selection_sig = None
        self._joint_rt_canvas = FigureCanvasTkAgg(self._joint_rt_fig, box)
        self._joint_rt_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=6, pady=(0, 6))

        self._clear_joint_rt_history()
        self._update_joint_rt_plot(force=True)

    def _destroy_joint_rt_plot(self, close_window: bool = True) -> None:
        container = getattr(self, "_joint_rt_container", None)
        if container is not None:
            try:
                container.destroy()
            except Exception:
                pass
        self._joint_rt_container = None
        canvas = getattr(self, "_joint_rt_canvas", None)
        if canvas is not None:
            try:
                widget = canvas.get_tk_widget()
                widget.destroy()
            except Exception:
                pass
        fig = getattr(self, "_joint_rt_fig", None)
        if fig is not None:
            try:
                plt.close(fig)
            except Exception:
                pass
        self._joint_rt_canvas = None
        self._joint_rt_fig = None
        self._joint_rt_ax = None
        self._joint_rt_line_target = None
        self._joint_rt_line_actual = None
        self._joint_rt_lines = {}
        self._joint_rt_selection_sig = None
        if close_window:
            win = getattr(self, "_joint_rt_window", None)
            self._joint_rt_window = None
            if win is not None:
                try:
                    if win.winfo_exists():
                        win.destroy()
                except Exception:
                    pass

    @staticmethod
    def _normalize_joint_rt_value(value: str) -> str:
        text = str(value or "").strip().upper()
        if text in ("", "NONE", "N", "-"):
            return "None"
        if text.startswith("J"):
            try:
                idx = int(text[1:])
            except ValueError:
                return "None"
            if 0 <= idx < ENCODER_COUNT:
                return f"J{idx:02d}"
        return "None"

    def _selected_joint_rt_indices(self) -> List[int]:
        selected: List[int] = []
        for var in getattr(self, "_joint_rt_joint_vars", []):
            norm = self._normalize_joint_rt_value(var.get())
            var.set(norm)
            if norm == "None":
                continue
            idx = int(norm[1:])
            if idx not in selected:
                selected.append(idx)
        return selected[:JOINT_RT_MAX_PLOT_JOINTS]

    def _on_joint_rt_selection_changed(self, slot: Optional[int] = None) -> None:
        if slot is not None and 0 <= int(slot) < len(self._joint_rt_joint_vars):
            slot_idx = int(slot)
            current = self._normalize_joint_rt_value(self._joint_rt_joint_vars[slot_idx].get())
            self._joint_rt_joint_vars[slot_idx].set(current)
            if current != "None":
                for i, var in enumerate(self._joint_rt_joint_vars):
                    if i == slot_idx:
                        continue
                    if self._normalize_joint_rt_value(var.get()) == current:
                        self._joint_rt_joint_vars[slot_idx].set("None")
                        if hasattr(self, "status_var"):
                            self.status_var.set(f"Plot selection duplicate joint: {current} already selected")
                        break
        selected = self._selected_joint_rt_indices()
        if selected:
            self._joint_rt_joint_index = int(selected[0])
            self._joint_rt_joint_var.set(f"J{self._joint_rt_joint_index:02d}")
        else:
            self._joint_rt_joint_index = 0
            self._joint_rt_joint_var.set("J00")
        self._clear_joint_rt_history()
        self._update_joint_rt_plot(force=True)

    def _on_joint_rt_clear(self) -> None:
        self._clear_joint_rt_history()
        self._update_joint_rt_plot(force=True)

    def _clear_joint_rt_history(self) -> None:
        self._joint_rt_series_t.clear()
        self._joint_rt_series_actual.clear()
        self._joint_rt_series_target.clear()
        self._joint_rt_t0 = 0.0
        self._joint_rt_t0_map.clear()
        self._joint_rt_last_plot_ts = 0.0

    def _ensure_joint_rt_series(self, joint_idx: int) -> None:
        if joint_idx not in self._joint_rt_series_t:
            self._joint_rt_series_t[joint_idx] = deque(maxlen=1800)
            self._joint_rt_series_actual[joint_idx] = deque(maxlen=1800)
            self._joint_rt_series_target[joint_idx] = deque(maxlen=1800)

    def _append_joint_rt_sample(self, s: HandModel) -> None:
        selected = self._selected_joint_rt_indices()
        if not selected:
            return

        mode = self._ctrl_mode_var.get() if hasattr(self, "_ctrl_mode_var") else "joint"
        targets: Optional[List[float]] = None
        if mode in ("joint", "test"):
            try:
                targets = self._resolved_joint_targets()
            except Exception:
                targets = None

        for idx in selected:
            self._ensure_joint_rt_series(idx)
            t_rel = float("nan")
            actual = float("nan")
            target = float("nan")

            debug_ts_ms = 0
            if idx < len(getattr(s, "joint_debug_timestamp_ms", [])):
                try:
                    debug_ts_ms = int(s.joint_debug_timestamp_ms[idx])
                except Exception:
                    debug_ts_ms = 0

            if debug_ts_ms > 0:
                if idx not in self._joint_rt_t0_map:
                    self._joint_rt_t0_map[idx] = float(debug_ts_ms)
                t_rel = (float(debug_ts_ms) - self._joint_rt_t0_map[idx]) / 1000.0
                if idx < len(getattr(s, "joint_debug_actual_deg", [])):
                    try:
                        actual = float(s.joint_debug_actual_deg[idx])
                    except Exception:
                        actual = float("nan")
                if idx < len(getattr(s, "joint_debug_target_deg", [])):
                    try:
                        target = float(s.joint_debug_target_deg[idx])
                    except Exception:
                        target = float("nan")
                if self._joint_rt_series_t[idx] and t_rel <= float(self._joint_rt_series_t[idx][-1]):
                    continue
            else:
                now = time.time()
                if idx not in self._joint_rt_t0_map:
                    self._joint_rt_t0_map[idx] = now
                t_rel = now - self._joint_rt_t0_map[idx]
                if idx < len(getattr(s, "angles", [])):
                    try:
                        actual = float(s.angles[idx])
                    except Exception:
                        actual = float("nan")
                if targets is not None and idx < len(targets):
                    try:
                        target = float(targets[idx])
                    except Exception:
                        target = float("nan")

            self._joint_rt_series_t[idx].append(float(t_rel))
            self._joint_rt_series_actual[idx].append(actual)
            self._joint_rt_series_target[idx].append(target)

    def _update_joint_rt_plot(self, force: bool = False) -> None:
        ax = getattr(self, "_joint_rt_ax", None)
        canvas = getattr(self, "_joint_rt_canvas", None)
        if ax is None or canvas is None:
            return

        now = time.time()
        if not force and (now - float(getattr(self, "_joint_rt_last_plot_ts", 0.0))) < float(self._joint_rt_plot_interval):
            return
        self._joint_rt_last_plot_ts = now

        selected = self._selected_joint_rt_indices()
        selection_sig = tuple(selected)
        needs_rebuild = (
            force
            or getattr(self, "_joint_rt_selection_sig", None) != selection_sig
            or not hasattr(self, "_joint_rt_lines")
        )
        if needs_rebuild:
            ax.clear()
            ax.set_xlabel("Time (s)", fontsize=9)
            ax.set_ylabel("Angle (deg)", fontsize=9)
            ax.grid(True, alpha=0.28)
            self._joint_rt_lines = {}
            self._joint_rt_selection_sig = selection_sig

        x_max = 0.0
        vals: List[float] = []
        plotted = False
        cmap = plt.get_cmap("tab10")
        for idx in selected:
            t = list(self._joint_rt_series_t.get(idx, []))
            y_actual = list(self._joint_rt_series_actual.get(idx, []))
            y_target = list(self._joint_rt_series_target.get(idx, []))
            if not t:
                continue
            plotted = True
            x_max = max(x_max, float(t[-1]))
            base_color = cmap(idx % 10)
            actual_key = (idx, "actual")
            target_key = (idx, "target")
            if actual_key not in self._joint_rt_lines:
                (self._joint_rt_lines[actual_key],) = ax.plot(
                    [],
                    [],
                    linestyle="-",
                    linewidth=1.8,
                    color=base_color,
                    label=f"J{idx:02d} Actual",
                )
            if target_key not in self._joint_rt_lines:
                (self._joint_rt_lines[target_key],) = ax.plot(
                    [],
                    [],
                    linestyle="--",
                    linewidth=1.6,
                    color=base_color,
                    alpha=0.55,
                    label=f"J{idx:02d} Target",
                )
            self._joint_rt_lines[actual_key].set_data(t, y_actual)
            self._joint_rt_lines[target_key].set_data(t, y_target)
            for v in y_target:
                if np.isfinite(v):
                    vals.append(float(v))
            for v in y_actual:
                if np.isfinite(v):
                    vals.append(float(v))

        if x_max > 0.0:
            x_min = max(0.0, x_max - float(self._joint_rt_window_sec))
            ax.set_xlim(x_min, max(x_min + 0.8, x_max + 0.05))
        else:
            ax.set_xlim(0.0, float(self._joint_rt_window_sec))

        if vals:
            y_min = min(vals)
            y_max = max(vals)
            if y_min == y_max:
                y_max = y_min + 1.0
            margin = (y_max - y_min) * 0.12
            ax.set_ylim(y_min - margin, y_max + margin)
        else:
            ax.set_ylim(-1.0, 1.0)

        mode = self._ctrl_mode_var.get() if hasattr(self, "_ctrl_mode_var") else "joint"
        suffix = " (target N/A in motor mode)" if mode == "motor" else ""
        if selected:
            selection_label = ", ".join([f"J{i:02d}" for i in selected])
            ax.set_title(f"Target vs Actual: {selection_label}{suffix}", fontsize=9)
        else:
            ax.set_title(f"Target vs Actual: None selected{suffix}", fontsize=9)
        if plotted:
            ax.legend(loc="upper right", fontsize=8, ncol=2)
        canvas.draw_idle()



class _ComponentProxy:
    def __init__(self, owner):
        self.owner = owner
        self.root = owner.root

    def __getattr__(self, name):
        return getattr(self.owner, name)


class JointRealtimePlotView(_ComponentProxy, JointRealtimePlotMixin):
    def build(self, parent) -> None:
        self._setup_joint_realtime_plot(parent)

    def update(self, state: HandModel) -> None:
        self._append_joint_rt_sample(state)
        self._update_joint_rt_plot()

    def clear(self) -> None:
        self._clear_joint_rt_history()

    def close(self) -> None:
        self._destroy_joint_rt_plot(close_window=False)
