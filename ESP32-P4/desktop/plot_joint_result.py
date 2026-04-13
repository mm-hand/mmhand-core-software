#!/usr/bin/env python3
from __future__ import annotations

import csv
import math
import os
import re
import tkinter as tk
from dataclasses import dataclass
from tkinter import colorchooser, filedialog, messagebox, ttk
from typing import Dict, List, Optional

import matplotlib

matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


NA_MODE_MARKER = "NA_MODE"
CURR_COL_PATTERN = re.compile(r"^joint_curr_deg_j(\d{2})$")
TGT_COL_PATTERN = re.compile(r"^joint_target_deg_j(\d{2})$")


def _default_run_data_dir() -> str:
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(project_root, "run_data")


def _latest_csv_file(run_data_dir: str) -> str:
    if not os.path.isdir(run_data_dir):
        raise FileNotFoundError(f"run_data directory does not exist: {run_data_dir}")
    candidates = [
        os.path.join(run_data_dir, name)
        for name in os.listdir(run_data_dir)
        if name.lower().endswith(".csv") and os.path.isfile(os.path.join(run_data_dir, name))
    ]
    if not candidates:
        raise FileNotFoundError(f"No CSV files found in: {run_data_dir}")
    candidates.sort(key=os.path.getmtime, reverse=True)
    return candidates[0]


def _safe_float(value: str) -> float:
    try:
        v = float(value)
        if math.isfinite(v):
            return v
    except Exception:
        pass
    return float("nan")


def _joint_label(joint_idx: int) -> str:
    return f"J{joint_idx:02d}"


def _joint_idx_from_label(label: str) -> int:
    text = str(label).strip().upper()
    if not text.startswith("J"):
        raise ValueError(f"Invalid joint label: {label}")
    idx = int(text[1:])
    if idx < 0:
        raise ValueError(f"Invalid joint label: {label}")
    return idx


def _best_text_color(bg_hex: str) -> str:
    color = bg_hex.lstrip("#")
    if len(color) != 6:
        return "black"
    r = int(color[0:2], 16)
    g = int(color[2:4], 16)
    b = int(color[4:6], 16)
    luminance = 0.299 * r + 0.587 * g + 0.114 * b
    return "black" if luminance >= 160 else "white"


@dataclass
class RunData:
    csv_path: str
    time_rel: List[float]
    joint_indices: List[int]
    current_deg: Dict[int, List[float]]
    target_deg: Dict[int, List[float]]


@dataclass
class JointRow:
    frame: ttk.Frame
    joint_var: tk.StringVar
    combo: ttk.Combobox
    color_btn: tk.Button
    remove_btn: ttk.Button
    color_hex: str
    prev_joint: str


def _parse_joint_columns(fieldnames: List[str]) -> tuple[List[int], Dict[int, str], Dict[int, str]]:
    curr_map: Dict[int, str] = {}
    tgt_map: Dict[int, str] = {}
    for field in fieldnames:
        m_curr = CURR_COL_PATTERN.match(field)
        if m_curr:
            curr_map[int(m_curr.group(1))] = field
        m_tgt = TGT_COL_PATTERN.match(field)
        if m_tgt:
            tgt_map[int(m_tgt.group(1))] = field
    joint_indices = sorted(set(curr_map).intersection(tgt_map))
    if not joint_indices:
        raise ValueError("CSV has no matching joint_curr_deg_jXX / joint_target_deg_jXX columns.")
    return joint_indices, curr_map, tgt_map


def load_run_data(csv_path: str) -> RunData:
    with open(csv_path, "r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames:
            raise ValueError(f"CSV header is empty: {csv_path}")
        if "ts_wall" not in reader.fieldnames:
            raise ValueError(f"CSV is missing required column: ts_wall ({csv_path})")

        joint_indices, curr_map, tgt_map = _parse_joint_columns(reader.fieldnames)
        current_deg: Dict[int, List[float]] = {idx: [] for idx in joint_indices}
        target_deg: Dict[int, List[float]] = {idx: [] for idx in joint_indices}
        ts_wall: List[float] = []

        for row in reader:
            t = _safe_float(str(row.get("ts_wall", "")))
            if not math.isfinite(t):
                continue
            ts_wall.append(t)
            for idx in joint_indices:
                current_deg[idx].append(_safe_float(str(row.get(curr_map[idx], ""))))
                raw_tgt = str(row.get(tgt_map[idx], "")).strip()
                if raw_tgt == "" or raw_tgt.upper() == NA_MODE_MARKER:
                    target_deg[idx].append(float("nan"))
                else:
                    target_deg[idx].append(_safe_float(raw_tgt))

    if not ts_wall:
        raise ValueError(f"CSV has no valid data rows: {csv_path}")

    t0 = ts_wall[0]
    time_rel = [t - t0 for t in ts_wall]
    return RunData(
        csv_path=os.path.abspath(csv_path),
        time_rel=time_rel,
        joint_indices=joint_indices,
        current_deg=current_deg,
        target_deg=target_deg,
    )


class JointResultViewerApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Joint Result Viewer")
        self.root.geometry("1180x760")

        self.run_data: Optional[RunData] = None
        self.rows: List[JointRow] = []
        self.default_colors = [
            "#1f77b4",
            "#d62728",
            "#2ca02c",
            "#ff7f0e",
            "#9467bd",
            "#17becf",
            "#8c564b",
            "#e377c2",
            "#7f7f7f",
            "#bcbd22",
        ]

        self.csv_path_var = tk.StringVar(value="")
        self.start_time_var = tk.StringVar(value="")
        self.end_time_var = tk.StringVar(value="")
        self.status_var = tk.StringVar(value="Ready.")

        self._build_ui()
        self._load_initial_data()

    def _build_ui(self) -> None:
        root_frame = ttk.Frame(self.root)
        root_frame.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)

        control_box = ttk.LabelFrame(root_frame, text="Controls")
        control_box.pack(fill=tk.X, side=tk.TOP)

        file_row = ttk.Frame(control_box)
        file_row.pack(fill=tk.X, padx=8, pady=(8, 6))
        ttk.Button(file_row, text="Select CSV", command=self._on_select_csv).pack(side=tk.LEFT)
        ttk.Label(file_row, textvariable=self.csv_path_var).pack(side=tk.LEFT, padx=10, fill=tk.X, expand=True)

        options_row = ttk.Frame(control_box)
        options_row.pack(fill=tk.X, padx=8, pady=(0, 8))
        ttk.Label(options_row, text="Start (s)").pack(side=tk.LEFT)
        start_entry = ttk.Entry(options_row, textvariable=self.start_time_var, width=10)
        start_entry.pack(side=tk.LEFT, padx=(4, 12))
        ttk.Label(options_row, text="End (s)").pack(side=tk.LEFT)
        end_entry = ttk.Entry(options_row, textvariable=self.end_time_var, width=10)
        end_entry.pack(side=tk.LEFT, padx=(4, 12))
        ttk.Button(options_row, text="Refresh Plot", command=self._refresh_plot).pack(side=tk.LEFT)

        start_entry.bind("<Return>", lambda _e: self._refresh_plot())
        end_entry.bind("<Return>", lambda _e: self._refresh_plot())

        joint_box = ttk.LabelFrame(root_frame, text="Joints")
        joint_box.pack(fill=tk.X, side=tk.TOP, pady=(8, 8))
        joint_top = ttk.Frame(joint_box)
        joint_top.pack(fill=tk.X, padx=8, pady=(8, 4))
        ttk.Label(joint_top, text="Choose joints to plot.").pack(side=tk.LEFT)
        ttk.Button(joint_top, text="+", width=4, command=self._on_add_row).pack(side=tk.RIGHT)

        self.joint_rows_container = ttk.Frame(joint_box)
        self.joint_rows_container.pack(fill=tk.X, padx=8, pady=(0, 8))

        plot_box = ttk.LabelFrame(root_frame, text="Plot")
        plot_box.pack(fill=tk.BOTH, expand=True)
        self.figure = Figure(figsize=(10.8, 5.8), dpi=100)
        self.ax = self.figure.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.figure, plot_box)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=6, pady=6)

        status_label = ttk.Label(root_frame, textvariable=self.status_var, anchor="w")
        status_label.pack(fill=tk.X, side=tk.BOTTOM, pady=(6, 0))

    def _load_initial_data(self) -> None:
        run_data_dir = _default_run_data_dir()
        try:
            csv_path = _latest_csv_file(run_data_dir)
            self._load_csv(csv_path)
        except Exception:
            self.csv_path_var.set("No CSV loaded. Click 'Select CSV'.")
            self._add_joint_row(joint_label="J00")
            self._refresh_plot()

    def _set_status(self, text: str) -> None:
        self.status_var.set(text)

    def _on_select_csv(self) -> None:
        start_dir = _default_run_data_dir()
        file_path = filedialog.askopenfilename(
            title="Select runtime CSV",
            initialdir=start_dir if os.path.isdir(start_dir) else None,
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if not file_path:
            return
        self._load_csv(file_path)

    def _load_csv(self, csv_path: str) -> None:
        try:
            self.run_data = load_run_data(csv_path)
        except Exception as exc:
            messagebox.showerror("Load CSV", f"Failed to load CSV:\n{exc}")
            self._set_status("Load failed.")
            return

        assert self.run_data is not None
        self.csv_path_var.set(self.run_data.csv_path)
        self._sync_row_joint_options()
        if not self.rows:
            default_joint = _joint_label(self.run_data.joint_indices[0])
            self._add_joint_row(joint_label=default_joint)
        self._set_status(
            f"Loaded {os.path.basename(self.run_data.csv_path)} ({len(self.run_data.time_rel)} rows, "
            f"{len(self.run_data.joint_indices)} joints)."
        )
        self._refresh_plot()

    def _joint_labels(self) -> List[str]:
        if self.run_data and self.run_data.joint_indices:
            return [_joint_label(i) for i in self.run_data.joint_indices]
        return [_joint_label(i) for i in range(21)]

    def _next_default_color(self) -> str:
        return self.default_colors[len(self.rows) % len(self.default_colors)]

    def _next_default_joint(self) -> str:
        labels = self._joint_labels()
        used = {row.joint_var.get() for row in self.rows}
        for label in labels:
            if label not in used:
                return label
        return labels[0] if labels else "J00"

    def _style_color_button(self, button: tk.Button, color_hex: str) -> None:
        button.configure(
            bg=color_hex,
            activebackground=color_hex,
            fg=_best_text_color(color_hex),
            activeforeground=_best_text_color(color_hex),
            relief=tk.RAISED,
        )

    def _on_add_row(self) -> None:
        labels = self._joint_labels()
        if labels and len(self.rows) >= len(labels):
            messagebox.showinfo("Add Joint", "All available joints are already added.")
            return
        self._add_joint_row()
        self._refresh_plot()

    def _add_joint_row(self, joint_label: Optional[str] = None) -> None:
        labels = self._joint_labels()
        if not labels:
            messagebox.showwarning("Add Joint", "No joints are available for selection.")
            return

        selected_joint = joint_label or self._next_default_joint()
        if selected_joint not in labels:
            selected_joint = labels[0]

        color_hex = self._next_default_color()
        frame = ttk.Frame(self.joint_rows_container)
        frame.pack(fill=tk.X, pady=2)

        joint_var = tk.StringVar(value=selected_joint)
        ttk.Label(frame, text="Joint").pack(side=tk.LEFT)
        combo = ttk.Combobox(frame, textvariable=joint_var, values=labels, state="readonly", width=8)
        combo.pack(side=tk.LEFT, padx=(4, 12))

        ttk.Label(frame, text="Color").pack(side=tk.LEFT)
        color_btn = tk.Button(frame, text="Pick", width=7)
        color_btn.pack(side=tk.LEFT, padx=(4, 12))

        remove_btn = ttk.Button(frame, text="-", width=4)
        remove_btn.pack(side=tk.LEFT)

        row = JointRow(
            frame=frame,
            joint_var=joint_var,
            combo=combo,
            color_btn=color_btn,
            remove_btn=remove_btn,
            color_hex=color_hex,
            prev_joint=selected_joint,
        )
        combo.bind("<<ComboboxSelected>>", lambda _e, r=row: self._on_joint_changed(r))
        color_btn.configure(command=lambda r=row: self._on_pick_color(r))
        remove_btn.configure(command=lambda r=row: self._on_remove_row(r))

        self._style_color_button(color_btn, row.color_hex)
        self.rows.append(row)

    def _on_remove_row(self, row: JointRow) -> None:
        if len(self.rows) <= 1:
            messagebox.showinfo("Remove Joint", "At least one joint row is required.")
            return
        row.frame.destroy()
        self.rows = [r for r in self.rows if r is not row]
        self._refresh_plot()

    def _on_joint_changed(self, row: JointRow) -> None:
        new_joint = row.joint_var.get()
        for other in self.rows:
            if other is row:
                continue
            if other.joint_var.get() == new_joint:
                messagebox.showwarning("Duplicate Joint", f"{new_joint} is already selected.")
                row.joint_var.set(row.prev_joint)
                return
        row.prev_joint = new_joint
        self._refresh_plot()

    def _on_pick_color(self, row: JointRow) -> None:
        chosen = colorchooser.askcolor(color=row.color_hex, title=f"Choose color for {row.joint_var.get()}")
        picked = chosen[1] if chosen else None
        if not picked:
            return
        row.color_hex = picked
        self._style_color_button(row.color_btn, row.color_hex)
        self._refresh_plot()

    def _sync_row_joint_options(self) -> None:
        labels = self._joint_labels()
        for row in self.rows:
            row.combo.configure(values=labels)
            current = row.joint_var.get()
            if current not in labels:
                row.joint_var.set(labels[0] if labels else "J00")
            row.prev_joint = row.joint_var.get()

    def _parse_time_range(self) -> tuple[Optional[float], Optional[float]]:
        start_text = self.start_time_var.get().strip()
        end_text = self.end_time_var.get().strip()

        start = None if start_text == "" else float(start_text)
        end = None if end_text == "" else float(end_text)
        if start is not None and not math.isfinite(start):
            raise ValueError("Start time must be a finite number.")
        if end is not None and not math.isfinite(end):
            raise ValueError("End time must be a finite number.")
        if start is not None and end is not None and start > end:
            raise ValueError("Start time must be <= End time.")
        return start, end

    def _selected_rows(self) -> List[JointRow]:
        if self.run_data is None:
            return []
        selected: List[JointRow] = []
        seen: set[int] = set()
        for row in self.rows:
            idx = _joint_idx_from_label(row.joint_var.get())
            if idx not in self.run_data.joint_indices:
                continue
            if idx in seen:
                raise ValueError(f"Duplicate joint selected: {_joint_label(idx)}")
            seen.add(idx)
            selected.append(row)
        return selected

    def _refresh_plot(self) -> None:
        self.ax.clear()
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Angle (deg)")
        self.ax.grid(True, alpha=0.3)

        if self.run_data is None:
            self.ax.set_title("Joint Runtime Curve")
            self.ax.text(0.5, 0.5, "No CSV loaded", transform=self.ax.transAxes, ha="center", va="center")
            self.figure.tight_layout()
            self.canvas.draw_idle()
            return

        try:
            start_time, end_time = self._parse_time_range()
            selected_rows = self._selected_rows()
        except Exception as exc:
            messagebox.showerror("Plot Error", str(exc))
            self._set_status(f"Plot failed: {exc}")
            self.canvas.draw_idle()
            return

        if not selected_rows:
            self.ax.set_title(f"{os.path.basename(self.run_data.csv_path)} (no joints selected)")
            self.ax.text(0.5, 0.5, "No valid joint selected", transform=self.ax.transAxes, ha="center", va="center")
            self.figure.tight_layout()
            self.canvas.draw_idle()
            return

        indices = []
        for i, t in enumerate(self.run_data.time_rel):
            if start_time is not None and t < start_time:
                continue
            if end_time is not None and t > end_time:
                continue
            indices.append(i)

        if not indices:
            self.ax.set_title(f"{os.path.basename(self.run_data.csv_path)} (time range has no data)")
            self.ax.text(
                0.5,
                0.5,
                "No data points in selected time range",
                transform=self.ax.transAxes,
                ha="center",
                va="center",
            )
            self.figure.tight_layout()
            self.canvas.draw_idle()
            self._set_status("No data points in selected time range.")
            return

        time_vals = [self.run_data.time_rel[i] for i in indices]
        finite_y: List[float] = []
        for row in selected_rows:
            idx = _joint_idx_from_label(row.joint_var.get())
            curr_vals = [self.run_data.current_deg[idx][i] for i in indices]
            tgt_vals = [self.run_data.target_deg[idx][i] for i in indices]

            self.ax.plot(
                time_vals,
                curr_vals,
                color=row.color_hex,
                linestyle="-",
                linewidth=1.8,
                label=f"{_joint_label(idx)} Actual",
            )
            self.ax.plot(
                time_vals,
                tgt_vals,
                color=row.color_hex,
                linestyle="--",
                linewidth=1.7,
                alpha=0.72,
                label=f"{_joint_label(idx)} Target",
            )

            for v in curr_vals:
                if math.isfinite(v):
                    finite_y.append(v)
            for v in tgt_vals:
                if math.isfinite(v):
                    finite_y.append(v)

        x_min = start_time if start_time is not None else time_vals[0]
        x_max = end_time if end_time is not None else time_vals[-1]
        if x_min == x_max:
            x_max = x_min + 1e-3
        self.ax.set_xlim(x_min, x_max)

        if finite_y:
            y_min = min(finite_y)
            y_max = max(finite_y)
            if y_min == y_max:
                y_max = y_min + 1.0
            margin = (y_max - y_min) * 0.12
            self.ax.set_ylim(y_min - margin, y_max + margin)

        self.ax.legend(loc="upper right", fontsize=8, ncol=2)
        self.ax.set_title(f"Joint Runtime Curve: {os.path.basename(self.run_data.csv_path)}")
        self.figure.tight_layout()
        self.canvas.draw_idle()
        self._set_status(
            f"Plotted {len(selected_rows)} joints, time range "
            f"[{time_vals[0]:.3f}, {time_vals[-1]:.3f}] s, samples={len(indices)}."
        )


def main() -> int:
    root = tk.Tk()
    JointResultViewerApp(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
