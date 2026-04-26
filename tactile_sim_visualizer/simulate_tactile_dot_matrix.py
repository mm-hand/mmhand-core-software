#!/usr/bin/env python3
"""
Tactile simulator visualizer (standalone)

Reference behavior:
- sensor shapes match desktop tactile model
  tip: 5x5, pad1: 4x13, pad2: 4x13
- 5 fingers x 3 sensors, animated in dot-matrix style
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from typing import List, Tuple

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

FINGER_NAMES = ["thumb", "index", "middle", "ring", "little"]
SENSOR_NAMES = ["tip_5x5", "pad1_4x13", "pad2_4x13"]
SENSOR_SHAPES: List[Tuple[int, int]] = [(5, 5), (4, 13), (4, 13)]


@dataclass
class FrameData:
    # data[finger][sensor] -> (rows, cols, 3)
    data: List[List[np.ndarray]]


class TactileSignalGenerator:
    def __init__(self, seed: int, boost: float, speed: float) -> None:
        self.rng = np.random.default_rng(seed)
        self.boost = float(max(boost, 0.1))
        self.speed = float(max(speed, 0.1))

    def _sensor_tensor(self, t: float, finger_idx: int, sensor_idx: int, rows: int, cols: int) -> np.ndarray:
        y, x = np.mgrid[0:rows, 0:cols]
        phase = (t * 1.15 * self.speed) + finger_idx + (sensor_idx * 1.30)
        pulse = math.sin(t * 2.8 * self.speed + 0.6 * finger_idx + 0.4 * sensor_idx)

        # Keep thumb/index slightly stronger, similar to practical demo behavior.
        gain = 1.40 if finger_idx in (0, 1) else 1.00
        base_z = (0.30 if sensor_idx == 0 else 0.14) * gain

        cx = (cols - 1) * 0.5 + 0.95 * math.cos(phase)
        cy = (rows - 1) * 0.5 + 0.75 * math.sin(phase * 0.9)
        sigma = 0.95 if sensor_idx == 0 else 1.35
        bump = np.exp(-((x - cx) ** 2 + (y - cy) ** 2) / (2.0 * sigma * sigma))

        fx = self.boost * ((0.16 * np.sin(phase + 0.20 * x)) + (0.06 * (x - cx) / max(cols, 1)))
        fy = self.boost * ((0.16 * np.cos(phase * 0.72 + 0.16 * y)) + (0.06 * (y - cy) / max(rows, 1)))
        fz = (
            base_z
            + 0.85 * bump
            + 0.22 * pulse
            + 0.10 * np.sin(phase * 1.35 + 0.15 * x - 0.08 * y)
        ) * self.boost

        noise = self.rng.normal(0.0, 0.015, size=(rows, cols, 3))

        out = np.zeros((rows, cols, 3), dtype=float)
        out[:, :, 0] = np.clip(fx + noise[:, :, 0], -1.0, 1.0)
        out[:, :, 1] = np.clip(fy + noise[:, :, 1], -1.0, 1.0)
        out[:, :, 2] = np.clip(fz + noise[:, :, 2], 0.0, 2.0)
        return out

    def frame(self, t: float) -> FrameData:
        fingers: List[List[np.ndarray]] = []
        for fi in range(5):
            sensor_group: List[np.ndarray] = []
            for si, (rows, cols) in enumerate(SENSOR_SHAPES):
                sensor_group.append(self._sensor_tensor(t, fi, si, rows, cols))
            fingers.append(sensor_group)
        return FrameData(data=fingers)


class DotMatrixVisualizer:
    def __init__(self, fps: int, axis_name: str, seed: int, boost: float, speed: float) -> None:
        self.fps = fps
        self.axis_name = axis_name
        self.axis_idx = {"x": 0, "y": 1, "z": 2}[axis_name]
        self.generator = TactileSignalGenerator(seed, boost=boost, speed=speed)

        self.paused = False
        self.frame_idx = 0

        if self.axis_idx == 2:
            self.vmin, self.vmax = 0.0, 2.0
            self.cmap = "inferno"
        else:
            self.vmin, self.vmax = -1.0, 1.0
            self.cmap = "coolwarm"

        self.fig, self.axes = plt.subplots(3, 5, figsize=(15, 8), constrained_layout=True)
        self.scatters = []

        for row in range(3):
            scatter_row = []
            for col in range(5):
                ax = self.axes[row, col]
                rows, cols = SENSOR_SHAPES[row]
                yy, xx = np.mgrid[0:rows, 0:cols]
                vals = np.zeros(rows * cols, dtype=float)

                s = ax.scatter(
                    xx.flatten(),
                    yy.flatten(),
                    c=vals,
                    cmap=self.cmap,
                    vmin=self.vmin,
                    vmax=self.vmax,
                    s=260,
                    marker="s",
                    edgecolors="k",
                    linewidths=0.25,
                )

                ax.set_xlim(-0.75, cols - 0.25)
                ax.set_ylim(rows - 0.25, -0.75)
                ax.set_xticks(range(cols))
                ax.set_yticks(range(rows))
                ax.grid(alpha=0.25, linewidth=0.4)
                ax.set_aspect("equal")
                ax.tick_params(labelsize=6)
                if row == 0:
                    ax.set_title(FINGER_NAMES[col], fontsize=10)
                if col == 0:
                    ax.set_ylabel(SENSOR_NAMES[row], fontsize=9)

                scatter_row.append(s)
            self.scatters.append(scatter_row)

        self.status_text = self.fig.suptitle("", fontsize=11)
        cbar = self.fig.colorbar(self.scatters[0][0], ax=self.axes, location="right", shrink=0.85)
        cbar.set_label(f"axis={self.axis_name}")

        self.fig.canvas.mpl_connect("key_press_event", self._on_key_press)

    def _on_key_press(self, event) -> None:
        if event.key == " ":
            self.paused = not self.paused

    def _update(self, _frame_number: int):
        if not self.paused:
            self.frame_idx += 1
        t = self.frame_idx / max(self.fps, 1)

        frame = self.generator.frame(t)

        mean_values = []
        for col in range(5):
            for row in range(3):
                mat = frame.data[col][row][:, :, self.axis_idx]
                self.scatters[row][col].set_array(mat.flatten())
                mean_values.append(float(np.mean(mat)))

        global_mean = sum(mean_values) / max(len(mean_values), 1)
        self.status_text.set_text(
            f"t={t:6.2f}s  frame={self.frame_idx:05d}  axis={self.axis_name}  mean={global_mean:+.4f}  [space]=pause"
        )

        artists = [self.status_text]
        for row in self.scatters:
            artists.extend(row)
        return artists

    def run(self) -> None:
        interval_ms = 1000 / max(self.fps, 1)
        self.ani = FuncAnimation(self.fig, self._update, interval=interval_ms, blit=False)
        plt.show()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Standalone tactile dot-matrix simulator")
    parser.add_argument("--fps", type=int, default=20, help="animation FPS, default 20")
    parser.add_argument("--axis", choices=["x", "y", "z"], default="z", help="visualized axis")
    parser.add_argument("--seed", type=int, default=20260424, help="random seed")
    parser.add_argument("--boost", type=float, default=2.2, help="signal amplitude boost, default 2.2")
    parser.add_argument("--speed", type=float, default=1.8, help="animation speed multiplier, default 1.8")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    viz = DotMatrixVisualizer(
        fps=args.fps,
        axis_name=args.axis,
        seed=args.seed,
        boost=args.boost,
        speed=args.speed,
    )
    viz.run()


if __name__ == "__main__":
    main()
