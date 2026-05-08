import os
import tkinter as tk
from tkinter import ttk
from typing import List, Optional, Tuple

import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from data_models import HandModel
from protocol import ENCODER_COUNT
from plot_hand_from_urdf import (
    DISPLAY_TRANSFORM,
    POSE_VIEW_BG,
    UrdfKinematics,
    apply_display_transform,
    apply_mplot3d_box_aspect_fill_widget,
    apply_mplot3d_camera_zoom,
    apply_pose_view_background,
    compute_pose_view_camera_dist,
    draw_skeleton,
    fill_3d_axes_to_figure,
    load_motor_degrees_from_text,
    motor_deg_to_urdf_joint_dict,
)

DESKTOP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

class PoseViewMixin:
    def _should_use_realtime_pose(self) -> bool:
        # Runtime policy:
        # - No port/connection: use txt pose.
        # - Connected: use realtime angles.
        return self._is_comm_connected()

    def _setup_3d_display(self, parent):
        self._pose_static_scene_ready = False
        self._pose_view_dirty = True
        self._pose_link_artists = []
        self._pose_nodes_artist = None
        pose_frame = ttk.Frame(parent)
        pose_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 2))
        self.fig = plt.Figure(figsize=(6.5, 5.0), dpi=110, facecolor=POSE_VIEW_BG)
        self.ax = self.fig.add_axes((0.05, 0.10, 0.945, 0.82), projection="3d")
        self.canvas = FigureCanvasTkAgg(self.fig, pose_frame)
        cw = self.canvas.get_tk_widget()
        self._pose_canvas_widget = cw
        cw.pack(fill=tk.BOTH, expand=True)
        try:
            cw.configure(bg=POSE_VIEW_BG, highlightthickness=0)
        except (tk.TclError, AttributeError):
            pass
        cw.bind("<Configure>", self._on_pose_canvas_configure)
        self._pose_3d_target_dist = compute_pose_view_camera_dist()
        self._pose_3d_drag_dist_scale = 0.90
        self._pose_3d_rezoom_delay_ms = 500
        self._pose_3d_rezoom_after_drag_job = None
        self.canvas.mpl_connect("scroll_event", self._on_pose_3d_mpl_scroll)
        self.canvas.mpl_connect("motion_notify_event", self._on_pose_3d_mpl_motion)
        self.canvas.mpl_connect("button_release_event", self._on_pose_3d_mpl_release)

        urdf_path = os.path.join(DESKTOP_DIR, "hku_hand_v2_urdf", "urdf", "hand.urdf")
        try:
            self._urdf_kin = UrdfKinematics(urdf_path)
        except Exception as e:
            print(f"URDF load failed ({urdf_path}): {e}")
            self._urdf_kin = None
        self._draw_hand_model([0.0] * ENCODER_COUNT, None)

    def _on_pose_canvas_configure(self, event) -> None:
        """姿态 3D 区随窗口拉伸，Figure 尺寸与 Tk 画布一致。"""
        if event.widget is not getattr(self, "_pose_canvas_widget", None):
            return
        w, h = int(event.width), int(event.height)
        if w < 24 or h < 24:
            return
        key = (w, h)
        if key == getattr(self, "_pose_canvas_size", None):
            return
        self._pose_canvas_size = key
        dpi = float(self.fig.dpi)
        self.fig.set_size_inches(w / dpi, h / dpi)
        try:
            rw = int(round(float(self.fig.get_figwidth()) * dpi))
            rh = int(round(float(self.fig.get_figheight()) * dpi))
            if rw < w or rh < h:
                self.fig.set_size_inches((w + 1) / dpi, (h + 1) / dpi)
        except (TypeError, ValueError, AttributeError):
            pass
        fill_3d_axes_to_figure(self.ax, has_title=True)
        apply_pose_view_background(self.ax)
        apply_mplot3d_box_aspect_fill_widget(self.ax)
        apply_mplot3d_camera_zoom(self.ax, dist=self._pose_3d_target_dist)
        self._pose_view_dirty = True
        self.canvas.draw_idle()

    def _sync_pose_3d_target_dist_from_axes(self) -> None:
        ax = getattr(self, "ax", None)
        if ax is None or not hasattr(ax, "_dist"):
            return
        try:
            self._pose_3d_target_dist = float(ax._dist)
        except (TypeError, ValueError):
            pass

    def _cancel_pose_3d_rezoom_timer(self) -> None:
        job = getattr(self, "_pose_3d_rezoom_after_drag_job", None)
        if job is None:
            return
        try:
            self.root.after_cancel(job)
        except tk.TclError:
            pass
        self._pose_3d_rezoom_after_drag_job = None

    def _on_pose_3d_mpl_scroll(self, event) -> None:
        if event.inaxes != getattr(self, "ax", None):
            return
        self._cancel_pose_3d_rezoom_timer()
        self.root.after_idle(self._sync_pose_3d_target_dist_from_axes)

    @staticmethod
    def _pose_3d_mpl_left_drag(event) -> bool:
        if event.button == 1:
            return True
        btns = getattr(event, "buttons", None)
        if not btns:
            return False
        return 1 in btns

    def _on_pose_3d_mpl_motion(self, event) -> None:
        if event.inaxes != getattr(self, "ax", None):
            return
        if not self._pose_3d_mpl_left_drag(event):
            return
        self._cancel_pose_3d_rezoom_timer()
        drag_scale = float(getattr(self, "_pose_3d_drag_dist_scale", 1.0))
        drag_dist = float(self._pose_3d_target_dist) * max(0.5, min(1.0, drag_scale))
        apply_mplot3d_camera_zoom(self.ax, dist=drag_dist)

    def _on_pose_3d_mpl_release(self, event) -> None:
        if event.inaxes != getattr(self, "ax", None) or event.button != 1:
            return
        self._cancel_pose_3d_rezoom_timer()

        def rezoom_after_release() -> None:
            self._pose_3d_rezoom_after_drag_job = None
            apply_mplot3d_camera_zoom(self.ax, dist=self._pose_3d_target_dist)
            self.canvas.draw_idle()

        delay_ms = int(getattr(self, "_pose_3d_rezoom_delay_ms", 500))
        self._pose_3d_rezoom_after_drag_job = self.root.after(delay_ms, rezoom_after_release)

    def _safe_remove_artist(self, artist) -> None:
        if artist is None:
            return
        try:
            artist.remove()
        except (AttributeError, ValueError, NotImplementedError):
            pass

    def _get_generated_pose_angles(self) -> List[float]:
        """从 pose 文本缓存或重新加载；文件缺失或解析失败时退回全 0。"""
        path = self._pose_generate_path
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            if self._generated_pose_angles is not None:
                return self._generated_pose_angles
            print(f"pose_generate: 未找到文件 {path!r}，姿态用全 0", flush=True)
            return [0.0] * ENCODER_COUNT
        if self._generated_pose_mtime == mtime and self._generated_pose_angles is not None:
            return self._generated_pose_angles
        try:
            self._generated_pose_angles = load_motor_degrees_from_text(path)
        except Exception as e:
            print(f"pose_generate: 读取失败 {path!r}: {e}", flush=True)
            if self._generated_pose_angles is not None:
                return self._generated_pose_angles
            return [0.0] * ENCODER_COUNT
        self._generated_pose_mtime = mtime
        return self._generated_pose_angles

    def _draw_hand_model(
        self,
        angles: List[float],
        state: Optional[HandModel],
        *,
        generate: Optional[bool] = None,
    ):
        """根据 URDF 正向运动学绘制手部骨架。

        generate：None 时用实例属性 _pose_generate；True 强制从文本读角；False 强制用传入 angles。
        state 保留参数以便与旧调用兼容；触觉叠加已在独立子图中绘制。
        """
        _ = state
        if generate is None:
            use_gen = not self._should_use_realtime_pose()
        else:
            use_gen = bool(generate)
        if use_gen:
            angles = self._get_generated_pose_angles()
        if self._urdf_kin is None:
            self.ax.clear()
            apply_pose_view_background(self.ax)
            self.ax.set_title("URDF load failed")
            self.fig.canvas.draw_idle()
            return
        joint_pos = motor_deg_to_urdf_joint_dict(angles)
        # 大部分帧仅更新连杆/节点；只有视图变化（如拖拽、缩放、窗口尺寸变化）时才重建静态背景。
        self._ensure_pose_static_scene(joint_pos)
        self._update_pose_dynamic_artists(joint_pos)
        self.fig.canvas.draw_idle()

    def _clear_pose_dynamic_artists(self) -> None:
        for _pa, _ch, line in getattr(self, "_pose_link_artists", []):
            self._safe_remove_artist(line)
        self._pose_link_artists = []
        n = getattr(self, "_pose_nodes_artist", None)
        if n is not None:
            self._safe_remove_artist(n)
        self._pose_nodes_artist = None

    def _ensure_pose_static_scene(self, joint_pos: dict) -> None:
        if self._pose_static_scene_ready and not self._pose_view_dirty:
            return

        preserve_user_view = bool(self._pose_static_scene_ready)
        prev_elev = None
        prev_azim = None
        prev_roll = None
        prev_dist = None
        if preserve_user_view:
            try:
                prev_elev = float(self.ax.elev)
                prev_azim = float(self.ax.azim)
                prev_dist = float(self._pose_3d_target_dist)
                if hasattr(self.ax, "roll"):
                    prev_roll = float(self.ax.roll)
            except (TypeError, ValueError, AttributeError):
                prev_elev = None
                prev_azim = None
                prev_roll = None
                prev_dist = None

        # draw_skeleton 内部会清空 axes；先清理动态层避免对已脱管 artist 再次 remove。
        self._clear_pose_dynamic_artists()
        draw_skeleton(
            self.ax,
            self._urdf_kin,
            joint_pos,
            display_transform=DISPLAY_TRANSFORM,
            plot_title="Pose (URDF skeleton)",
            draw_links_nodes=False,
        )

        if preserve_user_view and prev_elev is not None and prev_azim is not None:
            try:
                if prev_roll is not None:
                    self.ax.view_init(elev=prev_elev, azim=prev_azim, roll=prev_roll)
                else:
                    self.ax.view_init(elev=prev_elev, azim=prev_azim)
            except TypeError:
                self.ax.view_init(elev=prev_elev, azim=prev_azim)
            if prev_dist is not None:
                apply_mplot3d_camera_zoom(self.ax, dist=prev_dist)
                self._pose_3d_target_dist = float(prev_dist)
            else:
                self._sync_pose_3d_target_dist_from_axes()
        else:
            cd = getattr(self.ax, "_pose_view_cam_dist", None)
            if cd is not None:
                try:
                    self._pose_3d_target_dist = float(cd)
                except (TypeError, ValueError):
                    self._sync_pose_3d_target_dist_from_axes()
            else:
                self._sync_pose_3d_target_dist_from_axes()

        self._pose_static_scene_ready = True
        self._pose_view_dirty = False

    def _update_pose_dynamic_artists(self, joint_pos: dict) -> None:
        poses = apply_display_transform(self._urdf_kin.compute_link_poses(joint_pos), DISPLAY_TRANSFORM)
        if not self._pose_link_artists:
            for j in self._urdf_kin.joints:
                pa, ch = j["parent"], j["child"]
                if pa not in poses or ch not in poses:
                    continue
                p0 = poses[pa][:3, 3]
                p1 = poses[ch][:3, 3]
                (line,) = self.ax.plot(
                    [p0[0], p1[0]],
                    [p0[1], p1[1]],
                    [p0[2], p1[2]],
                    color="black",
                    linewidth=2.0,
                    zorder=20,
                )
                self._pose_link_artists.append((pa, ch, line))
        else:
            for pa, ch, line in self._pose_link_artists:
                if pa not in poses or ch not in poses:
                    continue
                p0 = poses[pa][:3, 3]
                p1 = poses[ch][:3, 3]
                line.set_data_3d([p0[0], p1[0]], [p0[1], p1[1]], [p0[2], p1[2]])

        xs: List[float] = []
        ys: List[float] = []
        zs: List[float] = []
        for T in poses.values():
            p = T[:3, 3]
            xs.append(float(p[0]))
            ys.append(float(p[1]))
            zs.append(float(p[2]))
        if self._pose_nodes_artist is None:
            self._pose_nodes_artist = self.ax.scatter(xs, ys, zs, s=12, c="red", depthshade=True, zorder=21)
        else:
            self._pose_nodes_artist._offsets3d = (xs, ys, zs)



class _ComponentProxy:
    def __init__(self, owner):
        self.owner = owner
        self.root = owner.root

    def __getattr__(self, name):
        return getattr(self.owner, name)


class Pose3DView(_ComponentProxy, PoseViewMixin):
    def build(self, parent) -> None:
        self._setup_3d_display(parent)

    def update(self, state: HandModel) -> None:
        self._draw_hand_model(state.angles, None)

    def draw_angles(self, angles, state=None, *, generate=None) -> None:
        self._draw_hand_model(angles, state, generate=generate)

    def clear(self) -> None:
        self._draw_hand_model([0.0] * ENCODER_COUNT, None, generate=False)

    def close(self) -> None:
        self._cancel_pose_3d_rezoom_timer()
