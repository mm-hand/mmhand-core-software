import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  # 保持导入以兼容某些 matplotlib 版本
import matplotlib.animation as animation
import argparse
from typing import Sequence, Union


def _darken_color(color: str, factor: float = 0.6) -> str:
    """将颜色名转为较深色，用于区分指腹/指甲面"""
    import matplotlib.colors as mcolors
    try:
        rgb = np.array(mcolors.to_rgb(color))
        darker = rgb * factor
        return mcolors.to_hex(np.clip(darker, 0, 1))
    except Exception:
        return color


def _draw_xyz_indicator(ax, half_range: float, length_ratio: float = 0.08) -> None:
    """在角落绘制 XYZ 正方向小标签（右手螺旋：X×Y=Z）"""
    o = -half_range * 0.9
    L = half_range * length_ratio
    ax.quiver(o, o, o, L, 0, 0, color='red', arrow_length_ratio=0.25, linewidth=1)
    ax.quiver(o, o, o, 0, L, 0, color='green', arrow_length_ratio=0.25, linewidth=1)
    ax.quiver(o, o, o, 0, 0, L, color='blue', arrow_length_ratio=0.25, linewidth=1)
    ax.text(o + L * 1.3, o, o, 'X', fontsize=8, color='red')
    ax.text(o, o + L * 1.3, o, 'Y', fontsize=8, color='green')
    ax.text(o, o, o + L * 1.3, 'Z', fontsize=8, color='blue')


class RobotHandVisualizer:
    """机器人手可视化器 - 基于关节角度绘制3D手模型。

    坐标系：右手螺旋 (X×Y=Z)，显示为左手。
    X 朝前（掌心朝向），Y 向右（拇指→小指），Z 向上（指尖方向）。
    """
    
    def __init__(self):
        # 手指指节长度 (米)，参考解剖数据：掌骨+指骨
        # 拇指：第1掌骨45mm + 近节30mm + 远节22mm
        # 四指：近节+中节+远节 (食指84/中指93/无名指86/小指67mm)
        self.finger_lengths = {
            'thumb':  [0.045, 0.030, 0.022],  # 第1掌骨, 近节, 远节 (总长97mm)
            'index':  [0.042, 0.024, 0.018],  # 近节, 中节, 远节 (总长84mm)
            'middle': [0.046, 0.027, 0.020],  # 近节, 中节, 远节 (总长93mm)
            'ring':   [0.042, 0.025, 0.019],  # 近节, 中节, 远节 (总长86mm)
            'pinky':  [0.034, 0.017, 0.016]   # 近节, 中节, 远节 (总长67mm)
        }
        
        # 手掌尺寸 (米)：腕骨~25-30mm + 掌骨区，掌根在腕部
        # 局部坐标系 (X,Y,Z) = (厚度, 宽度, 长度)，与世界坐标系一致：Y 向右，Z 向上
        self.palm_thickness = 0.028  # X：厚度（掌骨整体高度）
        self.palm_width = 0.090     # Y：宽度（拇指→小指）
        self.palm_length = 0.100   # Z：长度（腕到指根）
        
        # 指根相对掌心中心的偏移 (米)，(X,Y,Z) = (厚度, 宽度, 长度)
        # 左手：掌心朝向观察者时，拇指在左侧 (-Y)，四指在右侧 (+Y)，Y 向右
        # 指根到掌根距离：thumb ~38mm, index 78mm, middle 83mm, ring 73mm, pinky 62mm
        palm_center_z = self.palm_length / 2
        self.finger_origins = {
            'thumb':  np.array([0.0, -0.038, -palm_center_z]),   # Y=-38mm 左侧（桡侧），缩短至掌根
            'index':  np.array([0.0, 0.012, 0.0273]),           # Y=+12mm 右侧，dist 78mm
            'middle': np.array([0.0, 0.028, 0.0281]),           # Y=+28mm，dist 83mm
            'ring':   np.array([0.0, 0.040, 0.0111]),            # Y=+40mm，dist 73mm
            'pinky':  np.array([0.0, 0.045, -0.0075])           # Y=+45mm，dist 62mm
        }
        
        # 关节旋转轴定义 (局部坐标系)
        # 对于大多数手指: MCP关节有2个自由度, PIP和DIP各有1个自由度
        # 简化起见，我们主要控制弯曲自由度
        
        # 关节角度存储 [MCP_abduct, MCP_flex, PIP_flex, DIP_flex]
        # 对于拇指稍有不同: [CMC_abduct, CMC_flex, MCP_flex, IP_flex]
        self.joint_angles = {
            'thumb':  np.array([0.1, 0.0, 0.0, 0.0]),   # 初始张开，与四指一致
            'index':  np.array([0.1, 0.8, 0.6, 0.4]),
            'middle': np.array([0.0, 0.5, 0.5, 0.3]),
            'ring':   np.array([-0.1, 0.4, 0.4, 0.2]),
            'pinky':  np.array([-0.2, 0.3, 0.3, 0.1])
        }
        
        # 手掌姿态 (相对于世界坐标系，坐标原点在掌根)
        self.palm_pos = np.array([0.0, 0.0, 0.0])  # 掌根位置（世界坐标原点）
        # 世界坐标系：右手螺旋 (X×Y=Z)，手为左手
        #   X 朝前（掌心朝向），Y 向右（拇指→小指），Z 向上（指尖方向）
        # 局部坐标系 (X,Y,Z)=(厚度,宽度,长度) 与世界一致，palm_rot 恒等
        self.palm_rot = np.eye(3, dtype=float)
        self.is_left_hand = True                   # 左手：拇指在-Y侧（左），四指在+Y侧（右）
        
    def set_joint_angles(self, finger, angles):
        """设置指定手指的关节角度"""
        if finger in self.joint_angles:
            self.joint_angles[finger] = np.array(angles)
    
    def set_all_joint_angles(
        self,
        thumb_angles: Sequence[float],
        index_angles: Sequence[float],
        middle_angles: Sequence[float],
        ring_angles: Sequence[float],
        pinky_angles: Sequence[float],
    ):
        """一次性设置所有手指的关节角度（各 4 个弧度值）"""
        self.joint_angles['thumb'] = np.array(thumb_angles, dtype=float)
        self.joint_angles['index'] = np.array(index_angles, dtype=float)
        self.joint_angles['middle'] = np.array(middle_angles, dtype=float)
        self.joint_angles['ring'] = np.array(ring_angles, dtype=float)
        self.joint_angles['pinky'] = np.array(pinky_angles, dtype=float)

    def set_angles_from_row(self, row: Sequence[float]) -> None:
        """
        从单行关节数据设置手姿。支持 20 或 21 个关节角（弧度）。
        顺序: thumb(4), index(4), middle(4), ring(4), pinky(4) [，第21关节可选]
        """
        a = list(row)[:21]
        while len(a) < 20:
            a.append(0.0)
        self.set_all_joint_angles(
            a[0:4], a[4:8], a[8:12], a[12:16], a[16:20]
        )
    
    def set_palm_pose(self, position, rotation_matrix=None, euler_angles=None):
        """设置手掌的位置和姿态"""
        self.palm_pos = np.array(position)
        
        if rotation_matrix is not None:
            self.palm_rot = np.array(rotation_matrix)
        elif euler_angles is not None:
            # 从欧拉角创建旋转矩阵 (ZYX顺序)
            roll, pitch, yaw = euler_angles
            self.palm_rot = self._euler_to_rotation_matrix(roll, pitch, yaw)
    
    def _euler_to_rotation_matrix(self, roll, pitch, yaw):
        """将欧拉角转换为旋转矩阵"""
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(roll), -np.sin(roll)],
                       [0, np.sin(roll), np.cos(roll)]])
        
        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                       [0, 1, 0],
                       [-np.sin(pitch), 0, np.cos(pitch)]])
        
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                       [np.sin(yaw), np.cos(yaw), 0],
                       [0, 0, 1]])
        
        return Rz @ Ry @ Rx
    
    def _rotate_point(self, point, rotation_matrix, translation):
        """应用旋转和平移到点"""
        return rotation_matrix @ point + translation
    
    def _compute_finger_joints(self, finger_name, base_pos, base_rot):
        """
        计算手指所有关节的位置
        返回: [掌指关节, 近指关节, 中指关节, 指尖] 的世界坐标
        """
        lengths = self.finger_lengths[finger_name]
        angles = self.joint_angles[finger_name]
        
        # 获取该手指的关节角度
        if finger_name == 'thumb':
            # 拇指: CMC_abduct, CMC_flex, MCP_flex, IP_flex
            # 屈曲方向：正角度向中指/无名指方向弯曲（对握），而非向掌心
            abduct = angles[0]
            flex1, flex2, flex3 = angles[1], angles[2], angles[3]
            if getattr(self, 'is_left_hand', False):
                abduct = -abduct  # 外展方向取反

            R_abduct = np.array([[np.cos(abduct), -np.sin(abduct), 0],
                                 [np.sin(abduct), np.cos(abduct), 0],
                                 [0, 0, 1]])
            
            # 屈曲：绕局部 Y 轴，正角度使指尖从 +Y 弯向 +Z（朝中指/无名指方向对握）
            # R_y(θ)：局部 X→+Z，世界 +Y→+Z
            R_flex1 = np.array([[np.cos(flex1), 0, -np.sin(flex1)],
                                [0, 1, 0],
                                [np.sin(flex1), 0, np.cos(flex1)]])
            R_flex2 = np.array([[np.cos(flex2), 0, -np.sin(flex2)],
                                [0, 1, 0],
                                [np.sin(flex2), 0, np.cos(flex2)]])
            R_flex3 = np.array([[np.cos(flex3), 0, -np.sin(flex3)],
                                [0, 1, 0],
                                [np.sin(flex3), 0, np.cos(flex3)]])
            
            # 计算关节位置 (在局部坐标系中)
            joint1_local = np.array([0, 0, 0])
            joint2_local = joint1_local + R_abduct @ R_flex1 @ np.array([lengths[0], 0, 0])
            joint3_local = joint2_local + R_abduct @ R_flex1 @ R_flex2 @ np.array([lengths[1], 0, 0])
            tip_local = joint3_local + R_abduct @ R_flex1 @ R_flex2 @ R_flex3 @ np.array([lengths[2], 0, 0])
            
        else:
            # 四指：指根在 XOZ 平面，弯曲在 XZ 平面（绕 Y 轴屈曲）
            # 正角度屈曲时指尖从 +Z 向 +X 弯曲，即朝向掌心/屏幕外侧
            abduct = angles[0]
            flex1, flex2, flex3 = angles[1], angles[2], angles[3]
            if getattr(self, 'is_left_hand', False):
                abduct = -abduct
                # 屈曲方向已由绕 Y 轴正确设定，无需取反

            R_abduct = np.array([[np.cos(abduct), 0, np.sin(abduct)],
                                 [0, 1, 0],
                                 [-np.sin(abduct), 0, np.cos(abduct)]])  # 绕 Y 轴外展

            # 屈曲方向：绕 Y 轴，正角度使指尖从 +Z 弯向 +X（掌心/屏幕方向）
            # R_y(θ): 标准绕 Y 轴旋转，正 θ 使 +Z 转向 +X
            R_flex1 = np.array([[np.cos(flex1), 0, np.sin(flex1)],
                               [0, 1, 0],
                               [-np.sin(flex1), 0, np.cos(flex1)]])
            R_flex2 = np.array([[np.cos(flex2), 0, np.sin(flex2)],
                               [0, 1, 0],
                               [-np.sin(flex2), 0, np.cos(flex2)]])
            R_flex3 = np.array([[np.cos(flex3), 0, np.sin(flex3)],
                               [0, 1, 0],
                               [-np.sin(flex3), 0, np.cos(flex3)]])

            seg = np.array([0.0, 0.0, 1.0])  # 段沿局部 Z
            joint1_local = np.array([0.0, 0.0, 0.0])
            joint2_local = joint1_local + R_abduct @ R_flex1 @ (lengths[0] * seg)
            joint3_local = joint2_local + R_abduct @ R_flex1 @ R_flex2 @ (lengths[1] * seg)
            tip_local = joint3_local + R_abduct @ R_flex1 @ R_flex2 @ R_flex3 @ (lengths[2] * seg)
        
        # 将局部坐标转换到世界坐标
        joints_local = [joint1_local, joint2_local, joint3_local, tip_local]
        joints_world = [self._rotate_point(j, base_rot, base_pos) for j in joints_local]
        
        return joints_world
    
    def get_all_joint_positions(self):
        """获取所有关节的世界坐标位置"""
        all_points = []
        
        # 1. 手掌顶点（局部 (X,Y,Z)=(厚度,宽度,长度)，掌根 z=0）
        palm_half_t = self.palm_thickness / 2  # X
        palm_half_w = self.palm_width / 2      # Y
        palm_corners_local = [
            [-palm_half_t, -palm_half_w, 0.0],           # 0 掌根面(z=0)
            [palm_half_t, -palm_half_w, 0.0],           # 1
            [palm_half_t, palm_half_w, 0.0],            # 2
            [-palm_half_t, palm_half_w, 0.0],           # 3
            [-palm_half_t, -palm_half_w, self.palm_length],  # 4 指根面(z=L)
            [palm_half_t, -palm_half_w, self.palm_length],   # 5
            [palm_half_t, palm_half_w, self.palm_length],    # 6
            [-palm_half_t, palm_half_w, self.palm_length]    # 7
        ]
        
        palm_corners_world = [self._rotate_point(np.array(c), self.palm_rot, self.palm_pos) 
                              for c in palm_corners_local]
        all_points.extend(palm_corners_world)
        
        # 2. 为每根手指计算关节位置
        # 局部 (X,Y,Z)=(厚度,宽度,长度)，四指沿 Z 生长，拇指沿 X 生长需旋转
        palm_center_local = np.array([0.0, 0.0, self.palm_length / 2])
        # 左手拇指在左侧(-Y)，伸展方向应向四指(+Y)以对握
        R_thumb_forward = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], dtype=float)  # 绕 Z +90°，局部X→世界+Y

        finger_joints = {}
        for finger_name, origin_local in self.finger_origins.items():
            finger_base_local = palm_center_local + np.array(origin_local)
            finger_base_world = self._rotate_point(finger_base_local, self.palm_rot, self.palm_pos)
            base_rot = self.palm_rot @ R_thumb_forward if finger_name == 'thumb' else self.palm_rot
            joints = self._compute_finger_joints(finger_name, finger_base_world, base_rot)
            finger_joints[finger_name] = joints
            all_points.extend(joints)
        
        return all_points, finger_joints, palm_corners_world
    
    def draw(self, ax, color_scheme='default'):
        """
        在指定的3D坐标轴上绘制手
        
        参数:
            ax: matplotlib 3D 坐标轴
            color_scheme: 颜色方案
        """
        # 获取所有点位置
        _, finger_joints, _ = self.get_all_joint_positions()
        palm_root = self.palm_pos

        # 1. 从手掌根部到五根手指指根的连线
        colors = {
            'thumb': 'red',
            'index': 'blue',
            'middle': 'green',
            'ring': 'orange',
            'pinky': 'purple'
        }
        for finger_name, joints in finger_joints.items():
            finger_root = joints[0]
            ax.plot(
                [palm_root[0], finger_root[0]],
                [palm_root[1], finger_root[1]],
                [palm_root[2], finger_root[2]],
                color=colors.get(finger_name, 'gray'),
                linewidth=2,
                linestyle='--',
                alpha=0.7,
            )

        # 2. 绘制手指
        for finger_name, joints in finger_joints.items():
            color = colors.get(finger_name, 'black')
            x = [j[0] for j in joints]
            y = [j[1] for j in joints]
            z = [j[2] for j in joints]
            ax.plot(x, y, z, color=color, linewidth=3, marker='o',
                    markersize=6, label=finger_name)

            # 3. 指尖：小锥体 + 指腹/指甲双色面，便于区分正反面与空间前后
            if len(joints) >= 2:
                tip = np.array(joints[-1])
                prev = np.array(joints[-2])
                direction = tip - prev
                d_norm = np.linalg.norm(direction)
                if d_norm > 1e-6:
                    direction = direction / d_norm
                    # 构建正交基：u,v 垂直于 direction
                    n_ref = np.array([0.0, 1.0, 0.0]) if abs(direction[1]) < 0.9 else np.array([1.0, 0.0, 0.0])
                    u = np.cross(direction, n_ref)
                    u_norm = np.linalg.norm(u)
                    if u_norm > 1e-6:
                        u = u / u_norm
                        v = np.cross(direction, u)
                        v = v / np.linalg.norm(v)
                        # 左手拇指：翻转 u 使指尖锥体与右手对称
                        if finger_name == 'thumb' and getattr(self, 'is_left_hand', False):
                            u = -u
                        R = np.column_stack([u, v, direction])

                        # 小锥体：底面在指尖，顶点朝手指方向延伸
                        h_cone, r_cone = 0.006, 0.004
                        n_seg = 12
                        theta = np.linspace(0, 2 * np.pi, n_seg + 1)[:-1]
                        base_pts = tip + (R @ np.vstack([r_cone * np.cos(theta), r_cone * np.sin(theta), np.zeros(n_seg)])).T
                        apex_pt = tip + h_cone * direction

                        # 底面（指腹/指甲面）：朝向手掌方向，用较深色区分
                        base_face = [base_pts.tolist()]
                        poly_base = Poly3DCollection(
                            base_face,
                            facecolor=_darken_color(color, 0.5),
                            alpha=0.85,
                            edgecolor=color,
                            linewidth=1.2,
                        )
                        ax.add_collection3d(poly_base)

                        # 锥体侧面：每块三角面，便于看出空间前后
                        for i in range(n_seg):
                            j = (i + 1) % n_seg
                            side_face = [[base_pts[i], base_pts[j], apex_pt]]
                            poly_side = Poly3DCollection(
                                side_face,
                                facecolor=color,
                                alpha=0.7,
                                edgecolor=color,
                                linewidth=0.8,
                            )
                            ax.add_collection3d(poly_side)

        
        # 设置图例
        ax.legend()
        
        # 不显示背景网格和坐标轴
        ax.grid(False)
        ax.axis('off')
        
        # 固定视口范围，使手在画面中占比更大（不随姿态变化）
        half_range = 0.06  # 固定半宽约 6cm，手占满画面
        ax.set_xlim([-half_range, half_range])
        ax.set_ylim([-half_range, half_range])
        ax.set_zlim([-half_range, half_range])
        
        # 在角落绘制 XYZ 正方向小标签
        _draw_xyz_indicator(ax, half_range)
        
        # 设置视角：掌心(+X)朝向观察者；matplotlib YZ 视图 azim=0 使相机在 +X 侧
        ax.view_init(elev=20, azim=0)
        
        ax.set_title('Robot Hand Visualization')
    
    def animate_hand(self, fig, ax, frames=100, interval=50):
        """
        创建手部动画
        """
        view_state = [20.0, 0.0]  # elev, azim，掌心朝向观察者

        def update(frame):
            if fig.axes:
                try:
                    view_state[0] = fig.axes[0].elev
                    view_state[1] = fig.axes[0].azim
                except AttributeError:
                    pass
            fig.clf()
            ax = fig.add_subplot(111, projection="3d")
            
            # 随时间改变关节角度
            t = frame / frames * 2 * np.pi
            
            # 设置不同的抓取模式
            if frame < frames/3:
                # 模式1: 握拳
                self.set_all_joint_angles(
                    [0.2, 1.2, 1.0, 0.8],  # 拇指
                    [0.1, 1.5, 1.2, 1.0],  # 食指
                    [0.0, 1.5, 1.2, 1.0],  # 中指
                    [-0.1, 1.5, 1.2, 1.0], # 无名指
                    [-0.2, 1.4, 1.1, 0.9]  # 小指
                )
            elif frame < 2*frames/3:
                # 模式2: 半握
                self.set_all_joint_angles(
                    [0.2, 0.8, 0.6, 0.4],
                    [0.1, 1.0, 0.8, 0.6],
                    [0.0, 1.0, 0.8, 0.6],
                    [-0.1, 1.0, 0.8, 0.6],
                    [-0.2, 0.9, 0.7, 0.5]
                )
            else:
                # 模式3: 伸展
                self.set_all_joint_angles(
                    [0.1, 0.0, 0.0, 0.0],   # 拇指张开
                    [0.0, 0.2, 0.1, 0.0],
                    [0.0, 0.1, 0.1, 0.0],
                    [0.0, 0.1, 0.1, 0.0],
                    [-0.1, 0.1, 0.1, 0.0]
                )
            
            # 绘制手
            self.draw(ax)
            ax.set_title(f'Robot Hand Animation - Frame {frame}')
            ax.view_init(elev=view_state[0], azim=view_state[1])

        ani = animation.FuncAnimation(
            fig,
            update,
            frames=frames,
            interval=interval,
            repeat=True,
        )
        return ani

    def animate_sequence(
        self,
        angles_seq: Union[np.ndarray, Sequence[Sequence[float]]],
        fig=None,
        ax=None,
        interval: int = 300,
    ):
        """
        按给定关节序列动画显示手部运动。
        angles_seq: (N, 20) 或 (N, 21) 的矩阵，每行为一帧目标角度（弧度）
        """
        seq = np.asarray(angles_seq)
        if seq.ndim != 2 or seq.shape[1] < 20:
            raise ValueError("angles_seq 需为 (N, 20) 或 (N, 21) 的矩阵")
        n_frames = seq.shape[0]

        if fig is None:
            fig = plt.figure(figsize=(10, 8))
        if ax is None:
            ax = fig.add_subplot(111, projection="3d")

        view_state = [20.0, 0.0]  # [elev, azim]，掌心朝向观察者，跨帧保持

        def update(frame):
            if fig.axes:
                try:
                    view_state[0] = fig.axes[0].elev
                    view_state[1] = fig.axes[0].azim
                except AttributeError:
                    pass
            fig.clf()
            ax = fig.add_subplot(111, projection="3d")
            self.set_angles_from_row(seq[frame])
            self.draw(ax)
            ax.set_title(f"帧 {frame + 1}/{n_frames}")
            ax.view_init(elev=view_state[0], azim=view_state[1])

        ani = animation.FuncAnimation(
            fig, update, frames=n_frames, interval=interval, repeat=True, blit=False
        )
        return ani


def _demo_sequence_10x21() -> np.ndarray:
    """示例：10 帧 x 21 关节，从全手张开到握拳的完整过程"""
    # 全手张开
    open_pose = [
        0.1, 0.0, 0.0, 0.0,   # thumb 张开
        0.0, 0.2, 0.1, 0.0,   # index
        0.0, 0.1, 0.1, 0.0,   # middle
        0.0, 0.1, 0.1, 0.0,   # ring
        -0.1, 0.1, 0.1, 0.0,  # pinky
        0.0,
    ]
    # 握拳
    fist_pose = [
        0.2, 1.2, 1.0, 0.8,   # thumb
        0.1, 1.5, 1.2, 1.0,   # index
        0.0, 1.5, 1.2, 1.0,   # middle
        -0.1, 1.5, 1.2, 1.0,  # ring
        -0.2, 1.4, 1.1, 0.9,  # pinky
        0.0,
    ]
    open_pose = np.array(open_pose, dtype=float)
    fist_pose = np.array(fist_pose, dtype=float)
    # 10 帧线性插值：0=张开 -> 9=握拳
    n_frames = 10
    return np.array([
        open_pose + (fist_pose - open_pose) * (i / (n_frames - 1))
        for i in range(n_frames)
    ], dtype=float)


def _parse_args():
    """
    命令行参数：
      - 可选预设姿态: fist / open
      - 或传入 20 个自定义关节角（弧度），顺序:
        thumb(4), index(4), middle(4), ring(4), pinky(4)
      - 可设置手掌欧拉角 (roll, pitch, yaw, 单位弧度)
      - 可选是否播放动画
    """
    parser = argparse.ArgumentParser(description="Robot hand 3D visualizer")
    parser.add_argument(
        "--preset",
        choices=["fist", "open"],
        default="open",
        help="预设姿态: fist(握拳) 或 open(伸展)，默认 open 便于观察掌心与屈曲方向",
    )
    parser.add_argument(
        "--angles",
        type=float,
        nargs=20,
        metavar="angle",
        help="自定义 20 个关节角（弧度），顺序 thumb/index/middle/ring/pinky 各 4 个，若提供则覆盖 preset",
    )
    parser.add_argument(
        "--palm-euler",
        type=float,
        nargs=3,
        default=[0.0, 0.0, 0.0],
        metavar=("roll", "pitch", "yaw"),
        help="手掌欧拉角 (roll, pitch, yaw)，单位弧度，默认 [0,0,0]",
    )
    parser.add_argument(
        "--animate",
        action="store_true",
        help="播放抓取动画（忽略 angles/preset 的单帧静态姿态，仅用于演示）",
    )
    parser.add_argument(
        "--sequence",
        action="store_true",
        help="播放 10 帧动画：从全手张开到握拳的完整过程",
    )
    parser.add_argument(
        "--sequence-file",
        type=str,
        metavar="PATH",
        help="从文件读取关节序列 (每行 20 或 21 个浮点数，空格/逗号分隔)，模拟读取并动画显示",
    )
    parser.add_argument(
        "--interval",
        type=int,
        default=300,
        help="序列动画每帧间隔 (ms)，默认 300",
    )
    return parser.parse_args()


def _apply_preset_angles(hand: RobotHandVisualizer, preset: str):
    """根据预设名称设置一组合理的测试关节角"""
    if preset == "open":
        hand.set_all_joint_angles(
            [0.1, 0.0, 0.0, 0.0],   # 拇指张开
            [0.0, 0.2, 0.1, 0.0],
            [0.0, 0.1, 0.1, 0.0],
            [0.0, 0.1, 0.1, 0.0],
            [-0.1, 0.1, 0.1, 0.0],
        )
    else:  # fist
        hand.set_all_joint_angles(
            [0.2, 1.2, 1.0, 0.8],   # 拇指 [外展, 弯曲1, 弯曲2, 弯曲3]
            [0.1, 1.5, 1.2, 1.0],   # 食指
            [0.0, 1.5, 1.2, 1.0],   # 中指
            [-0.1, 1.5, 1.2, 1.0],  # 无名指
            [-0.2, 1.4, 1.1, 0.9],  # 小指
        )


def _apply_custom_angles(hand: RobotHandVisualizer, angles20: Sequence[float]):
    """按顺序拆分 20 个角并设置到 5 个手指"""
    if len(angles20) != 20:
        raise ValueError("需要恰好 20 个关节角 (thumb/index/middle/ring/pinky 各 4 个)")
    a = list(angles20)
    thumb = a[0:4]
    index = a[4:8]
    middle = a[8:12]
    ring = a[12:16]
    pinky = a[16:20]
    hand.set_all_joint_angles(thumb, index, middle, ring, pinky)


def main():
    args = _parse_args()

    hand = RobotHandVisualizer()

    # 设置手掌姿态
    hand.set_palm_pose([0.0, 0.0, 0.0], euler_angles=args.palm_euler)

    if args.animate:
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection="3d")
        ani = hand.animate_hand(fig, ax, frames=90, interval=100)
        plt.show()
        return

    if args.sequence or args.sequence_file is not None:
        if args.sequence_file:
            # 从文件读取：每行 20 或 21 个浮点数
            data = []
            with open(args.sequence_file, "r", encoding="utf-8") as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith("#"):
                        continue
                    parts = line.replace(",", " ").split()
                    row = [float(x) for x in parts]
                    if len(row) >= 20:
                        data.append(row[:21])
            if not data:
                raise ValueError(f"文件 {args.sequence_file} 中无有效数据行")
            angles_seq = np.array(data)
        else:
            angles_seq = _demo_sequence_10x21()
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection="3d")
        ani = hand.animate_sequence(angles_seq, fig=fig, ax=ax, interval=args.interval)
        plt.show()
        return

    # 静态单帧：使用 angles 或 preset
    if args.angles is not None:
        _apply_custom_angles(hand, args.angles)
    else:
        _apply_preset_angles(hand, args.preset)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    hand.draw(ax)
    title = f"Robot Hand - {args.preset}"
    plt.title(title)
    plt.show()


if __name__ == "__main__":
    main()