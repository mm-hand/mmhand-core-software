"""Human hand landmarks -> robot hand joint retargeting and IK solving."""

from __future__ import annotations

import numpy as np

from .constants import HUMAN_FINGER_ANCHOR, HUMAN_FINGER_LANDMARKS
from .math_utils import angle_between, clamp, normalize
from .urdf_model import URDFRobotHandModel


class HandRetargeter:
    """把人手关键点 retarget 到 URDF 机械手，并解算关节角。"""

    def __init__(self, robot_model: URDFRobotHandModel) -> None:
        self.robot = robot_model
        self.last_joint_angles = {name: 0.0 for name in self.robot.revolute_joint_order}

        index_base = self.robot.finger_chains["index"].base_origin
        pinky_base = self.robot.finger_chains["pinky"].base_origin
        self.robot_palm_width = float(np.linalg.norm(index_base - pinky_base))

    def _compute_palm_frame(self, points_3d: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        wrist = points_3d[0]
        index_mcp = points_3d[5]
        pinky_mcp = points_3d[17]

        x_axis = normalize(index_mcp - wrist)
        raw_y = pinky_mcp - wrist

        z_axis = normalize(np.cross(x_axis, raw_y))
        if float(np.linalg.norm(z_axis)) < 1e-8:
            z_axis = np.array([0.0, 0.0, 1.0], dtype=float)

        y_axis = normalize(np.cross(z_axis, x_axis))
        if float(np.linalg.norm(y_axis)) < 1e-8:
            y_axis = np.array([0.0, 1.0, 0.0], dtype=float)

        rotation = np.stack([x_axis, y_axis, z_axis], axis=1)
        return wrist, rotation

    def _to_palm_local(
        self,
        points_3d: np.ndarray,
        palm_origin: np.ndarray,
        palm_rotation: np.ndarray,
    ) -> np.ndarray:
        return (points_3d - palm_origin) @ palm_rotation

    def _finger_reference_angles(
        self,
        finger_name: str,
        local_points: np.ndarray,
    ) -> list[float]:
        landmarks = HUMAN_FINGER_LANDMARKS[finger_name]

        if finger_name == "thumb":
            prev_for_mcp = local_points[0]
            mcp, ip, tip = landmarks[1], landmarks[2], landmarks[3]

            mcp_flex = np.pi - angle_between(prev_for_mcp - local_points[mcp], local_points[ip] - local_points[mcp])
            ip_flex = np.pi - angle_between(local_points[mcp] - local_points[ip], local_points[tip] - local_points[ip])
            return [max(0.0, float(mcp_flex)), max(0.0, float(ip_flex))]

        mcp, pip, dip, tip = landmarks
        wrist = 0

        mcp_flex = np.pi - angle_between(local_points[wrist] - local_points[mcp], local_points[pip] - local_points[mcp])
        pip_flex = np.pi - angle_between(local_points[mcp] - local_points[pip], local_points[dip] - local_points[pip])
        dip_flex = np.pi - angle_between(local_points[pip] - local_points[dip], local_points[tip] - local_points[dip])
        return [
            max(0.0, float(mcp_flex)),
            max(0.0, float(pip_flex)),
            max(0.0, float(dip_flex)),
        ]

    def _build_reference_joint_angles(
        self,
        finger_name: str,
        tip_target: np.ndarray,
        local_points: np.ndarray,
    ) -> dict[str, float]:
        chain = self.robot.finger_chains[finger_name]
        ref: dict[str, float] = {}

        # 基于指尖在基座平面内的方向，为第一关节提供侧摆初值。
        base = chain.base_origin
        direction = tip_target - base
        abduction = float(np.arctan2(direction[1], direction[0]))

        first_joint = chain.revolute_joint_names[0]
        first_spec = self.robot.joint_specs[first_joint]
        ref[first_joint] = clamp(abduction, first_spec.lower, first_spec.upper)

        flex_hints = self._finger_reference_angles(finger_name, local_points)

        if finger_name == "thumb":
            # 拇指链：5-1/5-2 作为基座方向，5-3/5-4/5-5 主要承担弯曲。
            if len(chain.revolute_joint_names) >= 2:
                base2_name = chain.revolute_joint_names[1]
                base2_spec = self.robot.joint_specs[base2_name]
                ref[base2_name] = clamp(self.last_joint_angles.get(base2_name, 0.0), base2_spec.lower, base2_spec.upper)

            if len(chain.revolute_joint_names) >= 3:
                mcp_hint = flex_hints[0] if flex_hints else 0.0
                spec_53 = self.robot.joint_specs[chain.revolute_joint_names[2]]
                normalized_53 = clamp(mcp_hint / (np.pi * 0.95), 0.0, 1.0)
                ref[chain.revolute_joint_names[2]] = clamp(
                    spec_53.lower + normalized_53 * (spec_53.upper - spec_53.lower),
                    spec_53.lower,
                    spec_53.upper,
                )

            ip_hint = flex_hints[1] if len(flex_hints) > 1 else 0.0
            for joint_name in chain.revolute_joint_names[3:]:
                spec = self.robot.joint_specs[joint_name]
                normalized = clamp(ip_hint / (np.pi * 0.95), 0.0, 1.0)
                mapped = spec.lower + normalized * (spec.upper - spec.lower)
                ref[joint_name] = clamp(mapped, spec.lower, spec.upper)
        else:
            # 余下关节用人手弯曲角作为参考，缩放到关节上限。
            for joint_name, hint in zip(chain.revolute_joint_names[1:], flex_hints):
                spec = self.robot.joint_specs[joint_name]
                normalized = clamp(hint / (np.pi * 0.95), 0.0, 1.0)
                mapped = spec.lower + normalized * (spec.upper - spec.lower)
                ref[joint_name] = clamp(mapped, spec.lower, spec.upper)

        # 若某条链关节数 > hint 数量（如拇指），其余关节延用上一帧值。
        for joint_name in chain.revolute_joint_names:
            ref.setdefault(joint_name, self.last_joint_angles.get(joint_name, 0.0))

        return ref

    def _solve_finger_ik(
        self,
        finger_name: str,
        tip_target: np.ndarray,
        reference: dict[str, float],
        max_iters: int = 12,
    ) -> dict[str, float]:
        chain = self.robot.finger_chains[finger_name]
        joint_names = chain.revolute_joint_names

        q = np.array([self.last_joint_angles.get(name, reference[name]) for name in joint_names], dtype=float)
        q_ref = np.array([reference[name] for name in joint_names], dtype=float)

        damping = 1e-3
        regularization = 2e-2
        eps = 1e-4

        for _ in range(max_iters):
            joint_map = {name: float(value) for name, value in zip(joint_names, q)}
            current_tip = self.robot.forward_tip_position(finger_name, joint_map)
            error = current_tip - tip_target

            if float(np.linalg.norm(error)) < 1e-4:
                break

            jacobian = np.zeros((3, len(joint_names)), dtype=float)
            for i, _ in enumerate(joint_names):
                q_delta = q.copy()
                q_delta[i] += eps
                map_delta = {jname: float(val) for jname, val in zip(joint_names, q_delta)}
                tip_delta = self.robot.forward_tip_position(finger_name, map_delta)
                jacobian[:, i] = (tip_delta - current_tip) / eps

            lhs = jacobian.T @ jacobian + (damping + regularization) * np.eye(len(joint_names))
            rhs = jacobian.T @ error + regularization * (q - q_ref)
            delta_q = -np.linalg.solve(lhs, rhs)

            # 限制每步增量，提升稳定性。
            delta_q = np.clip(delta_q, -0.12, 0.12)
            q = q + delta_q

            for i, joint_name in enumerate(joint_names):
                spec = self.robot.joint_specs[joint_name]
                q[i] = clamp(float(q[i]), spec.lower, spec.upper)

        return {name: float(val) for name, val in zip(joint_names, q)}

    def _retarget_tip_targets(self, local_points: np.ndarray) -> dict[str, np.ndarray]:
        human_palm_width = float(np.linalg.norm(local_points[17] - local_points[5]))
        scale = self.robot_palm_width / max(human_palm_width, 1e-6)

        targets: dict[str, np.ndarray] = {}
        for finger_name, chain in self.robot.finger_chains.items():
            lm_indices = HUMAN_FINGER_LANDMARKS[finger_name]
            human_anchor_local = local_points[HUMAN_FINGER_ANCHOR[finger_name]]
            human_tip_local = local_points[lm_indices[-1]]
            tip_delta_local = (human_tip_local - human_anchor_local) * scale

            # 以对应机器人指根为锚点，把人手局部位移映射为机器人目标指尖位置。
            target = chain.base_origin + tip_delta_local

            reach = float(np.linalg.norm(target - chain.base_origin))
            if reach > chain.max_reach * 0.995:
                target = chain.base_origin + normalize(target - chain.base_origin) * chain.max_reach * 0.995

            targets[finger_name] = target

        return targets

    def solve_joint_angles(self, points_3d: np.ndarray, max_iters: int = 12) -> dict[str, float]:
        palm_origin, palm_rotation = self._compute_palm_frame(points_3d)
        local_points = self._to_palm_local(points_3d, palm_origin, palm_rotation)

        tip_targets = self._retarget_tip_targets(local_points)
        solved_angles: dict[str, float] = self.last_joint_angles.copy()

        for finger_name in ("index", "middle", "ring", "pinky", "thumb"):
            if finger_name not in self.robot.finger_chains:
                continue
            target = tip_targets[finger_name]
            reference = self._build_reference_joint_angles(finger_name, target, local_points)
            finger_solution = self._solve_finger_ik(finger_name, target, reference, max_iters=max_iters)
            solved_angles.update(finger_solution)

        # 全局关节角按 URDF 顺序返回。
        ordered = {
            joint_name: self.robot.clamp_joint_value(joint_name, solved_angles.get(joint_name, 0.0))
            for joint_name in self.robot.revolute_joint_order
        }
        self.last_joint_angles = ordered.copy()
        return ordered
