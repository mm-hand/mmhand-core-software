"""
Manipulability Ellipsoid Analysis for HKU Hand

Calculates the manipulability measure w = sqrt(det(J*J^T)) for the
index finger (1-1_Link) and thumb (5-1_Link) under three configurations:
1. Up-pose: distal joints fully extended (at minimum)
2. Down-pose: distal joints fully flexed (at maximum)
3. Curled-pose: distal joints at intermediate state

Separates analysis into:
- Linear velocity Jacobian (first 3 rows): position manipulability
- Angular velocity Jacobian (last 3 rows): rotation manipulability
"""

import mujoco
import numpy as np


def get_joint_info(model):
    """Print joint information for debugging."""
    print("\n=== Joint Information ===")
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        range_min = model.jnt_range[i, 0]
        range_max = model.jnt_range[i, 1]
        print(f"Joint {i}: {name}, range: [{range_min:.4f}, {range_max:.4f}]")
    print()


def get_body_id(model, body_name):
    """Get body ID by name."""
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)


def get_joint_id(model, joint_name):
    """Get joint ID by name."""
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)


def get_dof_adr_for_joint(model, joint_name):
    """
    Get the DOF (velocity) address for a joint.
    For simple joints (1 DOF), this gives the column index in the Jacobian.
    """
    joint_id = get_joint_id(model, joint_name)
    if joint_id < 0:
        raise ValueError(f"Joint '{joint_name}' not found")
    return model.jnt_dofadr[joint_id]


def set_joint_positions(model, data, joint_positions):
    """
    Set joint positions in the data structure.

    Args:
        model: MuJoCo model
        data: MuJoCo data
        joint_positions: dict mapping joint names to positions
    """
    for joint_name, position in joint_positions.items():
        joint_id = get_joint_id(model, joint_name)
        if joint_id >= 0:
            # Get the qpos address for this joint
            qpos_adr = model.jnt_qposadr[joint_id]
            data.qpos[qpos_adr] = position

    # Forward kinematics to update positions
    mujoco.mj_forward(model, data)


def compute_jacobians_for_joints(model, data, body_name, joint_names):
    """
    Compute the linear and angular Jacobians for a specific body (end-effector)
    with respect to only the specified joints.

    Args:
        model: MuJoCo model
        data: MuJoCo data
        body_name: Name of the end-effector body
        joint_names: List of joint names to include in the Jacobian

    Returns:
        J_linear: Linear velocity Jacobian (3 x num_joints)
        J_angular: Angular velocity Jacobian (3 x num_joints)
    """
    body_id = get_body_id(model, body_name)
    if body_id < 0:
        raise ValueError(f"Body '{body_name}' not found")

    # Allocate full Jacobian matrices
    nv = model.nv  # number of degrees of freedom
    Jp = np.zeros((3, nv))  # position/linear Jacobian
    Jr = np.zeros((3, nv))  # rotation/angular Jacobian

    # Compute full Jacobian
    mujoco.mj_jacBody(model, data, Jp, Jr, body_id)

    # Extract columns corresponding to the specified joints
    dof_indices = []
    for joint_name in joint_names:
        dof_adr = get_dof_adr_for_joint(model, joint_name)
        dof_indices.append(dof_adr)

    J_linear = Jp[:, dof_indices]
    J_angular = Jr[:, dof_indices]

    return J_linear, J_angular


def compute_manipulability(J):
    """
    Compute the manipulability measure from the Jacobian.

    w = sqrt(det(J * J^T))

    Args:
        J: Jacobian matrix (m x n)

    Returns:
        w: manipulability measure
    """
    JJT = J @ J.T
    det = np.linalg.det(JJT)

    # Handle numerical issues
    if det < 0:
        print(f"  Warning: Negative determinant detected: {det:.6e}")
        det = abs(det)

    w = np.sqrt(det)
    return w


def get_index_finger_config(model, pose_type):
    """
    Get joint configuration for index finger based on pose type.

    Index finger joints:
    - 1-1: side-sway, range [-0.8727, 0.5236] -> keep at 0
    - 1-2: distal 1, range [0, 1.5708]
    - 1-3: distal 2, range [0, 1.4835]
    - 1-4: distal 3, range [0, 1.5708]
    """
    # Get joint ranges
    jnt_1_2_range = model.jnt_range[get_joint_id(model, "1-2")]
    jnt_1_3_range = model.jnt_range[get_joint_id(model, "1-3")]
    jnt_1_4_range = model.jnt_range[get_joint_id(model, "1-4")]

    if pose_type == "up":
        # Fully extended (minimum values for distal joints)
        return {
            "1-1": 0.0,  # side-sway at neutral
            "1-2": jnt_1_2_range[0],  # 0
            "1-3": jnt_1_3_range[0],  # 0
            "1-4": jnt_1_4_range[0],  # 0
        }
    elif pose_type == "down":
        # Fully flexed (maximum values for distal joints)
        return {
            "1-1": 0.0,  # side-sway at neutral
            "1-2": jnt_1_2_range[1],  # 1.5708
            "1-3": jnt_1_3_range[1],  # 1.4835
            "1-4": jnt_1_4_range[1],  # 1.5708
        }
    elif pose_type == "curled":
        # Intermediate (midpoint for distal joints)
        return {
            "1-1": 0.0,  # side-sway at neutral
            "1-2": (jnt_1_2_range[0] + jnt_1_2_range[1]) / 2,
            "1-3": (jnt_1_3_range[0] + jnt_1_3_range[1]) / 2,
            "1-4": (jnt_1_4_range[0] + jnt_1_4_range[1]) / 2,
        }
    else:
        raise ValueError(f"Unknown pose type: {pose_type}")


def get_thumb_config(model, pose_type):
    """
    Get joint configuration for thumb based on pose type.

    Thumb joints:
    - 5-1: base 1, range [-1.0308, 0.54] -> keep at 0
    - 5-2: base 2, range [-1.0472, 1.0472] -> keep at 0
    - 5-3: distal 1, range [0, 1.5708]
    - 5-4: distal 2, range [0, 1.4835]
    - 5-5: distal 3, range [0, 1.5708]
    """
    # Get joint ranges
    jnt_5_3_range = model.jnt_range[get_joint_id(model, "5-3")]
    jnt_5_4_range = model.jnt_range[get_joint_id(model, "5-4")]
    jnt_5_5_range = model.jnt_range[get_joint_id(model, "5-5")]

    if pose_type == "up":
        # Fully extended (minimum values for distal joints)
        return {
            "5-1": 0.0,  # base joint at neutral
            "5-2": 0.0,  # base joint at neutral
            "5-3": jnt_5_3_range[0],  # 0
            "5-4": jnt_5_4_range[0],  # 0
            "5-5": jnt_5_5_range[0],  # 0
        }
    elif pose_type == "down":
        # Fully flexed (maximum values for distal joints)
        return {
            "5-1": 0.0,  # base joint at neutral
            "5-2": 0.0,  # base joint at neutral
            "5-3": jnt_5_3_range[1],  # 1.5708
            "5-4": jnt_5_4_range[1],  # 1.4835
            "5-5": jnt_5_5_range[1],  # 1.5708
        }
    elif pose_type == "curled":
        # Intermediate (midpoint for distal joints)
        return {
            "5-1": 0.0,  # base joint at neutral
            "5-2": 0.0,  # base joint at neutral
            "5-3": (jnt_5_3_range[0] + jnt_5_3_range[1]) / 2,
            "5-4": (jnt_5_4_range[0] + jnt_5_4_range[1]) / 2,
            "5-5": (jnt_5_5_range[0] + jnt_5_5_range[1]) / 2,
        }
    else:
        raise ValueError(f"Unknown pose type: {pose_type}")


def analyze_manipulability(model_path, body_name, joint_config, joint_names, finger_name, pose_name):
    """
    Analyze manipulability for a specific configuration, computing both
    linear and angular components.

    Args:
        model_path: Path to MuJoCo XML file
        body_name: End-effector body name
        joint_config: Dictionary of joint positions
        joint_names: List of joint names to include in Jacobian
        finger_name: Name of finger for display
        pose_name: Name of pose for display

    Returns:
        dict with linear and angular manipulability values and Jacobians
    """
    # Load model
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    # Set joint configuration
    set_joint_positions(model, data, joint_config)

    # Compute both linear and angular Jacobians
    J_linear, J_angular = compute_jacobians_for_joints(model, data, body_name, joint_names)

    # Compute manipulability for each component
    w_linear = compute_manipulability(J_linear)
    w_angular = compute_manipulability(J_angular)

    # Print details
    print(f"\n{finger_name} - {pose_name} pose:")
    print(f"  Joint configuration (rad): {joint_config}")
    # Convert to degrees for readability
    config_deg = {k: np.degrees(v) for k, v in joint_config.items()}
    print(f"  Joint configuration (deg): {config_deg}")
    print(f"  Linear Jacobian shape: {J_linear.shape}")
    print(f"  Angular Jacobian shape: {J_angular.shape}")
    print(f"  Linear manipulability (position):  w = {w_linear:.6e}")
    print(f"  Angular manipulability (rotation): w = {w_angular:.6e}")

    return {
        "w_linear": w_linear,
        "w_angular": w_angular,
        "J_linear": J_linear,
        "J_angular": J_angular
    }


def main():
    # Path to the HKU hand model
    model_path = "hku_hand.xml"

    poses = ["up", "down", "curled"]

    # Joint names for each finger's kinematic chain
    index_joints = ["1-1", "1-2", "1-3", "1-4"]  # All index finger joints
    thumb_joints = ["5-1", "5-2", "5-3", "5-4", "5-5"]  # All thumb joints

    results = {}

    print("=" * 70)
    print("HKU Hand Manipulability Ellipsoid Analysis")
    print("=" * 70)
    print("\nFormula: w = sqrt(det(J * J^T))")
    print("Computing separate manipulability for:")
    print("  - Linear velocity Jacobian (first 3 rows): position manipulability")
    print("  - Angular velocity Jacobian (last 3 rows): rotation manipulability")

    # Load model once to get joint info
    model = mujoco.MjModel.from_xml_path(model_path)
    get_joint_info(model)

    # Analyze Index Finger (end-effector: 1-4_Link)
    print("\n" + "=" * 70)
    print("INDEX FINGER (End-effector: 1-4_Link)")
    print("Kinematic chain: joints 1-1, 1-2, 1-3, 1-4")
    print("  - Joint 1-1 (side-sway): kept at 0 (neutral)")
    print("  - Joints 1-2, 1-3, 1-4 (distal): configured per pose")
    print("=" * 70)

    results["index"] = {}
    for pose in poses:
        config = get_index_finger_config(model, pose)
        result = analyze_manipulability(model_path, "1-4_Link", config, index_joints, "Index Finger", pose.capitalize())
        results["index"][pose] = {
            "config": config,
            **result
        }

    # Analyze Thumb (end-effector: 5-5_Link)
    print("\n" + "=" * 70)
    print("THUMB (End-effector: 5-5_Link)")
    print("Kinematic chain: joints 5-1, 5-2, 5-3, 5-4, 5-5")
    print("  - Joints 5-1, 5-2 (base): kept at 0 (neutral)")
    print("  - Joints 5-3, 5-4, 5-5 (distal): configured per pose")
    print("=" * 70)

    results["thumb"] = {}
    for pose in poses:
        config = get_thumb_config(model, pose)
        result = analyze_manipulability(model_path, "5-5_Link", config, thumb_joints, "Thumb", pose.capitalize())
        results["thumb"][pose] = {
            "config": config,
            **result
        }

    # Summary
    print("\n" + "=" * 70)
    print("SUMMARY")
    print("=" * 70)

    # Linear (Position) Manipulability Summary
    print("\nLinear (Position) Manipulability Values:")
    print("-" * 55)
    print(f"{'Finger':<15} {'Up-pose':<15} {'Down-pose':<15} {'Curled-pose':<15}")
    print("-" * 55)
    print(f"{'Index Finger':<15} {results['index']['up']['w_linear']:<15.6e} "
          f"{results['index']['down']['w_linear']:<15.6e} "
          f"{results['index']['curled']['w_linear']:<15.6e}")
    print(f"{'Thumb':<15} {results['thumb']['up']['w_linear']:<15.6e} "
          f"{results['thumb']['down']['w_linear']:<15.6e} "
          f"{results['thumb']['curled']['w_linear']:<15.6e}")
    print("-" * 55)

    # Angular (Rotation) Manipulability Summary
    print("\nAngular (Rotation) Manipulability Values:")
    print("-" * 55)
    print(f"{'Finger':<15} {'Up-pose':<15} {'Down-pose':<15} {'Curled-pose':<15}")
    print("-" * 55)
    print(f"{'Index Finger':<15} {results['index']['up']['w_angular']:<15.6e} "
          f"{results['index']['down']['w_angular']:<15.6e} "
          f"{results['index']['curled']['w_angular']:<15.6e}")
    print(f"{'Thumb':<15} {results['thumb']['up']['w_angular']:<15.6e} "
          f"{results['thumb']['down']['w_angular']:<15.6e} "
          f"{results['thumb']['curled']['w_angular']:<15.6e}")
    print("-" * 55)

    return results


if __name__ == "__main__":
    main()
