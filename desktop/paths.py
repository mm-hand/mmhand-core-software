"""Shared filesystem paths for the desktop package."""

from pathlib import Path

PACKAGE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = PACKAGE_DIR.parent
RESOURCE_DIR = PACKAGE_DIR / "resources"
CONFIG_DIR = RESOURCE_DIR / "config"
URDF_DIR = RESOURCE_DIR / "hku_hand_v2_urdf"
LOG_DIR = PACKAGE_DIR / "logs"

POSE_FIST_PATH = RESOURCE_DIR / "pose_fist_21deg.txt"
JOINT_SEQUENCE_EXAMPLE_PATH = RESOURCE_DIR / "joint_sequence_example.txt"
HAND_GEO_PATH = CONFIG_DIR / "hand_geo.json"
LEGACY_CALIB_DEG_PATH = CONFIG_DIR / "legacy_calib_deg.json"
CALIB_DEG_PATH = CONFIG_DIR / "calib_deg.json"
CALIB_LOAD_SNAPSHOT_PATH = CONFIG_DIR / "calib_load_snapshot.json"
TENDON_MANUAL_CALIB_PATH = CONFIG_DIR / "tendon_manual_calib.json"
MANUS_CALIBRATION_PATH = CONFIG_DIR / "manus_calibration.json"
MOTOR_MID_CALIB_PATH = CONFIG_DIR / "motor_mid_calib.json"
RAW_IK_Q_TARGET_PATH = CONFIG_DIR / "raw_ik_q_target.json"
RAW_IK_Q_URDF_PATH = CONFIG_DIR / "raw_ik_q_urdf.json"
URDF_JOINT_MAPPING_PATH = CONFIG_DIR / "urdf_joint_mapping_v2.json"
DEFAULT_URDF_PATH = URDF_DIR / "urdf" / "hand.urdf"
