"""paths.py - MMHand Desktop 模块契约。

职责：集中解析仓库、资源、配置和运行数据目录。
线程：纯模块初始化或工具调用线程执行，不创建后台线程。
输入：包配置、路径参数或文本流。
输出：稳定导入边界、路径或过滤后的数据。
状态所有权：只拥有模块级不可变配置，不拥有设备运行状态。
禁止依赖：禁止导入 Qt 控件或直接操作下位机通信。
"""

from pathlib import Path

PACKAGE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = PACKAGE_DIR.parent
RESOURCE_DIR = PACKAGE_DIR / "resources"
CONFIG_DIR = RESOURCE_DIR / "config"
DEVICE_STATE_DIR = CONFIG_DIR / "device_state"
URDF_DIR = RESOURCE_DIR / "hku_hand_v2_urdf"
RUN_DATA_DIR = PROJECT_ROOT / "run_data"
LOG_DIR = RUN_DATA_DIR / "运行日志"

# 唯一受支持的便携式 Calib Load V6 快照。旧 calib_load_snapshot.json
# 不再作为活动配置读取，避免旧格式被误加载或覆盖新格式。
CALIB_LOAD_SNAPSHOT_PATH = CONFIG_DIR / "calibration_bundle.mmcal.json"
# 运行期多圈锚点独立于体积较大的标定快照，以便运动时低开销原子更新。
MULTITURN_RUNTIME_PATH = DEVICE_STATE_DIR / "multiturn_runtime.json"
URDF_JOINT_MAPPING_PATH = CONFIG_DIR / "urdf_joint_mapping.json"
ROBOT_MODEL_MANIFEST_PATH = CONFIG_DIR / "robot_model_manifest.json"
PRODUCT_FINGERPRINT_PATH = CONFIG_DIR / "product_fingerprint.json"
DEFAULT_URDF_PATH = URDF_DIR / "urdf" / "hand.urdf"
