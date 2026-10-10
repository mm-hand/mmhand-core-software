#!/usr/bin/env python3
"""从语义清单和活动 URDF 统一生成 Desktop 与 P4 模型产物。"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import struct
import sys
import tempfile
import xml.etree.ElementTree as ET
import zlib
from pathlib import Path
from typing import Iterable

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tools.robot_model_source import inspect_source, validate_tree


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MANIFEST = ROOT / "desktop/resources/config/robot_model_manifest.json"
DEFAULT_URDF = ROOT / "desktop/resources/hku_hand_v2_urdf/urdf/hand.urdf"
DEFAULT_SOURCE = ROOT / "desktop/resources/config/robot_model_source.json"
DEFAULT_MAPPING = ROOT / "desktop/resources/config/urdf_joint_mapping.json"
DEFAULT_FINGERPRINT = ROOT / "desktop/resources/config/product_fingerprint.json"
DEFAULT_P4_LIMITS = (
    ROOT / "ESP32-P4/ServoBoardMain/src/generated/UrdfJointLimits.generated.h"
)
DEFAULT_P4_FINGERPRINT = (
    ROOT / "ESP32-P4/ServoBoardMain/src/generated/ProductFingerprint.generated.h"
)
DEFAULT_P4_BINDINGS = ROOT / "ESP32-P4/ServoBoardMain/src/generated/HardwareBindings.generated.h"
EXPECTED_JOINT_COUNT = 21


class ModelGenerationError(ValueError):
    """模型源缺失、格式错误或跨端关系不一致。"""


def _json_bytes(value: object) -> bytes:
    """生成不受字典插入顺序影响的 CRC 输入。"""
    return json.dumps(
        value, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")


def _crc32(value: object) -> int:
    return zlib.crc32(_json_bytes(value)) & 0xFFFFFFFF


def _hex32(value: int) -> str:
    return f"{int(value) & 0xFFFFFFFF:08X}"


def _read_json(path: Path) -> dict:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise ModelGenerationError(f"cannot read {path}: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise ModelGenerationError(f"invalid JSON {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ModelGenerationError(f"JSON root must be an object: {path}")
    return value


def load_manifest(path: Path) -> dict:
    """严格验证 J00-J20 语义顺序，避免生成时静默错绑舵机。"""
    data = _read_json(path)
    joints = data.get("joints")
    if int(data.get("schema_version", 0)) != 1 or not isinstance(joints, list):
        raise ModelGenerationError("robot model manifest schema must be version 1")
    if len(joints) != EXPECTED_JOINT_COUNT:
        raise ModelGenerationError(
            f"manifest needs {EXPECTED_JOINT_COUNT} joints, got {len(joints)}"
        )
    used_motors: set[int] = set()
    for expected, joint in enumerate(joints):
        if not isinstance(joint, dict) or int(joint.get("joint_index", -1)) != expected:
            raise ModelGenerationError(f"manifest joint order mismatch at J{expected:02d}")
        for key in ("display_name", "urdf_joint_name"):
            if not str(joint.get(key, "")).strip():
                raise ModelGenerationError(f"J{expected:02d} missing {key}")
        motors = joint.get("motor_indices")
        if not isinstance(motors, list) or len(motors) not in (1, 2):
            raise ModelGenerationError(f"J{expected:02d} motor_indices is invalid")
        for motor in motors:
            motor = int(motor)
            if not 0 <= motor < 27 or motor in used_motors:
                raise ModelGenerationError(f"J{expected:02d} invalid/duplicate M{motor:02d}")
            used_motors.add(motor)
    if used_motors != set(range(27)):
        raise ModelGenerationError("manifest must bind every motor M00-M26 exactly once")
    if len({item["urdf_joint_name"] for item in joints}) != EXPECTED_JOINT_COUNT:
        raise ModelGenerationError("duplicate control joint names in manifest")
    motors = data.get("motors", [])
    if len(motors) != 27 or [item.get("channel") for item in motors] != list(range(27)):
        raise ModelGenerationError("manifest needs ordered motor channels M00-M26")
    if len({item.get("servo_id") for item in motors}) != 27:
        raise ModelGenerationError("duplicate physical servo IDs")
    for motor in motors:
        if not 0 <= int(motor["servo_id"]) <= 26 or int(motor["raw_modulo"]) != 4096:
            raise ModelGenerationError(f"invalid servo topology: {motor}")
        if motor.get("feedback_abs_sign") not in (-1, 1) or motor.get("tendon_shorten_abs_sign") not in (-1, 1):
            raise ModelGenerationError(f"invalid motor direction: {motor}")
    return data


def load_urdf_limits(path: Path, manifest: dict) -> list[tuple[int, float, float]]:
    """从活动 URDF 读取真实限位；语义清单不再复制这些数值。"""
    try:
        root = ET.parse(path).getroot()
    except (OSError, ET.ParseError) as exc:
        raise ModelGenerationError(f"cannot parse active URDF {path}: {exc}") from exc
    validate_tree(path, [item["urdf_joint_name"] for item in manifest["joints"]])
    by_name = {joint.get("name", ""): joint for joint in root.findall("joint")}
    limits: list[tuple[int, float, float]] = []
    for item in manifest["joints"]:
        index = int(item["joint_index"])
        name = str(item["urdf_joint_name"])
        joint = by_name.get(name)
        if joint is None:
            raise ModelGenerationError(f"active URDF is missing J{index:02d} {name}")
        limit = joint.find("limit")
        if limit is None:
            raise ModelGenerationError(f"active URDF joint has no limit: {name}")
        try:
            lower_rad = float(limit.get("lower", ""))
            upper_rad = float(limit.get("upper", ""))
        except ValueError as exc:
            raise ModelGenerationError(f"non-numeric URDF limit: {name}") from exc
        if not math.isfinite(lower_rad) or not math.isfinite(upper_rad) or lower_rad >= upper_rad:
            raise ModelGenerationError(f"invalid URDF limit: {name}")
        def normalize_deg(value: float) -> float:
            # CAD 常用截断弧度表示整数角（例如 1.570796 ≈ 90°），消除无意义的 1e-5° 尾差。
            nearest_integer = round(value)
            if abs(value - nearest_integer) < 0.001:
                return float(nearest_integer)
            return round(value, 6)

        # 同一规范化结果同时进入 JSON 和 P4 CRC，避免跨语言浮点尾差。
        limits.append((
            index,
            normalize_deg(math.degrees(lower_rad)),
            normalize_deg(math.degrees(upper_rad)),
        ))
    return limits


def mapping_crc32(version: int, limits: list[tuple[int, float, float]]) -> int:
    """保持与 P4/现有标定快照一致的二进制 CRC 算法。"""
    payload = bytearray(struct.pack(">BB", version & 0xFF, len(limits) & 0xFF))
    for joint_index, lower, upper in limits:
        payload.extend(struct.pack(">Bff", joint_index, lower, upper))
    return zlib.crc32(payload) & 0xFFFFFFFF


def render_mapping(manifest: dict, limits: list[tuple[int, float, float]],
                   source_commit: str, source_urdf_path: str = "") -> str:
    joints = []
    for semantic, (_, lower, upper) in zip(manifest["joints"], limits):
        min_endpoint = str(semantic["calibrated_min_endpoint"])
        max_endpoint = str(semantic["calibrated_max_endpoint"])
        if min_endpoint not in ("lower", "upper") or max_endpoint not in ("lower", "upper"):
            raise ModelGenerationError("calibrated endpoints must be lower or upper")
        endpoint = {"lower": lower, "upper": upper}
        joints.append(
            {
                "joint_index": int(semantic["joint_index"]),
                "display_name": str(semantic["display_name"]),
                "urdf_joint_name": str(semantic["urdf_joint_name"]),
                "urdf_limit_deg": [round(lower, 6), round(upper, 6)],
                "calibrated_default_range_deg": [
                    endpoint[min_endpoint], endpoint[max_endpoint],
                ],
                "urdf_at_calibrated_min_deg": round(endpoint[min_endpoint], 6),
                "urdf_at_calibrated_max_deg": round(endpoint[max_endpoint], 6),
                "is_coupled": bool(semantic["is_coupled"]),
                "motor_indices": [int(v) for v in semantic["motor_indices"]],
                "mapping_source": "original URDF and explicit motor binding",
                "mapping_status": "unverified",
            }
        )
    data = {
        "schema_version": 4,
        "model_id": str(manifest["model_id"]),
        "mapping_id": str(manifest["mapping_id"]),
        "feedback_mapping_version": int(manifest["feedback_mapping_version"]),
        "manual_scaled_mapping_version": int(manifest["manual_scaled_mapping_version"]),
        "p4_sync_required": False,
        "p4_generated_header_required": True,
        "description": "Generated from original URDF and robot_model_manifest.json; do not edit manually.",
        "source_urdf": "desktop/resources/hku_hand_v2_urdf/urdf/hand.urdf",
        "source_commit": source_commit,
        "source_urdf_path": source_urdf_path,
        "motors": manifest["motors"],
        "joints": joints,
    }
    return json.dumps(data, ensure_ascii=False, indent=2) + "\n"


def render_p4_limits(manifest: dict, limits: list[tuple[int, float, float]]) -> str:
    feedback_version = int(manifest["feedback_mapping_version"])
    manual_version = int(manifest["manual_scaled_mapping_version"])
    manual_crc = mapping_crc32(manual_version, limits)
    rows = "\n".join(
        f"    {{{lower:.6f}f, {upper:.6f}f}},  // J{index:02d}"
        for index, lower, upper in limits
    )
    return f"""/**
 * @file UrdfJointLimits.generated.h
 * @brief 活动 URDF 的 21 关节静态角度上下限和 Manual 映射 CRC（自动生成）。
 *
 * 由 tools/generate_robot_model.py 生成，禁止手工编辑。
 */
#ifndef URDF_JOINT_LIMITS_GENERATED_H
#define URDF_JOINT_LIMITS_GENERATED_H

#include <Arduino.h>

#include "../runtime/SystemRuntimeContext.h"

struct UrdfJointLimitDeg {{
    float minDeg;
    float maxDeg;
}};

static const uint8_t kUrdfFeedbackMappingVersion = {feedback_version};
static const uint8_t kManualScaledUrdfFeedbackMappingVersion = {manual_version};
static const uint32_t kManualScaledUrdfConfigCrc = 0x{manual_crc:08X}UL;
static const UrdfJointLimitDeg kUrdfJointLimits[ENCODER_TOTAL_NUM] = {{
{rows}
}};

static_assert(
    sizeof(kUrdfJointLimits) / sizeof(kUrdfJointLimits[0]) == ENCODER_TOTAL_NUM,
    "URDF joint limit table must match ENCODER_TOTAL_NUM");

#endif // URDF_JOINT_LIMITS_GENERATED_H
"""


def _tree_crc(paths: Iterable[Path], base: Path) -> int:
    """对相对路径和文件内容共同计算 CRC，文件改名也会改变指纹。"""
    crc = 0
    for path in sorted(paths, key=lambda item: item.relative_to(base).as_posix()):
        relative = path.relative_to(base).as_posix().encode("utf-8")
        crc = zlib.crc32(relative + b"\0", crc)
        crc = zlib.crc32(path.read_bytes().replace(b"\r\n", b"\n"), crc)
    return crc & 0xFFFFFFFF


def topology_crc32(motors: list[dict]) -> int:
    """拓扑 v2: >BB 版本/数量，随后按 Mxx 排序的 >BBHb 通道/ID/模数/反馈方向。"""
    payload = bytearray(struct.pack(">BB", 2, len(motors)))
    for motor in sorted(motors, key=lambda item: item["channel"]):
        payload.extend(struct.pack(">BBHb", motor["channel"], motor["servo_id"],
                                   motor["raw_modulo"], motor["feedback_abs_sign"]))
    return zlib.crc32(payload) & 0xFFFFFFFF


def render_p4_bindings(manifest: dict) -> str:
    primary = ", ".join("{" + str(joint["motor_indices"][0]) + "}" for joint in manifest["joints"])
    servo_ids = ", ".join("{" + str(motor["servo_id"]) + "}" for motor in manifest["motors"])
    signs = ", ".join(str(motor["tendon_shorten_abs_sign"]) for motor in manifest["motors"])
    groups = []
    for joint in manifest["joints"]:
        indices = joint["motor_indices"]
        actuators = [f"{{{motor}, {1 if index == 0 else -1}, 0}}" for index, motor in enumerate(indices)]
        if len(actuators) == 1:
            actuators.append("{0xFF, 1, 0}")
        groups.append("    {" + str(len(indices)) + ", {" + ", ".join(actuators) + "}}")
    return """/**
 * @file HardwareBindings.generated.h
 * @brief 原始关节绑定表生成的 P4 拓扑数据，禁止手工编辑。
 */
#ifndef MMHAND_HARDWARE_BINDINGS_GENERATED_H
#define MMHAND_HARDWARE_BINDINGS_GENERATED_H
#include "../data/HardwareTypes.h"
""" + f"JointMapItem jointMap[ENCODER_TOTAL_NUM] = {{{primary}}};\n" + \
        f"MotorMapItem motorMap[SERVO_TOTAL_NUM] = {{{servo_ids}}};\n" + \
        f"static constexpr int8_t kMotorTendonShortenAbsSign[SERVO_TOTAL_NUM] = {{{signs}}};\n" + \
        "JointActuatorGroup jointActuatorGroups[ENCODER_TOTAL_NUM] = {\n" + ",\n".join(groups) + \
        "\n};\n#endif // MMHAND_HARDWARE_BINDINGS_GENERATED_H\n"


def build_fingerprint(manifest: dict, limits: list[tuple[int, float, float]],
                      urdf_path: Path, source_commit: str) -> dict:
    p4_root = ROOT / "ESP32-P4/ServoBoardMain"
    source_files = [p4_root / "ServoBoardMain.ino"]
    source_files.extend(
        path for path in (p4_root / "src").rglob("*")
        if path.is_file() and path.suffix.lower() in (".c", ".cpp", ".h")
        and "generated" not in path.relative_to(p4_root / "src").parts
    )
    # 协议 schema 同样决定固件对外行为。虽然生成头位于 generated/ 并被排除，
    # 唯一源文件必须进入固件源码指纹，避免协议变化后仍显示旧 fingerprint。
    source_files.append(ROOT / "desktop/resources/config/upper_protocol_schema.json")
    config_root = p4_root / "src/config"
    config_files = [path for path in config_root.rglob("*.h") if path.is_file()]
    control_contract = {
        "version": 2,
        "mapping_version": int(manifest["manual_scaled_mapping_version"]),
        "joints": [{key: entry[key] for key in (
            "joint_index", "urdf_joint_name", "motor_indices",
            "calibrated_min_endpoint", "calibrated_max_endpoint",
        )} for entry in manifest["joints"]],
        "limits": limits,
        "tendon_directions": [(motor["channel"], motor["tendon_shorten_abs_sign"])
                              for motor in sorted(manifest["motors"], key=lambda item: item["channel"])],
    }
    control_model_crc = zlib.crc32(json.dumps(
        control_contract, sort_keys=True, separators=(",", ":"), ensure_ascii=False
    ).encode("utf-8")) & 0xFFFFFFFF
    return {
        "schema_version": 2,
        "model_id": str(manifest["model_id"]),
        "mapping_id": str(manifest["mapping_id"]),
        "feedback_mapping_version": int(manifest["feedback_mapping_version"]),
        "manual_scaled_mapping_version": int(manifest["manual_scaled_mapping_version"]),
        "control_model_crc": _hex32(control_model_crc),
        "visual_model_crc": _hex32(zlib.crc32(urdf_path.read_bytes()) & 0xFFFFFFFF),
        "hardware_map_crc": _hex32(topology_crc32(manifest["motors"])),
        "parameter_crc": _hex32(_tree_crc(config_files, ROOT)),
        "firmware_source_crc": _hex32(_tree_crc(source_files, ROOT)),
        "structure_commit_crc": _hex32(zlib.crc32(source_commit.encode("utf-8")) & 0xFFFFFFFF),
        "source_commit": source_commit,
    }


def render_fingerprint_json(fingerprint: dict) -> str:
    return json.dumps(fingerprint, ensure_ascii=False, indent=2) + "\n"


def render_p4_fingerprint(fingerprint: dict) -> str:
    def value(name: str) -> int:
        return int(str(fingerprint[name]), 16)

    return f"""/**
 * @file ProductFingerprint.generated.h
 * @brief 固件源码、参数、硬件映射与 URDF 模型的可复现实验指纹（自动生成）。
 *
 * 由 tools/generate_robot_model.py 生成，禁止手工编辑。
 */
#ifndef PRODUCT_FINGERPRINT_GENERATED_H
#define PRODUCT_FINGERPRINT_GENERATED_H

#include <stdint.h>

static const uint8_t kProductFingerprintSchemaVersion = {fingerprint['schema_version']};
static const uint32_t kProductControlModelCrc = 0x{value('control_model_crc'):08X}UL;
static const uint32_t kProductVisualModelCrc = 0x{value('visual_model_crc'):08X}UL;
static const uint32_t kProductHardwareMapCrc = 0x{value('hardware_map_crc'):08X}UL;
static const uint32_t kProductParameterCrc = 0x{value('parameter_crc'):08X}UL;
static const uint32_t kProductFirmwareSourceCrc = 0x{value('firmware_source_crc'):08X}UL;
static const uint32_t kProductStructureCommitCrc = 0x{value('structure_commit_crc'):08X}UL;

#endif // PRODUCT_FINGERPRINT_GENERATED_H
"""


def _atomic_write(path: Path, content: str) -> None:
    """同目录临时文件替换，避免中途退出留下半份 JSON/头文件。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temp_name = tempfile.mkstemp(prefix=path.name + ".", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as stream:
            stream.write(content)
        Path(temp_name).replace(path)
    except Exception:
        Path(temp_name).unlink(missing_ok=True)
        raise


def build_outputs(args: argparse.Namespace) -> dict[Path, str]:
    manifest = load_manifest(args.manifest)
    limits = load_urdf_limits(args.urdf, manifest)
    source = _read_json(DEFAULT_SOURCE)
    source_commit = source["source_commit"]
    fingerprint = build_fingerprint(manifest, limits, args.urdf, source_commit)
    outputs = {
        args.mapping_output: render_mapping(manifest, limits, source_commit, source["source_urdf_path"]),
        args.p4_limits_output: render_p4_limits(manifest, limits),
        args.fingerprint_output: render_fingerprint_json(fingerprint),
        args.p4_fingerprint_output: render_p4_fingerprint(fingerprint),
        getattr(args, "p4_bindings_output", DEFAULT_P4_BINDINGS): render_p4_bindings(manifest),
    }
    if getattr(args, "firmware_only", False):
        outputs = {path: text for path, text in outputs.items()
                   if path in (args.p4_limits_output, args.p4_fingerprint_output)}
        if any(path.resolve().is_relative_to((ROOT / "desktop").resolve()) for path in outputs):
            raise ModelGenerationError("--firmware-only output cannot be inside desktop")
    return outputs



def default_args() -> argparse.Namespace:
    return argparse.Namespace(manifest=DEFAULT_MANIFEST, urdf=DEFAULT_URDF,
        mapping_output=DEFAULT_MAPPING, fingerprint_output=DEFAULT_FINGERPRINT,
        p4_limits_output=DEFAULT_P4_LIMITS, p4_fingerprint_output=DEFAULT_P4_FINGERPRINT,
        p4_bindings_output=DEFAULT_P4_BINDINGS)


def check_outputs() -> list[Path]:
    source = _read_json(DEFAULT_SOURCE)
    stale = []
    resource_root = DEFAULT_URDF.parent.parent
    for relative, digest in source["resource_sha256"].items():
        path = resource_root / Path(relative).relative_to(source["package_path"])
        if relative == source["source_urdf_path"]:
            path = DEFAULT_URDF
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            stale.append(path)
    stale.extend(path for path, content in build_outputs(default_args()).items()
                 if not path.is_file() or path.read_text(encoding="utf-8") != content)
    return stale


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("urdf", nargs="?", type=Path, help="Original committed structure URDF to import")
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--refresh", action="store_true", help="Regenerate from the committed local URDF")
    parser.add_argument("--firmware-only", action="store_true")
    for name, default in (("manifest", DEFAULT_MANIFEST), ("mapping-output", DEFAULT_MAPPING),
                          ("fingerprint-output", DEFAULT_FINGERPRINT), ("p4-limits-output", DEFAULT_P4_LIMITS),
                          ("p4-fingerprint-output", DEFAULT_P4_FINGERPRINT)):
        parser.add_argument("--" + name, type=Path, default=default)
    options = parser.parse_args()
    source_path = options.urdf
    try:
        if source_path is None:
            options.urdf = DEFAULT_URDF
            outputs = build_outputs(options)
            if options.check:
                stale = [path for path, content in outputs.items()
                         if not path.is_file() or path.read_text(encoding="utf-8") != content]
                if not options.firmware_only:
                    stale.extend(check_outputs())
                for path in sorted(set(stale)):
                    print(f"generated robot-model file is stale: {path}", file=sys.stderr)
                return int(bool(stale))
            publish_outputs({path: text.encode("utf-8") for path, text in outputs.items()})
            return 0
        manifest = load_manifest(DEFAULT_MANIFEST)
        limits = load_urdf_limits(source_path, manifest)
        source, repository, resources = inspect_source(source_path)
        package = source_path.resolve().parent.parent
        source["package_path"] = package.relative_to(repository).as_posix()
        # 所有输入、资源和生成文本验证完成后才开始发布。
        args = default_args()
        args.urdf = source_path
        fingerprint = build_fingerprint(manifest, limits, source_path, source["source_commit"])
        outputs = {
            DEFAULT_MAPPING: render_mapping(manifest, limits, source["source_commit"], source["source_urdf_path"]),
            DEFAULT_P4_LIMITS: render_p4_limits(manifest, limits),
            DEFAULT_FINGERPRINT: render_fingerprint_json(fingerprint),
            DEFAULT_P4_FINGERPRINT: render_p4_fingerprint(fingerprint),
            DEFAULT_P4_BINDINGS: render_p4_bindings(manifest),
            DEFAULT_SOURCE: json.dumps(source, ensure_ascii=False, indent=2) + "\n",
        }
        resource_bytes = {}
        for relative in source["resource_sha256"]:
            original = repository / relative
            destination = DEFAULT_URDF.parent.parent / original.relative_to(package)
            if original == source_path.resolve():
                destination = DEFAULT_URDF
            resource_bytes[destination] = original.read_bytes()
    except (OSError, ValueError, ET.ParseError) as exc:
        print(f"robot model generation failed: {exc}", file=sys.stderr)
        return 2

    publish_outputs({**resource_bytes, **{path: text.encode("utf-8") for path, text in outputs.items()}})
    return 0


def publish_outputs(outputs: dict[Path, bytes]) -> None:
    """先暂存完整输出集合；替换失败时恢复原文件，避免半套模型。"""
    staged, previous, installed = {}, {}, []
    try:
        for path, content in outputs.items():
            path.parent.mkdir(parents=True, exist_ok=True)
            previous[path] = path.read_bytes() if path.exists() else None
            fd, name = tempfile.mkstemp(prefix=path.name + ".", suffix=".tmp", dir=path.parent)
            staged[path] = Path(name)
            with os.fdopen(fd, "wb") as stream:
                stream.write(content)
                stream.flush()
                os.fsync(stream.fileno())
        for path, temporary in staged.items():
            os.replace(temporary, path)
            installed.append(path)
    except OSError:
        for path in reversed(installed):
            if previous[path] is None:
                path.unlink(missing_ok=True)
            else:
                path.write_bytes(previous[path])
        raise
    finally:
        for temporary in staged.values():
            temporary.unlink(missing_ok=True)


if __name__ == "__main__":
    raise SystemExit(main())
