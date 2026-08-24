"""从唯一 schema 生成 P4 C++ 与 Desktop Python 的协议常量和长度表。"""

from __future__ import annotations

import argparse
import json
import os
import re
import tempfile
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCHEMA = PROJECT_ROOT / "desktop/resources/config/upper_protocol_schema.json"
DEFAULT_CPP = PROJECT_ROOT / "ESP32-P4/ServoBoardMain/src/generated/UpperProtocol.generated.h"
DEFAULT_PYTHON = PROJECT_ROOT / "desktop/core/generated_upper_protocol.py"

# 协议说明最终会生成到 C++ 与 Python 源码中，因此要求至少包含一个中文字符；
# 这样新增编号时不能只复制英文常量名来敷衍用途说明。
_CHINESE_DESCRIPTION_RE = re.compile(r"[\u4e00-\u9fff]")


def _load_schema(path: Path) -> dict:
    """读取并验证人工维护的协议契约，拒绝重复编号和错误产品维度。"""
    data = json.loads(path.read_text(encoding="utf-8"))
    if int(data.get("schema_version", 0)) != 1:
        raise ValueError("upper protocol schema_version must be 1")
    packets = list(data.get("packets", []))
    commands = list(data.get("commands", []))
    for label, items in (("packet", packets), ("command", commands)):
        names = [str(item["name"]) for item in items]
        values = [int(item["value"]) for item in items]
        if len(names) != len(set(names)):
            raise ValueError(f"duplicate {label} name")
        if len(values) != len(set(values)):
            raise ValueError(f"duplicate {label} value")
        # 注释也是协议契约的一部分：新增编号时必须同时说明方向、单位或用途。
        for item in items:
            description = str(item.get("description", "")).strip()
            if not description:
                raise ValueError(f"{label} {item['name']} missing description")
            if not _CHINESE_DESCRIPTION_RE.search(description):
                raise ValueError(f"{label} {item['name']} description must contain Chinese")
            if "\n" in description or "\r" in description or "*/" in description:
                raise ValueError(f"{label} {item['name']} description must be one safe comment line")
    for group in ("proto_ack_status", "manual_save_status", "state_sync_masks"):
        descriptions = data.get(f"{group}_descriptions", {})
        missing = [name for name in data.get(group, {}) if not str(descriptions.get(name, "")).strip()]
        if missing:
            raise ValueError(f"{group} missing descriptions: {', '.join(missing)}")
        non_chinese = [
            name
            for name in data.get(group, {})
            if not _CHINESE_DESCRIPTION_RE.search(str(descriptions.get(name, "")))
        ]
        if non_chinese:
            raise ValueError(
                f"{group} descriptions must contain Chinese: {', '.join(non_chinese)}"
            )
    dimensions = data.get("dimensions", {})
    if int(dimensions.get("encoder_count", 0)) != 21 or int(dimensions.get("motor_count", 0)) != 27:
        raise ValueError("current protocol requires exactly 21 joints and 27 motors")
    return data


def _render_cpp(schema: dict) -> str:
    """生成 P4 使用的 constexpr 常量与命令 payload 长度查询函数。"""
    frame = schema["frame"]
    dimensions = schema["dimensions"]
    lines = [
        "/**",
        " * @file UpperProtocol.generated.h",
        " * @brief 由 upper_protocol_schema.json 自动生成的 P4 协议常量和长度表。",
        " * @warning 禁止手工编辑；请运行 python tools/generate_upper_protocol.py。",
        " */",
        "#ifndef MMHAND_UPPER_PROTOCOL_GENERATED_H",
        "#define MMHAND_UPPER_PROTOCOL_GENERATED_H",
        "",
        "#include <stddef.h>",
        "#include <stdint.h>",
        "#include \"../config/DimensionsConfig.h\"",
        "",
        f"inline constexpr uint8_t PROTOCOL_HEADER = 0x{int(frame['header']):02X};",
        f"inline constexpr uint8_t PROTOCOL_TAIL = 0x{int(frame['tail']):02X};",
        f"inline constexpr int16_t PROTOCOL_DISCONNECT_SENTINEL = {int(frame['disconnect_sentinel'])};",
        f"inline constexpr size_t kP4TimestampBytes = {int(frame['p4_timestamp_bytes'])};",
        "",
        f"static_assert(ENCODER_TOTAL_NUM == {int(dimensions['encoder_count'])}, \"Protocol joint count drift\");",
        f"static_assert(SERVO_TOTAL_NUM == {int(dimensions['motor_count'])}, \"Protocol motor count drift\");",
        "",
        "// P4 -> Desktop 包类型。",
    ]
    for item in schema["packets"]:
        name = item.get("cpp_name", item["name"])
        description = str(item.get("description", "未填写用途"))
        lines.append(
            f"inline constexpr uint8_t {name} = 0x{int(item['value']):02X}; "
            f"///< {description}"
        )
    lines.extend(["", "// Desktop -> P4 命令及 ACK 状态。"])
    for item in schema["commands"]:
        description = str(item.get("description", "未填写用途"))
        lines.append(
            f"inline constexpr uint8_t {item['name']} = 0x{int(item['value']):02X}; "
            f"///< {description}"
        )
    for group in ("proto_ack_status", "manual_save_status", "state_sync_masks"):
        descriptions = schema.get(f"{group}_descriptions", {})
        for name, value in schema[group].items():
            description = str(descriptions.get(name, "未填写用途"))
            lines.append(
                f"inline constexpr uint8_t {name} = {int(value)}; "
                f"///< {description}"
            )

    # 命名长度常量供业务解析器和生成长度查询函数共享，避免头文件内出现第二份魔数。
    lengths = {item["name"]: int(item["payload_bytes"]) for item in schema["commands"]}
    aliases = {
        "kFloatPayloadBytes": "CMD_ANGLE_CTRL",
        "kMaskedFloatPayloadBytes": "CMD_ANGLE_CTRL_MASKED",
        "kMotorPosPayloadBytes": "CMD_MOTOR_POS",
        "kMotorPosAbsPayloadBytes": "CMD_MOTOR_POS_ABS",
        "kMotorPosAbsOnePayloadBytes": "CMD_MOTOR_POS_ABS_ONE",
        "kActiveScopePayloadBytes": "CMD_ACTIVE_SCOPE",
        "kRestoreMultiTurnPayloadBytes": "CMD_RESTORE_MULTITURN",
        "kCalibLoadJointPayloadBytes": "CMD_CALIB_LOAD_JOINT",
        "kCalibLoadLimitsPayloadBytes": "CMD_CALIB_LOAD_LIMITS",
        "kManualEncoderReferencePayloadBytes": "CMD_MANUAL_ENCODER_REFERENCE",
        "kManualEncoderSinglePayloadBytes": "CMD_MANUAL_ENCODER_SINGLE",
        "kRopeInitPayloadBytes": "CMD_ROPE_INIT",
        "kClearCalibrationScopePayloadBytes": "CMD_CLEAR_CALIBRATION_SCOPE",
        "kServoResetZeroPayloadBytes": "CMD_SERVO_RESET_ZERO",
        "kStateSyncRequestPayloadBytes": "CMD_STATE_SYNC_REQUEST",
        "kTaskDiagSubscribePayloadBytes": "CMD_TASK_DIAG_SUBSCRIBE",
        "kControlTraceSubscribePayloadBytes": "CMD_CONTROL_TRACE_SUBSCRIBE",
    }
    lines.extend(["", "// 固定 payload 长度；单位 byte，不包含命令字本身。"])
    for alias, command in aliases.items():
        lines.append(f"inline constexpr size_t {alias} = {lengths[command]};")
    lines.extend(
        [
            f"inline constexpr size_t kSerialRxBufferSize = {int(schema['p4_runtime']['serial_rx_buffer_size'])};",
            f"inline constexpr int32_t kEncoderModulo = {int(dimensions['encoder_raw_modulo'])};",
            "",
            "/** 查询固定命令 payload 长度；未知命令返回 SIZE_MAX。 */",
            "inline constexpr size_t generatedCommandPayloadLength(uint8_t command)",
            "{",
            "    switch (command) {",
        ]
    )
    alias_for_command = {command: alias for alias, command in aliases.items()}
    # 连续直控与普通单圈直控使用完全相同的 27 路 uint16 payload。
    alias_for_command["CMD_MOTOR_POS_SWEEP"] = "kMotorPosPayloadBytes"
    for item in schema["commands"]:
        name = item["name"]
        return_value = alias_for_command.get(name, str(int(item["payload_bytes"])))
        lines.append(f"        case {name}: return {return_value};")
    lines.extend(["        default: return SIZE_MAX;", "    }", "}", ""])
    for name, value in schema["p4_runtime"]["heartbeats_ms"].items():
        lines.append(
            f"inline constexpr uint32_t {name} = {int(value)}; "
            "///< 对应上行包无变化时的强制心跳周期，单位 ms。"
        )
    lines.extend(["", "#endif // MMHAND_UPPER_PROTOCOL_GENERATED_H", ""])
    return "\n".join(lines)


def _render_python(schema: dict) -> str:
    """生成 Desktop 使用的协议常量、时间戳包集合与长度字典。"""
    frame = schema["frame"]
    dimensions = schema["dimensions"]
    lines = [
        '"""由 upper_protocol_schema.json 自动生成的 Desktop 协议常量。',
        '',
        '职责：提供上下位机共享的包类型、命令编号和固定长度。',
        '线程：纯常量模块，不创建线程，可由任意线程只读导入。',
        '输入：tools/generate_upper_protocol.py 读取的唯一协议 schema。',
        '输出：供 protocol.py 和协议测试读取的不可变常量。',
        '状态所有权：不拥有运行状态；文件内容由生成器独占维护。',
        '禁止依赖：禁止手工编辑，禁止导入 UI、通信对象或业务状态。',
        '"""',
        "",
        f"PROTOCOL_HEADER = 0x{int(frame['header']):02X}",
        f"PROTOCOL_TAIL = 0x{int(frame['tail']):02X}",
        f"DISCONNECT_SENTINEL = {int(frame['disconnect_sentinel'])}",
        f"P4_TIMESTAMP_BYTES = {int(frame['p4_timestamp_bytes'])}",
        f"ENCODER_COUNT = {int(dimensions['encoder_count'])}",
        f"MOTOR_COUNT = {int(dimensions['motor_count'])}",
        f"ENCODER_RAW_MODULO = {int(dimensions['encoder_raw_modulo'])}",
        "",
        "# P4 -> Desktop 包类型。",
    ]
    for item in schema["packets"]:
        lines.append(
            f"{item['name']} = 0x{int(item['value']):02X}  "
            f"# {item.get('description', '未填写用途')}"
        )
    lines.extend(["", "# Desktop -> P4 命令。"])
    for item in schema["commands"]:
        lines.append(
            f"{item['name']} = 0x{int(item['value']):02X}  "
            f"# {item.get('description', '未填写用途')}"
        )
    for group in ("proto_ack_status", "manual_save_status", "state_sync_masks"):
        lines.append("")
        descriptions = schema.get(f"{group}_descriptions", {})
        for name, value in schema[group].items():
            lines.append(
                f"{name} = {int(value)}  "
                f"# {descriptions.get(name, '未填写用途')}"
            )
    lines.extend(
        [
            "",
            "# 当前 P4 会在这些上行包的业务负载末尾追加 uint32 毫秒时间戳。",
            "P4_TRAILING_TIMESTAMP_PACKET_TYPES = frozenset({",
        ]
    )
    for item in schema["packets"]:
        if item.get("timestamp"):
            lines.append(f"    {item['name']},")
    lines.extend(["})", "", "# 固定下行 payload 长度，不包含命令字。", "COMMAND_PAYLOAD_LENGTHS = {"])
    for item in schema["commands"]:
        lines.append(f"    {item['name']}: {int(item['payload_bytes'])},")
    lines.extend(["}", ""])
    return "\n".join(lines)


def _write_atomic(path: Path, text: str) -> None:
    """同目录临时文件写完后原子替换，避免中断时留下半个生成文件。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temp_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent, text=True)
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="\n") as handle:
            handle.write(text)
        os.replace(temp_name, path)
    finally:
        if os.path.exists(temp_name):
            os.unlink(temp_name)


def build_outputs(schema_path: Path) -> dict[Path, str]:
    """构造两个目标文件内容；测试可直接调用而不触碰正式文件。"""
    schema = _load_schema(schema_path)
    return {
        DEFAULT_CPP: _render_cpp(schema),
        DEFAULT_PYTHON: _render_python(schema),
    }


def main() -> int:
    """执行生成或 --check 陈旧检查，并以进程退出码供 CI/架构门禁使用。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--schema", type=Path, default=DEFAULT_SCHEMA)
    parser.add_argument("--check", action="store_true", help="只检查生成物是否陈旧")
    args = parser.parse_args()
    outputs = build_outputs(args.schema.resolve())
    stale = [path for path, content in outputs.items() if not path.exists() or path.read_text(encoding="utf-8") != content]
    if args.check:
        if stale:
            for path in stale:
                print(f"STALE: {path.relative_to(PROJECT_ROOT)}")
            return 1
        print("Upper protocol generated artifacts are current.")
        return 0
    for path, content in outputs.items():
        _write_atomic(path, content)
        print(f"WROTE: {path.relative_to(PROJECT_ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
