"""上下位机协议 schema 与生成产物回归测试。"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest

from desktop.core import protocol
from tools import generate_upper_protocol as generator


def _load_generated_python(text: str, path: Path):
    path.write_text(text, encoding="utf-8")
    spec = importlib.util.spec_from_file_location("generated_protocol_test", path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_schema_generates_matching_cpp_and_python_contracts(tmp_path: Path) -> None:
    outputs = generator.build_outputs(generator.DEFAULT_SCHEMA)
    cpp = outputs[generator.DEFAULT_CPP]
    python_text = outputs[generator.DEFAULT_PYTHON]
    module = _load_generated_python(python_text, tmp_path / "generated_protocol.py")

    assert module.ENCODER_COUNT == 21
    assert module.MOTOR_COUNT == 27
    assert module.COMMAND_PAYLOAD_LENGTHS[module.CMD_ANGLE_CTRL] == 84
    assert module.COMMAND_PAYLOAD_LENGTHS[module.CMD_CALIB_LOAD_LIMITS] == 224
    assert "case CMD_ANGLE_CTRL:" in cpp
    assert "case CMD_ANGLE_CTRL: return kFloatPayloadBytes;" in cpp
    assert "static_assert(ENCODER_TOTAL_NUM == 21" in cpp


def test_every_packet_and_command_value_is_unique() -> None:
    schema = json.loads(generator.DEFAULT_SCHEMA.read_text(encoding="utf-8"))
    for key in ("packets", "commands"):
        names = [item["name"] for item in schema[key]]
        values = [item["value"] for item in schema[key]]
        assert len(names) == len(set(names))
        assert len(values) == len(set(values))


def test_duplicate_command_value_is_rejected(tmp_path: Path) -> None:
    schema = json.loads(generator.DEFAULT_SCHEMA.read_text(encoding="utf-8"))
    schema["commands"][1]["value"] = schema["commands"][0]["value"]
    bad_schema = tmp_path / "bad_schema.json"
    bad_schema.write_text(json.dumps(schema), encoding="utf-8")

    with pytest.raises(ValueError, match="duplicate command value"):
        generator.build_outputs(bad_schema)


def test_new_protocol_item_requires_chinese_maintenance_description(tmp_path: Path) -> None:
    """防止以后只增加编号，却没有把命令用途同步进两端生成注释。"""
    schema = json.loads(generator.DEFAULT_SCHEMA.read_text(encoding="utf-8"))
    schema["commands"][0].pop("description")
    bad_schema = tmp_path / "missing_description.json"
    bad_schema.write_text(json.dumps(schema), encoding="utf-8")

    with pytest.raises(ValueError, match="missing description"):
        generator.build_outputs(bad_schema)


def test_protocol_description_cannot_be_english_identifier_only(tmp_path: Path) -> None:
    """维护说明必须真正包含中文，不能退化成重复常量名。"""
    schema = json.loads(generator.DEFAULT_SCHEMA.read_text(encoding="utf-8"))
    schema["packets"][0]["description"] = "sensor packet"
    bad_schema = tmp_path / "english_description.json"
    bad_schema.write_text(json.dumps(schema), encoding="utf-8")

    with pytest.raises(ValueError, match="must contain Chinese"):
        generator.build_outputs(bad_schema)


def test_desktop_frame_builder_rejects_unknown_command() -> None:
    with pytest.raises(ValueError, match="unknown upper command"):
        protocol._build_command_frame(0xFE, b"")


def test_removed_commands_have_no_api_and_retired_numbers_cannot_be_reassigned(tmp_path: Path) -> None:
    schema = json.loads(generator.DEFAULT_SCHEMA.read_text(encoding="utf-8"))
    assert not hasattr(protocol, 'CMD_PAUSE')
    assert not hasattr(protocol, 'CMD_CALIBRATE')
    assert not hasattr(protocol, 'build_calibrate_cmd')
    with pytest.raises(ValueError, match='unknown upper command'):
        protocol._build_command_frame(202)
    assert not any(cmd['name'] in ('CMD_CALIBRATE', 'CMD_PAUSE', 'CMD_FEEDBACK_HOLD', 'CMD_DIRECT_HOLD',
                                  'CMD_CONTROL_TRACE_SUBSCRIBE') for cmd in schema['commands'])
    assert {202, 217, 239, 245, 247} <= set(schema['retired_command_values'])
    assert schema["p4_runtime"]["heartbeats_ms"]["kSystemStatusHeartbeatMs"] == 200
    schema["retired_command_values"] = [250]
    schema["commands"][0]["value"] = 250
    path = tmp_path / "reused.json"
    path.write_text(json.dumps(schema), encoding="utf-8")
    with pytest.raises(ValueError, match="retired command"):
        generator.build_outputs(path)


def test_desktop_frame_builder_rejects_wrong_payload_length() -> None:
    with pytest.raises(ValueError, match="payload length mismatch"):
        protocol._build_command_frame(protocol.CMD_ANGLE_CTRL, b"\x00")
